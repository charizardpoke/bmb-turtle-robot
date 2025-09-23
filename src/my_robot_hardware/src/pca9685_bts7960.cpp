#include "my_robot_hardware/pca9685_bts7960.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace mrh = my_robot_hardware;

// PCA9685 registers
static constexpr std::uint8_t MODE1     = 0x00;
static constexpr std::uint8_t MODE2     = 0x01;
static constexpr std::uint8_t PRESCALE  = 0xFE;
static constexpr std::uint8_t LED0_ON_L = 0x06;

static inline std::uint8_t REG_ON_L (int ch){ return LED0_ON_L + 4*ch + 0; }
static inline std::uint8_t REG_ON_H (int ch){ return LED0_ON_L + 4*ch + 1; }
static inline std::uint8_t REG_OFF_L(int ch){ return LED0_ON_L + 4*ch + 2; }
static inline std::uint8_t REG_OFF_H(int ch){ return LED0_ON_L + 4*ch + 3; }

// I2C helpers
namespace {
inline int i2c_write8(int fd, std::uint8_t reg, std::uint8_t val) {
  std::uint8_t b[2] = {reg, val};
  return (::write(fd, b, 2) == 2) ? 0 : -1;
}
inline int i2c_read8(int fd, std::uint8_t reg, std::uint8_t &val) {
  if (::write(fd, &reg, 1) != 1) return -1;
  return (::read(fd, &val, 1) == 1) ? 0 : -1;
}
inline int i2c_write_block(int fd, std::uint8_t start_reg, const std::uint8_t *data, std::size_t len) {
  if (len == 0 || len > 4) return -1;
  std::uint8_t buf[1 + 4];
  buf[0] = start_reg;
  std::memcpy(buf + 1, data, len);
  return (::write(fd, buf, 1 + len) == static_cast<ssize_t>(1 + len)) ? 0 : -1;
}
} // namespace

// --- lifecycle ---
CallbackReturn mrh::PCA9685BTS7960::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  auto parse_int = [](const std::string &s){ return std::stoi(s, nullptr, 0); };
  auto parse_bool = [](const std::string &s){
    return s=="1"||s=="true"||s=="True"||s=="TRUE"||s=="yes"||s=="on";
  };

  try {
    if (auto it=info.hardware_parameters.find("dry_run"); it!=info.hardware_parameters.end()) dry_run_ = parse_bool(it->second);
    if (auto it=info.hardware_parameters.find("i2c_bus"); it!=info.hardware_parameters.end()) i2c_bus_ = parse_int(it->second);
    if (auto it=info.hardware_parameters.find("i2c_addr"); it!=info.hardware_parameters.end()) addr_ = static_cast<std::uint8_t>(parse_int(it->second));
    if (auto it=info.hardware_parameters.find("pwm_hz"); it!=info.hardware_parameters.end()) pwm_hz_ = std::stod(it->second);

    if (auto it=info.hardware_parameters.find("left_lpw"); it!=info.hardware_parameters.end())  ch_[0] = parse_int(it->second);
    if (auto it=info.hardware_parameters.find("left_rpw"); it!=info.hardware_parameters.end())  ch_[1] = parse_int(it->second);
    if (auto it=info.hardware_parameters.find("right_lpw"); it!=info.hardware_parameters.end()) ch_[2] = parse_int(it->second);
    if (auto it=info.hardware_parameters.find("right_rpw"); it!=info.hardware_parameters.end()) ch_[3] = parse_int(it->second);

    if (auto it=info.hardware_parameters.find("max_speed_rad_s"); it!=info.hardware_parameters.end()) max_speed_rad_s_ = std::max(0.1, std::stod(it->second));
    if (auto it=info.hardware_parameters.find("deadband"); it!=info.hardware_parameters.end())       deadband_ = std::clamp(std::stod(it->second), 0.0, 0.3);
    if (auto it=info.hardware_parameters.find("invert_left"); it!=info.hardware_parameters.end())    invert_left_  = parse_bool(it->second);
    if (auto it=info.hardware_parameters.find("invert_right"); it!=info.hardware_parameters.end())   invert_right_ = parse_bool(it->second);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Param parse error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Expect exactly 2 joints, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto &ji : info_.joints) {
    if (ji.command_interfaces.size() != 1 || ji.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Joint %s must have velocity command", ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (ji.state_interfaces.size() < 2) {
      RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Joint %s must have position and velocity state", ji.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  pos_.fill(0.0);
  vel_.fill(0.0);
  cmd_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
mrh::PCA9685BTS7960::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  v.reserve(4);
  v.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_[0]);
  v.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_[0]);
  v.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &pos_[1]);
  v.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_[1]);
  return v;
}

std::vector<hardware_interface::CommandInterface>
mrh::PCA9685BTS7960::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  v.reserve(2);
  v.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_[0]);
  v.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_[1]);
  return v;
}

CallbackReturn mrh::PCA9685BTS7960::on_activate(const rclcpp_lifecycle::State &)
{
  if (dry_run_) {
    RCLCPP_WARN(rclcpp::get_logger("PCA9685BTS7960"), "dry_run=true: skipping I2C init");
    return CallbackReturn::SUCCESS;
  }

  const std::string dev = "/dev/i2c-" + std::to_string(i2c_bus_);
  i2c_fd_ = ::open(dev.c_str(), O_RDWR);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Open %s failed", dev.c_str());
    return CallbackReturn::ERROR;
  }
  if (::ioctl(i2c_fd_, I2C_SLAVE, addr_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Set I2C addr 0x%02X failed", addr_);
    ::close(i2c_fd_); i2c_fd_ = -1;
    return CallbackReturn::ERROR;
  }

  // Push-pull outputs (OUTDRV=1), non-inverted (INVRT=0)
  (void)i2c_write8(i2c_fd_, MODE2, 0x04);

  // Wake and set PWM frequency
  std::uint8_t oldmode = 0;
  if (i2c_read8(i2c_fd_, MODE1, oldmode) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Read MODE1 failed");
    return CallbackReturn::ERROR;
  }
  if (i2c_write8(i2c_fd_, MODE1, static_cast<std::uint8_t>(oldmode & ~0x10)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Write MODE1 failed");
    return CallbackReturn::ERROR;
  }
  pca_set_freq(pwm_hz_);
  // Defensive: reassert MODE2 after prescale sequence
  (void)i2c_write8(i2c_fd_, MODE2, 0x04);

  // Coast all channels
  for (int i = 0; i < 4; ++i) pca_set_pwm(ch_[i], 0, 0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn mrh::PCA9685BTS7960::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (!dry_run_) {
    for (int i = 0; i < 4; ++i) if (i2c_fd_ >= 0) pca_set_pwm(ch_[i], 0, 0);
    if (i2c_fd_ >= 0) { ::close(i2c_fd_); i2c_fd_ = -1; }
  }
  return CallbackReturn::SUCCESS;
}

return_type mrh::PCA9685BTS7960::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  const double dt = period.seconds();
  vel_[0] = cmd_[0];
  vel_[1] = cmd_[1];
  pos_[0] += vel_[0] * dt;
  pos_[1] += vel_[1] * dt;
  return return_type::OK;
}

return_type mrh::PCA9685BTS7960::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto to_duty = [&](double cmd, bool invert){
    if (invert) cmd = -cmd;
    double u = cmd / std::max(0.1, max_speed_rad_s_);
    u = std::clamp(u, -1.0, 1.0);
    if (std::fabs(u) < deadband_) u = 0.0;
    return u;
  };

  const double uL = to_duty(cmd_[0], invert_left_);
  const double uR = to_duty(cmd_[1], invert_right_);

  RCLCPP_DEBUG(rclcpp::get_logger("PCA9685BTS7960"),
               "cmd L=%.3f R=%.3f  uL=%.3f uR=%.3f", cmd_[0], cmd_[1], uL, uR);

  if (!dry_run_) {
    drive_motor(0, uL);
    drive_motor(1, uR);
  }
  return return_type::OK;
}

// --- PCA9685 helpers ---
void mrh::PCA9685BTS7960::pca_set_freq(double hz) noexcept
{
  if (dry_run_ || i2c_fd_ < 0) return;

  hz = std::clamp(hz, 24.0, 1526.0);
  double prescale_d = std::round(25e6 / (4096.0 * hz)) - 1.0;
  std::uint8_t prescale = static_cast<std::uint8_t>(std::clamp(prescale_d, 3.0, 255.0));

  std::uint8_t oldmode = 0;
  if (i2c_read8(i2c_fd_, MODE1, oldmode) < 0) return;

  const std::uint8_t sleep = static_cast<std::uint8_t>((oldmode & 0x7F) | 0x10);
  (void)i2c_write8(i2c_fd_, MODE1, sleep);
  (void)i2c_write8(i2c_fd_, PRESCALE, prescale);
  (void)i2c_write8(i2c_fd_, MODE1, oldmode);
  ::usleep(5000);
  (void)i2c_write8(i2c_fd_, MODE1, static_cast<std::uint8_t>(oldmode | 0xA1)); // AI|RESTART|ALLCALL
}

void mrh::PCA9685BTS7960::pca_set_pwm(int ch, std::uint16_t on, std::uint16_t off) noexcept
{
  if (dry_run_ || i2c_fd_ < 0) return;

  // exact 0% and 100% use FULL_OFF / FULL_ON
  if (off == 0) {
    (void)i2c_write8(i2c_fd_, REG_ON_H(ch),  0x00);
    (void)i2c_write8(i2c_fd_, REG_OFF_H(ch), 0x10);
    return;
  }
  if (off >= 4095) {
    (void)i2c_write8(i2c_fd_, REG_OFF_H(ch), 0x00);
    (void)i2c_write8(i2c_fd_, REG_ON_H(ch),  0x10);
    return;
  }
  const std::uint8_t data[4] = { 0x00, 0x00,
    static_cast<std::uint8_t>(off & 0xFF),
    static_cast<std::uint8_t>((off >> 8) & 0x0F) };
  (void)i2c_write_block(i2c_fd_, REG_ON_L(ch), data, 4);
}

// --- BTS7960 mapping ---
void mrh::PCA9685BTS7960::drive_motor(int idx, double u) noexcept
{
  const int lp = (idx == 0) ? ch_[0] : ch_[2];
  const int rp = (idx == 0) ? ch_[1] : ch_[3];

  const double duty = std::fabs(u);
  const double duty_clamped = std::min(1.0, duty);
  std::uint16_t off = static_cast<std::uint16_t>(std::round(duty_clamped * 4095.0));
  if (duty_clamped == 0.0) off = 0;
  if (duty_clamped == 1.0) off = 4095;

  if (u > 0.0) {              // forward: RPWM = duty, LPWM = 0
    pca_set_pwm(lp, 0, 0);
    pca_set_pwm(rp, 0, off);
  } else if (u < 0.0) {       // reverse: LPWM = duty, RPWM = 0
    pca_set_pwm(rp, 0, 0);
    pca_set_pwm(lp, 0, off);
  } else {                    // coast
    pca_set_pwm(lp, 0, 0);
    pca_set_pwm(rp, 0, 0);
  }
}

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::PCA9685BTS7960,
                       hardware_interface::SystemInterface)
