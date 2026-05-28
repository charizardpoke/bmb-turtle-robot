#include "my_robot_hardware/pca9685_bts7960.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <lgpio.h>
#include <pluginlib/class_list_macros.hpp>

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
namespace mrh = my_robot_hardware;

// PCA9685 registers
static constexpr std::uint8_t MODE1     = 0x00;
static constexpr std::uint8_t MODE2     = 0x01;
static constexpr std::uint8_t PRESCALE  = 0xFE;
static constexpr std::uint8_t LED0_ON_L = 0x06;

static inline std::uint8_t REG_ON_L(int ch)  { return LED0_ON_L + 4 * ch + 0; }
static inline std::uint8_t REG_ON_H(int ch)  { return LED0_ON_L + 4 * ch + 1; }
static inline std::uint8_t REG_OFF_H(int ch) { return LED0_ON_L + 4 * ch + 3; }

namespace
{
inline int i2c_write8(int fd, std::uint8_t reg, std::uint8_t val)
{
  std::uint8_t b[2] = {reg, val};
  return (::write(fd, b, 2) == 2) ? 0 : -1;
}

inline int i2c_read8(int fd, std::uint8_t reg, std::uint8_t & val)
{
  if (::write(fd, &reg, 1) != 1) return -1;
  return (::read(fd, &val, 1) == 1) ? 0 : -1;
}

inline int i2c_write_block(
  int fd,
  std::uint8_t start_reg,
  const std::uint8_t * data,
  std::size_t len)
{
  if (len == 0 || len > 4) return -1;

  std::uint8_t buf[1 + 4];
  buf[0] = start_reg;
  std::memcpy(buf + 1, data, len);

  return (::write(fd, buf, 1 + len) == static_cast<ssize_t>(1 + len)) ? 0 : -1;
}
}  // namespace

CallbackReturn mrh::PCA9685BTS7960::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto parse_int = [](const std::string & s) {
    return std::stoi(s, nullptr, 0);
  };

  auto parse_bool = [](const std::string & s) {
    return s == "1" || s == "true" || s == "True" || s == "TRUE" ||
           s == "yes" || s == "on";
  };

  try {
    // Basic hardware params
    if (auto it = info.hardware_parameters.find("dry_run"); it != info.hardware_parameters.end()) {
      dry_run_ = parse_bool(it->second);
    }

    if (auto it = info.hardware_parameters.find("i2c_bus"); it != info.hardware_parameters.end()) {
      i2c_bus_ = parse_int(it->second);
    }

    if (auto it = info.hardware_parameters.find("i2c_addr"); it != info.hardware_parameters.end()) {
      addr_ = static_cast<std::uint8_t>(parse_int(it->second));
    }

    if (auto it = info.hardware_parameters.find("pwm_hz"); it != info.hardware_parameters.end()) {
      pwm_hz_ = std::stod(it->second);
    }

    // PCA9685 channels
    if (auto it = info.hardware_parameters.find("left_lpw"); it != info.hardware_parameters.end()) {
      ch_[0] = parse_int(it->second);
    }

    if (auto it = info.hardware_parameters.find("left_rpw"); it != info.hardware_parameters.end()) {
      ch_[1] = parse_int(it->second);
    }

    if (auto it = info.hardware_parameters.find("right_lpw"); it != info.hardware_parameters.end()) {
      ch_[2] = parse_int(it->second);
    }

    if (auto it = info.hardware_parameters.find("right_rpw"); it != info.hardware_parameters.end()) {
      ch_[3] = parse_int(it->second);
    }

    // Motor control params
    if (auto it = info.hardware_parameters.find("max_speed_rad_s"); it != info.hardware_parameters.end()) {
      max_speed_rad_s_ = std::max(0.1, std::stod(it->second));
    }

    if (auto it = info.hardware_parameters.find("deadband"); it != info.hardware_parameters.end()) {
      deadband_ = std::clamp(std::stod(it->second), 0.0, 0.3);
    }

    if (auto it = info.hardware_parameters.find("invert_left"); it != info.hardware_parameters.end()) {
      invert_left_ = parse_bool(it->second);
    }

    if (auto it = info.hardware_parameters.find("invert_right"); it != info.hardware_parameters.end()) {
      invert_right_ = parse_bool(it->second);
    }

    // Encoder params
    if (auto it = info.hardware_parameters.find("use_encoders"); it != info.hardware_parameters.end()) {
      use_encoders_ = parse_bool(it->second);
    }

    if (auto it = info.hardware_parameters.find("gpio_chip"); it != info.hardware_parameters.end()) {
      gpio_chip_ = parse_int(it->second);
    }

    // Your current encoder wiring:
    // Left:  Yellow A = GPIO22, White B = GPIO23
    // Right: Yellow A = GPIO17, White B = GPIO27
    if (auto it = info.hardware_parameters.find("left_encoder_a"); it != info.hardware_parameters.end()) {
      left_encoder_a_ = parse_int(it->second);
    } else {
      left_encoder_a_ = 22;
    }

    if (auto it = info.hardware_parameters.find("left_encoder_b"); it != info.hardware_parameters.end()) {
      left_encoder_b_ = parse_int(it->second);
    } else {
      left_encoder_b_ = 23;
    }

    if (auto it = info.hardware_parameters.find("right_encoder_a"); it != info.hardware_parameters.end()) {
      right_encoder_a_ = parse_int(it->second);
    } else {
      right_encoder_a_ = 17;
    }

    if (auto it = info.hardware_parameters.find("right_encoder_b"); it != info.hardware_parameters.end()) {
      right_encoder_b_ = parse_int(it->second);
    } else {
      right_encoder_b_ = 27;
    }

    if (auto it = info.hardware_parameters.find("encoder_cpr"); it != info.hardware_parameters.end()) {
      encoder_cpr_ = std::max(1.0, std::stod(it->second));
    }

    if (auto it = info.hardware_parameters.find("invert_left_encoder"); it != info.hardware_parameters.end()) {
      invert_left_encoder_ = parse_bool(it->second);
    }

    if (auto it = info.hardware_parameters.find("invert_right_encoder"); it != info.hardware_parameters.end()) {
      invert_right_encoder_ = parse_bool(it->second);
    }

    // Relay params
    if (auto it = info.hardware_parameters.find("use_relays"); it != info.hardware_parameters.end()) {
      use_relays_ = parse_bool(it->second);
    }

    // Your relay wiring:
    // IN2 / GPIO25 = left motor power relay
    // IN1 / GPIO24 = right motor power relay
    if (auto it = info.hardware_parameters.find("left_relay_pin"); it != info.hardware_parameters.end()) {
      left_relay_pin_ = parse_int(it->second);
    } else {
      left_relay_pin_ = 25;
    }

    if (auto it = info.hardware_parameters.find("right_relay_pin"); it != info.hardware_parameters.end()) {
      right_relay_pin_ = parse_int(it->second);
    } else {
      right_relay_pin_ = 24;
    }

    if (auto it = info.hardware_parameters.find("relay_active_low"); it != info.hardware_parameters.end()) {
      relay_active_low_ = parse_bool(it->second);
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Param parse error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PCA9685BTS7960"),
      "Expect exactly 2 joints, got %zu",
      info_.joints.size());
    return CallbackReturn::ERROR;
  }

  // Detect left/right joint index from URDF joint names
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & name = info_.joints[i].name;

    if (name.find("left") != std::string::npos) {
      left_joint_index_ = static_cast<int>(i);
    }

    if (name.find("right") != std::string::npos) {
      right_joint_index_ = static_cast<int>(i);
    }
  }

  for (const auto & ji : info_.joints) {
    if (ji.command_interfaces.size() != 1 ||
        ji.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("PCA9685BTS7960"),
        "Joint %s must have velocity command",
        ji.name.c_str());
      return CallbackReturn::ERROR;
    }

    if (ji.state_interfaces.size() < 2) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PCA9685BTS7960"),
        "Joint %s must have position and velocity state",
        ji.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  pos_.fill(0.0);
  last_pos_.fill(0.0);
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
    RCLCPP_WARN(rclcpp::get_logger("PCA9685BTS7960"), "dry_run=true: skipping hardware init");
    return CallbackReturn::SUCCESS;
  }

  // Open PCA9685 I2C
  const std::string dev = "/dev/i2c-" + std::to_string(i2c_bus_);
  i2c_fd_ = ::open(dev.c_str(), O_RDWR);

  if (i2c_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Open %s failed", dev.c_str());
    return CallbackReturn::ERROR;
  }

  if (::ioctl(i2c_fd_, I2C_SLAVE, addr_) < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("PCA9685BTS7960"),
      "Set I2C addr 0x%02X failed",
      addr_);

    ::close(i2c_fd_);
    i2c_fd_ = -1;
    return CallbackReturn::ERROR;
  }

  // Push-pull outputs, non-inverted
  (void)i2c_write8(i2c_fd_, MODE2, 0x04);

  std::uint8_t oldmode = 0;
  if (i2c_read8(i2c_fd_, MODE1, oldmode) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Read MODE1 failed");
    ::close(i2c_fd_);
    i2c_fd_ = -1;
    return CallbackReturn::ERROR;
  }

  if (i2c_write8(i2c_fd_, MODE1, static_cast<std::uint8_t>(oldmode & ~0x10)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685BTS7960"), "Write MODE1 failed");
    ::close(i2c_fd_);
    i2c_fd_ = -1;
    return CallbackReturn::ERROR;
  }

  pca_set_freq(pwm_hz_);

  // Reassert MODE2 after frequency setup
  (void)i2c_write8(i2c_fd_, MODE2, 0x04);

  // Stop all motor PWM before turning relay power on
  for (int i = 0; i < 4; ++i) {
    pca_set_pwm(ch_[i], 0, 0);
  }

  // Open Raspberry Pi GPIO chip for encoders and relays
  if (use_encoders_ || use_relays_) {
    gpio_handle_ = lgGpiochipOpen(gpio_chip_);

    if (gpio_handle_ < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PCA9685BTS7960"),
        "Could not open gpiochip%d. Check GPIO permission.",
        gpio_chip_);

      ::close(i2c_fd_);
      i2c_fd_ = -1;
      return CallbackReturn::ERROR;
    }

    // Claim relay outputs first.
    // Keep relays OFF during startup.
    if (use_relays_) {
      const int relay_off = relay_active_low_ ? 1 : 0;

      lgGpioClaimOutput(gpio_handle_, 0, left_relay_pin_, relay_off);
      lgGpioClaimOutput(gpio_handle_, 0, right_relay_pin_, relay_off);

      relays_claimed_ = true;

      RCLCPP_INFO(
        rclcpp::get_logger("PCA9685BTS7960"),
        "Relays ready: LEFT IN2 GPIO%d, RIGHT IN1 GPIO%d",
        left_relay_pin_,
        right_relay_pin_);
    }

    // Claim encoder inputs
    if (use_encoders_) {
      encoder_enabled_[0] = (left_encoder_a_ >= 0 || left_encoder_b_ >= 0);
      encoder_enabled_[1] = (right_encoder_a_ >= 0 || right_encoder_b_ >= 0);
      
      if (encoder_enabled_[0]) {
        if (left_encoder_a_ >= 0) {
          lgGpioClaimInput(gpio_handle_, 0, left_encoder_a_);
        }
        if (left_encoder_b_ >= 0) {
          lgGpioClaimInput(gpio_handle_, 0, left_encoder_b_);
        }
      }

      if (encoder_enabled_[1]) {
        if (right_encoder_a_ >= 0) {
          lgGpioClaimInput(gpio_handle_, 0, right_encoder_a_);
        }
        if (right_encoder_b_ >= 0) {
          lgGpioClaimInput(gpio_handle_, 0, right_encoder_b_);
        }
      }

      encoder_running_ = true;
      encoder_thread_ = std::thread(&PCA9685BTS7960::encoder_loop, this);

      RCLCPP_INFO(
        rclcpp::get_logger("PCA9685BTS7960"),
        "Encoder thread started. LEFT A=%d B=%d, RIGHT A=%d B=%d",
        left_encoder_a_,
        left_encoder_b_,
        right_encoder_a_,
        right_encoder_b_);
    }

    // Turn motor power relays ON after PWM outputs are stopped
    set_motor_power(true, true);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn mrh::PCA9685BTS7960::on_deactivate(const rclcpp_lifecycle::State &)
{
  // First stop PWM so motors are not commanded
  if (!dry_run_ && i2c_fd_ >= 0) {
    for (int i = 0; i < 4; ++i) {
      pca_set_pwm(ch_[i], 0, 0);
    }
  }

  // Small delay before cutting motor power
  ::usleep(100000);

  // Then turn motor relays OFF
  set_motor_power(false, false);

  // Stop encoder thread
  encoder_running_ = false;

  if (encoder_thread_.joinable()) {
    encoder_thread_.join();
  }

  // Close GPIO
  if (gpio_handle_ >= 0) {
    lgGpiochipClose(gpio_handle_);
    gpio_handle_ = -1;
  }

  relays_claimed_ = false;

  // Close I2C
  if (!dry_run_) {
    if (i2c_fd_ >= 0) {
      ::close(i2c_fd_);
      i2c_fd_ = -1;
    }
  }

  return CallbackReturn::SUCCESS;
}

return_type mrh::PCA9685BTS7960::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  double dt = period.seconds();

  if (dt <= 0.0) {
    dt = 0.02;
  }

  constexpr double TWO_PI = 6.28318530717958647692;

  long long left_count = 0;
  long long right_count = 0;

  {
    std::lock_guard<std::mutex> lock(encoder_mutex_);
    left_count = encoder_count_[0];
    right_count = encoder_count_[1];
  }

  // Left wheel odometry from encoder
  if (encoder_enabled_[0]) {
    pos_[left_joint_index_] =
      static_cast<double>(left_count) * TWO_PI / encoder_cpr_;

    vel_[left_joint_index_] =
      (pos_[left_joint_index_] - last_pos_[left_joint_index_]) / dt;

    last_pos_[left_joint_index_] = pos_[left_joint_index_];
  } else {
    // Fallback fake odometry
    vel_[left_joint_index_] = cmd_[left_joint_index_];
    pos_[left_joint_index_] += vel_[left_joint_index_] * dt;
  }

  // Right wheel odometry from encoder
  if (encoder_enabled_[1]) {
    pos_[right_joint_index_] =
      static_cast<double>(right_count) * TWO_PI / encoder_cpr_;

    vel_[right_joint_index_] =
      (pos_[right_joint_index_] - last_pos_[right_joint_index_]) / dt;

    last_pos_[right_joint_index_] = pos_[right_joint_index_];
  } else {
    // Fallback fake odometry
    vel_[right_joint_index_] = cmd_[right_joint_index_];
    pos_[right_joint_index_] += vel_[right_joint_index_] * dt;
  }

  return return_type::OK;
}

return_type mrh::PCA9685BTS7960::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto to_duty = [&](double command, bool invert) {
    if (invert) {
      command = -command;
    }

    double u = command / std::max(0.1, max_speed_rad_s_);
    u = std::clamp(u, -1.0, 1.0);

    if (std::fabs(u) < deadband_) {
      u = 0.0;
    }

    return u;
  };

  const double uL = to_duty(cmd_[left_joint_index_], invert_left_);
  const double uR = to_duty(cmd_[right_joint_index_], invert_right_);

  RCLCPP_DEBUG(
    rclcpp::get_logger("PCA9685BTS7960"),
    "cmd L=%.3f R=%.3f uL=%.3f uR=%.3f",
    cmd_[left_joint_index_],
    cmd_[right_joint_index_],
    uL,
    uR);

  if (!dry_run_) {
    drive_motor(0, uL);
    drive_motor(1, uR);
  }

  return return_type::OK;
}

void mrh::PCA9685BTS7960::pca_set_freq(double hz) noexcept
{
  if (dry_run_ || i2c_fd_ < 0) {
    return;
  }

  hz = std::clamp(hz, 24.0, 1526.0);

  double prescale_d = std::round(25000000.0 / (4096.0 * hz)) - 1.0;

  std::uint8_t prescale =
    static_cast<std::uint8_t>(std::clamp(prescale_d, 3.0, 255.0));

  std::uint8_t oldmode = 0;

  if (i2c_read8(i2c_fd_, MODE1, oldmode) < 0) {
    return;
  }

  const std::uint8_t sleep =
    static_cast<std::uint8_t>((oldmode & 0x7F) | 0x10);

  (void)i2c_write8(i2c_fd_, MODE1, sleep);
  (void)i2c_write8(i2c_fd_, PRESCALE, prescale);
  (void)i2c_write8(i2c_fd_, MODE1, oldmode);

  ::usleep(5000);

  // Auto-increment, restart, all-call
  (void)i2c_write8(i2c_fd_, MODE1, static_cast<std::uint8_t>(oldmode | 0xA1));
}

void mrh::PCA9685BTS7960::pca_set_pwm(
  int ch,
  std::uint16_t on,
  std::uint16_t off) noexcept
{
  if (dry_run_ || i2c_fd_ < 0) {
    return;
  }

  // Exact 0%
  if (off == 0) {
    (void)i2c_write8(i2c_fd_, REG_ON_H(ch), 0x00);
    (void)i2c_write8(i2c_fd_, REG_OFF_H(ch), 0x10);
    return;
  }

  // Exact 100%
  if (off >= 4095) {
    (void)i2c_write8(i2c_fd_, REG_OFF_H(ch), 0x00);
    (void)i2c_write8(i2c_fd_, REG_ON_H(ch), 0x10);
    return;
  }

  const std::uint8_t data[4] = {
    static_cast<std::uint8_t>(on & 0xFF),
    static_cast<std::uint8_t>((on >> 8) & 0x0F),
    static_cast<std::uint8_t>(off & 0xFF),
    static_cast<std::uint8_t>((off >> 8) & 0x0F)
  };

  (void)i2c_write_block(i2c_fd_, REG_ON_L(ch), data, 4);
}

void mrh::PCA9685BTS7960::drive_motor(int idx, double u) noexcept
{
  // idx 0 = left motor
  // idx 1 = right motor

  const int lp = (idx == 0) ? ch_[0] : ch_[2];
  const int rp = (idx == 0) ? ch_[1] : ch_[3];

  const double duty = std::fabs(u);
  const double duty_clamped = std::min(1.0, duty);

  std::uint16_t off =
    static_cast<std::uint16_t>(std::round(duty_clamped * 4095.0));

  if (duty_clamped == 0.0) {
    off = 0;
  }

  if (duty_clamped == 1.0) {
    off = 4095;
  }

  if (u > 0.0) {
    // Forward: RPWM = duty, LPWM = 0
    pca_set_pwm(lp, 0, 0);
    pca_set_pwm(rp, 0, off);
  } else if (u < 0.0) {
    // Reverse: LPWM = duty, RPWM = 0
    pca_set_pwm(rp, 0, 0);
    pca_set_pwm(lp, 0, off);
  } else {
    // Coast / stop
    pca_set_pwm(lp, 0, 0);
    pca_set_pwm(rp, 0, 0);
  }
}

void mrh::PCA9685BTS7960::encoder_sample(int idx) noexcept
{
  if (gpio_handle_ < 0 || !encoder_enabled_[idx]) {
    return;
  }

  const int pin_a = (idx == 0) ? left_encoder_a_ : right_encoder_a_;
  const int pin_b = (idx == 0) ? left_encoder_b_ : right_encoder_b_;

  const bool has_a = pin_a >= 0;
  const bool has_b = pin_b >= 0;

  if (!has_a && !has_b) {
    return;
  }

  const bool invert =
    (idx == 0) ? invert_left_encoder_ : invert_right_encoder_;

  // If both A and B work, use normal quadrature.
  if (has_a && has_b) {
    const int a = lgGpioRead(gpio_handle_, pin_a);
    const int b = lgGpioRead(gpio_handle_, pin_b);

    if (a < 0 || b < 0) {
      return;
    }

    const int state = ((a & 1) << 1) | (b & 1);

    if (encoder_last_state_[idx] < 0) {
      encoder_last_state_[idx] = state;
      return;
    }

    const int transition = (encoder_last_state_[idx] << 2) | state;

    static const int table[16] = {
       0, -1,  1,  0,
       1,  0,  0, -1,
      -1,  0,  0,  1,
       0,  1, -1,  0
    };

    int delta = table[transition];

    if (invert) {
      delta = -delta;
    }

    if (delta != 0) {
      std::lock_guard<std::mutex> lock(encoder_mutex_);
      encoder_count_[idx] += delta;
    }

    encoder_last_state_[idx] = state;
    return;
  }

  // If only one channel works, count edges from that one channel.
  // Direction comes from motor command, because one channel alone cannot know direction.
  const int pin = has_a ? pin_a : pin_b;
  const int signal = lgGpioRead(gpio_handle_, pin);

  if (signal < 0) {
    return;
  }

  const int state = signal & 1;

  if (encoder_last_state_[idx] < 0) {
    encoder_last_state_[idx] = state;
    return;
  }

  if (state != encoder_last_state_[idx]) {
    int direction = (cmd_[idx] >= 0.0) ? 1 : -1;

    if (invert) {
      direction = -direction;
    }

    std::lock_guard<std::mutex> lock(encoder_mutex_);
    encoder_count_[idx] += direction;
  }

  encoder_last_state_[idx] = state;
}

void mrh::PCA9685BTS7960::encoder_loop() noexcept
{
  while (encoder_running_) {
    encoder_sample(0);  // left encoder
    encoder_sample(1);  // right encoder

    // 50 us = about 20 kHz polling
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  }
}

void mrh::PCA9685BTS7960::set_motor_power(bool left_on, bool right_on) noexcept
{
  if (!use_relays_ || !relays_claimed_ || gpio_handle_ < 0) {
    return;
  }

  const int relay_on = relay_active_low_ ? 0 : 1;
  const int relay_off = relay_active_low_ ? 1 : 0;

  // IN2 / GPIO25 controls LEFT motor relay
  lgGpioWrite(gpio_handle_, left_relay_pin_, left_on ? relay_on : relay_off);

  // IN1 / GPIO24 controls RIGHT motor relay
  lgGpioWrite(gpio_handle_, right_relay_pin_, right_on ? relay_on : relay_off);
}

PLUGINLIB_EXPORT_CLASS(
  my_robot_hardware::PCA9685BTS7960,
  hardware_interface::SystemInterface)