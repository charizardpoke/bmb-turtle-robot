#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace my_robot_hardware {

class PCA9685BTS7960 : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PCA9685BTS7960)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // PCA9685 helpers
  void pca_set_freq(double hz) noexcept;
  void pca_set_pwm(int ch, std::uint16_t on, std::uint16_t off) noexcept;
  // BTS7960 mapping
  void drive_motor(int idx, double u) noexcept;

  // Params/state
  bool dry_run_{false};
  int  i2c_fd_{-1};
  int  i2c_bus_{1};
  std::uint8_t addr_{0x40};
  double pwm_hz_{1000.0};
  int ch_[4]{0,1,2,3}; // LPWM_L, RPWM_L, LPWM_R, RPWM_R

  // Control mapping
  double max_speed_rad_s_{10.0};
  double deadband_{0.05};
  bool invert_left_{false};
  bool invert_right_{false};

  // states/commands
  std::array<double,2> pos_{};
  std::array<double,2> vel_{};
  std::array<double,2> cmd_{};
};

} // namespace my_robot_hardware
