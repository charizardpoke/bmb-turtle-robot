#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <thread>
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

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

  hardware_interface::return_type write(
    const rclcpp::Time &,
    const rclcpp::Duration &) override;

private:
  // PCA9685 helpers
  void pca_set_freq(double hz) noexcept;
  void pca_set_pwm(int ch, std::uint16_t on, std::uint16_t off) noexcept;

  // BTS7960 motor helpers
  void drive_motor(int idx, double u) noexcept;

  // Encoder helpers
  void encoder_loop() noexcept;
  void encoder_sample(int idx) noexcept;

  // Relay helper
  void set_motor_power(bool left_on, bool right_on) noexcept;

  // PCA9685 / BTS7960 settings
  bool dry_run_{false};
  int i2c_fd_{-1};
  int i2c_bus_{1};
  std::uint8_t addr_{0x40};
  double pwm_hz_{1000.0};

  // PCA9685 channels:
  // ch_[0] = left LPWM
  // ch_[1] = left RPWM
  // ch_[2] = right LPWM
  // ch_[3] = right RPWM
  int ch_[4]{0, 1, 2, 3};

  // Motor control
  double max_speed_rad_s_{10.0};
  double deadband_{0.05};
  bool invert_left_{false};
  bool invert_right_{false};

  // GPIO chip
  int gpio_chip_{4};  // Raspberry Pi 5 usually uses gpiochip4
  int gpio_handle_{-1};

  // Encoder settings
  bool use_encoders_{true};

  // Left encoder:
  // Yellow A -> GPIO22
  // White  B -> GPIO23
  int left_encoder_a_{22};
  int left_encoder_b_{23};

  // Right encoder:
  // Yellow A -> GPIO17
  // White  B -> GPIO27
  int right_encoder_a_{17};
  int right_encoder_b_{27};

  // GM37 encoder count
  double encoder_cpr_{5760.0};

  bool invert_left_encoder_{false};
  bool invert_right_encoder_{false};

  bool encoder_enabled_[2]{false, false};
  int encoder_last_state_[2]{-1, -1};
  long long encoder_count_[2]{0, 0};

  std::thread encoder_thread_;
  std::atomic_bool encoder_running_{false};
  std::mutex encoder_mutex_;

  // Relay settings
  bool use_relays_{true};

  // Relay wiring:
  // IN2 / GPIO25 -> left motor relay
  // IN1 / GPIO24 -> right motor relay
  int left_relay_pin_{25};
  int right_relay_pin_{24};

  // Most relay modules are active LOW:
  // GPIO 0 = relay ON
  // GPIO 1 = relay OFF
  bool relay_active_low_{true};
  bool relays_claimed_{false};

  // Joint indexes from URDF
  int left_joint_index_{0};
  int right_joint_index_{1};

  // Joint states / commands
  std::array<double, 2> pos_{};
  std::array<double, 2> last_pos_{};
  std::array<double, 2> vel_{};
  std::array<double, 2> cmd_{};
};

}  // namespace my_robot_hardware