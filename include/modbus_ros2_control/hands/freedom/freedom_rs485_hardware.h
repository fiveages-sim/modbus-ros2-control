#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace modbus_ros2_control
{

class FreedomRS485Hardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kJointCount = 6;
  static constexpr uint8_t kFrameHead = 0x5A;
  static constexpr uint8_t kFrameTail = 0x5D;
  static constexpr uint8_t kHostToDevice = 0x00;
  static constexpr uint8_t kDeviceToHost = 0x01;
  static constexpr uint8_t kMoveCommand = 0x10;
  static constexpr uint8_t kAngleQueryCommand = 0xF1;
  static constexpr uint8_t kMoveFrameLength = 0x13;
  static constexpr uint8_t kQueryFrameLength = 0x08;
  static constexpr uint8_t kAngleResponseLength = 0x0D;
  static constexpr uint8_t kMaxProtocolAngle = 90;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool open_serial();
  void close_serial();
  bool configure_serial();
  void start_background_thread();
  void stop_background_thread();
  void background_loop();
  bool send_frame(const std::vector<uint8_t>& frame);
  bool read_frame(std::vector<uint8_t>& frame, int timeout_ms);
  bool query_positions(std::array<double, kJointCount>& positions);
  bool send_position_command(const std::array<uint8_t, kJointCount>& angles);

  uint8_t radians_to_protocol_angle(double radians, std::size_t joint_index) const;
  double protocol_angle_to_radians(uint8_t angle, std::size_t joint_index) const;
  bool command_changed(const std::array<uint8_t, kJointCount>& angles) const;

  static uint8_t checksum(const std::vector<uint8_t>& frame);
  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);

  std::string serial_port_ = "/dev/ttyUSB0";
  std::string hand_side_ = "left";
  int baudrate_ = 115200;
  uint8_t slave_id_ = 1;
  bool read_feedback_ = true;
  int feedback_timeout_ms_ = 20;
  int background_period_ms_ = 20;
  int command_deadband_deg_ = 0;
  int serial_fd_ = -1;
  bool command_sent_ = false;
  bool pending_command_valid_ = false;
  bool pending_command_dirty_ = false;
  std::atomic_bool background_running_{false};
  std::thread background_thread_;
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{0.785, 0.29, 1.24, 1.24, 1.24, 1.24};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<uint8_t, kJointCount> last_command_angles_{0, 0, 0, 0, 0, 0};
  std::array<uint8_t, kJointCount> pending_command_angles_{0, 0, 0, 0, 0, 0};
  std::array<double, kJointCount> feedback_positions_{};
  bool feedback_positions_valid_ = false;
};

}  // namespace modbus_ros2_control
