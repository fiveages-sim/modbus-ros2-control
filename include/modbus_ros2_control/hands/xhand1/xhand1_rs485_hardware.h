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
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace modbus_ros2_control
{

class XHand1RS485Hardware : public hardware_interface::SystemInterface
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
  static constexpr std::size_t kJointCount = 12;
  static constexpr std::size_t kRealtimeCommandBytesPerJoint = 24;
  static constexpr std::size_t kRealtimeStateBytesPerJoint = 24;
  static constexpr std::size_t kRealtimeCommandDataLength =
    kJointCount * kRealtimeCommandBytesPerJoint;
  static constexpr std::size_t kRealtimeJointStateDataLength =
    kJointCount * kRealtimeStateBytesPerJoint;
  static constexpr std::size_t kTactileSensorDataLength = 384 * 5;
  static constexpr std::size_t kRealtimeResponseDataLength =
    kRealtimeJointStateDataLength + kTactileSensorDataLength;
  static constexpr std::size_t kFrameHeaderLength = 7;
  static constexpr std::size_t kCrcLength = 2;
  static constexpr uint8_t kFrameHead0 = 0x55;
  static constexpr uint8_t kFrameHead1 = 0xAA;
  static constexpr uint8_t kHostId = 0xFE;
  static constexpr uint8_t kRealtimeCommand = 0x02;
  static constexpr uint16_t kPositionMode = 3;
  static constexpr double kMaxVelocityRadPerSec = 10.0;
  static constexpr uint16_t kMaxTorqueLimit = 300;

  void load_parameters();
  void declare_tool_parameters();
  rcl_interfaces::msg::SetParametersResult on_tool_parameters(
    const std::vector<rclcpp::Parameter>& parameters);
  void apply_tool_parameters(double torque_scale, double velocity_scale);
  bool validate_joint_interfaces() const;
  bool open_serial();
  void close_serial();
  bool configure_serial();
  void start_background_thread();
  void stop_background_thread();
  void background_loop();
  std::array<double, kJointCount> compute_limited_command_positions(
    const std::array<double, kJointCount>& target_positions,
    double period_seconds) const;
  bool send_realtime_command(
    const std::array<double, kJointCount>& commands,
    const std::array<uint16_t, kJointCount>& torque_limits);
  bool send_frame(const std::vector<uint8_t>& frame);
  bool read_frame(std::vector<uint8_t>& frame, int timeout_ms);
  bool parse_realtime_response(
    const std::vector<uint8_t>& frame,
    std::array<double, kJointCount>& positions,
    std::array<double, kJointCount>& efforts) const;
  bool command_changed(
    const std::array<double, kJointCount>& commands,
    const std::array<uint16_t, kJointCount>& torque_limits) const;

  static void append_u16_le(std::vector<uint8_t>& frame, uint16_t value);
  static void append_i16_le(std::vector<uint8_t>& frame, int16_t value);
  static void append_float_le(std::vector<uint8_t>& frame, float value);
  static uint16_t read_u16_le(const std::vector<uint8_t>& frame, std::size_t offset);
  static float read_float_le(const std::vector<uint8_t>& frame, std::size_t offset);
  static uint16_t crc16_ccitt_zero(const uint8_t* data, std::size_t length);
  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double parse_double(const std::string& value, double default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_lower_limits();
  static std::array<double, kJointCount> default_upper_limits();

  std::string serial_port_ = "/dev/ttyUSB0";
  int baudrate_ = 3000000;
  uint8_t host_id_ = kHostId;
  uint8_t hand_id_ = 0;
  bool read_feedback_ = true;
  int feedback_timeout_ms_ = 20;
  int background_period_ms_ = 12;
  double command_deadband_rad_ = 0.001;
  int16_t kp_ = 100;
  int16_t ki_ = 0;
  int16_t kd_ = 0;
  uint16_t torque_limit_ = kMaxTorqueLimit;
  uint16_t control_mode_ = kPositionMode;
  double tool_torque_scale_ = 1.0;
  double tool_velocity_scale_ = 1.0;
  int serial_fd_ = -1;
  bool command_sent_ = false;
  bool pending_command_valid_ = false;
  bool pending_command_dirty_ = false;
  std::atomic_bool background_running_{false};
  std::thread background_thread_;
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;
  std::mutex error_mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;
  std::string last_exchange_error_;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> hw_velocity_scales_{};
  std::array<double, kJointCount> hw_effort_scales_{};
  std::array<double, kJointCount> last_command_positions_{};
  std::array<uint16_t, kJointCount> last_command_torque_limits_{};
  std::array<double, kJointCount> pending_command_positions_{};
  std::array<uint16_t, kJointCount> pending_command_torque_limits_{};
  std::array<double, kJointCount> feedback_positions_{};
  std::array<double, kJointCount> feedback_efforts_{};
  bool feedback_positions_valid_ = false;
};

}  // namespace modbus_ros2_control
