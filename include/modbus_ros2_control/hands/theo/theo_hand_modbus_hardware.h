#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "modbus_ros2_control/communicator/modbus_rtu_communicator.h"
#include "modbus_ros2_control/hands/theo/theo_hand_conversion.h"

namespace modbus_ros2_control
{

class TheoHandModbusHardware : public hardware_interface::SystemInterface
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
  static constexpr std::size_t kJointCount = 16;
  static constexpr uint16_t kControlWordRegister = 0x0000;
  static constexpr uint16_t kTargetPositionRegister = 0x0001;
  static constexpr uint16_t kRealPositionRegister = 0x0051;
  static constexpr uint16_t kModbusLeftRightRegister = 0x00cf;
  static constexpr uint16_t kObjectDictionaryLeftRightRegister = 0x9007;
  static constexpr uint16_t kEnableControlWord = 0x000f;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool connect_modbus();
  void disconnect_modbus();
  bool initialize_hand();
  void detect_hand_side();
  void start_background_thread();
  void stop_background_thread();
  void update_background_period(const rclcpp::Duration& period);
  void background_loop();
  bool read_feedback(std::array<double, kJointCount>& positions);
  int send_command(const std::array<uint16_t, kJointCount>& target_positions);
  bool command_changed(const std::array<uint16_t, kJointCount>& target_positions) const;
  bool feedback_quiet_period_elapsed(
    std::chrono::steady_clock::time_point now,
    std::chrono::milliseconds quiet_period) const;

  std::array<uint16_t, kJointCount> joints_to_registers(
    const std::array<double, kJointCount>& joints) const;
  std::array<double, kJointCount> registers_to_joints(
    const std::array<uint16_t, kJointCount>& registers) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_lower_limits(
    const std::vector<std::string>& joint_names);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);

  std::string serial_port_ = "/dev/ttyUSB0";
  int baudrate_ = 115200;
  int slave_id_ = 1;
  bool read_feedback_enabled_ = true;
  int background_period_ms_ = 200;
  int feedback_quiet_after_write_ms_ = 500;
  std::atomic_bool background_period_initialized_{false};
  int command_deadband_raw_ = 1;

  std::unique_ptr<ModbusRtuCommunicator> modbus_;
  std::atomic_bool background_running_{false};
  std::thread background_thread_;
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> feedback_positions_{};

  std::array<uint16_t, kJointCount> last_command_values_{};
  std::array<uint16_t, kJointCount> pending_command_values_{};
  std::atomic<int64_t> last_command_request_ms_{0};
  bool command_sent_ = false;
  bool pending_command_valid_ = false;
  bool pending_command_dirty_ = false;
  bool feedback_valid_ = false;
  theo_hand::HandSide detected_hand_side_ = theo_hand::HandSide::UNKNOWN;
};

}  // namespace modbus_ros2_control
