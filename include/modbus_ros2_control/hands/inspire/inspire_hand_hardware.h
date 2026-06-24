#pragma once

#include <array>
#include <atomic>
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

namespace modbus_ros2_control
{

class InspireHandHardware : public hardware_interface::SystemInterface
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
  static constexpr std::size_t kActuatorCount = 6;

  static constexpr uint16_t kModeRegister = 1100;
  static constexpr uint16_t kAngleSetRegister = 1040;
  static constexpr uint16_t kForceSetRegister = 1046;
  static constexpr uint16_t kSpeedSetRegister = 1052;
  static constexpr uint16_t kAngleActRegister = 1064;
  static constexpr uint16_t kForceActRegister = 1070;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool connect_modbus();
  void disconnect_modbus();
  void start_background_thread();
  void stop_background_thread();
  void update_background_period(const rclcpp::Duration& period);
  void background_loop();
  bool initialize_hand();
  bool read_feedback(std::array<double, kJointCount>& positions,
    std::array<double, kJointCount>& efforts);
  bool send_command(const std::array<uint16_t, kActuatorCount>& actuator_values);
  bool command_changed(const std::array<uint16_t, kActuatorCount>& actuator_values) const;

  std::array<uint16_t, kActuatorCount> joints_to_actuators(
    const std::array<double, kJointCount>& joints) const;
  std::array<double, kJointCount> actuators_to_joints(
    const std::array<uint16_t, kActuatorCount>& actuators) const;
  uint16_t joint_to_actuator_value(double radians, std::size_t joint_index) const;
  double actuator_value_to_joint(uint16_t value, std::size_t joint_index) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);

  std::string serial_port_ = "/dev/ttyUSB0";
  std::string hand_side_ = "left";
  int baudrate_ = 115200;
  int slave_id_ = 1;
  bool read_feedback_enabled_ = true;
  int background_period_ms_ = 20;
  std::atomic_bool background_period_initialized_{false};
  int command_deadband_raw_ = 1;
  uint16_t default_speed_ = 4000;
  uint16_t default_force_ = 6000;

  std::unique_ptr<ModbusRtuCommunicator> modbus_;
  std::atomic_bool background_running_{false};
  std::thread background_thread_;
  std::mutex command_mutex_;
  std::mutex feedback_mutex_;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> feedback_positions_{};
  std::array<double, kJointCount> feedback_efforts_{};

  std::array<uint16_t, kActuatorCount> last_command_values_{};
  std::array<uint16_t, kActuatorCount> pending_command_values_{};
  bool command_sent_ = false;
  bool pending_command_valid_ = false;
  bool pending_command_dirty_ = false;
  bool feedback_valid_ = false;
};

}  // namespace modbus_ros2_control
