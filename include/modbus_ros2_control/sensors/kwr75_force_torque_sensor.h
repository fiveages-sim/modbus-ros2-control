#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "modbus_ros2_control/sensors/kwr75_serial_client.h"

namespace modbus_ros2_control
{

/** KWR75 six-axis force/torque sensor over custom RS485 (USB adapter). */
class Kwr75ForceTorqueSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kAxisCount = 6;
  static constexpr std::array<const char*, kAxisCount> kInterfaceNames = {
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

  void load_parameters();
  void publish_wrench(const rclcpp::Time& time);
  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);

  std::string sensor_name_;
  std::string serial_port_ = "/dev/ttyUSB0";
  std::string wrench_topic_;
  std::string frame_id_ = "ft_sensor";
  int baudrate_ = 115200;
  uint8_t command_code_ = 0x49;
  bool convert_to_si_ = true;
  double gravity_ = 9.80665;
  int response_timeout_ms_ = 10;
  bool zero_mode_ = false;
  std::unique_ptr<Kwr75SerialClient> client_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  std::atomic<bool> has_valid_sample_{false};
  std::array<double, kAxisCount> wrench_state_{};
};

}  // namespace modbus_ros2_control
