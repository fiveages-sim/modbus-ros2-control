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

#include "modbus_ros2_control/sensors/weili_serial_client.h"

namespace modbus_ros2_control
{

/**
 * @brief Weili 六维力/力矩传感器 ROS2 Control 插件（Modbus RTU，通用协议 1）。
 *
 * 机器人侧作为 Modbus 主站，通过 USB-RS485 转换器与传感器通信：
 *   - 串口：8N1，默认 115200（USB-RS485 默认波特率，无需写死；机器人内部 485 可配 921600），从站 9。
 *   - 启动数据回传：FC 0x10 写寄存器 0x019A（250/500/1000 Hz 可选，默认 1000 Hz）。
 *   - 传感器回传 0x20 0x4E 帧（6×float32 + CRC16/Modbus），帧头不符 / CRC 失败自动丢包。
 *   - 可选激活时回零（发送回零指令并等待回零结果，500 ms 超时）。
 *   - 传感器自身错误时自动回传错误状态帧（功能位 0x01），此处记录并告警。
 *
 * 数据中断 / 串口不可用时输出全 0（与 Kwr75ForceTorqueSensor 行为一致）。
 */
class WeiliForceTorqueSensor : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kAxisCount = 6;
  static constexpr std::array<const char *, kAxisCount> kInterfaceNames = {
    "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

  void load_parameters();
  void publish_wrench(const rclcpp::Time & time);
  void stop_io_and_zero(const std::string & reason);
  static uint16_t sample_rate_to_register(int sample_rate);
  static bool parse_bool(const std::string & value, bool default_value);
  static int parse_int(const std::string & value, int default_value);

  std::string sensor_name_;
  std::string serial_port_ = "/dev/ttyUSB0";
  std::string wrench_topic_;
  std::string frame_id_ = "ft_sensor";
  int baudrate_ = 115200;
  uint8_t slave_id_ = WeiliSerialClient::kDefaultSlaveId;
  int sample_rate_ = 1000;
  uint16_t rate_register_ = 0;  // 0 = 由 sample_rate_ 推导
  std::size_t data_offset_ = 2;
  std::size_t frame_length_ = WeiliSerialClient::kDefaultFrameLength;
  double force_scale_ = 1.0;
  double torque_scale_ = 1.0;
  int zero_timeout_ms_ = 500;
  int read_timeout_ms_ = 4;
  int startup_delay_ms_ = 100;
  int warmup_attempts_ = 20;
  int data_timeout_ms_ = 200;   // 数据中断看门狗
  int max_read_failures_ = 3;
  bool zero_on_activate_ = false;
  bool zero_mode_ = false;
  int consecutive_read_failures_ = 0;
  uint8_t last_reported_error_ = 0;
  std::unique_ptr<WeiliSerialClient> client_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  std::atomic<bool> has_valid_sample_{false};
  std::array<double, kAxisCount> wrench_state_{};
};

}  // namespace modbus_ros2_control
