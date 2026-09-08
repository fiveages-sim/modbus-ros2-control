#include "modbus_ros2_control/sensors/weili_force_torque_sensor.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <cerrno>
#include <unistd.h>

#include <pluginlib/class_list_macros.hpp>

namespace modbus_ros2_control
{

hardware_interface::CallbackReturn WeiliForceTorqueSensor::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SensorInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1)
  {
    RCLCPP_ERROR(
      get_logger(),
      "WeiliForceTorqueSensor expects exactly one <sensor>, got %zu",
      info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_name_ = info_.sensors.front().name;
  const auto & declared = info_.sensors.front().state_interfaces;
  if (declared.size() != kAxisCount)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Sensor '%s' must declare 6 state interfaces, got %zu",
      sensor_name_.c_str(),
      declared.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kAxisCount; ++i)
  {
    if (declared[i].name != kInterfaceNames[i])
    {
      RCLCPP_ERROR(
        get_logger(),
        "Sensor '%s' interface[%zu] must be '%s', got '%s'",
        sensor_name_.c_str(),
        i,
        kInterfaceNames[i],
        declared[i].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  load_parameters();
  wrench_state_.fill(0.0);

  RCLCPP_INFO(
    get_logger(),
    "Configured Weili FT sensor '%s': port=%s, baudrate=%d, slave_id=%d, "
    "rate=%dHz(0x%04X), topic=%s",
    sensor_name_.c_str(),
    serial_port_.c_str(),
    baudrate_,
    slave_id_,
    sample_rate_,
    rate_register_,
    wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WeiliForceTorqueSensor::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  if (SensorInterface::on_configure(previous_state) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!wrench_topic_.empty())
  {
    wrench_pub_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, rclcpp::SensorDataQoS());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WeiliForceTorqueSensor::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (hardware_interface::SensorInterface::on_activate(previous_state) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  client_ = std::make_unique<WeiliSerialClient>(
    serial_port_,
    baudrate_,
    slave_id_,
    rate_register_,
    data_offset_,
    frame_length_,
    zero_timeout_ms_,
    read_timeout_ms_,
    startup_delay_ms_,
    warmup_attempts_);

  if (::access(serial_port_.c_str(), F_OK) != 0)
  {
    stop_io_and_zero(
      "Weili 传感器 '" + sensor_name_ + "' 未启动：未找到 USB 串口 " + serial_port_ +
      "，力/力矩输出保持为 0");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!client_->connect())
  {
    stop_io_and_zero(
      "Weili 传感器 '" + sensor_name_ + "' 未启动：无法打开 USB 串口 " + serial_port_ + " (" +
      std::strerror(errno) + ")，力/力矩输出保持为 0");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // 可选：激活时回零（发送回零指令并等待回零结果，超时 500 ms）
  if (zero_on_activate_)
  {
    uint8_t zero_status = 0xFF;
    const bool zero_ok = client_->request_zero(zero_status);
    if (zero_ok)
    {
      RCLCPP_INFO(
        get_logger(),
        "Weili 传感器 '%s' 回零成功", sensor_name_.c_str());
    }
    else
    {
      RCLCPP_WARN(
        get_logger(),
        "Weili 传感器 '%s' 回零%s（状态码 0x%02X，0=正常/非0=失败；500ms 内未收到回零结果亦视为失败），"
        "继续启动数据回传",
        sensor_name_.c_str(),
        (zero_status == 0) ? "超时" : "失败",
        zero_status);
    }
    // 回零后清空旧帧，等待新一轮数据
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // 握手：发送启动指令并等待首帧有效力值
  std::array<double, kAxisCount> sample {};
  bool got_first = false;
  for (int attempt = 0; attempt < warmup_attempts_; ++attempt)
  {
    if (client_->read_wrench(sample, 100))
    {
      got_first = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));
  }

  if (!got_first)
  {
    const std::string sample_hex = client_->last_io_sample_hex();
    if (sample_hex.empty())
    {
      stop_io_and_zero(
        "Weili 传感器 '" + sensor_name_ + "' 未就绪：串口 " + serial_port_ +
        " 已打开但未收到有效回传帧（未收到 0x20 0x4E 数据帧）。"
        "请确认 USB-RS485 已接、传感器已上电、波特率/采样率与传感器一致，"
        "且 robot.local.yaml 中端口映射正确。已停止串口读取。");
    }
    else
    {
      stop_io_and_zero(
        "Weili 传感器 '" + sensor_name_ + "' 未就绪：串口 " + serial_port_ +
        " 收到非 Weili 数据 [" + sample_hex + "]。该端口可能接的是其他设备或帧格式不符。"
        "已停止串口读取。");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  for (std::size_t i = 0; i < kAxisCount; ++i)
  {
    wrench_state_[i] = sample[i];
  }
  has_valid_sample_.store(true);
  consecutive_read_failures_ = 0;
  zero_mode_ = false;
  last_reported_error_ = 0;
  RCLCPP_INFO(
    get_logger(),
    "Weili 传感器 '%s' 已启动：串口 %s，%d Hz（0x%04X），topic %s",
    sensor_name_.c_str(),
    serial_port_.c_str(),
    sample_rate_,
    rate_register_,
    wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WeiliForceTorqueSensor::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  if (client_)
  {
    // 协议：机器人发送“停止数据传输”，传感器停止回传
    client_->stop_streaming();
    client_->disconnect();
    client_.reset();
  }
  wrench_pub_.reset();
  zero_mode_ = false;
  consecutive_read_failures_ = 0;
  has_valid_sample_.store(false);
  wrench_state_.fill(0.0);
  return hardware_interface::SensorInterface::on_deactivate(previous_state);
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
WeiliForceTorqueSensor::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.reserve(kAxisCount);
  for (std::size_t i = 0; i < kAxisCount; ++i)
  {
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      sensor_name_, kInterfaceNames[i], &wrench_state_[i]));
  }
  return interfaces;
}

hardware_interface::return_type WeiliForceTorqueSensor::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & /* period */)
{
  if (zero_mode_ || !client_ || !client_->is_connected())
  {
    wrench_state_.fill(0.0);
    publish_wrench(time);
    return hardware_interface::return_type::OK;
  }

  // 非阻塞读最新帧（采样保持）。read_wrench 总回填最近一帧，返回是否解析到新帧。
  std::array<double, kAxisCount> sample {};
  const bool got_new = client_->read_wrench(sample, 0);

  // 传感器错误状态帧：记录并告警（错误码含义取决于具体固件）
  if (client_->has_error_report())
  {
    const uint8_t err = client_->last_error_code();
    if (err != last_reported_error_)
    {
      last_reported_error_ = err;
      RCLCPP_WARN(
        get_logger(),
        "Weili 传感器 '%s' 回传错误状态：错误码 0x%02X（非 0 表示传感器自身异常），"
        "上层需按错误码含义处理",
        sensor_name_.c_str(), err);
    }
    client_->clear_error_report();
  }

  if (!got_new)
  {
    // 无新帧：若已长期无数据（超过 data_timeout），按通讯中断处理
    if (!client_->has_valid_sample() || client_->millis_since_last_data() > data_timeout_ms_)
    {
      ++consecutive_read_failures_;
      if (consecutive_read_failures_ >= max_read_failures_)
      {
        const std::string reason =
          "Weili 传感器 '" + sensor_name_ + "' 连续 " + std::to_string(consecutive_read_failures_) +
          " 次数据中断（超过 " + std::to_string(data_timeout_ms_) + " ms 未收到回传帧，" +
          serial_port_ + "），已停止串口读取，力/力矩输出保持为 0";
        stop_io_and_zero(reason);
      }
    }
    // 未到看门狗阈值：保持最近值输出（采样保持）
    if (client_->has_valid_sample())
    {
      for (std::size_t i = 0; i < kAxisCount; ++i)
      {
        wrench_state_[i] = sample[i];
      }
    }
    publish_wrench(time);
    return hardware_interface::return_type::OK;
  }

  consecutive_read_failures_ = 0;
  has_valid_sample_.store(true);
  for (std::size_t i = 0; i < kAxisCount; ++i)
  {
    wrench_state_[i] = sample[i];
  }
  publish_wrench(time);
  return hardware_interface::return_type::OK;
}

void WeiliForceTorqueSensor::stop_io_and_zero(const std::string & reason)
{
  if (!zero_mode_)
  {
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }
  zero_mode_ = true;
  consecutive_read_failures_ = 0;
  has_valid_sample_.store(false);
  if (client_)
  {
    client_->stop_streaming();
    client_->disconnect();
    client_.reset();
  }
  wrench_state_.fill(0.0);
}

void WeiliForceTorqueSensor::publish_wrench(const rclcpp::Time & time)
{
  if (!wrench_pub_)
  {
    return;
  }

  geometry_msgs::msg::WrenchStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = frame_id_;
  msg.wrench.force.x = wrench_state_[0];
  msg.wrench.force.y = wrench_state_[1];
  msg.wrench.force.z = wrench_state_[2];
  msg.wrench.torque.x = wrench_state_[3];
  msg.wrench.torque.y = wrench_state_[4];
  msg.wrench.torque.z = wrench_state_[5];
  wrench_pub_->publish(msg);
}

void WeiliForceTorqueSensor::load_parameters()
{
  const auto & params = info_.hardware_parameters;
  const auto get_param = [&params](const std::string & key, const std::string & default_value) {
    const auto it = params.find(key);
    return it != params.end() ? it->second : default_value;
  };

  serial_port_ = get_param("serial_port", serial_port_);
  wrench_topic_ = get_param("wrench_topic", "");
  frame_id_ = get_param("frame_id", frame_id_);
  baudrate_ = parse_int(get_param("baudrate", std::to_string(baudrate_)), baudrate_);
  slave_id_ = static_cast<uint8_t>(
    parse_int(get_param("slave_id", std::to_string(slave_id_)), slave_id_));
  sample_rate_ = parse_int(get_param("sample_rate", std::to_string(sample_rate_)), sample_rate_);
  if (sample_rate_ < 1)
  {
    sample_rate_ = 250;
  }
  rate_register_ = static_cast<uint16_t>(parse_int(get_param("rate_register", "0"), 0));
  if (rate_register_ == 0)
  {
    rate_register_ = sample_rate_to_register(sample_rate_);
  }
  data_offset_ = static_cast<std::size_t>(
    parse_int(get_param("data_offset", std::to_string(data_offset_)), data_offset_));
  frame_length_ = static_cast<std::size_t>(
    parse_int(get_param("frame_length", std::to_string(frame_length_)), frame_length_));
  if (frame_length_ < WeiliSerialClient::kDefaultFrameLength ||
    frame_length_ > WeiliSerialClient::kMaxFrameLength)
  {
    RCLCPP_WARN(
      get_logger(),
      "Sensor '%s': frame_length=%zu 越界，复位为默认 %zu",
      sensor_name_.c_str(), frame_length_, static_cast<std::size_t>(WeiliSerialClient::kDefaultFrameLength));
    frame_length_ = WeiliSerialClient::kDefaultFrameLength;
  }
  force_scale_ = std::stod(get_param("force_scale", "1.0"));
  torque_scale_ = std::stod(get_param("torque_scale", "1.0"));
  zero_on_activate_ = parse_bool(get_param("zero_on_activate", "false"), zero_on_activate_);
  zero_timeout_ms_ = parse_int(
    get_param("zero_timeout_ms", std::to_string(zero_timeout_ms_)),
    zero_timeout_ms_);
  read_timeout_ms_ = parse_int(
    get_param("read_timeout_ms", std::to_string(read_timeout_ms_)),
    read_timeout_ms_);
  if (read_timeout_ms_ < 1)
  {
    read_timeout_ms_ = 1;
  }
  startup_delay_ms_ = parse_int(
    get_param("startup_delay_ms", std::to_string(startup_delay_ms_)),
    startup_delay_ms_);
  warmup_attempts_ = parse_int(
    get_param("warmup_attempts", std::to_string(warmup_attempts_)),
    warmup_attempts_);
  data_timeout_ms_ = parse_int(
    get_param("data_timeout_ms", std::to_string(data_timeout_ms_)),
    data_timeout_ms_);
  if (data_timeout_ms_ < 10)
  {
    data_timeout_ms_ = 10;
  }
  max_read_failures_ = parse_int(
    get_param("max_read_failures", std::to_string(max_read_failures_)),
    max_read_failures_);
  if (max_read_failures_ < 1)
  {
    max_read_failures_ = 1;
  }
}

uint16_t WeiliForceTorqueSensor::sample_rate_to_register(int sample_rate)
{
  switch (sample_rate)
  {
    case 250:
      return WeiliSerialClient::kRate250Hz;
    case 500:
      return WeiliSerialClient::kRate500Hz;
    case 1000:
      return WeiliSerialClient::kRate1000Hz;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("WeiliForceTorqueSensor"),
        "sample_rate=%d 未在 {250, 500, 1000} 中，回退为 250 Hz (0x0301)",
        sample_rate);
      return WeiliSerialClient::kRate250Hz;
  }
}

bool WeiliForceTorqueSensor::parse_bool(const std::string & value, bool default_value)
{
  if (value == "true" || value == "1")
  {
    return true;
  }
  if (value == "false" || value == "0")
  {
    return false;
  }
  return default_value;
}

int WeiliForceTorqueSensor::parse_int(const std::string & value, int default_value)
{
  try
  {
    // base 0：兼容十进制与 0x 十六进制（如 rate_register=0x0301）
    std::size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed, 0);
    return (consumed > 0) ? parsed : default_value;
  }
  catch (const std::exception &)
  {
    return default_value;
  }
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::WeiliForceTorqueSensor,
  hardware_interface::SensorInterface)
