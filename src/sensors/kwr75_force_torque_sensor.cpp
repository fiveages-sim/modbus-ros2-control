#include "modbus_ros2_control/sensors/kwr75_force_torque_sensor.h"

#include "modbus_ros2_control/sensors/kwr75_serial_client.h"

#include <cerrno>
#include <cstring>
#include <unistd.h>

#include <pluginlib/class_list_macros.hpp>

namespace modbus_ros2_control
{

hardware_interface::CallbackReturn Kwr75ForceTorqueSensor::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
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
      "Kwr75ForceTorqueSensor expects exactly one <sensor>, got %zu",
      info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_name_ = info_.sensors.front().name;
  const auto& declared = info_.sensors.front().state_interfaces;
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
    "Configured KWR75 FT sensor '%s': port=%s, topic=%s, baudrate=%d, command=0x%02X",
    sensor_name_.c_str(),
    serial_port_.c_str(),
    wrench_topic_.c_str(),
    baudrate_,
    command_code_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Kwr75ForceTorqueSensor::on_configure(
  const rclcpp_lifecycle::State& previous_state)
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

hardware_interface::CallbackReturn Kwr75ForceTorqueSensor::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  if (hardware_interface::SensorInterface::on_activate(previous_state) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  client_ = std::make_unique<Kwr75SerialClient>(
    serial_port_,
    baudrate_,
    command_code_,
    convert_to_si_,
    gravity_,
    response_timeout_ms_,
    read_timeout_ms_,
    startup_delay_ms_,
    warmup_attempts_);

  if (::access(serial_port_.c_str(), F_OK) != 0)
  {
    stop_io_and_zero(
      "KWR75 传感器 '" + sensor_name_ + "' 未启动：未找到 USB 串口 " + serial_port_ +
      "，力/力矩输出保持为 0");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!client_->connect())
  {
    stop_io_and_zero(
      "KWR75 传感器 '" + sensor_name_ + "' 未启动：无法打开 USB 串口 " + serial_port_ + " (" +
      std::strerror(errno) + ")，力/力矩输出保持为 0");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  if (!client_->warmup())
  {
    const std::string sample = client_->last_io_sample_hex();
    if (sample.empty())
    {
      stop_io_and_zero(
        "KWR75 传感器 '" + sensor_name_ + "' 未就绪：串口 " + serial_port_ +
        " 已打开但无有效 28 字节帧（未收到数据）。"
        "请确认 KWR75 USB 线已接、传感器已上电，且 robot.local.yaml 中 usb_*_ft_port 映射正确。"
        "已停止串口读取。");
    }
    else
    {
      stop_io_and_zero(
        "KWR75 传感器 '" + sensor_name_ + "' 未就绪：串口 " + serial_port_ + " 收到非 KWR75 数据 [" +
        sample + "]。该端口可能接的是其他设备。"
        "已停止串口读取。");
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  consecutive_read_failures_ = 0;
  zero_mode_ = false;
  RCLCPP_INFO(
    get_logger(),
    "KWR75 传感器 '%s' 已启动：串口 %s，topic %s",
    sensor_name_.c_str(),
    serial_port_.c_str(),
    wrench_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Kwr75ForceTorqueSensor::on_deactivate(
  const rclcpp_lifecycle::State& previous_state)
{
  if (client_)
  {
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
Kwr75ForceTorqueSensor::on_export_state_interfaces()
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

hardware_interface::return_type Kwr75ForceTorqueSensor::read(
  const rclcpp::Time& time,
  const rclcpp::Duration& /* period */)
{
  if (zero_mode_ || !client_)
  {
    wrench_state_.fill(0.0);
    publish_wrench(time);
    return hardware_interface::return_type::OK;
  }

  // timeout_ms = 0：实时环里绝不阻塞。串口由 Kwr75SerialClient 的后台线程持续
  // 排空并缓存最新帧，这里只取当前缓存（28 字节帧在 115200 下需要 2.43 ms 传输，
  // 同步等待必然超过 500 Hz 的 2 ms 周期预算）。
  if (!client_->read_wrench(wrench_state_, 0))
  {
    wrench_state_.fill(0.0);
    ++consecutive_read_failures_;
    if (consecutive_read_failures_ >= max_read_failures_)
    {
      const std::string sample = client_->last_io_sample_hex();
      std::string reason =
        "KWR75 传感器 '" + sensor_name_ + "' 连续 " + std::to_string(consecutive_read_failures_) +
        " 次读取失败（" + serial_port_ + "），已停止串口读取，力/力矩输出保持为 0";
      if (!sample.empty())
      {
        reason += "，最近收到: [" + sample + "]";
      }
      stop_io_and_zero(reason);
    }
  }
  else
  {
    consecutive_read_failures_ = 0;

    // ── 单帧跳变抑制（EMI / 错帧）──
    // 现场实测会偶发 +15 N / +60 N 的孤立跳变（force-spike）：控制器拿到这种跳变后
    // 导纳会给出 v = 60/D 的速度（D=300 时 0.2 m/s），被钳到 force_vmax 后表现为
    // 3 cm/s 的猛烈抽动。这里在源头上丢弃「单帧跳变超过阈值」的样本，保持上一次
    // 有效值；连续丢弃超过 5 帧才接受（避免真的阶跃变化被永久挡住）。
    bool plausible = true;
    if (have_good_)
    {
      for (std::size_t i = 0; i < kAxisCount; ++i)
      {
        const double limit = i < 3 ? spike_limit_force_ : spike_limit_torque_;
        // limit <= 0 → 该轴关闭跳变抑制（真正的一键关掉；之前把阈值钳到 0 会让
        // “任何变化”都被判为不可信，等于永远开着）。
        if (limit <= 0.0)
        {
          continue;
        }
        if (std::abs(wrench_state_[i] - last_good_[i]) > limit)
        {
          plausible = false;
          break;
        }
      }
    }
    if (plausible || ++rejected_in_a_row_ > 5)
    {
      last_good_ = wrench_state_;
      have_good_ = true;
      rejected_in_a_row_ = 0;
      has_valid_sample_.store(true);
    }
    else
    {
      wrench_state_ = last_good_;   // 丢弃本帧，保持上次有效值
    }
  }

  publish_wrench(time);
  return hardware_interface::return_type::OK;
}

void Kwr75ForceTorqueSensor::stop_io_and_zero(const std::string& reason)
{
  if (!zero_mode_)
  {
    RCLCPP_WARN(get_logger(), "%s", reason.c_str());
  }
  zero_mode_ = true;
  consecutive_read_failures_ = 0;
  if (client_)
  {
    client_->disconnect();
    client_.reset();
  }
  wrench_state_.fill(0.0);
}

void Kwr75ForceTorqueSensor::publish_wrench(const rclcpp::Time& time)
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

void Kwr75ForceTorqueSensor::load_parameters()
{
  const auto& params = info_.hardware_parameters;
  const auto get_param = [&params](const std::string& key, const std::string& default_value) {
    const auto it = params.find(key);
    return it != params.end() ? it->second : default_value;
  };

  serial_port_ = get_param("serial_port", serial_port_);
  wrench_topic_ = get_param("wrench_topic", "");
  frame_id_ = get_param("frame_id", frame_id_);
  baudrate_ = parse_int(get_param("baudrate", std::to_string(baudrate_)), baudrate_);
  command_code_ = static_cast<uint8_t>(
    parse_int(get_param("command_code", std::to_string(command_code_)), command_code_));
  convert_to_si_ = parse_bool(get_param("convert_to_si", "true"), convert_to_si_);
  // 单帧跳变抑制阈值（抗 EMI 尖峰）。≤0 关闭。
  spike_limit_force_ = std::stod(get_param("spike_limit_force", std::to_string(spike_limit_force_)));
  spike_limit_torque_ = std::stod(get_param("spike_limit_torque", std::to_string(spike_limit_torque_)));
  if (spike_limit_force_ < 0.0) spike_limit_force_ = 0.0;
  if (spike_limit_torque_ < 0.0) spike_limit_torque_ = 0.0;
  last_good_.fill(0.0);
  have_good_ = false;
  rejected_in_a_row_ = 0;
  gravity_ = std::stod(get_param("gravity", std::to_string(gravity_)));
  response_timeout_ms_ = parse_int(
    get_param("response_timeout_ms", std::to_string(response_timeout_ms_)),
    response_timeout_ms_);
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
  max_read_failures_ = parse_int(
    get_param("max_read_failures", std::to_string(max_read_failures_)),
    max_read_failures_);
  if (max_read_failures_ < 1)
  {
    max_read_failures_ = 1;
  }
}

bool Kwr75ForceTorqueSensor::parse_bool(const std::string& value, bool default_value)
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

int Kwr75ForceTorqueSensor::parse_int(const std::string& value, int default_value)
{
  try
  {
    return std::stoi(value);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::Kwr75ForceTorqueSensor,
  hardware_interface::SensorInterface)
