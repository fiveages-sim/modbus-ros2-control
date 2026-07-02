#include "modbus_ros2_control/sensors/kwr75_force_torque_sensor.h"

#include "modbus_ros2_control/sensors/kwr75_serial_client.h"

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
    serial_port_, baudrate_, command_code_, convert_to_si_, gravity_, response_timeout_ms_);

  if (!client_->connect())
  {
    RCLCPP_WARN(
      get_logger(),
      "KWR75 port '%s' unavailable; state interfaces and %s stay zero",
      serial_port_.c_str(),
      wrench_topic_.c_str());
    zero_mode_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  zero_mode_ = !client_->warmup();
  if (zero_mode_)
  {
    RCLCPP_WARN(
      get_logger(),
      "KWR75 warmup failed on '%s'; outputs stay zero until reads succeed",
      serial_port_.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "KWR75 FT sensor activated on %s", serial_port_.c_str());
  }

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
  if (!client_ || zero_mode_)
  {
    wrench_state_.fill(0.0);
    publish_wrench(time);
    return hardware_interface::return_type::OK;
  }

  if (!client_->read_wrench(wrench_state_))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_node()->get_clock(),
      2000,
      "Failed to read KWR75 frame from %s",
      serial_port_.c_str());
    wrench_state_.fill(0.0);
  }
  else
  {
    has_valid_sample_.store(true);
    zero_mode_ = false;
  }

  publish_wrench(time);
  return hardware_interface::return_type::OK;
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
  gravity_ = std::stod(get_param("gravity", std::to_string(gravity_)));
  response_timeout_ms_ = parse_int(
    get_param("response_timeout_ms", std::to_string(response_timeout_ms_)),
    response_timeout_ms_);
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
