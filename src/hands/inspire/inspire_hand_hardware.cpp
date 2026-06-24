#include "modbus_ros2_control/hands/inspire/inspire_hand_hardware.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace modbus_ros2_control
{
namespace
{
constexpr auto kLoggerName = "InspireHandHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

int16_t to_signed(uint16_t value)
{
  return static_cast<int16_t>(value);
}
}  // namespace

hardware_interface::CallbackReturn InspireHandHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Inspire hand hardware expects exactly %zu joints, got %zu",
      kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(kJointCount);
  for (const auto& joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (!validate_joint_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();
  upper_limits_ = default_upper_limits(joint_names_);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_positions_[i] = std::clamp(
      initial_value_for_joint(info_.joints[i]), lower_limits_[i], upper_limits_[i]);
    previous_positions_[i] = hw_positions_[i];
    hw_commands_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    feedback_positions_[i] = hw_positions_[i];
    feedback_efforts_[i] = 0.0;
  }

  last_command_values_ = joints_to_actuators(hw_commands_);
  pending_command_values_ = last_command_values_;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured Inspire Modbus RTU hand: port=%s, baudrate=%d, id=%d, feedback=%s",
    serial_port_.c_str(),
    baudrate_,
    slave_id_,
    read_feedback_enabled_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireHandHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!connect_modbus())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initialize_hand())
  {
    disconnect_modbus();
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  pending_command_valid_ = false;
  pending_command_dirty_ = false;
  feedback_valid_ = false;

  std::array<double, kJointCount> initial_positions{};
  std::array<double, kJointCount> initial_efforts{};
  if (read_feedback_enabled_ && read_feedback(initial_positions, initial_efforts))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_positions_[i] = initial_positions[i];
      hw_commands_[i] = hw_positions_[i];
      feedback_positions_[i] = initial_positions[i];
      feedback_efforts_[i] = initial_efforts[i];
    }
    feedback_valid_ = true;
  }

  last_command_values_ = joints_to_actuators(hw_commands_);
  pending_command_values_ = last_command_values_;
  start_background_thread();

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Inspire Modbus RTU hand activated on %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireHandHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  stop_background_thread();
  disconnect_modbus();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Inspire Modbus RTU hand deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
InspireHandHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(kJointCount * 3);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
InspireHandHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type InspireHandHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (!modbus_ || !modbus_->isConnected())
  {
    return hardware_interface::return_type::ERROR;
  }

  if (background_running_.load())
  {
    update_background_period(period);
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_enabled_)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (feedback_valid_)
    {
      hw_positions_ = feedback_positions_;
      hw_efforts_ = feedback_efforts_;
    }
  }
  else
  {
    hw_positions_ = hw_commands_;
    hw_efforts_.fill(0.0);
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InspireHandHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (!modbus_ || !modbus_->isConnected())
  {
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
  }

  const auto actuator_values = joints_to_actuators(hw_commands_);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sent_ && !command_changed(actuator_values))
    {
      return hardware_interface::return_type::OK;
    }

    pending_command_values_ = actuator_values;
    pending_command_valid_ = true;
    pending_command_dirty_ = true;
  }

  return hardware_interface::return_type::OK;
}

void InspireHandHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_parameter("serial_port", serial_port_);
  hand_side_ = get_parameter("hand_side", hand_side_);
  baudrate_ = parse_int(get_parameter("baudrate", std::to_string(baudrate_)), baudrate_);
  background_period_ms_ = std::max(
    1, parse_int(get_parameter("background_period_ms", std::to_string(background_period_ms_)),
                 background_period_ms_));
  command_deadband_raw_ = std::clamp(
    parse_int(get_parameter("command_deadband_raw", std::to_string(command_deadband_raw_)),
              command_deadband_raw_),
    0,
    1000);
  default_speed_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_speed", std::to_string(default_speed_)), default_speed_),
    0,
    4000));
  default_force_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_force", std::to_string(default_force_)), default_force_),
    0,
    12000));
  read_feedback_enabled_ = parse_bool(
    get_parameter("read_feedback", read_feedback_enabled_ ? "true" : "false"),
    read_feedback_enabled_);

  auto side_lower = hand_side_;
  std::transform(side_lower.begin(), side_lower.end(), side_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  const auto side_default_slave_id =
    side_lower == "right" || side_lower == "right_hand" ? 1 : 2;
  const auto slave_id_value = get_parameter("slave_id", "auto");
  if (slave_id_value == "auto" || slave_id_value == "AUTO" || slave_id_value.empty())
  {
    slave_id_ = side_default_slave_id;
  }
  else
  {
    slave_id_ = std::clamp(parse_int(slave_id_value, side_default_slave_id), 1, 254);
  }
}

bool InspireHandHardware::validate_joint_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return false;
    }

    const auto has_state_interface = [&joint](const std::string& interface_name) {
      return std::any_of(
        joint.state_interfaces.begin(),
        joint.state_interfaces.end(),
        [&interface_name](const auto& interface) { return interface.name == interface_name; });
    };

    if (!has_state_interface(hardware_interface::HW_IF_POSITION) ||
        !has_state_interface(hardware_interface::HW_IF_VELOCITY) ||
        !has_state_interface(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position, velocity, and effort state interfaces",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool InspireHandHardware::connect_modbus()
{
  disconnect_modbus();

  modbus_ = std::make_unique<ModbusRtuCommunicator>(
    serial_port_,
    static_cast<uint32_t>(baudrate_),
    slave_id_,
    'N',
    8,
    1);

  if (!modbus_->connect())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to connect Inspire Modbus RTU hand: %s",
      modbus_->getLastError().c_str());
    modbus_.reset();
    return false;
  }

  return true;
}

void InspireHandHardware::disconnect_modbus()
{
  if (modbus_)
  {
    modbus_->disconnect();
    modbus_.reset();
  }
}

void InspireHandHardware::start_background_thread()
{
  if (background_running_.load())
  {
    return;
  }

  background_running_.store(true);
  background_thread_ = std::thread(&InspireHandHardware::background_loop, this);
}

void InspireHandHardware::stop_background_thread()
{
  background_running_.store(false);
  if (background_thread_.joinable())
  {
    background_thread_.join();
  }
}

void InspireHandHardware::update_background_period(const rclcpp::Duration& period)
{
  if (background_period_initialized_.load())
  {
    return;
  }

  int period_ms = static_cast<int>(period.seconds() * 1000.0);
  if (period_ms < 10)
  {
    period_ms = 10;
  }

  background_period_ms_ = period_ms;
  background_period_initialized_.store(true);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Inspire Modbus RTU background loop configured: %d ms (%.1f Hz)",
    background_period_ms_,
    1000.0 / static_cast<double>(background_period_ms_));
}

void InspireHandHardware::background_loop()
{
  while (background_running_.load())
  {
    const auto loop_start = std::chrono::steady_clock::now();

    if (read_feedback_enabled_)
    {
      std::array<double, kJointCount> positions{};
      std::array<double, kJointCount> efforts{};
      if (read_feedback(positions, efforts))
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_positions_ = positions;
        feedback_efforts_ = efforts;
        feedback_valid_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          2000,
          "No valid Inspire Modbus RTU feedback received from ID %d",
          slave_id_);
      }
    }

    std::array<uint16_t, kActuatorCount> actuator_values{};
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_valid_ && pending_command_dirty_ &&
          (!command_sent_ || command_changed(pending_command_values_)))
      {
        actuator_values = pending_command_values_;
        should_send = true;
        pending_command_dirty_ = false;
      }
    }

    if (should_send)
    {
      if (send_command(actuator_values))
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_values_ = actuator_values;
        command_sent_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          1000,
          "Failed to write Inspire Modbus RTU command to ID %d; command will be retried",
          slave_id_);
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_dirty_ = true;
      }
    }

    const auto elapsed = std::chrono::steady_clock::now() - loop_start;
    const auto target_period = std::chrono::milliseconds(background_period_ms_);
    if (elapsed < target_period)
    {
      std::this_thread::sleep_for(target_period - elapsed);
    }
    else
    {
      const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        *get_node()->get_clock(),
        5000,
        "Inspire Modbus RTU background loop is slower than expected: %ld ms (expected: %d ms)",
        elapsed_ms,
        background_period_ms_);
    }
  }
}

bool InspireHandHardware::initialize_hand()
{
  std::array<uint16_t, kActuatorCount> modes{};
  modes.fill(0);
  if (modbus_->writeRegisters(kModeRegister, modes.size(), modes.data()) !=
      static_cast<int>(modes.size()))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire hand mode register; continuing with current device mode");
  }

  std::array<uint16_t, kActuatorCount> speeds{};
  speeds.fill(default_speed_);
  if (modbus_->writeRegisters(kSpeedSetRegister, speeds.size(), speeds.data()) !=
      static_cast<int>(speeds.size()))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire hand speed register; continuing");
  }

  std::array<uint16_t, kActuatorCount> forces{};
  forces.fill(default_force_);
  if (modbus_->writeRegisters(kForceSetRegister, forces.size(), forces.data()) !=
      static_cast<int>(forces.size()))
  {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Failed to set Inspire hand force register; continuing");
  }

  return true;
}

bool InspireHandHardware::read_feedback(
  std::array<double, kJointCount>& positions,
  std::array<double, kJointCount>& efforts)
{
  std::array<uint16_t, kActuatorCount> angle_registers{};
  const auto angle_count = modbus_->readHoldingRegisters(
    kAngleActRegister, angle_registers.size(), angle_registers.data());
  if (angle_count != static_cast<int>(angle_registers.size()))
  {
    return false;
  }

  positions = actuators_to_joints(angle_registers);

  std::array<uint16_t, kActuatorCount> force_registers{};
  const auto force_count = modbus_->readHoldingRegisters(
    kForceActRegister, force_registers.size(), force_registers.data());
  if (force_count == static_cast<int>(force_registers.size()))
  {
    const std::array<std::size_t, kJointCount> joint_to_actuator{5, 4, 3, 2, 1, 0};
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
      efforts[joint_index] = static_cast<double>(to_signed(force_registers[joint_to_actuator[joint_index]]));
    }
  }
  else
  {
    efforts.fill(0.0);
  }

  return true;
}

bool InspireHandHardware::send_command(
  const std::array<uint16_t, kActuatorCount>& actuator_values)
{
  const auto result = modbus_->writeRegisters(
    kAngleSetRegister, actuator_values.size(), actuator_values.data());
  return result == static_cast<int>(actuator_values.size());
}

bool InspireHandHardware::command_changed(
  const std::array<uint16_t, kActuatorCount>& actuator_values) const
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
  {
    if (std::abs(static_cast<int>(actuator_values[i]) - static_cast<int>(last_command_values_[i])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

std::array<uint16_t, InspireHandHardware::kActuatorCount> InspireHandHardware::joints_to_actuators(
  const std::array<double, kJointCount>& joints) const
{
  return {
    joint_to_actuator_value(joints[5], 5),
    joint_to_actuator_value(joints[4], 4),
    joint_to_actuator_value(joints[3], 3),
    joint_to_actuator_value(joints[2], 2),
    joint_to_actuator_value(joints[1], 1),
    joint_to_actuator_value(joints[0], 0)};
}

std::array<double, InspireHandHardware::kJointCount> InspireHandHardware::actuators_to_joints(
  const std::array<uint16_t, kActuatorCount>& actuators) const
{
  return {
    actuator_value_to_joint(actuators[5], 0),
    actuator_value_to_joint(actuators[4], 1),
    actuator_value_to_joint(actuators[3], 2),
    actuator_value_to_joint(actuators[2], 3),
    actuator_value_to_joint(actuators[1], 4),
    actuator_value_to_joint(actuators[0], 5)};
}

uint16_t InspireHandHardware::joint_to_actuator_value(
  double radians,
  std::size_t joint_index) const
{
  constexpr std::array<double, kJointCount> raw_open{1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
  constexpr std::array<double, kJointCount> raw_closed{500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return static_cast<uint16_t>(std::lround(raw_open[joint_index]));
  }

  const auto ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  const auto value = raw_open[joint_index] + ratio * (raw_closed[joint_index] - raw_open[joint_index]);
  return static_cast<uint16_t>(std::clamp<long>(std::lround(value), 0, 2000));
}

double InspireHandHardware::actuator_value_to_joint(
  uint16_t value,
  std::size_t joint_index) const
{
  constexpr std::array<double, kJointCount> raw_open{1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
  constexpr std::array<double, kJointCount> raw_closed{500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower || std::abs(raw_closed[joint_index] - raw_open[joint_index]) <
                          std::numeric_limits<double>::epsilon())
  {
    return lower;
  }

  const auto ratio = std::clamp(
    (static_cast<double>(value) - raw_open[joint_index]) /
      (raw_closed[joint_index] - raw_open[joint_index]),
    0.0,
    1.0);
  return lower + ratio * (upper - lower);
}

bool InspireHandHardware::parse_bool(const std::string& value, bool default_value)
{
  if (value == "true" || value == "1" || value == "True" || value == "TRUE")
  {
    return true;
  }
  if (value == "false" || value == "0" || value == "False" || value == "FALSE")
  {
    return false;
  }
  return default_value;
}

int InspireHandHardware::parse_int(const std::string& value, int default_value)
{
  try
  {
    return std::stoi(value, nullptr, 0);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double InspireHandHardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
    {
      try
      {
        return std::stod(interface.initial_value);
      }
      catch (const std::exception&)
      {
        return 0.0;
      }
    }
  }
  return 0.0;
}

std::array<double, InspireHandHardware::kJointCount> InspireHandHardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 1.1641;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 0.5864;
    }
    else
    {
      limits[i] = 1.4381;
    }
  }

  return limits;
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::InspireHandHardware,
  hardware_interface::SystemInterface)
