#include "modbus_ros2_control/hands/theo/theo_hand_modbus_hardware.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace modbus_ros2_control
{
namespace
{
constexpr auto kLoggerName = "TheoHandModbusHardware";
constexpr auto kCommandPollPeriod = std::chrono::milliseconds(5);

int64_t steady_ms()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

const char* hand_side_to_string(theo_hand::HandSide side)
{
  switch (side)
  {
    case theo_hand::HandSide::LEFT:
      return "left";
    case theo_hand::HandSide::RIGHT:
      return "right";
    case theo_hand::HandSide::UNKNOWN:
      return "unknown";
  }
  return "unknown";
}
}  // namespace

hardware_interface::CallbackReturn TheoHandModbusHardware::on_init(
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
      "TheoHand STD16A Modbus hardware expects exactly %zu joints, got %zu",
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
  lower_limits_ = default_lower_limits(joint_names_);
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
  }

  last_command_values_ = joints_to_registers(hw_commands_);
  pending_command_values_ = last_command_values_;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured TheoHand STD16A Modbus RTU hardware: joints=%zu, port=%s, baudrate=%d, slave_id=%d, feedback=%s",
    kJointCount,
    serial_port_.c_str(),
    baudrate_,
    slave_id_,
    read_feedback_enabled_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TheoHandModbusHardware::on_activate(
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
  if (read_feedback_enabled_ && read_feedback(initial_positions))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_positions_[i] = initial_positions[i];
      hw_commands_[i] = hw_positions_[i];
      feedback_positions_[i] = initial_positions[i];
    }
    feedback_valid_ = true;
  }

  last_command_values_ = joints_to_registers(hw_commands_);
  pending_command_values_ = last_command_values_;
  command_sent_ = feedback_valid_;
  start_background_thread();

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "TheoHand STD16A Modbus RTU hardware activated on %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TheoHandModbusHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  stop_background_thread();
  disconnect_modbus();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "TheoHand STD16A Modbus RTU hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
TheoHandModbusHardware::on_export_state_interfaces()
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
TheoHandModbusHardware::on_export_command_interfaces()
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

hardware_interface::return_type TheoHandModbusHardware::read(
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
    }
  }
  else
  {
    hw_positions_ = hw_commands_;
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_efforts_[i] = 0.0;
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TheoHandModbusHardware::write(
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

  const auto target_positions = joints_to_registers(hw_commands_);
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sent_ && !command_changed(target_positions))
    {
      return hardware_interface::return_type::OK;
    }

    pending_command_values_ = target_positions;
    pending_command_valid_ = true;
    pending_command_dirty_ = true;
    last_command_request_ms_.store(steady_ms(), std::memory_order_relaxed);
  }

  return hardware_interface::return_type::OK;
}

void TheoHandModbusHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_parameter("serial_port", serial_port_);
  baudrate_ = parse_int(get_parameter("baudrate", std::to_string(baudrate_)), baudrate_);
  slave_id_ = std::clamp(
    parse_int(get_parameter("slave_id", std::to_string(slave_id_)), slave_id_),
    1,
    254);
  background_period_ms_ = std::max(
    1, parse_int(get_parameter("background_period_ms", std::to_string(background_period_ms_)),
                 background_period_ms_));
  feedback_quiet_after_write_ms_ = std::max(
    0,
    parse_int(
      get_parameter(
        "feedback_quiet_after_write_ms",
        std::to_string(feedback_quiet_after_write_ms_)),
      feedback_quiet_after_write_ms_));
  command_deadband_raw_ = std::clamp(
    parse_int(get_parameter("command_deadband_raw", std::to_string(command_deadband_raw_)),
              command_deadband_raw_),
    0,
    static_cast<int>(theo_hand::kDefaultProtocolPositionMax));
  read_feedback_enabled_ = parse_bool(
    get_parameter("read_feedback", read_feedback_enabled_ ? "true" : "false"),
    read_feedback_enabled_);
}

bool TheoHandModbusHardware::validate_joint_interfaces() const
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

bool TheoHandModbusHardware::connect_modbus()
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
      "Failed to connect TheoHand STD16A Modbus RTU hand: %s",
      modbus_->getLastError().c_str());
    modbus_.reset();
    return false;
  }

  return true;
}

void TheoHandModbusHardware::disconnect_modbus()
{
  if (modbus_)
  {
    modbus_->disconnect();
    modbus_.reset();
  }
}

bool TheoHandModbusHardware::initialize_hand()
{
  if (!modbus_->writeRegister(kControlWordRegister, kEnableControlWord))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write TheoHand control word 0x%04X: %s",
      kEnableControlWord,
      modbus_->getLastError().c_str());
    return false;
  }

  return true;
}

void TheoHandModbusHardware::detect_hand_side()
{
  uint16_t side_register = 0;
  uint16_t register_address = kModbusLeftRightRegister;
  auto read_count = modbus_->readHoldingRegisters(register_address, 1, &side_register);
  if (read_count != 1)
  {
    register_address = kObjectDictionaryLeftRightRegister;
    read_count = modbus_->readHoldingRegisters(register_address, 1, &side_register);
    if (read_count != 1)
    {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Failed to read TheoHand LeftRight registers 0x%04X and 0x%04X: %s",
        kModbusLeftRightRegister,
        kObjectDictionaryLeftRightRegister,
        modbus_->getLastError().c_str());
      detected_hand_side_ = theo_hand::HandSide::UNKNOWN;
      return;
    }
  }

  detected_hand_side_ = theo_hand::hand_side_from_register(side_register);
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Detected TheoHand side from register 0x%04X: %s (%u)",
    register_address,
    hand_side_to_string(detected_hand_side_),
    side_register);
}

void TheoHandModbusHardware::start_background_thread()
{
  if (background_running_.load())
  {
    return;
  }

  background_running_ = true;
  background_thread_ = std::thread(&TheoHandModbusHardware::background_loop, this);
}

void TheoHandModbusHardware::stop_background_thread()
{
  background_running_ = false;
  if (background_thread_.joinable())
  {
    background_thread_.join();
  }
}

void TheoHandModbusHardware::update_background_period(const rclcpp::Duration& period)
{
  if (background_period_initialized_.exchange(true))
  {
    return;
  }

  const auto period_ms = std::max(1, static_cast<int>(period.seconds() * 1000.0));
  background_period_ms_ = std::max(background_period_ms_, period_ms);
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "TheoHand STD16A background period configured to %d ms",
    background_period_ms_);
}

void TheoHandModbusHardware::background_loop()
{
  auto next_feedback_time = std::chrono::steady_clock::time_point::min();

  while (background_running_.load())
  {
    const auto now = std::chrono::steady_clock::now();
    const auto target_period = std::chrono::milliseconds(background_period_ms_);
    const auto feedback_quiet_period = std::chrono::milliseconds(feedback_quiet_after_write_ms_);

    std::array<uint16_t, kJointCount> target_positions{};
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      should_send = pending_command_valid_ && pending_command_dirty_;
      if (should_send)
      {
        target_positions = pending_command_values_;
        pending_command_dirty_ = false;
      }
    }

    if (should_send)
    {
      const auto written_count = send_command(target_positions);
      if (written_count == static_cast<int>(target_positions.size()))
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_values_ = target_positions;
        command_sent_ = true;
      }
      else
      {
        const auto error = modbus_->getLastError();
        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_command_dirty_ = true;
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          1000,
          "Failed to write TheoHand STD16A target positions: %s (wrote %d registers, expected %zu); command will be retried",
          error.c_str(),
          written_count,
          target_positions.size());
      }
      next_feedback_time = std::chrono::steady_clock::now() + target_period;
    }
    else if (
      read_feedback_enabled_ && now >= next_feedback_time &&
      feedback_quiet_period_elapsed(now, feedback_quiet_period))
    {
      std::array<double, kJointCount> positions{};
      if (read_feedback(positions))
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_positions_ = positions;
        feedback_valid_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          2000,
          "No valid TheoHand STD16A position feedback received");
      }
      next_feedback_time = std::chrono::steady_clock::now() + target_period;
    }

    auto sleep_duration = kCommandPollPeriod;
    if (read_feedback_enabled_)
    {
      const auto remaining = next_feedback_time - std::chrono::steady_clock::now();
      if (remaining > std::chrono::steady_clock::duration::zero())
      {
        sleep_duration = std::min(
          kCommandPollPeriod,
          std::chrono::duration_cast<std::chrono::milliseconds>(remaining));
      }
    }

    if (sleep_duration > std::chrono::milliseconds(0))
    {
      std::this_thread::sleep_for(sleep_duration);
    }
  }
}

bool TheoHandModbusHardware::read_feedback(std::array<double, kJointCount>& positions)
{
  std::array<uint16_t, kJointCount> position_registers{};
  const auto count = modbus_->readHoldingRegisters(
    kRealPositionRegister, position_registers.size(), position_registers.data());
  if (count != static_cast<int>(position_registers.size()))
  {
    return false;
  }

  positions = registers_to_joints(position_registers);
  return true;
}

int TheoHandModbusHardware::send_command(
  const std::array<uint16_t, kJointCount>& target_positions)
{
  return modbus_->writeRegisters(
    kTargetPositionRegister, target_positions.size(), target_positions.data());
}

bool TheoHandModbusHardware::command_changed(
  const std::array<uint16_t, kJointCount>& target_positions) const
{
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (std::abs(static_cast<int>(target_positions[i]) - static_cast<int>(last_command_values_[i])) >
        command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

bool TheoHandModbusHardware::feedback_quiet_period_elapsed(
  std::chrono::steady_clock::time_point now,
  std::chrono::milliseconds quiet_period) const
{
  if (quiet_period <= std::chrono::milliseconds(0))
  {
    return true;
  }

  const auto last_request_ms = last_command_request_ms_.load(std::memory_order_relaxed);
  if (last_request_ms == 0)
  {
    return true;
  }

  const auto now_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
  return (now_ms - last_request_ms) >= quiet_period.count();
}

std::array<uint16_t, TheoHandModbusHardware::kJointCount>
TheoHandModbusHardware::joints_to_registers(
  const std::array<double, kJointCount>& joints) const
{
  std::array<uint16_t, kJointCount> registers{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    registers[i] = theo_hand::radians_to_protocol_position(
      joints[i], lower_limits_[i], upper_limits_[i]);
  }
  return registers;
}

std::array<double, TheoHandModbusHardware::kJointCount>
TheoHandModbusHardware::registers_to_joints(
  const std::array<uint16_t, kJointCount>& registers) const
{
  std::array<double, kJointCount> joints{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    joints[i] = theo_hand::protocol_position_to_radians(
      static_cast<int>(static_cast<int16_t>(registers[i])),
      lower_limits_[i],
      upper_limits_[i]);
  }
  return joints;
}

bool TheoHandModbusHardware::parse_bool(const std::string& value, bool default_value)
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

int TheoHandModbusHardware::parse_int(const std::string& value, int default_value)
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

double TheoHandModbusHardware::initial_value_for_joint(
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

std::array<double, TheoHandModbusHardware::kJointCount>
TheoHandModbusHardware::default_lower_limits(const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{};
  limits.fill(0.0);

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = -0.001;
    }
    else if (has_suffix(name, "thumb_joint4"))
    {
      limits[i] = -0.610865;
    }
  }

  return limits;
}

std::array<double, TheoHandModbusHardware::kJointCount>
TheoHandModbusHardware::default_upper_limits(const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{};
  limits.fill(1.5708);

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 0.04;
    }
    else if (has_suffix(name, "thumb_joint4"))
    {
      limits[i] = 1.48353;
    }
  }

  return limits;
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::TheoHandModbusHardware,
  hardware_interface::SystemInterface)
