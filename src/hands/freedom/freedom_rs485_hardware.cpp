#include "modbus_ros2_control/hands/freedom/freedom_rs485_hardware.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <pluginlib/class_list_macros.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace modbus_ros2_control
{
namespace
{
constexpr auto kLoggerName = "FreedomRS485Hardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

speed_t baud_to_constant(int baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return B115200;
  }
}
}  // namespace

hardware_interface::CallbackReturn FreedomRS485Hardware::on_init(
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
      "Freedom RS485 hardware expects exactly %zu joints, got %zu",
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
    last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured Freedom RS485 hardware: port=%s, baudrate=%d, id=%u, feedback=%s",
    serial_port_.c_str(),
    baudrate_,
    slave_id_,
    read_feedback_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FreedomRS485Hardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  pending_command_valid_ = false;
  pending_command_dirty_ = false;
  feedback_positions_valid_ = false;
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
    pending_command_angles_[i] = last_command_angles_[i];
    feedback_positions_[i] = hw_positions_[i];
  }

  std::array<double, kJointCount> initial_positions{};
  if (read_feedback_ && query_positions(initial_positions))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_positions_[i] = initial_positions[i];
      hw_commands_[i] = hw_positions_[i];
      last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
      pending_command_angles_[i] = last_command_angles_[i];
      feedback_positions_[i] = hw_positions_[i];
    }
    feedback_positions_valid_ = true;
  }

  start_background_thread();

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Freedom RS485 hardware activated on %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FreedomRS485Hardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  stop_background_thread();
  close_serial();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Freedom RS485 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
FreedomRS485Hardware::on_export_state_interfaces()
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
FreedomRS485Hardware::on_export_command_interfaces()
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

hardware_interface::return_type FreedomRS485Hardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_)
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (feedback_positions_valid_)
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

hardware_interface::return_type FreedomRS485Hardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  std::array<uint8_t, kJointCount> angles{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
    angles[i] = radians_to_protocol_angle(hw_commands_[i], i);
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sent_ && !command_changed(angles))
    {
      return hardware_interface::return_type::OK;
    }

    pending_command_angles_ = angles;
    pending_command_valid_ = true;
    pending_command_dirty_ = true;
  }

  return hardware_interface::return_type::OK;
}

void FreedomRS485Hardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_parameter("serial_port", serial_port_);
  hand_side_ = get_parameter("hand_side", hand_side_);
  baudrate_ = parse_int(get_parameter("baudrate", std::to_string(baudrate_)), baudrate_);
  feedback_timeout_ms_ = std::max(
    0, parse_int(get_parameter("feedback_timeout_ms", std::to_string(feedback_timeout_ms_)),
                 feedback_timeout_ms_));
  background_period_ms_ = std::max(
    1, parse_int(get_parameter("background_period_ms", std::to_string(background_period_ms_)),
                 background_period_ms_));
  command_deadband_deg_ = std::clamp(
    parse_int(get_parameter("command_deadband_deg", std::to_string(command_deadband_deg_)),
              command_deadband_deg_),
    0,
    static_cast<int>(kMaxProtocolAngle));
  read_feedback_ = parse_bool(
    get_parameter("read_feedback", read_feedback_ ? "true" : "false"), read_feedback_);

  auto side_lower = hand_side_;
  std::transform(side_lower.begin(), side_lower.end(), side_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  const auto side_default_slave_id = side_lower == "right" || side_lower == "right_hand" ? 0 : 1;
  const auto slave_id_value = get_parameter("slave_id", "auto");
  if (slave_id_value == "auto" || slave_id_value == "AUTO" || slave_id_value.empty())
  {
    slave_id_ = static_cast<uint8_t>(side_default_slave_id);
  }
  else
  {
    const auto slave_id = parse_int(slave_id_value, side_default_slave_id);
    slave_id_ = static_cast<uint8_t>(std::clamp(slave_id, 0, 247));
  }
}

bool FreedomRS485Hardware::validate_joint_interfaces() const
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

bool FreedomRS485Hardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to open serial port '%s': %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  if (!configure_serial())
  {
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void FreedomRS485Hardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool FreedomRS485Hardware::configure_serial()
{
  termios options {};
  if (tcgetattr(serial_fd_, &options) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }

  cfmakeraw(&options);
  const auto baud = baud_to_constant(baudrate_);
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

void FreedomRS485Hardware::start_background_thread()
{
  if (background_running_.load())
  {
    return;
  }

  background_running_.store(true);
  background_thread_ = std::thread(&FreedomRS485Hardware::background_loop, this);
}

void FreedomRS485Hardware::stop_background_thread()
{
  background_running_.store(false);
  if (background_thread_.joinable())
  {
    background_thread_.join();
  }
}

void FreedomRS485Hardware::background_loop()
{
  while (background_running_.load())
  {
    const auto loop_start = std::chrono::steady_clock::now();

    if (read_feedback_)
    {
      std::array<double, kJointCount> positions{};
      if (query_positions(positions))
      {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        feedback_positions_ = positions;
        feedback_positions_valid_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          2000,
          "No valid Freedom RS485 angle feedback received from ID %u",
          slave_id_);
      }
    }

    std::array<uint8_t, kJointCount> angles{};
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_valid_ && pending_command_dirty_ &&
          (!command_sent_ || command_changed(pending_command_angles_)))
      {
        angles = pending_command_angles_;
        should_send = true;
        pending_command_dirty_ = false;
      }
    }

    if (should_send)
    {
      if (send_position_command(angles))
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_angles_ = angles;
        command_sent_ = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          1000,
          "Failed to write Freedom RS485 position command to ID %u; command will be retried",
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
  }
}

bool FreedomRS485Hardware::send_frame(const std::vector<uint8_t>& frame)
{
  const auto bytes_written = ::write(serial_fd_, frame.data(), frame.size());
  if (bytes_written != static_cast<ssize_t>(frame.size()))
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *get_node()->get_clock(),
      1000,
      "Failed to write Freedom RS485 frame: %s",
      std::strerror(errno));
    return false;
  }

  tcdrain(serial_fd_);
  return true;
}

bool FreedomRS485Hardware::read_frame(std::vector<uint8_t>& frame, int timeout_ms)
{
  frame.clear();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() <= deadline)
  {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(serial_fd_, &read_set);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    const auto select_result = select(serial_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
    if (select_result < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "select failed: %s", std::strerror(errno));
      return false;
    }

    if (select_result == 0 || !FD_ISSET(serial_fd_, &read_set))
    {
      continue;
    }

    uint8_t byte = 0;
    const auto bytes_read = ::read(serial_fd_, &byte, 1);
    if (bytes_read < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
      {
        continue;
      }
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "read failed: %s", std::strerror(errno));
      return false;
    }
    if (bytes_read == 0)
    {
      continue;
    }

    if (frame.empty())
    {
      if (byte != kFrameHead)
      {
        continue;
      }
      frame.push_back(byte);
      continue;
    }

    frame.push_back(byte);
    if (frame.size() == 4 && frame[3] < 7)
    {
      frame.clear();
      continue;
    }

    if (frame.size() >= 4 && frame.size() == frame[3])
    {
      return frame.back() == kFrameTail && checksum(frame) == frame[frame.size() - 2];
    }
  }

  return false;
}

bool FreedomRS485Hardware::query_positions(std::array<double, kJointCount>& positions)
{
  std::vector<uint8_t> query = {
    kFrameHead,
    slave_id_,
    kAngleQueryCommand,
    kQueryFrameLength,
    kHostToDevice,
    0x00,
    0x00,
    kFrameTail};
  query[query.size() - 2] = checksum(query);

  tcflush(serial_fd_, TCIFLUSH);
  if (!send_frame(query))
  {
    return false;
  }

  std::vector<uint8_t> response;
  if (!read_frame(response, feedback_timeout_ms_))
  {
    return false;
  }

  if (response.size() != kAngleResponseLength ||
      response[1] != slave_id_ ||
      response[2] != kAngleQueryCommand ||
      response[4] != kDeviceToHost)
  {
    return false;
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    positions[i] = protocol_angle_to_radians(response[5 + i], i);
  }

  return true;
}

bool FreedomRS485Hardware::send_position_command(
  const std::array<uint8_t, kJointCount>& angles)
{
  std::vector<uint8_t> frame = {
    kFrameHead,
    slave_id_,
    kMoveCommand,
    kMoveFrameLength,
    kHostToDevice,
    0x01,
    angles[0],
    0x01,
    angles[1],
    0x01,
    angles[2],
    0x01,
    angles[3],
    0x01,
    angles[4],
    0x01,
    angles[5],
    0x00,
    kFrameTail};
  frame[frame.size() - 2] = checksum(frame);
  return send_frame(frame);
}

uint8_t FreedomRS485Hardware::radians_to_protocol_angle(
  double radians,
  std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return 0;
  }

  const auto normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  const auto angle = std::lround(normalized * static_cast<double>(kMaxProtocolAngle));
  return static_cast<uint8_t>(std::clamp<long>(angle, 0, kMaxProtocolAngle));
}

double FreedomRS485Hardware::protocol_angle_to_radians(
  uint8_t angle,
  std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return lower;
  }

  const auto normalized = std::clamp(
    static_cast<double>(angle) / static_cast<double>(kMaxProtocolAngle), 0.0, 1.0);
  return lower + normalized * (upper - lower);
}

bool FreedomRS485Hardware::command_changed(
  const std::array<uint8_t, kJointCount>& angles) const
{
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (std::abs(static_cast<int>(angles[i]) - static_cast<int>(last_command_angles_[i])) >
        command_deadband_deg_)
    {
      return true;
    }
  }
  return false;
}

uint8_t FreedomRS485Hardware::checksum(const std::vector<uint8_t>& frame)
{
  if (frame.size() < 4)
  {
    return 0;
  }

  uint16_t sum = 0;
  for (std::size_t i = 1; i + 2 < frame.size(); ++i)
  {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

bool FreedomRS485Hardware::parse_bool(const std::string& value, bool default_value)
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

int FreedomRS485Hardware::parse_int(const std::string& value, int default_value)
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

double FreedomRS485Hardware::initial_value_for_joint(
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

std::array<double, FreedomRS485Hardware::kJointCount> FreedomRS485Hardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{0.785, 0.29, 1.24, 1.24, 1.24, 1.24};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 0.785;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 0.29;
    }
    else
    {
      limits[i] = 1.24;
    }
  }

  return limits;
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::FreedomRS485Hardware,
  hardware_interface::SystemInterface)
