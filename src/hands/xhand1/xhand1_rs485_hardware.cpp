#include "modbus_ros2_control/hands/xhand1/xhand1_rs485_hardware.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace modbus_ros2_control
{
namespace
{
constexpr auto kLoggerName = "XHand1RS485Hardware";

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
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
#ifdef B1500000
    case 1500000:
      return B1500000;
#endif
#ifdef B2000000
    case 2000000:
      return B2000000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
    default:
      return B115200;
  }
}

}  // namespace

hardware_interface::CallbackReturn XHand1RS485Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "XHAND1 RS485 hardware expects exactly %zu joints, got %zu",
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

  lower_limits_ = default_lower_limits();
  upper_limits_ = default_upper_limits();

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_positions_[i] = std::clamp(
      initial_value_for_joint(info_.joints[i]), lower_limits_[i], upper_limits_[i]);
    previous_positions_[i] = hw_positions_[i];
    hw_commands_[i] = hw_positions_[i];
    feedback_positions_[i] = hw_positions_[i];
    feedback_efforts_[i] = 0.0;
    last_command_positions_[i] = hw_positions_[i];
    pending_command_positions_[i] = hw_positions_[i];
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured XHAND1 RS485 hardware: joints=%zu, port=%s, baudrate=%d, host_id=0x%02X, hand_id=0x%02X, board_id=0x%02X, feedback=%s, kp=%d, ki=%d, kd=%d, torque_limit=%u",
    kJointCount,
    serial_port_.c_str(),
    baudrate_,
    host_id_,
    hand_id_,
    static_cast<uint8_t>(hand_id_ | 0x80),
    read_feedback_ ? "true" : "false",
    kp_,
    ki_,
    kd_,
    torque_limit_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XHand1RS485Hardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_serial())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  pending_command_valid_ = true;
  pending_command_dirty_ = true;
  feedback_positions_valid_ = false;
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    pending_command_positions_[i] = hw_positions_[i];
    last_command_positions_[i] = hw_positions_[i];
    feedback_positions_[i] = hw_positions_[i];
    feedback_efforts_[i] = 0.0;
  }

  start_background_thread();

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "XHAND1 RS485 hardware activated on %s",
    serial_port_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XHand1RS485Hardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  stop_background_thread();
  close_serial();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "XHAND1 RS485 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
XHand1RS485Hardware::on_export_state_interfaces()
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
XHand1RS485Hardware::on_export_command_interfaces()
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

hardware_interface::return_type XHand1RS485Hardware::read(
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

hardware_interface::return_type XHand1RS485Hardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kJointCount> commands{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
    commands[i] = hw_commands_[i];
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_sent_ && !command_changed(commands))
    {
      return hardware_interface::return_type::OK;
    }

    pending_command_positions_ = commands;
    pending_command_valid_ = true;
    pending_command_dirty_ = true;
  }

  return hardware_interface::return_type::OK;
}

void XHand1RS485Hardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_parameter("serial_port", serial_port_);
  baudrate_ = parse_int(get_parameter("baudrate", std::to_string(baudrate_)), baudrate_);
  host_id_ = static_cast<uint8_t>(std::clamp(
    parse_int(get_parameter("host_id", std::to_string(host_id_)), host_id_), 0, 255));
  hand_id_ = static_cast<uint8_t>(std::clamp(
    parse_int(get_parameter("hand_id", std::to_string(hand_id_)), hand_id_), 0, 127));
  feedback_timeout_ms_ = std::max(
    1, parse_int(get_parameter("feedback_timeout_ms", std::to_string(feedback_timeout_ms_)),
                 feedback_timeout_ms_));
  background_period_ms_ = std::max(
    1, parse_int(get_parameter("background_period_ms", std::to_string(background_period_ms_)),
                 background_period_ms_));
  command_deadband_rad_ = std::max(
    0.0,
    parse_double(
      get_parameter("command_deadband_rad", std::to_string(command_deadband_rad_)),
      command_deadband_rad_));
  kp_ = static_cast<int16_t>(std::clamp(
    parse_int(get_parameter("kp", std::to_string(kp_)), kp_),
    static_cast<int>(std::numeric_limits<int16_t>::min()),
    static_cast<int>(std::numeric_limits<int16_t>::max())));
  ki_ = static_cast<int16_t>(std::clamp(
    parse_int(get_parameter("ki", std::to_string(ki_)), ki_),
    static_cast<int>(std::numeric_limits<int16_t>::min()),
    static_cast<int>(std::numeric_limits<int16_t>::max())));
  kd_ = static_cast<int16_t>(std::clamp(
    parse_int(get_parameter("kd", std::to_string(kd_)), kd_),
    static_cast<int>(std::numeric_limits<int16_t>::min()),
    static_cast<int>(std::numeric_limits<int16_t>::max())));
  torque_limit_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("torque_limit", std::to_string(torque_limit_)), torque_limit_),
    0,
    static_cast<int>(std::numeric_limits<uint16_t>::max())));
  control_mode_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("control_mode", std::to_string(control_mode_)), control_mode_),
    0,
    static_cast<int>(std::numeric_limits<uint16_t>::max())));
  read_feedback_ = parse_bool(
    get_parameter("read_feedback", read_feedback_ ? "true" : "false"), read_feedback_);
}

bool XHand1RS485Hardware::validate_joint_interfaces() const
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

bool XHand1RS485Hardware::open_serial()
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

void XHand1RS485Hardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool XHand1RS485Hardware::configure_serial()
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

void XHand1RS485Hardware::start_background_thread()
{
  if (background_running_.load())
  {
    return;
  }

  background_running_.store(true);
  background_thread_ = std::thread(&XHand1RS485Hardware::background_loop, this);
}

void XHand1RS485Hardware::stop_background_thread()
{
  background_running_.store(false);
  if (background_thread_.joinable())
  {
    background_thread_.join();
  }
}

void XHand1RS485Hardware::background_loop()
{
  while (background_running_.load())
  {
    const auto loop_start = std::chrono::steady_clock::now();

    std::array<double, kJointCount> commands{};
    bool should_send = false;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      if (pending_command_valid_)
      {
        commands = pending_command_positions_;
        should_send = true;
        pending_command_dirty_ = false;
      }
    }

    if (should_send)
    {
      if (send_realtime_command(commands))
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        last_command_positions_ = commands;
        command_sent_ = true;
      }
      else
      {
        std::string error;
        {
          std::lock_guard<std::mutex> error_lock(error_mutex_);
          error = last_exchange_error_;
        }
        RCLCPP_WARN_THROTTLE(
          rclcpp::get_logger(kLoggerName),
          *get_node()->get_clock(),
          1000,
          "Failed to exchange XHAND1 RS485 realtime frame with board ID 0x%02X: %s; command will be retried",
          static_cast<uint8_t>(hand_id_ | 0x80),
          error.empty() ? "unknown error" : error.c_str());
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

bool XHand1RS485Hardware::send_realtime_command(
  const std::array<double, kJointCount>& commands)
{
  std::vector<uint8_t> frame;
  frame.reserve(kFrameHeaderLength + kRealtimeCommandDataLength + kCrcLength);
  frame.push_back(kFrameHead0);
  frame.push_back(kFrameHead1);
  frame.push_back(host_id_);
  frame.push_back(static_cast<uint8_t>(hand_id_ | 0x80));
  frame.push_back(kRealtimeCommand);
  append_u16_le(frame, static_cast<uint16_t>(kRealtimeCommandDataLength));

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    append_u16_le(frame, static_cast<uint16_t>(i));
    append_i16_le(frame, kp_);
    append_i16_le(frame, ki_);
    append_i16_le(frame, kd_);
    append_float_le(frame, static_cast<float>(commands[i]));
    append_u16_le(frame, torque_limit_);
    append_u16_le(frame, control_mode_);
    append_u16_le(frame, 0);
    append_u16_le(frame, 0);
    append_u16_le(frame, 0);
    append_u16_le(frame, 0);
  }

  const auto crc = crc16_ccitt_zero(frame.data(), frame.size());
  append_u16_le(frame, crc);

  tcflush(serial_fd_, TCIFLUSH);
  if (!send_frame(frame))
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_exchange_error_ = "serial write failed";
    return false;
  }

  if (!read_feedback_)
  {
    return true;
  }

  std::vector<uint8_t> response;
  if (!read_frame(response, feedback_timeout_ms_))
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_exchange_error_ = response.empty()
                             ? "feedback timeout/no frame"
                             : "feedback frame CRC/length error, bytes=" + std::to_string(response.size());
    return false;
  }

  std::array<double, kJointCount> positions{};
  std::array<double, kJointCount> efforts{};
  if (!parse_realtime_response(response, positions, efforts))
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_exchange_error_ = "feedback frame parsed but did not match XHAND1 realtime response";
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_positions_ = positions;
    feedback_efforts_ = efforts;
    feedback_positions_valid_ = true;
  }

  return true;
}

bool XHand1RS485Hardware::send_frame(const std::vector<uint8_t>& frame)
{
  std::size_t total_written = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

  while (total_written < frame.size() && std::chrono::steady_clock::now() <= deadline)
  {
    const auto bytes_written =
      ::write(serial_fd_, frame.data() + total_written, frame.size() - total_written);
    if (bytes_written > 0)
    {
      total_written += static_cast<std::size_t>(bytes_written);
      continue;
    }

    if (bytes_written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
    {
      fd_set write_set;
      FD_ZERO(&write_set);
      FD_SET(serial_fd_, &write_set);
      timeval timeout {};
      timeout.tv_sec = 0;
      timeout.tv_usec = 1000;
      select(serial_fd_ + 1, nullptr, &write_set, nullptr, &timeout);
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(error_mutex_);
      last_exchange_error_ = std::string("serial write failed: ") + std::strerror(errno);
    }
    return false;
  }

  if (total_written != frame.size())
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_exchange_error_ = "serial write timeout after " + std::to_string(total_written) +
                           "/" + std::to_string(frame.size()) + " bytes";
    return false;
  }

  tcdrain(serial_fd_);
  return true;
}

bool XHand1RS485Hardware::read_frame(std::vector<uint8_t>& frame, int timeout_ms)
{
  frame.clear();
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timeout_ms);
  std::size_t expected_size = 0;

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

    uint8_t buffer[256];
    const auto bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
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

    for (ssize_t i = 0; i < bytes_read; ++i)
    {
      const auto byte = buffer[i];
      if (frame.empty())
      {
        if (byte != kFrameHead0)
        {
          continue;
        }
        frame.push_back(byte);
        continue;
      }

      if (frame.size() == 1 && byte != kFrameHead1)
      {
        frame.clear();
        if (byte == kFrameHead0)
        {
          frame.push_back(byte);
        }
        continue;
      }

      frame.push_back(byte);

      if (frame.size() == kFrameHeaderLength)
      {
        const auto data_length = read_u16_le(frame, 5);
        expected_size = kFrameHeaderLength + data_length + kCrcLength;
        if (expected_size < kFrameHeaderLength + kCrcLength)
        {
          frame.clear();
          expected_size = 0;
          continue;
        }
        frame.reserve(expected_size);
      }

      if (expected_size > 0 && frame.size() == expected_size)
      {
        const auto expected_crc = read_u16_le(frame, frame.size() - kCrcLength);
        const auto actual_crc = crc16_ccitt_zero(frame.data(), frame.size() - kCrcLength);
        if (expected_crc != actual_crc)
        {
          std::lock_guard<std::mutex> lock(error_mutex_);
          last_exchange_error_ = "feedback CRC mismatch expected=0x" +
                                 std::to_string(expected_crc) + " actual=0x" +
                                 std::to_string(actual_crc);
          return false;
        }
        return true;
      }
    }
  }

  return false;
}

bool XHand1RS485Hardware::parse_realtime_response(
  const std::vector<uint8_t>& frame,
  std::array<double, kJointCount>& positions,
  std::array<double, kJointCount>& efforts) const
{
  const auto minimum_size = kFrameHeaderLength + kRealtimeJointStateDataLength + kCrcLength;
  if (frame.size() < minimum_size)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(kLoggerName),
      "XHAND1 response too short: got %zu, need at least %zu",
      frame.size(),
      minimum_size);
    return false;
  }

  if (frame[0] != kFrameHead0 || frame[1] != kFrameHead1 ||
      frame[2] != static_cast<uint8_t>(hand_id_ | 0x80) ||
      frame[3] != host_id_ ||
      frame[4] != kRealtimeCommand)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(kLoggerName),
      "Ignoring non-XHAND1 realtime response: src=0x%02X dst=0x%02X cmd=0x%02X",
      frame.size() > 2 ? frame[2] : 0,
      frame.size() > 3 ? frame[3] : 0,
      frame.size() > 4 ? frame[4] : 0);
    return false;
  }

  const auto data_length = read_u16_le(frame, 5);
  if (data_length < kRealtimeJointStateDataLength)
  {
    RCLCPP_DEBUG(
      rclcpp::get_logger(kLoggerName),
      "XHAND1 response data too short: %u",
      data_length);
    return false;
  }

  for (std::size_t record = 0; record < kJointCount; ++record)
  {
    const auto offset = kFrameHeaderLength + record * kRealtimeStateBytesPerJoint;
    const auto joint_id = frame[offset];
    if (joint_id >= kJointCount)
    {
      continue;
    }

    positions[joint_id] = read_float_le(frame, offset + 2);
    efforts[joint_id] = static_cast<double>(read_u16_le(frame, offset + 6));
  }

  return true;
}

bool XHand1RS485Hardware::command_changed(
  const std::array<double, kJointCount>& commands) const
{
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (std::abs(commands[i] - last_command_positions_[i]) > command_deadband_rad_)
    {
      return true;
    }
  }
  return false;
}

void XHand1RS485Hardware::append_u16_le(std::vector<uint8_t>& frame, uint16_t value)
{
  frame.push_back(static_cast<uint8_t>(value & 0xFF));
  frame.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void XHand1RS485Hardware::append_i16_le(std::vector<uint8_t>& frame, int16_t value)
{
  append_u16_le(frame, static_cast<uint16_t>(value));
}

void XHand1RS485Hardware::append_float_le(std::vector<uint8_t>& frame, float value)
{
  static_assert(sizeof(float) == 4, "XHAND1 protocol expects 32-bit floats");
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  frame.push_back(static_cast<uint8_t>(raw & 0xFF));
  frame.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
  frame.push_back(static_cast<uint8_t>((raw >> 16) & 0xFF));
  frame.push_back(static_cast<uint8_t>((raw >> 24) & 0xFF));
}

uint16_t XHand1RS485Hardware::read_u16_le(
  const std::vector<uint8_t>& frame,
  std::size_t offset)
{
  if (offset + 1 >= frame.size())
  {
    return 0;
  }
  return static_cast<uint16_t>(frame[offset]) |
         (static_cast<uint16_t>(frame[offset + 1]) << 8);
}

float XHand1RS485Hardware::read_float_le(
  const std::vector<uint8_t>& frame,
  std::size_t offset)
{
  if (offset + 3 >= frame.size())
  {
    return 0.0F;
  }
  const uint32_t raw = static_cast<uint32_t>(frame[offset]) |
                       (static_cast<uint32_t>(frame[offset + 1]) << 8) |
                       (static_cast<uint32_t>(frame[offset + 2]) << 16) |
                       (static_cast<uint32_t>(frame[offset + 3]) << 24);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

uint16_t XHand1RS485Hardware::crc16_ccitt_zero(const uint8_t* data, std::size_t length)
{
  static constexpr std::array<uint16_t, 256> kTable = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0};

  uint16_t crc = 0;
  for (std::size_t i = 0; i < length; ++i)
  {
    crc = static_cast<uint16_t>((crc << 8) ^ kTable[((crc >> 8) ^ data[i]) & 0x00FF]);
  }
  return crc;
}

bool XHand1RS485Hardware::parse_bool(const std::string& value, bool default_value)
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

int XHand1RS485Hardware::parse_int(const std::string& value, int default_value)
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

double XHand1RS485Hardware::parse_double(const std::string& value, double default_value)
{
  try
  {
    return std::stod(value);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double XHand1RS485Hardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
    {
      return parse_double(interface.initial_value, 0.0);
    }
  }
  return 0.0;
}

std::array<double, XHand1RS485Hardware::kJointCount>
XHand1RS485Hardware::default_lower_limits()
{
  return {0.0, -0.698, 0.0, -0.174, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

std::array<double, XHand1RS485Hardware::kJointCount>
XHand1RS485Hardware::default_upper_limits()
{
  return {1.832, 1.57, 1.57, 0.174, 1.919, 1.919, 1.919, 1.919, 1.919, 1.919, 1.919, 1.919};
}

}  // namespace modbus_ros2_control

PLUGINLIB_EXPORT_CLASS(
  modbus_ros2_control::XHand1RS485Hardware,
  hardware_interface::SystemInterface)
