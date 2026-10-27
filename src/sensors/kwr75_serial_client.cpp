#include "modbus_ros2_control/sensors/kwr75_serial_client.h"
#include "modbus_ros2_control/communicator/usb_serial_latency.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace modbus_ros2_control
{
namespace
{
constexpr uint8_t kFrameMarker = 0xAA;
constexpr uint8_t kFrameEnd0 = 0x0D;
constexpr uint8_t kFrameEnd1 = 0x0A;
constexpr std::size_t kPayloadOffset = 2;
constexpr std::size_t kFloatBytes = 4;

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
#ifdef B2500000
    case 2500000:
      return B2500000;
#endif
#ifdef B3000000
    case 3000000:
      return B3000000;
#endif
    default:
      return 0;
  }
}

bool frame_matches(
  const std::vector<uint8_t>& buffer,
  std::size_t start,
  uint8_t command_code)
{
  if (start + Kwr75SerialClient::kFrameLength > buffer.size())
  {
    return false;
  }
  const uint8_t header = buffer[start];
  if (header != command_code && header != 0x48 && header != 0x49)
  {
    return false;
  }
  if (buffer[start + 1] != kFrameMarker)
  {
    return false;
  }
  if (buffer[start + Kwr75SerialClient::kFrameLength - 2] != kFrameEnd0 ||
      buffer[start + Kwr75SerialClient::kFrameLength - 1] != kFrameEnd1)
  {
    return false;
  }
  return true;
}

}  // namespace

Kwr75SerialClient::Kwr75SerialClient(
  std::string serial_port,
  int baudrate,
  uint8_t command_code,
  bool convert_to_si,
  double gravity,
  int response_timeout_ms,
  int read_timeout_ms,
  int startup_delay_ms,
  int warmup_attempts)
: serial_port_(std::move(serial_port))
, baudrate_(baudrate)
, command_code_(command_code)
, convert_to_si_(convert_to_si)
, gravity_(gravity)
, response_timeout_ms_(response_timeout_ms)
, read_timeout_ms_(read_timeout_ms)
, startup_delay_ms_(startup_delay_ms)
, warmup_attempts_(warmup_attempts)
{
}

bool Kwr75SerialClient::connect()
{
  disconnect();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    return false;
  }

  if (!configure_serial())
  {
    disconnect();
    return false;
  }

  configure_usb_serial_latency(serial_port_);
  tcflush(serial_fd_, TCIOFLUSH);
  streaming_started_ = false;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    have_frame_ = false;
    frame_seq_ = 0;
  }
  return true;
}

void Kwr75SerialClient::disconnect()
{
  // 必须先停线程再关 fd，否则线程可能在 select() 里拿到已关闭的 fd。
  stop_reader();
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  streaming_started_ = false;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    have_frame_ = false;
  }
}

void Kwr75SerialClient::stop_reader()
{
  reader_running_.store(false, std::memory_order_relaxed);
  if (reader_thread_.joinable())
  {
    if (reader_thread_.get_id() == std::this_thread::get_id())
    {
      reader_thread_.detach();
    }
    else
    {
      reader_thread_.join();
    }
  }
  frame_cv_.notify_all();
}

bool Kwr75SerialClient::start_capture()
{
  if (!is_connected())
  {
    return false;
  }
  if (streaming_started_)
  {
    return true;
  }

  tcflush(serial_fd_, TCIOFLUSH);
  if (!send_start_command())
  {
    return false;
  }

  streaming_started_ = true;
  if (!reader_thread_.joinable())
  {
    reader_running_.store(true, std::memory_order_relaxed);
    reader_thread_ = std::thread(&Kwr75SerialClient::reader_loop, this);
  }
  return true;
}

void Kwr75SerialClient::reader_loop()
{
  std::vector<uint8_t> buffer;
  buffer.reserve(kFrameLength * 4);

  const int fd = serial_fd_;
  auto last_bytes_at = std::chrono::steady_clock::now();

  while (reader_running_.load(std::memory_order_relaxed))
  {
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd, &read_set);

    timeval timeout {};
    timeout.tv_sec = 0;
    timeout.tv_usec = 2000;  // 2 ms：无数据时也让出 CPU，有数据时立刻返回
    const auto select_result = select(fd + 1, &read_set, nullptr, nullptr, &timeout);
    if (select_result < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      break;
    }

    if (select_result == 0)
    {
      // 100 ms 一点数据都没有：传感器可能是「请求-应答」模式，补发一次启动命令。
      const auto idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - last_bytes_at).count();
      if (idle_ms > 100)
      {
        send_start_command();
        last_bytes_at = std::chrono::steady_clock::now();
      }
      continue;
    }

    uint8_t chunk[256];
    const auto bytes_read = ::read(fd, chunk, sizeof(chunk));
    if (bytes_read < 0)
    {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
      {
        continue;
      }
      break;
    }
    if (bytes_read == 0)
    {
      continue;
    }

    last_bytes_at = std::chrono::steady_clock::now();
    buffer.insert(buffer.end(), chunk, chunk + bytes_read);

    std::size_t consumed = 0;
    std::size_t start = 0;
    while (start + kFrameLength <= buffer.size())
    {
      if (!frame_matches(buffer, start, command_code_))
      {
        ++start;
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        std::copy_n(
          buffer.begin() + static_cast<std::ptrdiff_t>(start),
          kFrameLength,
          latest_frame_.begin());
        latest_frame_stamp_ = std::chrono::steady_clock::now();
        have_frame_ = true;
        ++frame_seq_;
      }
      frame_cv_.notify_all();
      start += kFrameLength;
      consumed = start;
    }

    if (consumed > 0)
    {
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(consumed));
    }
    if (buffer.size() > kFrameLength * 8)
    {
      // 长时间对不上帧（波特率/协议不符），丢掉重来，别让缓冲区无限长。
      record_io_sample(buffer);
      buffer.clear();
    }
  }
}

void Kwr75SerialClient::record_io_sample(const std::vector<uint8_t>& buffer)
{
  if (buffer.empty())
  {
    last_io_sample_hex_.clear();
    return;
  }
  const std::size_t sample_len = std::min(buffer.size(), std::size_t{32});
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < sample_len; ++i)
  {
    if (i > 0)
    {
      oss << ' ';
    }
    oss << std::setw(2) << static_cast<int>(buffer[i]);
  }
  if (buffer.size() > sample_len)
  {
    oss << " ...";
  }
  std::lock_guard<std::mutex> lock(io_mutex_);
  last_io_sample_hex_ = oss.str();
}

bool Kwr75SerialClient::warmup()
{
  if (!is_connected())
  {
    return false;
  }
  if (!start_capture())
  {
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(startup_delay_ms_));

  std::array<double, kAxisCount> wrench {};
  for (int attempt = 0; attempt < warmup_attempts_; ++attempt)
  {
    if (read_wrench(wrench, response_timeout_ms_))
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool Kwr75SerialClient::read_wrench(std::array<double, kAxisCount>& wrench_si, int timeout_ms)
{
  if (!is_connected())
  {
    return false;
  }
  if (!streaming_started_ && !start_capture())
  {
    return false;
  }

  const int effective_timeout = timeout_ms >= 0 ? timeout_ms : read_timeout_ms_;

  std::array<uint8_t, kFrameLength> frame {};
  if (!read_latest_frame(frame, effective_timeout))
  {
    return false;
  }

  std::array<float, kAxisCount> raw_values {};
  if (!parse_frame(frame, raw_values, command_code_))
  {
    return false;
  }

  const double force_scale = convert_to_si_ ? gravity_ : 1.0;
  const double torque_scale = convert_to_si_ ? gravity_ : 1.0;
  wrench_si[0] = static_cast<double>(raw_values[0]) * force_scale;
  wrench_si[1] = static_cast<double>(raw_values[1]) * force_scale;
  wrench_si[2] = static_cast<double>(raw_values[2]) * force_scale;
  wrench_si[3] = static_cast<double>(raw_values[3]) * torque_scale;
  wrench_si[4] = static_cast<double>(raw_values[4]) * torque_scale;
  wrench_si[5] = static_cast<double>(raw_values[5]) * torque_scale;
  return true;
}

bool Kwr75SerialClient::configure_serial()
{
  termios options {};
  if (tcgetattr(serial_fd_, &options) != 0)
  {
    return false;
  }

  cfmakeraw(&options);
  const auto baud = baud_to_constant(baudrate_);
  if (baud == 0)
  {
    return false;
  }

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
  return tcsetattr(serial_fd_, TCSANOW, &options) == 0;
}

bool Kwr75SerialClient::send_start_command()
{
  const uint8_t request[4] = {command_code_, kFrameMarker, kFrameEnd0, kFrameEnd1};
  std::size_t total_written = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

  while (total_written < sizeof(request) && std::chrono::steady_clock::now() <= deadline)
  {
    const auto bytes_written =
      ::write(serial_fd_, request + total_written, sizeof(request) - total_written);
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
      timeout.tv_usec = 1000;
      select(serial_fd_ + 1, nullptr, &write_set, nullptr, &timeout);
      continue;
    }

    return false;
  }

  return total_written == sizeof(request);
}

// 从后台线程的缓存里取最新一帧。timeout_ms 只是「等一帧新数据」的上限：
//   - timeout_ms == 0 → 立刻返回当前缓存（read() 走这条路径，不占用实时环时间）
//   - timeout_ms  > 0 → 最多等这么久（warmup / 诊断用）
// 拿不到新帧但缓存还没过期时，返回旧帧而不是 false —— 单帧丢失不应该把
// 力/力矩置 0（那会变成一次假的力阶跃，直接踢到导纳环）。
bool Kwr75SerialClient::read_latest_frame(
  std::array<uint8_t, kFrameLength>& frame,
  int timeout_ms)
{
  std::unique_lock<std::mutex> lock(frame_mutex_);
  const uint64_t seq_at_entry = frame_seq_;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(std::max(timeout_ms, 0));

  if (timeout_ms > 0)
  {
    frame_cv_.wait_until(lock, deadline, [this, seq_at_entry] {
      return !reader_running_.load(std::memory_order_relaxed) || frame_seq_ != seq_at_entry;
    });
  }

  if (!have_frame_)
  {
    return false;
  }

  if (frame_seq_ == seq_at_entry)
  {
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - latest_frame_stamp_).count();
    if (age_ms > stale_timeout_ms_)
    {
      return false;
    }
  }

  frame = latest_frame_;
  return true;
}

float Kwr75SerialClient::decode_wire_float(const uint8_t* wire_bytes)
{
  float value = 0.0F;
  std::memcpy(&value, wire_bytes, sizeof(value));
  return value;
}

bool Kwr75SerialClient::parse_frame(
  const std::array<uint8_t, kFrameLength>& frame,
  std::array<float, kAxisCount>& values,
  uint8_t command_code)
{
  const uint8_t header = frame[0];
  if (header != command_code && header != 0x48 && header != 0x49)
  {
    return false;
  }
  if (frame[1] != kFrameMarker)
  {
    return false;
  }
  if (frame[kFrameLength - 2] != kFrameEnd0 || frame[kFrameLength - 1] != kFrameEnd1)
  {
    return false;
  }

  for (std::size_t axis = 0; axis < kAxisCount; ++axis)
  {
    const std::size_t offset = kPayloadOffset + axis * kFloatBytes;
    values[axis] = decode_wire_float(frame.data() + offset);
  }
  return true;
}

}  // namespace modbus_ros2_control
