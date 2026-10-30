#include "modbus_ros2_control/sensors/kwr75_serial_client.h"
#include "modbus_ros2_control/communicator/usb_serial_latency.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

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
long long steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
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
  frames_received_.store(0, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(io_mutex_);
    reader_stop_reason_.clear();
  }
  latest_frame_.reset();
  last_delivered_valid_ = false;
  last_delivered_stamp_ns_ = -1;
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
  latest_frame_.reset();
  last_delivered_valid_ = false;
  last_delivered_stamp_ns_ = -1;
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
  const auto logger = rclcpp::get_logger("modbus_ros2_control.Kwr75SerialClient");
  auto last_bytes_at = std::chrono::steady_clock::now();
  std::string exit_reason = "reader loop finished";

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
      const int saved_errno = errno;
      exit_reason = "select() failed: errno=" + std::to_string(saved_errno) + " (" +
        std::strerror(saved_errno) + ")";
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
      const int saved_errno = errno;
      exit_reason = "read() failed: errno=" + std::to_string(saved_errno) + " (" +
        std::strerror(saved_errno) + ")";
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
      // 无锁发布：写者=reader 线程，读者=控制线程。
      std::array<uint8_t, kFrameLength> frame {};
      std::copy_n(
        buffer.begin() + static_cast<std::ptrdiff_t>(start),
        kFrameLength,
        frame.begin());
      const long long now_ns = steady_now_ns();
      latest_frame_.publish(frame, now_ns);
      frames_received_.fetch_add(1, std::memory_order_relaxed);
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

  const long exit_idle_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_bytes_at).count();
  set_reader_stop_state(exit_reason);

  if (exit_reason == "reader loop finished" &&
      !reader_running_.load(std::memory_order_relaxed))
  {
    RCLCPP_INFO(
      logger, "KWR75 %s reader thread stopped", serial_port_.c_str());
  }
  else
  {
    // 线程非正常退出（select/read 报错）：必须留下现场，否则缓存永远不更新但无任何提示。
    RCLCPP_ERROR(
      logger,
      "KWR75 %s reader thread exited: %s (frames=%llu, silent=%ld ms). "
      "No further frames will be cached until the sensor component is re-activated.",
      serial_port_.c_str(), exit_reason.c_str(),
      static_cast<unsigned long long>(frames_received_.load(std::memory_order_relaxed)),
      static_cast<long>(exit_idle_ms));
  }
}

// ── 无锁帧交接（seqlock）────────────────────────────────────────────────
// 写者（reader 线程）用奇数序号标记"更新中"，读者只在序号稳定且前后一致时
// 取走数据；拿不到就有界自旋后放弃，绝不阻塞实时环。
void Kwr75SerialClient::FrameSlot::reset()
{
  sequence.store(0, std::memory_order_relaxed);
  stamp_ns.store(0, std::memory_order_relaxed);
  has_data.store(false, std::memory_order_release);
}

void Kwr75SerialClient::FrameSlot::publish(
  const std::array<uint8_t, kFrameLength>& src,
  long long now_ns)
{
  const uint64_t s0 = sequence.load(std::memory_order_relaxed);
  sequence.store(s0 + 1, std::memory_order_relaxed);   // 奇数：正在写
  std::atomic_thread_fence(std::memory_order_release);
  frame = src;
  stamp_ns.store(now_ns, std::memory_order_relaxed);
  sequence.store(s0 + 2, std::memory_order_release);   // 偶数：数据稳定
  has_data.store(true, std::memory_order_release);
}

bool Kwr75SerialClient::FrameSlot::try_read(
  std::array<uint8_t, kFrameLength>& out,
  long long& stamp_ns_out) const
{
  if (!has_data.load(std::memory_order_acquire))
  {
    return false;
  }
  constexpr int kMaxSpins = 8;   // 一次 publish 只有几十 ns，8 次足够；绝不等待
  for (int spin = 0; spin < kMaxSpins; ++spin)
  {
    const uint64_t s1 = sequence.load(std::memory_order_acquire);
    if ((s1 & 1ULL) != 0ULL)
    {
      continue;   // 写者正在更新，重试
    }
    const auto copy = frame;
    const long long stamp = stamp_ns.load(std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint64_t s2 = sequence.load(std::memory_order_acquire);
    if (s1 == s2)
    {
      out = copy;
      stamp_ns_out = stamp;
      return true;
    }
  }
  return false;
}

bool Kwr75SerialClient::acquire_latest_frame(
  std::array<uint8_t, kFrameLength>& frame,
  int timeout_ms)
{
  std::array<uint8_t, kFrameLength> candidate {};
  long long stamp_ns = 0;

  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(std::max(timeout_ms, 0));

  for (;;)
  {
    if (latest_frame_.try_read(candidate, stamp_ns))
    {
      last_delivered_frame_ = candidate;
      last_delivered_stamp_ns_ = stamp_ns;
      last_delivered_valid_ = true;
      break;
    }
    if (timeout_ms > 0)
    {
      // warmup / 诊断路径：可以等，但不进实时环。
      if (!reader_running_.load(std::memory_order_relaxed) ||
          std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    // timeout_ms == 0：实时环不等待。写者恰好正在更新时复用上次取到的快照。
    break;
  }

  if (!last_delivered_valid_)
  {
    return false;
  }
  // 新鲜度判定：超过 stale_timeout_ms_ 视为没有数据。
  const long long age_ms = (steady_now_ns() - last_delivered_stamp_ns_) / 1000000LL;
  if (last_delivered_stamp_ns_ < 0 || age_ms > stale_timeout_ms_)
  {
    return false;
  }
  frame = last_delivered_frame_;
  return true;
}

long Kwr75SerialClient::last_frame_age_ms() const
{
  if (!latest_frame_.has_data.load(std::memory_order_acquire))
  {
    return -1;
  }
  const long long stamp = latest_frame_.stamp_ns.load(std::memory_order_relaxed);
  if (stamp <= 0)
  {
    return -1;
  }
  return static_cast<long>((steady_now_ns() - stamp) / 1000000LL);
}

std::string Kwr75SerialClient::reader_stop_reason() const
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  return reader_stop_reason_;
}

void Kwr75SerialClient::set_reader_stop_state(const std::string& reason)
{
  std::lock_guard<std::mutex> lock(io_mutex_);
  reader_stop_reason_ = reason;
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
  if (!acquire_latest_frame(frame, effective_timeout))
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

// 从后台线程的无锁缓存里取最新一帧（seqlock，读者永不阻塞）：
//   - timeout_ms == 0 → 实时环路径，立即返回；写者正在更新时复用上次快照
//   - timeout_ms  > 0 → warmup / 诊断路径，最多等到 deadline
// 单帧丢失不应该把力/力矩置 0（那会变成一次假的力阶跃，直接踢到导纳环）。
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
