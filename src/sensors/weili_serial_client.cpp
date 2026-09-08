#include "modbus_ros2_control/sensors/weili_serial_client.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>

#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace modbus_ros2_control
{
namespace
{
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
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
    default:
      return 0;
  }
}
}  // namespace

WeiliSerialClient::WeiliSerialClient(
  std::string serial_port,
  int baudrate,
  uint8_t slave_id,
  uint16_t start_value,
  std::size_t data_offset,
  std::size_t frame_length,
  int zero_timeout_ms,
  int read_timeout_ms,
  int startup_delay_ms,
  int warmup_attempts)
: serial_port_(std::move(serial_port))
, baudrate_(baudrate)
, slave_id_(slave_id)
, start_value_(start_value)
, data_offset_(data_offset)
, frame_length_(frame_length)
, zero_timeout_ms_(zero_timeout_ms)
, read_timeout_ms_(read_timeout_ms)
, startup_delay_ms_(startup_delay_ms)
, warmup_attempts_(warmup_attempts)
{
  rx_buffer_.reserve(frame_length_ * 8);
}

bool WeiliSerialClient::connect()
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

  tcflush(serial_fd_, TCIOFLUSH);
  streaming_started_ = false;
  rx_buffer_.clear();
  has_valid_sample_ = false;
  has_error_report_.store(false);
  last_error_code_ = 0;
  return true;
}

void WeiliSerialClient::disconnect()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
  streaming_started_ = false;
  rx_buffer_.clear();
}

bool WeiliSerialClient::configure_serial()
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
  options.c_cflag &= ~PARENB;   // 无校验
  options.c_cflag &= ~CSTOPB;   // 1 停止位
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;       // 8 数据位
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  return tcsetattr(serial_fd_, TCSANOW, &options) == 0;
}

bool WeiliSerialClient::start_streaming()
{
  if (!is_connected())
  {
    return false;
  }
  if (streaming_started_)
  {
    return true;
  }

  // 只发一次启动指令（与 KWR75 / 天机 485 一致：只发帧，不等待 Modbus ACK）
  tcflush(serial_fd_, TCIOFLUSH);
  rx_buffer_.clear();
  has_valid_sample_ = false;
  has_error_report_.store(false);
  last_error_code_ = 0;

  if (!send_command(start_value_))
  {
    return false;
  }

  streaming_started_ = true;
  return true;
}

bool WeiliSerialClient::stop_streaming()
{
  const bool ok = send_command(kStopValue);
  streaming_started_ = false;
  return ok;
}

bool WeiliSerialClient::send_command(uint16_t register_value)
{
  // Modbus RTU：站号 9, FC 0x10, 地址 0x019A, 1 个寄存器, 2 字节数据（大端）
  // 例：250 Hz 启动 = 09 10 01 9A 00 01 02 03 01 0D 9A
  uint8_t request[11] = {};
  request[0] = slave_id_;
  request[1] = kFunctionCode;
  request[2] = static_cast<uint8_t>((kControlRegister >> 8) & 0xFF);
  request[3] = static_cast<uint8_t>(kControlRegister & 0xFF);
  request[4] = 0x00;  // 寄存器数量 0x0001
  request[5] = 0x01;
  request[6] = 0x02;  // 字节数
  request[7] = static_cast<uint8_t>((register_value >> 8) & 0xFF);
  request[8] = static_cast<uint8_t>(register_value & 0xFF);

  const uint16_t crc = crc16_modbus(request, 9);
  request[9] = static_cast<uint8_t>(crc & 0xFF);          // CRC_L
  request[10] = static_cast<uint8_t>((crc >> 8) & 0xFF);  // CRC_H

  std::size_t total_written = 0;
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(20);

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

bool WeiliSerialClient::request_zero(uint8_t & status_out)
{
  status_out = 0xFF;
  if (!is_connected())
  {
    return false;
  }

  // 发送回零指令
  if (!send_command(kZeroValue))
  {
    return false;
  }

  // 丢弃缓冲中的旧数据流帧，避免干扰回零结果判断
  rx_buffer_.clear();

  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(zero_timeout_ms_);

  while (std::chrono::steady_clock::now() <= deadline)
  {
    // 从缓冲头部逐帧消费；丢弃力值帧 / 错误帧，命中“回零状态帧(功能位 0x00)”即返回。
    while (!rx_buffer_.empty())
    {
      if (rx_buffer_.front() != kFrameHeader1)
      {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() < 2 || rx_buffer_[1] != kFrameHeader2)
      {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() >= 2 && rx_buffer_[1] == kFrameHeader2)
      {
        // 完整力值帧？丢弃
        std::array<float, kAxisCount> ignored {};
        if (rx_buffer_.size() >= frame_length_ && try_parse_force_frame(0, ignored))
        {
          rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_length_);
          continue;
        }
        // 完整短帧？
        if (rx_buffer_.size() >= kShortFrameLength)
        {
          uint8_t tag = 0;
          uint8_t code = 0;
          if (try_parse_short_frame(0, tag, code))
          {
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kShortFrameLength);
            if (tag == kZeroStatusTag)
            {
              status_out = code;
              return code == 0;  // 0=回零正常，非 0=回零失败
            }
            if (tag == kErrorTag)
            {
              has_error_report_.store(true);
              last_error_code_ = code;
            }
            continue;
          }
        }
        // 帧头已到但帧不完整 → 等待更多字节
        break;
      }
      break;
    }

    // 安全上限：防止长时间无有效帧时缓冲无限增长
    const std::size_t max_buf = frame_length_ * 8;
    if (rx_buffer_.size() > max_buf)
    {
      rx_buffer_.erase(
        rx_buffer_.begin(), rx_buffer_.end() - static_cast<std::ptrdiff_t>(frame_length_ - 1));
    }

    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
    if (remaining <= std::chrono::milliseconds(0))
    {
      break;
    }
    if (!read_into_buffer(remaining))
    {
      break;
    }
  }

  // 超时未收到回零结果
  if (rx_buffer_.empty())
  {
    last_io_sample_hex_.clear();
  }
  else
  {
    record_io_sample(rx_buffer_);
  }
  return false;
}

bool WeiliSerialClient::read_wrench(
  std::array<double, kAxisCount> & out,
  int wait_ms)
{
  if (!is_connected())
  {
    return false;
  }
  if (!streaming_started_ && !start_streaming())
  {
    return false;
  }

  const auto wait = std::chrono::milliseconds(wait_ms > 0 ? wait_ms : 0);
  const auto deadline = std::chrono::steady_clock::now() + wait;

  bool got_new = false;
  int drain_passes = 0;
  while (true)
  {
    got_new = consume_latest_force_frame();
    if (got_new)
    {
      break;
    }

    if (wait_ms <= 0)
    {
      // 非阻塞：做一次零超时 drain 后即返回（采样保持，不占用控制周期）
      if (drain_passes++ == 0)
      {
        read_into_buffer(std::chrono::milliseconds(0));
        got_new = consume_latest_force_frame();
      }
      break;
    }

    if (std::chrono::steady_clock::now() >= deadline)
    {
      break;
    }
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
      deadline - std::chrono::steady_clock::now());
    if (!read_into_buffer(remaining))
    {
      break;
    }
  }

  out = has_valid_sample_ ? latest_wrench_ : std::array<double, kAxisCount>{};
  return got_new;
}

bool WeiliSerialClient::read_into_buffer(std::chrono::milliseconds wait)
{
  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(serial_fd_, &read_set);

  timeval timeout {};
  timeout.tv_sec = wait.count() / 1000;
  timeout.tv_usec = (wait.count() % 1000) * 1000;

  const auto select_result = select(serial_fd_ + 1, &read_set, nullptr, nullptr, &timeout);
  if (select_result < 0)
  {
    return errno == EINTR;
  }
  if (select_result == 0)
  {
    return true;  // 超时无数据，不算错误
  }

  uint8_t chunk[256];
  const auto bytes_read = ::read(serial_fd_, chunk, sizeof(chunk));
  if (bytes_read < 0)
  {
    return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
  }
  if (bytes_read == 0)
  {
    return true;
  }

  rx_buffer_.insert(rx_buffer_.end(), chunk, chunk + bytes_read);

  // 限制缓冲大小（保留帧尾，便于不完整帧跨调用补齐）
  const std::size_t max_buf = frame_length_ * 8;
  if (rx_buffer_.size() > max_buf)
  {
    rx_buffer_.erase(
      rx_buffer_.begin(), rx_buffer_.end() - static_cast<std::ptrdiff_t>(frame_length_ - 1));
  }
  return true;
}

bool WeiliSerialClient::consume_latest_force_frame()
{
  // 丢弃不可能成帧的前导字节
  while (!rx_buffer_.empty() && rx_buffer_.front() != kFrameHeader1)
  {
    rx_buffer_.erase(rx_buffer_.begin());
  }
  if (rx_buffer_.size() < 2 || rx_buffer_[1] != kFrameHeader2)
  {
    if (!rx_buffer_.empty())
    {
      rx_buffer_.erase(rx_buffer_.begin());
    }
    return false;
  }

  bool found = false;
  while (!rx_buffer_.empty())
  {
    if (rx_buffer_.front() != kFrameHeader1 || rx_buffer_.size() < 2 ||
      rx_buffer_[1] != kFrameHeader2)
    {
      break;  // 交给下方“清理前导”处理
    }

    // 完整力值帧
    if (rx_buffer_.size() >= frame_length_ && try_parse_force_frame(0, latest_raw_))
    {
      for (std::size_t axis = 0; axis < kAxisCount; ++axis)
      {
        latest_wrench_[axis] = static_cast<double>(latest_raw_[axis]);
      }
      last_data_time_ = std::chrono::steady_clock::now();
      has_valid_sample_ = true;
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + frame_length_);
      found = true;
      continue;
    }

    // 短帧：错误状态(功能位 0x01) / 回零状态(功能位 0x00)
    if (rx_buffer_.size() >= kShortFrameLength)
    {
      uint8_t tag = 0;
      uint8_t code = 0;
      if (try_parse_short_frame(0, tag, code))
      {
        if (tag == kErrorTag)
        {
          has_error_report_.store(true);
          last_error_code_ = code;
        }
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kShortFrameLength);
        continue;
      }
    }

    // 帧头已到但数据不完整
    break;
  }

  // 清理前导垃圾（保留可能的帧头）
  while (!rx_buffer_.empty() && rx_buffer_.front() != kFrameHeader1)
  {
    rx_buffer_.erase(rx_buffer_.begin());
  }
  if (!rx_buffer_.empty() && rx_buffer_.size() >= 2 && rx_buffer_[1] != kFrameHeader2)
  {
    rx_buffer_.erase(rx_buffer_.begin());
  }
  return found;
}

bool WeiliSerialClient::try_parse_force_frame(
  std::size_t start,
  std::array<float, kAxisCount> & values) const
{
  if (start + frame_length_ > rx_buffer_.size())
  {
    return false;
  }
  if (data_offset_ + kMaxDataBytes + kCrcBytes > frame_length_)
  {
    return false;
  }

  const uint8_t * f = rx_buffer_.data() + start;
  if (f[0] != kFrameHeader1 || f[1] != kFrameHeader2)
  {
    return false;
  }

  // CRC16/Modbus：覆盖除最后 2 字节外的全部帧数据，末两位为 CRC_L、CRC_H
  const uint16_t calc = crc16_modbus(f, frame_length_ - kCrcBytes);
  const uint16_t stored =
    (static_cast<uint16_t>(f[frame_length_ - 1]) << 8) | f[frame_length_ - 2];
  if (calc != stored)
  {
    return false;
  }

  for (std::size_t axis = 0; axis < kAxisCount; ++axis)
  {
    values[axis] = decode_wire_float(f + data_offset_ + axis * 4);
  }
  return true;
}

bool WeiliSerialClient::try_parse_short_frame(
  std::size_t start,
  uint8_t & tag,
  uint8_t & code) const
{
  if (start + kShortFrameLength > rx_buffer_.size())
  {
    return false;
  }

  const uint8_t * f = rx_buffer_.data() + start;
  if (f[0] != kFrameHeader1 || f[1] != kFrameHeader2)
  {
    return false;
  }

  tag = f[2];
  code = f[3];
  if (tag != kZeroStatusTag && tag != kErrorTag)
  {
    return false;
  }

  const uint16_t calc = crc16_modbus(f, 4);
  const uint16_t stored = (static_cast<uint16_t>(f[5]) << 8) | f[4];
  return calc == stored;
}

uint16_t WeiliSerialClient::crc16_modbus(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
    {
      if (crc & 0x0001)
      {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      }
      else
      {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }
  return crc;
}

float WeiliSerialClient::decode_wire_float(const uint8_t * wire_bytes)
{
  // 小端 4 字节 → float32
  const uint32_t raw =
    static_cast<uint32_t>(wire_bytes[0]) | (static_cast<uint32_t>(wire_bytes[1]) << 8) |
    (static_cast<uint32_t>(wire_bytes[2]) << 16) | (static_cast<uint32_t>(wire_bytes[3]) << 24);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

void WeiliSerialClient::record_io_sample(const std::vector<uint8_t> & buffer)
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
  last_io_sample_hex_ = oss.str();
}

long long WeiliSerialClient::millis_since_last_data() const
{
  if (!has_valid_sample_)
  {
    return std::numeric_limits<long long>::max();
  }
  const auto elapsed = std::chrono::steady_clock::now() - last_data_time_;
  return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
}

}  // namespace modbus_ros2_control
