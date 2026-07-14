#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace modbus_ros2_control
{

/** Low-level KWR75 RS485 client (28-byte float frame, default 115200 8N1). */
class Kwr75SerialClient
{
public:
  static constexpr std::size_t kAxisCount = 6;
  static constexpr std::size_t kFrameLength = 28;

  Kwr75SerialClient(
    std::string serial_port,
    int baudrate = 115200,
    uint8_t command_code = 0x48,
    bool convert_to_si = true,
    double gravity = 9.80665,
    int response_timeout_ms = 50,
    int read_timeout_ms = 3,
    int startup_delay_ms = 50,
    int warmup_attempts = 20);

  bool connect();
  void disconnect();
  bool is_connected() const { return serial_fd_ >= 0; }
  bool warmup();
  bool read_wrench(std::array<double, kAxisCount>& wrench_si, int timeout_ms = -1);
  /** Hex dump of recent serial bytes when warmup/read fails (for diagnostics). */
  std::string last_io_sample_hex() const { return last_io_sample_hex_; }
  const std::string& serial_port() const { return serial_port_; }

private:
  bool configure_serial();
  bool start_capture();
  bool send_start_command();
  bool read_latest_frame(std::array<uint8_t, kFrameLength>& frame, int timeout_ms);
  void record_io_sample(const std::vector<uint8_t>& buffer);
  static float decode_wire_float(const uint8_t* wire_bytes);
  static bool parse_frame(
    const std::array<uint8_t, kFrameLength>& frame,
    std::array<float, kAxisCount>& values,
    uint8_t command_code);

  std::string serial_port_;
  int baudrate_;
  uint8_t command_code_;
  bool convert_to_si_;
  double gravity_;
  int response_timeout_ms_;
  int read_timeout_ms_;
  int startup_delay_ms_;
  int warmup_attempts_;
  bool streaming_started_ = false;
  int serial_fd_ = -1;
  std::string last_io_sample_hex_;
};

}  // namespace modbus_ros2_control
