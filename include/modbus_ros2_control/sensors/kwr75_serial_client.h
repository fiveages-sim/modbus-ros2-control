#pragma once

#include <array>
#include <cstdint>
#include <string>

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
    uint8_t command_code = 0x49,
    bool convert_to_si = true,
    double gravity = 9.80665,
    int response_timeout_ms = 10);

  bool connect();
  void disconnect();
  bool is_connected() const { return serial_fd_ >= 0; }
  bool warmup();
  bool read_wrench(std::array<double, kAxisCount>& wrench_si);
  const std::string& serial_port() const { return serial_port_; }

private:
  bool configure_serial();
  bool send_request();
  bool read_response_frame(std::array<uint8_t, kFrameLength>& frame, int timeout_ms);
  static float decode_wire_float(const uint8_t* wire_bytes);
  static bool parse_frame(
    const std::array<uint8_t, kFrameLength>& frame,
    std::array<float, kAxisCount>& values);

  std::string serial_port_;
  int baudrate_;
  uint8_t command_code_;
  bool convert_to_si_;
  double gravity_;
  int response_timeout_ms_;
  int serial_fd_ = -1;
};

}  // namespace modbus_ros2_control
