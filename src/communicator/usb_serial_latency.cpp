#include "modbus_ros2_control/communicator/usb_serial_latency.h"

#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

namespace modbus_ros2_control
{

void configure_usb_serial_latency(const std::string& serial_port)
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const auto resolved_port = fs::canonical(serial_port, ec);
  if (ec)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("UsbSerialLatency"),
      "Cannot resolve serial port '%s' while checking FTDI latency_timer: %s",
      serial_port.c_str(), ec.message().c_str());
    return;
  }

  const auto latency_path = fs::path("/sys/bus/usb-serial/devices") /
    resolved_port.filename() / "latency_timer";
  if (!fs::exists(latency_path, ec) || ec)
  {
    return;
  }

  int previous = -1;
  {
    std::ifstream input(latency_path);
    input >> previous;
  }
  if (previous == 1)
  {
    return;
  }

  {
    std::ofstream output(latency_path);
    if (!output || !(output << 1 << '\n' << std::flush))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("UsbSerialLatency"),
        "Unable to set '%s' to 1 ms; continuing with latency_timer=%d ms (check sysfs/udev permissions)",
        latency_path.c_str(), previous);
      return;
    }
  }

  int verified = -1;
  {
    std::ifstream input(latency_path);
    input >> verified;
  }
  if (verified == 1)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("UsbSerialLatency"),
      "FTDI latency_timer changed from %d ms to 1 ms for '%s'",
      previous, resolved_port.c_str());
  }
  else
  {
    RCLCPP_WARN(
      rclcpp::get_logger("UsbSerialLatency"),
      "FTDI latency_timer verification failed for '%s' (read back %d ms)",
      resolved_port.c_str(), verified);
  }
}

}  // namespace modbus_ros2_control
