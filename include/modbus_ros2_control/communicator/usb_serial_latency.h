#pragma once

#include <string>

namespace modbus_ros2_control
{

// Set supported USB serial adapters to 1 ms and verify the result.
// Resolves device symlinks. Unsupported adapters are skipped; permission or
// verification failures are logged and do not prevent serial communication.
void configure_usb_serial_latency(const std::string& serial_port);

}  // namespace modbus_ros2_control
