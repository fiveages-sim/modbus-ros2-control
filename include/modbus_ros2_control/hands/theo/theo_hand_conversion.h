#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace modbus_ros2_control::theo_hand
{

static constexpr uint16_t kDefaultProtocolPositionMax = 9000;

enum class HandSide
{
  LEFT,
  RIGHT,
  UNKNOWN,
};

inline uint16_t radians_to_protocol_position(
  double position,
  double lower,
  double upper,
  uint16_t protocol_max = kDefaultProtocolPositionMax)
{
  if (upper <= lower)
  {
    return 0;
  }

  const double ratio = std::clamp((position - lower) / (upper - lower), 0.0, 1.0);
  return static_cast<uint16_t>(std::lround(ratio * static_cast<double>(protocol_max)));
}

inline double protocol_position_to_radians(
  int raw_position,
  double lower,
  double upper,
  uint16_t protocol_max = kDefaultProtocolPositionMax)
{
  if (upper <= lower)
  {
    return lower;
  }

  const double max_value = static_cast<double>(protocol_max);
  const double clamped = std::clamp(static_cast<double>(raw_position), 0.0, max_value);
  const double ratio = clamped / max_value;
  return lower + ratio * (upper - lower);
}

inline HandSide hand_side_from_register(uint16_t value)
{
  if (value == 0)
  {
    return HandSide::LEFT;
  }
  if (value == 1)
  {
    return HandSide::RIGHT;
  }
  return HandSide::UNKNOWN;
}

}  // namespace modbus_ros2_control::theo_hand
