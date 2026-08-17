#include "ros/livox_point_time.h"

#include <algorithm>
#include <cmath>

namespace LI2Sup {
namespace {

constexpr double kMaxPointOffsetNanoseconds = 1e9;
constexpr double kMinPointOffsetSeconds = -1e-3;
constexpr double kMaxPointOffsetSeconds = 1.0;

}  // namespace

std::optional<double> decodeLivoxPointOffsetSeconds(
    double point_timestamp_ns,
    double frame_start_ns) {
  if (!std::isfinite(point_timestamp_ns) || !std::isfinite(frame_start_ns)) {
    return std::nullopt;
  }

  // Driver2 1.2.x writes StoragePacket::offset_time directly. Values larger
  // than one scan are absolute timestamps, including low simulation clocks.
  double offset_seconds =
      std::abs(point_timestamp_ns) <= kMaxPointOffsetNanoseconds
          ? point_timestamp_ns * 1e-9
          : (point_timestamp_ns - frame_start_ns) * 1e-9;

  if (!std::isfinite(offset_seconds) ||
      offset_seconds < kMinPointOffsetSeconds ||
      offset_seconds > kMaxPointOffsetSeconds) {
    return std::nullopt;
  }
  return std::max(0.0, offset_seconds);
}

}  // namespace LI2Sup
