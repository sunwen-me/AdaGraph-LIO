#ifndef LI2SUP_LIVOX_POINT_TIME_H_
#define LI2SUP_LIVOX_POINT_TIME_H_

#include <optional>

namespace LI2Sup {

// Decodes Livox PointXYZRTLT's timestamp field to seconds from the beginning
// of the current scan. Livox Driver2 stores an offset in nanoseconds, while
// some converted datasets store an absolute nanosecond timestamp.
std::optional<double> decodeLivoxPointOffsetSeconds(
    double point_timestamp_ns,
    double frame_start_ns);

}  // namespace LI2Sup

#endif
