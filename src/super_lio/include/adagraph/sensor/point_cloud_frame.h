#ifndef ADAGRAPH_SENSOR_POINT_CLOUD_FRAME_H_
#define ADAGRAPH_SENSOR_POINT_CLOUD_FRAME_H_

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

#include "adagraph/common/timestamp.h"
#include "adagraph/geometry/transform3.h"

namespace adagraph::sensor {

struct PointXYZIT {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
  // Signed because some sensors (notably Velodyne bags) timestamp a scan
  // relative to its end and therefore expose negative per-point offsets.
  std::int64_t offset_nanoseconds = 0;
};

using PointCloudBuffer = std::vector<PointXYZIT>;

// Mutable construction has one owner. Moving this frame transfers the vector
// allocation in O(1); copying is deliberately forbidden. Same-call consumers
// borrow points() and Phase 3 async fan-out will explicitly freeze/share a
// cloud buffer.
class PointCloudFrame {
 public:
  PointCloudFrame() = default;

  PointCloudFrame(adagraph::common::Timestamp start_time,
                  adagraph::common::Timestamp end_time,
                  adagraph::geometry::FrameId frame,
                  PointCloudBuffer points) noexcept
      : start_time_(start_time),
        end_time_(end_time),
        frame_(frame),
        points_(std::move(points)) {}

  PointCloudFrame(const PointCloudFrame&) = delete;
  PointCloudFrame& operator=(const PointCloudFrame&) = delete;
  PointCloudFrame(PointCloudFrame&&) noexcept = default;
  PointCloudFrame& operator=(PointCloudFrame&&) noexcept = default;

  [[nodiscard]] adagraph::common::Timestamp startTime() const noexcept {
    return start_time_;
  }
  [[nodiscard]] adagraph::common::Timestamp endTime() const noexcept {
    return end_time_;
  }
  [[nodiscard]] adagraph::geometry::FrameId frame() const noexcept {
    return frame_;
  }
  [[nodiscard]] std::span<const PointXYZIT> points() const noexcept {
    return points_;
  }
  [[nodiscard]] bool valid() const noexcept {
    return frame_ != adagraph::geometry::FrameId::kUnknown &&
           end_time_ >= start_time_;
  }

  [[nodiscard]] PointCloudBuffer releasePoints() && noexcept {
    return std::move(points_);
  }

 private:
  adagraph::common::Timestamp start_time_;
  adagraph::common::Timestamp end_time_;
  adagraph::geometry::FrameId frame_ =
      adagraph::geometry::FrameId::kUnknown;
  PointCloudBuffer points_;
};

}  // namespace adagraph::sensor

#endif  // ADAGRAPH_SENSOR_POINT_CLOUD_FRAME_H_
