#ifndef ADAGRAPH_MAPPING_KEYFRAME_H_
#define ADAGRAPH_MAPPING_KEYFRAME_H_

#include <cstddef>
#include <utility>

#include "adagraph/common/domain_ids.h"
#include "adagraph/geometry/transform3.h"
#include "adagraph/sensor/immutable_point_cloud.h"

namespace adagraph::mapping {

struct KeyFrameQuality {
  std::size_t input_point_count = 0;
  std::size_t matching_point_count = 0;
  std::size_t effective_constraint_count = 0;
  double inlier_ratio = 0.0;
  double condition_number = 1.0;
};

// Immutable-after-construction durable input to backend/map components.
// T_odom_base and cloud_base must describe the same timestamp/base frame.
class KeyFrame {
 public:
  KeyFrame(adagraph::common::KeyFrameId id,
           adagraph::common::TrajectoryId trajectory_id,
           adagraph::common::SessionId session_id,
           adagraph::geometry::StampedTransform3d T_odom_base,
           adagraph::sensor::ImmutablePointCloud cloud_base,
           KeyFrameQuality quality) noexcept
      : id_(id),
        trajectory_id_(trajectory_id),
        session_id_(session_id),
        T_odom_base_(std::move(T_odom_base)),
        cloud_base_(std::move(cloud_base)),
        quality_(quality) {}

  [[nodiscard]] bool valid() const noexcept {
    return T_odom_base_.valid() &&
           T_odom_base_.target_frame == adagraph::geometry::FrameId::kOdom &&
           T_odom_base_.source_frame == adagraph::geometry::FrameId::kBase &&
           cloud_base_.valid() && !cloud_base_.empty() &&
           cloud_base_.frame() == adagraph::geometry::FrameId::kBase;
  }

  [[nodiscard]] adagraph::common::KeyFrameId id() const noexcept {
    return id_;
  }
  [[nodiscard]] adagraph::common::TrajectoryId trajectoryId() const noexcept {
    return trajectory_id_;
  }
  [[nodiscard]] adagraph::common::SessionId sessionId() const noexcept {
    return session_id_;
  }
  [[nodiscard]] adagraph::common::Timestamp timestamp() const noexcept {
    return T_odom_base_.timestamp;
  }
  [[nodiscard]] const adagraph::geometry::StampedTransform3d& T_odom_base()
      const noexcept {
    return T_odom_base_;
  }
  [[nodiscard]] const adagraph::sensor::ImmutablePointCloud& cloudBase()
      const noexcept {
    return cloud_base_;
  }
  [[nodiscard]] const KeyFrameQuality& quality() const noexcept {
    return quality_;
  }

 private:
  adagraph::common::KeyFrameId id_;
  adagraph::common::TrajectoryId trajectory_id_;
  adagraph::common::SessionId session_id_;
  adagraph::geometry::StampedTransform3d T_odom_base_;
  adagraph::sensor::ImmutablePointCloud cloud_base_;
  KeyFrameQuality quality_;
};

}  // namespace adagraph::mapping

#endif  // ADAGRAPH_MAPPING_KEYFRAME_H_
