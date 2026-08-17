#ifndef HYBRID_SLAM_MAPPING_KEYFRAME_H_
#define HYBRID_SLAM_MAPPING_KEYFRAME_H_

#include <cstddef>
#include <utility>

#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"

namespace hybrid_slam::mapping {

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
  KeyFrame(hybrid_slam::common::KeyFrameId id,
           hybrid_slam::common::TrajectoryId trajectory_id,
           hybrid_slam::common::SessionId session_id,
           hybrid_slam::geometry::StampedTransform3d T_odom_base,
           hybrid_slam::sensor::ImmutablePointCloud cloud_base,
           KeyFrameQuality quality) noexcept
      : id_(id),
        trajectory_id_(trajectory_id),
        session_id_(session_id),
        T_odom_base_(std::move(T_odom_base)),
        cloud_base_(std::move(cloud_base)),
        quality_(quality) {}

  [[nodiscard]] bool valid() const noexcept {
    return T_odom_base_.valid() &&
           T_odom_base_.target_frame == hybrid_slam::geometry::FrameId::kOdom &&
           T_odom_base_.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           cloud_base_.valid() && !cloud_base_.empty() &&
           cloud_base_.frame() == hybrid_slam::geometry::FrameId::kBase;
  }

  [[nodiscard]] hybrid_slam::common::KeyFrameId id() const noexcept {
    return id_;
  }
  [[nodiscard]] hybrid_slam::common::TrajectoryId trajectoryId() const noexcept {
    return trajectory_id_;
  }
  [[nodiscard]] hybrid_slam::common::SessionId sessionId() const noexcept {
    return session_id_;
  }
  [[nodiscard]] hybrid_slam::common::Timestamp timestamp() const noexcept {
    return T_odom_base_.timestamp;
  }
  [[nodiscard]] const hybrid_slam::geometry::StampedTransform3d& T_odom_base()
      const noexcept {
    return T_odom_base_;
  }
  [[nodiscard]] const hybrid_slam::sensor::ImmutablePointCloud& cloudBase()
      const noexcept {
    return cloud_base_;
  }
  [[nodiscard]] const KeyFrameQuality& quality() const noexcept {
    return quality_;
  }

 private:
  hybrid_slam::common::KeyFrameId id_;
  hybrid_slam::common::TrajectoryId trajectory_id_;
  hybrid_slam::common::SessionId session_id_;
  hybrid_slam::geometry::StampedTransform3d T_odom_base_;
  hybrid_slam::sensor::ImmutablePointCloud cloud_base_;
  KeyFrameQuality quality_;
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_KEYFRAME_H_
