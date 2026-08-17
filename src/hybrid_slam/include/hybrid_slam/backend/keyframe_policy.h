#ifndef HYBRID_SLAM_BACKEND_KEYFRAME_POLICY_H_
#define HYBRID_SLAM_BACKEND_KEYFRAME_POLICY_H_

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>

#include "hybrid_slam/backend/backend_config.h"
#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::backend {

// Stateful keyframe admission policy. It is owned by the system/backend
// scheduler, not by a frontend or optimizer. Rigid transforms deliberately
// omit a frame label here so IMU-pose cadence can be preserved until
// the explicit base-frame adapter is introduced at the KeyFrame boundary.
class KeyFramePolicy {
 public:
  explicit KeyFramePolicy(const BackendConfig& config)
      : translation_threshold_m_(config.keyframe_translation_m),
        rotation_threshold_rad_(config.keyframe_rotation_rad),
        maximum_rejected_frames_(config.keyframe_max_frames) {
    if (const auto error = config.validate()) {
      throw std::invalid_argument("invalid keyframe policy config: " + *error);
    }
  }

  [[nodiscard]] bool shouldAccept(
      const hybrid_slam::geometry::RigidTransform3d& current_pose) const {
    const double translation =
        (current_pose.translation - last_accepted_pose_.translation).norm();
    const Eigen::Quaterniond rotation_delta =
        current_pose.rotation * last_accepted_pose_.rotation.conjugate();
    const double rotation =
        std::abs(Eigen::AngleAxisd(rotation_delta.normalized()).angle());
    return translation > translation_threshold_m_ ||
           rotation > rotation_threshold_rad_ ||
           rejected_frames_ >= maximum_rejected_frames_;
  }

  void accept(hybrid_slam::geometry::RigidTransform3d pose) noexcept {
    last_accepted_pose_ = std::move(pose);
    rejected_frames_ = 0;
  }

  void reject() noexcept { ++rejected_frames_; }

  [[nodiscard]] const hybrid_slam::geometry::RigidTransform3d& lastAcceptedPose()
      const noexcept {
    return last_accepted_pose_;
  }
  [[nodiscard]] std::size_t rejectedFrameCount() const noexcept {
    return rejected_frames_;
  }

 private:
  double translation_threshold_m_ = 0.5;
  double rotation_threshold_rad_ = 0.1;
  std::size_t maximum_rejected_frames_ = 30;
  hybrid_slam::geometry::RigidTransform3d last_accepted_pose_;
  std::size_t rejected_frames_ = 0;
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_KEYFRAME_POLICY_H_
