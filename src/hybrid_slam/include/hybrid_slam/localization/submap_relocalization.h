#ifndef HYBRID_SLAM_LOCALIZATION_SUBMAP_RELOCALIZATION_H_
#define HYBRID_SLAM_LOCALIZATION_SUBMAP_RELOCALIZATION_H_

#include <cmath>

#include "hybrid_slam/backend/constraint.h"

namespace hybrid_slam::localization {

// A geometrically verified result bound to the exact historical submap that
// supplied the registration target. Carrying this identity prevents a valid
// pose from accidentally being paired with a different local-map seed.
struct SubmapRelocalizationResult {
  hybrid_slam::geometry::StampedTransform3d T_map_base;
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::common::SubmapId submap_id;
  hybrid_slam::backend::Covariance6d covariance =
      hybrid_slam::backend::Covariance6d::Identity();
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  double overlap = 0.0;
  bool observable = false;

  [[nodiscard]] bool valid() const noexcept {
    return T_map_base.valid() &&
           T_map_base.target_frame == hybrid_slam::geometry::FrameId::kMap &&
           T_map_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           session_id.value() != 0 && trajectory_id.value() != 0 &&
           hybrid_slam::backend::IsValidCovariance6d(covariance) &&
           std::isfinite(fitness) && fitness >= 0.0 &&
           std::isfinite(inlier_ratio) && inlier_ratio >= 0.0 &&
           inlier_ratio <= 1.0 && std::isfinite(overlap) && overlap >= 0.0 &&
           overlap <= 1.0 && observable;
  }
};

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_SUBMAP_RELOCALIZATION_H_
