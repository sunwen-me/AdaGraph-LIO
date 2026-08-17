#ifndef HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_UPDATE_H_
#define HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_UPDATE_H_

#include <optional>

#include "hybrid_slam/localization/relocalizer.h"

namespace hybrid_slam::localization {

// Transitional event carrying the local pose that corresponds exactly to a
// relocalization measurement. SlamSystem uses the pair to derive T_map_odom;
// Relocalizer itself still returns only the global result.
struct RelocalizationUpdate {
  RelocalizationResult result;
  hybrid_slam::geometry::StampedTransform3d T_odom_base;

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] std::optional<RelocalizationUpdate>
BuildRelocalizationUpdateFromMapOdom(
    const hybrid_slam::geometry::StampedTransform3d& T_map_odom,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_imu,
    const hybrid_slam::geometry::StaticTransform3d& T_base_imu,
    double fitness,
    double inlier_ratio = 0.0,
    Confidence confidence = Confidence::kUnknown);

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_UPDATE_H_
