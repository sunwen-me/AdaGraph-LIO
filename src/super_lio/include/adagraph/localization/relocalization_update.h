#ifndef ADAGRAPH_LOCALIZATION_RELOCALIZATION_UPDATE_H_
#define ADAGRAPH_LOCALIZATION_RELOCALIZATION_UPDATE_H_

#include <optional>

#include "adagraph/localization/relocalizer.h"

namespace adagraph::localization {

// Transitional event carrying the local pose that corresponds exactly to a
// relocalization measurement. SlamSystem uses the pair to derive T_map_odom;
// Relocalizer itself still returns only the global result.
struct RelocalizationUpdate {
  RelocalizationResult result;
  adagraph::geometry::StampedTransform3d T_odom_base;

  [[nodiscard]] bool valid() const noexcept;
};

[[nodiscard]] std::optional<RelocalizationUpdate>
BuildRelocalizationUpdateFromMapOdom(
    const adagraph::geometry::StampedTransform3d& T_map_odom,
    const adagraph::geometry::StampedTransform3d& T_odom_imu,
    const adagraph::geometry::StaticTransform3d& T_base_imu,
    double fitness,
    double inlier_ratio = 0.0,
    Confidence confidence = Confidence::kUnknown);

}  // namespace adagraph::localization

#endif  // ADAGRAPH_LOCALIZATION_RELOCALIZATION_UPDATE_H_
