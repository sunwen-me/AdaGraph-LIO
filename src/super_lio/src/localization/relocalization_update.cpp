#include "adagraph/localization/relocalization_update.h"

#include <cmath>

namespace adagraph::localization {

bool RelocalizationUpdate::valid() const noexcept {
  return result.success() && result.T_map_base &&
         result.T_map_base->valid() && T_odom_base.valid() &&
         std::isfinite(result.fitness) && result.fitness >= 0.0 &&
         std::isfinite(result.inlier_ratio) &&
         result.inlier_ratio >= 0.0 && result.inlier_ratio <= 1.0 &&
         result.T_map_base->timestamp == T_odom_base.timestamp &&
         result.T_map_base->target_frame ==
             adagraph::geometry::FrameId::kMap &&
         result.T_map_base->source_frame ==
             adagraph::geometry::FrameId::kBase &&
         T_odom_base.target_frame == adagraph::geometry::FrameId::kOdom &&
         T_odom_base.source_frame == adagraph::geometry::FrameId::kBase;
}

std::optional<RelocalizationUpdate> BuildRelocalizationUpdateFromMapOdom(
    const adagraph::geometry::StampedTransform3d& T_map_odom,
    const adagraph::geometry::StampedTransform3d& T_odom_imu,
    const adagraph::geometry::StaticTransform3d& T_base_imu,
    double fitness,
    double inlier_ratio,
    Confidence confidence) {
  if (!T_map_odom.valid() || !T_odom_imu.valid() ||
      !T_base_imu.valid() || !std::isfinite(fitness) || fitness < 0.0 ||
      !std::isfinite(inlier_ratio) || inlier_ratio < 0.0 ||
      inlier_ratio > 1.0 ||
      T_map_odom.timestamp != T_odom_imu.timestamp ||
      T_map_odom.target_frame != adagraph::geometry::FrameId::kMap ||
      T_map_odom.source_frame != adagraph::geometry::FrameId::kOdom ||
      T_odom_imu.target_frame != adagraph::geometry::FrameId::kOdom ||
      T_odom_imu.source_frame != adagraph::geometry::FrameId::kImu ||
      T_base_imu.target_frame != adagraph::geometry::FrameId::kBase ||
      T_base_imu.source_frame != adagraph::geometry::FrameId::kImu) {
    return std::nullopt;
  }
  const auto T_map_imu = T_map_odom.compose(T_odom_imu);
  const auto T_odom_base = T_odom_imu.compose(T_base_imu.inverse());
  const auto T_map_base =
      T_map_imu ? T_map_imu->compose(T_base_imu.inverse()) : std::nullopt;
  if (!T_odom_base || !T_map_base) {
    return std::nullopt;
  }

  RelocalizationUpdate update;
  update.result.status = RelocalizationStatus::kSuccess;
  update.result.T_map_base = *T_map_base;
  update.result.fitness = fitness;
  update.result.inlier_ratio = inlier_ratio;
  update.result.confidence = confidence;
  update.T_odom_base = *T_odom_base;
  return update;
}

}  // namespace adagraph::localization
