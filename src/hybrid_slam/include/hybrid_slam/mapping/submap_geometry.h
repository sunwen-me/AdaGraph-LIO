#ifndef HYBRID_SLAM_MAPPING_SUBMAP_GEOMETRY_H_
#define HYBRID_SLAM_MAPPING_SUBMAP_GEOMETRY_H_

#include <cstddef>
#include <optional>

#include "hybrid_slam/mapping/map_snapshot.h"

namespace hybrid_slam::mapping {

struct SubmapGeometryBuildConfig {
  double voxel_resolution_m = 0.20;
  std::size_t descriptor_radial_bins = 32;
  std::size_t max_points = 250'000;

  [[nodiscard]] bool valid() const noexcept;
};

struct SubmapGeometryBuildResult {
  hybrid_slam::geometry::RigidTransform3d T_map_submap =
      hybrid_slam::geometry::RigidTransform3d::Identity();
  hybrid_slam::sensor::ImmutablePointCloud cloud_submap;
  // The initial implementation uses the same immutable downsampled storage
  // for both registration and seed use.  Separate LODs can be added without
  // changing the snapshot contract.
  hybrid_slam::sensor::ImmutablePointCloud seed_cloud_submap;
  SubmapBounds bounds;
  std::vector<float> descriptor;
  std::size_t input_point_count = 0;
  std::size_t keyframe_count = 0;
  double voxel_resolution_m = 0.0;

  [[nodiscard]] bool valid() const noexcept {
    return T_map_submap.isFinite() && cloud_submap.valid() &&
           seed_cloud_submap.valid() && !cloud_submap.empty() &&
           !seed_cloud_submap.empty() &&
           cloud_submap.frame() == hybrid_slam::geometry::FrameId::kSubmap &&
           seed_cloud_submap.frame() ==
               hybrid_slam::geometry::FrameId::kSubmap &&
           bounds.valid() && !descriptor.empty() && input_point_count > 0 &&
           keyframe_count > 0 && std::isfinite(voxel_resolution_m) &&
           voxel_resolution_m > 0.0;
  }
};

// Builds a local, immutable submap from the member keyframes. The submap
// frame is the base frame of anchor_keyframe_id, so graph optimization can
// update T_map_submap without rewriting the stored points.
[[nodiscard]] std::optional<SubmapGeometryBuildResult>
BuildSubmapGeometry(
    const std::vector<KeyFrameSnapshot>& keyframes,
    const SubmapSnapshot& submap,
    SubmapGeometryBuildConfig config = {});

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_SUBMAP_GEOMETRY_H_
