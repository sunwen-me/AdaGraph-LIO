#ifndef HYBRID_SLAM_MAPPING_MAP_SNAPSHOT_H_
#define HYBRID_SLAM_MAPPING_MAP_SNAPSHOT_H_

#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "hybrid_slam/backend/constraint.h"
#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/mapping/keyframe.h"

namespace hybrid_slam::mapping {

struct KeyFrameSnapshot {
  hybrid_slam::common::NodeId node_id;
  hybrid_slam::common::KeyFrameId keyframe_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::common::SubmapId submap_id;
  hybrid_slam::geometry::StampedTransform3d T_odom_base;
  hybrid_slam::geometry::StampedTransform3d T_map_base;
  hybrid_slam::sensor::ImmutablePointCloud cloud_base;
  KeyFrameQuality quality;

  [[nodiscard]] bool valid() const noexcept {
    return T_odom_base.valid() && T_map_base.valid() &&
           T_odom_base.timestamp == T_map_base.timestamp &&
           T_odom_base.target_frame == hybrid_slam::geometry::FrameId::kOdom &&
           T_odom_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           T_map_base.target_frame == hybrid_slam::geometry::FrameId::kMap &&
           T_map_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           cloud_base.valid() && !cloud_base.empty() &&
           cloud_base.frame() == hybrid_slam::geometry::FrameId::kBase;
  }
};

struct SubmapBounds {
  Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
  Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
  bool has_points = false;

  [[nodiscard]] bool valid() const noexcept {
    return !has_points ||
           (minimum.allFinite() && maximum.allFinite() &&
            (minimum.array() <= maximum.array()).all());
  }
};

enum class SubmapState {
  kActive = 0,
  kFinished,
};

struct SubmapSnapshot {
  hybrid_slam::common::SubmapId id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::common::SessionId session_id;
  SubmapState state = SubmapState::kActive;
  std::vector<hybrid_slam::common::KeyFrameId> keyframe_ids;

  // Geometry is materialized when a submap becomes finished; active submaps
  // intentionally keep only membership metadata. anchor_keyframe_id remains
  // the primary retrieval representative even when rolling geometry also
  // covers the preceding primary submap.
  hybrid_slam::common::KeyFrameId anchor_keyframe_id;
  hybrid_slam::geometry::RigidTransform3d T_map_submap =
      hybrid_slam::geometry::RigidTransform3d::Identity();
  hybrid_slam::sensor::ImmutablePointCloud cloud_submap;
  hybrid_slam::sensor::ImmutablePointCloud seed_cloud_submap;
  SubmapBounds bounds;
  std::vector<float> descriptor;
  bool geometry_ready = false;
  // Keep every extension after geometry_ready. Several integration fixtures
  // intentionally aggregate-initialize the original field sequence ending in
  // `{descriptor, true}`; inserting fields before it silently changed the
  // meaning of valid legacy snapshots.
  std::size_t geometry_keyframe_count = 0;
  std::size_t geometry_input_point_count = 0;
  double geometry_voxel_resolution_m = 0.0;
  // anchor_keyframe_id remains the retrieval/primary-membership identity.
  // Rolling geometry can start in the preceding primary submap, in which case
  // this separate anchor defines the immutable cloud_submap coordinate frame.
  std::optional<hybrid_slam::common::KeyFrameId> geometry_anchor_keyframe_id;

  [[nodiscard]] bool valid() const noexcept {
    if (!bounds.valid()) {
      return false;
    }
    if (!geometry_ready) {
      return true;
    }
    return T_map_submap.isFinite() && cloud_submap.valid() &&
           seed_cloud_submap.valid() && !cloud_submap.empty() &&
           !seed_cloud_submap.empty() &&
           cloud_submap.frame() == hybrid_slam::geometry::FrameId::kSubmap &&
           seed_cloud_submap.frame() ==
               hybrid_slam::geometry::FrameId::kSubmap &&
           !descriptor.empty();
  }
};

struct TrajectorySnapshot {
  hybrid_slam::common::TrajectoryId id;
  hybrid_slam::common::SessionId session_id;
  std::vector<hybrid_slam::common::SubmapId> submap_ids;
};

// Immutable, internally consistent committed map view. Cloud payload copies
// increment one shared owner count; point storage is never deep-copied.
struct MapSnapshot {
  std::uint64_t version = 0;
  std::vector<TrajectorySnapshot> trajectories;
  std::vector<SubmapSnapshot> submaps;
  std::vector<KeyFrameSnapshot> keyframes;
  std::vector<hybrid_slam::backend::Constraint> constraints;

  [[nodiscard]] bool valid() const noexcept {
    for (const auto& submap : submaps) {
      if (!submap.valid()) {
        return false;
      }
    }
    for (const auto& keyframe : keyframes) {
      if (!keyframe.valid()) {
        return false;
      }
    }
    return true;
  }

  // O(metadata) structural audit intended for tests, IO and diagnostics, not
  // for the per-keyframe publish path.
  [[nodiscard]] bool consistent() const;
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_MAP_SNAPSHOT_H_
