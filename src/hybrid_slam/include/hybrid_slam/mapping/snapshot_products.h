#ifndef HYBRID_SLAM_MAPPING_SNAPSHOT_PRODUCTS_H_
#define HYBRID_SLAM_MAPPING_SNAPSHOT_PRODUCTS_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/mapping/map_snapshot.h"

namespace hybrid_slam::mapping {

struct TrajectoryPose {
  hybrid_slam::common::NodeId node_id;
  hybrid_slam::common::KeyFrameId keyframe_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::geometry::StampedTransform3d T_map_base;
};

// Lightweight adapter product for ROS paths, diagnostics and trajectory IO.
// It copies only fixed-size pose metadata and never retains or copies clouds.
struct SnapshotTrajectory {
  std::uint64_t snapshot_version = 0;
  std::vector<TrajectoryPose> poses;
};

struct MapPoint3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

// Materialized derived representation. Building it intentionally performs one
// allocation and one transform/copy per keyframe point; callers must schedule
// it outside the frontend/backend critical path and may discard stale output.
struct SnapshotPointCloudMap {
  std::uint64_t snapshot_version = 0;
  std::vector<MapPoint3f> points;
  Eigen::Vector3f minimum = Eigen::Vector3f::Zero();
  Eigen::Vector3f maximum = Eigen::Vector3f::Zero();
};

class SnapshotProductBuilder {
 public:
  [[nodiscard]] static std::optional<SnapshotTrajectory> BuildTrajectory(
      const MapSnapshot& snapshot);

  [[nodiscard]] static std::optional<SnapshotPointCloudMap> BuildPointCloudMap(
      const MapSnapshot& snapshot);

  // Materialize a chronological half-open keyframe range.  This is useful for
  // diagnosing whether two visits overlap after graph optimization without
  // losing provenance in the final global voxel filter.
  [[nodiscard]] static std::optional<SnapshotPointCloudMap> BuildPointCloudMap(
      const MapSnapshot& snapshot, std::size_t keyframe_begin,
      std::size_t keyframe_end);
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_SNAPSHOT_PRODUCTS_H_
