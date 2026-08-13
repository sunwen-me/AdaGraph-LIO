#ifndef ADAGRAPH_MAPPING_SNAPSHOT_PRODUCTS_H_
#define ADAGRAPH_MAPPING_SNAPSHOT_PRODUCTS_H_

#include <cstdint>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "adagraph/common/domain_ids.h"
#include "adagraph/geometry/transform3.h"
#include "adagraph/mapping/map_snapshot.h"

namespace adagraph::mapping {

struct TrajectoryPose {
  adagraph::common::NodeId node_id;
  adagraph::common::KeyFrameId keyframe_id;
  adagraph::common::TrajectoryId trajectory_id;
  adagraph::common::SessionId session_id;
  adagraph::geometry::StampedTransform3d T_map_base;
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
};

}  // namespace adagraph::mapping

#endif  // ADAGRAPH_MAPPING_SNAPSHOT_PRODUCTS_H_
