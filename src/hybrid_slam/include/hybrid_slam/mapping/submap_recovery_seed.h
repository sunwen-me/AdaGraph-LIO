#ifndef HYBRID_SLAM_MAPPING_SUBMAP_RECOVERY_SEED_H_
#define HYBRID_SLAM_MAPPING_SUBMAP_RECOVERY_SEED_H_

#include <optional>
#include <vector>

#include <Eigen/Core>

#include "hybrid_slam/mapping/map_snapshot.h"

namespace hybrid_slam::mapping {

// Materializes a finished submap seed in a new frontend odom frame. The
// immutable submap points are never rewritten: T_map_submap carries their
// graph-updated pose while this function creates the transient iVox payload.
[[nodiscard]] std::optional<std::vector<Eigen::Vector3f>>
BuildRecoverySeedInOdom(const SubmapSnapshot& submap,
                        const hybrid_slam::geometry::RigidTransform3d&
                            T_map_odom);

// Active submaps do not own a materialized submap cloud yet.  Build a
// transient seed directly from their committed keyframes in that case.
[[nodiscard]] std::optional<std::vector<Eigen::Vector3f>>
BuildRecoverySeedInOdom(const SubmapSnapshot& submap,
                        const std::vector<KeyFrameSnapshot>& keyframes,
                        const hybrid_slam::geometry::RigidTransform3d&
                            T_map_odom);

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_SUBMAP_RECOVERY_SEED_H_
