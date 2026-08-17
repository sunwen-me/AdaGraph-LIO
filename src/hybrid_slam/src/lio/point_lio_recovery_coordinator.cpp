#include "lio/point_lio_recovery_coordinator.h"

#include <algorithm>

#include "hybrid_slam/mapping/submap_recovery_seed.h"

namespace LI2Sup {

bool PointLIORecoveryCoordinator::Recover(
    const PointLIORecoveryRequest& request,
    const hybrid_slam::mapping::MapSnapshot& map,
    PointLIOFrontend& frontend,
    PointLIOPipeline& pipeline) {
  if (!request.valid() || !map.consistent() || frontend.inputEnabled()) {
    return false;
  }
  const auto found = std::find_if(
      map.submaps.begin(), map.submaps.end(),
      [&request](const hybrid_slam::mapping::SubmapSnapshot& submap) {
        return submap.id == request.relocalization.submap_id &&
               submap.session_id == request.relocalization.session_id &&
               submap.trajectory_id == request.relocalization.trajectory_id &&
               submap.valid();
      });
  if (found == map.submaps.end()) {
    return false;
  }

  const hybrid_slam::geometry::StampedTransform3d T_odom_base{
      request.relocalization.T_map_base.timestamp,
      hybrid_slam::geometry::FrameId::kOdom,
      hybrid_slam::geometry::FrameId::kBase,
      request.frontend_state.T_odom_base,
  };
  const auto T_map_odom = request.relocalization.T_map_base.compose(
      T_odom_base.inverse());
  if (!T_map_odom) {
    return false;
  }
  const auto seed = hybrid_slam::mapping::BuildRecoverySeedInOdom(
      *found, map.keyframes, T_map_odom->T_target_source);
  if (!seed) {
    return false;
  }

  RecoveredTrajectoryStart start;
  start.session_id = request.relocalization.session_id;
  start.trajectory_id = request.new_trajectory_id;
  start.T_map_base = request.relocalization.T_map_base;
  start.T_odom_base = T_odom_base;
  start.covariance = request.relocalization.covariance;
  start.score = request.relocalization.fitness;
  if (!pipeline.prepareRecoveredTrajectory(start)) {
    return false;
  }

  frontend.setLocalMapWriteEnabled(false);
  try {
    frontend.resetAndSeed(request.frontend_state, *seed);
  } catch (...) {
    // The pipeline has not changed trajectory, alignment, or graph state yet.
    // Discard only the prepared transition and let the caller decide whether
    // to retry or keep the old segment stopped.
    pipeline.abortPreparedRecoveredTrajectory();
    return false;
  }

  if (!pipeline.commitPreparedRecoveredTrajectory()) {
    pipeline.abortPreparedRecoveredTrajectory();
    return false;
  }
  return true;
}

}  // namespace LI2Sup
