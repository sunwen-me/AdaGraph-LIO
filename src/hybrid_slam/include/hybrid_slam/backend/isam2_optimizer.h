#ifndef HYBRID_SLAM_BACKEND_ISAM2_OPTIMIZER_H_
#define HYBRID_SLAM_BACKEND_ISAM2_OPTIMIZER_H_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "hybrid_slam/backend/backend_config.h"
#include "hybrid_slam/backend/optimizer.h"

namespace hybrid_slam::backend {

struct Isam2OptimizerStats {
  std::size_t node_count = 0;
  std::size_t factor_count = 0;
  std::size_t update_count = 0;
  bool healthy = true;
};

// GTSAM implementation detail behind the optimizer boundary. It owns solver
// state but no PoseGraph, point cloud, ROS publisher or registration logic.
class Isam2Optimizer final : public Optimizer {
 public:
  explicit Isam2Optimizer(BackendConfig config);

  [[nodiscard]] OptimizerOutput apply(const GraphUpdate& update) override;
  [[nodiscard]] Isam2OptimizerStats stats() const noexcept;

 private:
  struct TrajectoryKey {
    hybrid_slam::common::SessionId session_id;
    hybrid_slam::common::TrajectoryId trajectory_id;
    bool operator==(const TrajectoryKey&) const = default;
  };

  struct TrajectoryKeyHash {
    [[nodiscard]] std::size_t operator()(
        const TrajectoryKey& key) const noexcept;
  };

  struct StoredLoopClosure {
    hybrid_slam::common::ConstraintId constraint_id;
    RelativePoseMeasurement measurement;
  };

  [[nodiscard]] static gtsam::Pose3 toGtsamPose(
      const hybrid_slam::geometry::RigidTransform3d& pose);
  [[nodiscard]] static hybrid_slam::geometry::RigidTransform3d fromGtsamPose(
      const gtsam::Pose3& pose);

  BackendConfig config_;
  gtsam::ISAM2 isam2_;
  std::unordered_map<std::uint64_t, PoseGraphNode> nodes_;
  std::vector<std::uint64_t> node_order_;
  std::unordered_map<TrajectoryKey, bool, TrajectoryKeyHash>
      anchored_trajectories_;
  std::vector<StoredLoopClosure> loop_closures_;
  std::size_t factor_count_ = 0;
  std::size_t update_count_ = 0;
  bool healthy_ = true;
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_ISAM2_OPTIMIZER_H_
