#ifndef HYBRID_SLAM_MAPPING_MAP_MANAGER_H_
#define HYBRID_SLAM_MAPPING_MAP_MANAGER_H_

#include <atomic>
#include <cstddef>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "hybrid_slam/backend/optimization_result.h"
#include "hybrid_slam/backend/pose_graph.h"
#include "hybrid_slam/mapping/map_snapshot.h"

namespace hybrid_slam::mapping {

struct MapMemoryStats {
  std::size_t keyframe_cloud_count = 0;
  std::size_t unique_keyframe_cloud_count = 0;
  std::size_t unique_keyframe_cloud_points = 0;
  std::size_t unique_keyframe_cloud_logical_bytes = 0;
  std::size_t unique_keyframe_cloud_backing_bytes = 0;
};

// Single-writer keyframe payload/optimized-pose owner. BackendScheduler calls
// mutation methods; arbitrary readers only load immutable snapshots.
class MapManager {
 public:
  MapManager();

  [[nodiscard]] bool insertKeyFrame(
      const KeyFrame& keyframe,
      const hybrid_slam::backend::PoseGraphNode& node,
      hybrid_slam::common::SubmapId submap_id);
  [[nodiscard]] bool addConstraint(
      const hybrid_slam::backend::Constraint& constraint);
  [[nodiscard]] bool applyOptimization(
      const hybrid_slam::backend::OptimizationResult& result);
  [[nodiscard]] bool finishSubmap(
      hybrid_slam::common::SessionId session_id,
      hybrid_slam::common::TrajectoryId trajectory_id,
      hybrid_slam::common::SubmapId submap_id);

  // Explicit commit point. Copies light metadata/vectors and shares immutable
  // cloud payloads, then atomically publishes the complete new version.
  [[nodiscard]] std::shared_ptr<const MapSnapshot> publishSnapshot();
  [[nodiscard]] std::shared_ptr<const MapSnapshot> snapshot() const noexcept;
  // Constant-time single-writer accounting. Shared Snapshot references do not
  // increase these values; every cloud backing store is counted once.
  [[nodiscard]] MapMemoryStats memoryStats() const noexcept;

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

  struct SubmapKey {
    TrajectoryKey trajectory;
    hybrid_slam::common::SubmapId submap_id;
    bool operator==(const SubmapKey&) const = default;
  };

  struct SubmapKeyHash {
    [[nodiscard]] std::size_t operator()(
        const SubmapKey& key) const noexcept;
  };

  std::vector<TrajectorySnapshot> trajectories_;
  std::vector<SubmapSnapshot> submaps_;
  std::vector<KeyFrameSnapshot> keyframes_;
  std::vector<hybrid_slam::backend::Constraint> constraints_;
  std::unordered_map<TrajectoryKey, std::size_t, TrajectoryKeyHash>
      trajectory_index_;
  std::unordered_map<SubmapKey, std::size_t, SubmapKeyHash> submap_index_;
  std::unordered_map<std::uint64_t, std::size_t> keyframe_index_;
  std::unordered_map<std::uint64_t, std::size_t> node_index_;
  std::unordered_set<const void*> unique_keyframe_cloud_storage_;
  MapMemoryStats memory_stats_;
  std::uint64_t next_snapshot_version_ = 1;
  std::atomic<std::shared_ptr<const MapSnapshot>> published_snapshot_;
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_MAP_MANAGER_H_
