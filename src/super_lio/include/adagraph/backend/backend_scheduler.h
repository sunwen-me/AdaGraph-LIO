#ifndef ADAGRAPH_BACKEND_BACKEND_SCHEDULER_H_
#define ADAGRAPH_BACKEND_BACKEND_SCHEDULER_H_

#include <condition_variable>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <variant>

#include "adagraph/backend/backend_config.h"
#include "adagraph/backend/constraint_builder.h"
#include "adagraph/backend/optimizer.h"
#include "adagraph/backend/pose_graph.h"
#include "adagraph/mapping/keyframe.h"
#include "adagraph/mapping/map_manager.h"

namespace adagraph::backend {

enum class SubmissionStatus {
  kAccepted = 0,
  kInvalid,
  kClosed,
};

struct BackendSchedulerStats {
  std::uint64_t enqueued = 0;
  std::uint64_t processed = 0;
  std::uint64_t failed = 0;
  std::uint64_t capacity_waits = 0;
  std::size_t maximum_queue_depth = 0;
  std::size_t node_count = 0;
  std::size_t constraint_count = 0;
  std::uint64_t optimizer_updates = 0;
  std::uint64_t optimizer_failures = 0;
  double maximum_optimizer_translation_delta_m = 0.0;
  double maximum_optimizer_rotation_delta_rad = 0.0;
  std::uint64_t global_constraints_dropped = 0;
  std::uint64_t map_snapshot_version = 0;
  std::size_t submap_count = 0;
  std::size_t finished_submap_count = 0;
  std::uint64_t snapshot_observer_failures = 0;
};

struct KeyFrameSubmission {
  adagraph::mapping::KeyFrame keyframe;
  Covariance6d odometry_covariance = Covariance6d::Identity();

  [[nodiscard]] bool valid() const noexcept {
    return keyframe.valid() && IsValidCovariance6d(odometry_covariance);
  }
};

struct TimedGlobalPoseConstraint {
  ConstraintType type = ConstraintType::kRelocalization;
  adagraph::common::SessionId session_id;
  adagraph::common::TrajectoryId trajectory_id;
  adagraph::geometry::StampedTransform3d T_map_base;
  Covariance6d covariance = Covariance6d::Identity();
  double score = 0.0;
  double maximum_time_difference_s = 0.15;

  [[nodiscard]] bool valid() const noexcept {
    return (type == ConstraintType::kPrior ||
            type == ConstraintType::kRelocalization) &&
           T_map_base.valid() &&
           T_map_base.target_frame == adagraph::geometry::FrameId::kMap &&
           T_map_base.source_frame == adagraph::geometry::FrameId::kBase &&
           IsValidCovariance6d(covariance) && std::isfinite(score) &&
           std::isfinite(maximum_time_difference_s) &&
           maximum_time_difference_s >= 0.0;
  }
};

// Reliable bounded single-writer scheduler for accepted KeyFrames and typed
// constraints. It owns the only mutable PoseGraph instance. submit* may wait
// for bounded capacity, but never performs graph work under the queue mutex.
// The intended caller is the system/downstream execution domain, not the
// frontend estimator's per-point loop.
class BackendScheduler {
 public:
  using MapSnapshotObserver = std::function<void(
      std::shared_ptr<const adagraph::mapping::MapSnapshot>)>;

  explicit BackendScheduler(
      BackendConfig config,
      std::unique_ptr<Optimizer> optimizer = nullptr,
      MapSnapshotObserver snapshot_observer = {});
  ~BackendScheduler();

  BackendScheduler(const BackendScheduler&) = delete;
  BackendScheduler& operator=(const BackendScheduler&) = delete;

  [[nodiscard]] SubmissionStatus submitKeyFrame(
      adagraph::mapping::KeyFrame keyframe);
  [[nodiscard]] SubmissionStatus submitKeyFrame(
      adagraph::mapping::KeyFrame keyframe,
      Covariance6d odometry_covariance);
  [[nodiscard]] SubmissionStatus submitConstraint(Constraint constraint);
  [[nodiscard]] SubmissionStatus submitGlobalPoseConstraint(
      TimedGlobalPoseConstraint request);

  // Idempotent close/drain/join. Required tasks already accepted are
  // processed before the worker exits.
  void finish();

  [[nodiscard]] BackendSchedulerStats stats() const;
  [[nodiscard]] std::shared_ptr<const adagraph::mapping::MapSnapshot>
  mapSnapshot() const noexcept;

  // A consistent full graph copy is available only after finish(), avoiding
  // an O(graph size) snapshot allocation on every keyframe. Phase 5 adds the
  // versioned runtime MapSnapshot publication boundary.
  [[nodiscard]] std::optional<PoseGraphState> finalState() const;

 private:
  struct TrajectoryKey {
    adagraph::common::SessionId session_id;
    adagraph::common::TrajectoryId trajectory_id;

    bool operator==(const TrajectoryKey&) const = default;
  };

  struct TrajectoryKeyHash {
    [[nodiscard]] std::size_t operator()(
        const TrajectoryKey& key) const noexcept;
  };

  struct SubmapRuntimeState {
    adagraph::common::SubmapId active_id{0};
    std::size_t keyframe_count = 0;
  };

  enum class State {
    kRunning = 0,
    kClosing,
    kStopped,
  };

  using WorkItem =
      std::variant<KeyFrameSubmission, Constraint, TimedGlobalPoseConstraint>;

  [[nodiscard]] SubmissionStatus enqueue(WorkItem item);
  void workerLoop();
  [[nodiscard]] bool process(KeyFrameSubmission&& submission);
  [[nodiscard]] bool process(Constraint&& constraint);
  [[nodiscard]] bool process(TimedGlobalPoseConstraint&& request);
  [[nodiscard]] bool applyOptimizer(
      GraphUpdate update,
      const std::optional<PoseGraphNode>& parity_node = std::nullopt);
  [[nodiscard]] bool publishMapSnapshot();

  BackendConfig config_;
  ConstraintBuilder constraint_builder_;
  PoseGraph pose_graph_;
  adagraph::mapping::MapManager map_manager_;
  std::unique_ptr<Optimizer> optimizer_;
  MapSnapshotObserver snapshot_observer_;
  std::unordered_map<TrajectoryKey, PoseGraphNode, TrajectoryKeyHash>
      last_nodes_;
  std::unordered_map<TrajectoryKey, SubmapRuntimeState, TrajectoryKeyHash>
      submap_states_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_not_empty_cv_;
  std::condition_variable queue_not_full_cv_;
  std::deque<WorkItem> queue_;
  State state_ = State::kRunning;
  std::thread worker_;
  BackendSchedulerStats stats_;
  std::uint64_t optimizer_updates_ = 0;
  std::uint64_t optimizer_failures_ = 0;
  double maximum_optimizer_translation_delta_m_ = 0.0;
  double maximum_optimizer_rotation_delta_rad_ = 0.0;
  std::uint64_t global_constraints_dropped_ = 0;
  std::uint64_t snapshot_observer_failures_ = 0;
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_BACKEND_SCHEDULER_H_
