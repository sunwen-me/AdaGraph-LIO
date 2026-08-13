#include "adagraph/backend/backend_scheduler.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace adagraph::backend {

BackendScheduler::BackendScheduler(
    BackendConfig config, std::unique_ptr<Optimizer> optimizer,
    MapSnapshotObserver snapshot_observer)
    : config_(std::move(config)),
      optimizer_(std::move(optimizer)),
      snapshot_observer_(std::move(snapshot_observer)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid backend scheduler config: " +
                                *error);
  }
  worker_ = std::thread(&BackendScheduler::workerLoop, this);
}

BackendScheduler::~BackendScheduler() { finish(); }

SubmissionStatus BackendScheduler::submitKeyFrame(
    adagraph::mapping::KeyFrame keyframe) {
  Covariance6d covariance = Covariance6d::Zero();
  covariance.diagonal().head<3>().setConstant(
      config_.odometry_rotation_sigma_rad *
      config_.odometry_rotation_sigma_rad);
  covariance.diagonal().tail<3>().setConstant(
      config_.odometry_translation_sigma_m *
      config_.odometry_translation_sigma_m);
  return submitKeyFrame(std::move(keyframe), covariance);
}

SubmissionStatus BackendScheduler::submitKeyFrame(
    adagraph::mapping::KeyFrame keyframe,
    Covariance6d odometry_covariance) {
  KeyFrameSubmission submission{
      std::move(keyframe), std::move(odometry_covariance)};
  if (!submission.valid()) {
    return SubmissionStatus::kInvalid;
  }
  return enqueue(WorkItem(std::in_place_type<KeyFrameSubmission>,
                          std::move(submission)));
}

SubmissionStatus BackendScheduler::submitConstraint(Constraint constraint) {
  if (!constraint.valid()) {
    return SubmissionStatus::kInvalid;
  }
  return enqueue(WorkItem(std::in_place_type<Constraint>,
                          std::move(constraint)));
}

SubmissionStatus BackendScheduler::submitGlobalPoseConstraint(
    TimedGlobalPoseConstraint request) {
  if (!request.valid()) {
    return SubmissionStatus::kInvalid;
  }
  return enqueue(WorkItem(std::in_place_type<TimedGlobalPoseConstraint>,
                          std::move(request)));
}

SubmissionStatus BackendScheduler::enqueue(WorkItem item) {
  std::unique_lock<std::mutex> lock(queue_mutex_);
  if (state_ != State::kRunning) {
    return SubmissionStatus::kClosed;
  }
  if (queue_.size() >= config_.submission_queue_capacity) {
    ++stats_.capacity_waits;
  }
  queue_not_full_cv_.wait(lock, [this] {
    return state_ != State::kRunning ||
           queue_.size() < config_.submission_queue_capacity;
  });
  if (state_ != State::kRunning) {
    return SubmissionStatus::kClosed;
  }

  queue_.push_back(std::move(item));
  ++stats_.enqueued;
  stats_.maximum_queue_depth =
      std::max(stats_.maximum_queue_depth, queue_.size());
  lock.unlock();
  queue_not_empty_cv_.notify_one();
  return SubmissionStatus::kAccepted;
}

void BackendScheduler::finish() {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (state_ == State::kStopped) {
      return;
    }
    state_ = State::kClosing;
  }
  queue_not_empty_cv_.notify_all();
  queue_not_full_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    state_ = State::kStopped;
  }
}

BackendSchedulerStats BackendScheduler::stats() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return stats_;
}

std::shared_ptr<const adagraph::mapping::MapSnapshot>
BackendScheduler::mapSnapshot() const noexcept {
  return map_manager_.snapshot();
}

std::optional<PoseGraphState> BackendScheduler::finalState() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  if (state_ != State::kStopped) {
    return std::nullopt;
  }
  return pose_graph_.buildState();
}

std::size_t BackendScheduler::TrajectoryKeyHash::operator()(
    const TrajectoryKey& key) const noexcept {
  const std::size_t first =
      std::hash<std::uint64_t>{}(key.session_id.value());
  const std::size_t second =
      std::hash<std::uint64_t>{}(key.trajectory_id.value());
  return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) +
                  (first >> 2U));
}

void BackendScheduler::workerLoop() {
  for (;;) {
    std::optional<WorkItem> item;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_not_empty_cv_.wait(lock, [this] {
        return state_ != State::kRunning || !queue_.empty();
      });
      if (queue_.empty()) {
        if (state_ != State::kRunning) {
          return;
        }
        continue;
      }
      item.emplace(std::move(queue_.front()));
      queue_.pop_front();
    }
    queue_not_full_cv_.notify_one();

    const bool success = std::visit(
        [this](auto&& value) {
          return process(std::move(value));
        },
        std::move(*item));

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      ++stats_.processed;
      if (!success) {
        ++stats_.failed;
      }
      stats_.node_count = pose_graph_.nodeCount();
      stats_.constraint_count = pose_graph_.constraintCount();
      stats_.optimizer_updates = optimizer_updates_;
      stats_.optimizer_failures = optimizer_failures_;
      stats_.maximum_optimizer_translation_delta_m =
          maximum_optimizer_translation_delta_m_;
      stats_.maximum_optimizer_rotation_delta_rad =
          maximum_optimizer_rotation_delta_rad_;
      stats_.global_constraints_dropped = global_constraints_dropped_;
      const auto map_snapshot = map_manager_.snapshot();
      stats_.map_snapshot_version = map_snapshot->version;
      stats_.submap_count = map_snapshot->submaps.size();
      stats_.finished_submap_count = static_cast<std::size_t>(std::count_if(
          map_snapshot->submaps.begin(), map_snapshot->submaps.end(),
          [](const adagraph::mapping::SubmapSnapshot& submap) {
            return submap.state == adagraph::mapping::SubmapState::kFinished;
          }));
      stats_.snapshot_observer_failures = snapshot_observer_failures_;
    }
  }
}

bool BackendScheduler::process(KeyFrameSubmission&& submission) {
  const auto& keyframe = submission.keyframe;
  PoseGraphNode node;
  node.keyframe_id = keyframe.id();
  node.trajectory_id = keyframe.trajectoryId();
  node.session_id = keyframe.sessionId();
  node.T_odom_base = keyframe.T_odom_base();

  const TrajectoryKey key{node.session_id, node.trajectory_id};
  auto& submap = submap_states_[key];

  const auto node_id = pose_graph_.addNode(node);
  if (!node_id) {
    return false;
  }
  node.id = *node_id;

  GraphUpdate optimizer_update;
  optimizer_update.added_nodes.push_back(node);
  if (!map_manager_.insertKeyFrame(
          keyframe, node, submap.active_id)) {
    return false;
  }

  const auto previous = last_nodes_.find(key);
  if (previous != last_nodes_.end()) {
    const auto odometry = constraint_builder_.makeOdometry(
        previous->second, node, submission.odometry_covariance);
    if (!odometry) {
      return false;
    }
    const auto constraint_id = pose_graph_.addConstraint(*odometry);
    if (!constraint_id) {
      return false;
    }
    Constraint committed = *odometry;
    committed.id = *constraint_id;
    if (!map_manager_.addConstraint(committed)) {
      return false;
    }
    optimizer_update.added_constraints.push_back(std::move(committed));
  }
  last_nodes_.insert_or_assign(key, node);
  optimizer_update.input_graph_version = pose_graph_.version();
  if (!applyOptimizer(std::move(optimizer_update), node)) {
    return false;
  }
  ++submap.keyframe_count;
  if (submap.keyframe_count >= config_.submap_max_keyframes) {
    if (!map_manager_.finishSubmap(
            node.session_id, node.trajectory_id, submap.active_id)) {
      return false;
    }
    submap.active_id = adagraph::common::SubmapId(
        submap.active_id.value() + 1);
    submap.keyframe_count = 0;
  }
  return publishMapSnapshot();
}

bool BackendScheduler::process(Constraint&& constraint) {
  Constraint optimizer_constraint = constraint;
  const auto constraint_id =
      pose_graph_.addConstraint(std::move(constraint));
  if (!constraint_id) {
    return false;
  }
  optimizer_constraint.id = *constraint_id;
  if (!map_manager_.addConstraint(optimizer_constraint)) {
    return false;
  }
  GraphUpdate optimizer_update;
  optimizer_update.input_graph_version = pose_graph_.version();
  optimizer_update.added_constraints.push_back(
      std::move(optimizer_constraint));
  if (!applyOptimizer(std::move(optimizer_update))) {
    return false;
  }
  return publishMapSnapshot();
}

bool BackendScheduler::process(TimedGlobalPoseConstraint&& request) {
  const auto node = pose_graph_.findClosestNode(
      request.session_id, request.trajectory_id,
      request.T_map_base.timestamp, request.maximum_time_difference_s);
  if (!node) {
    ++global_constraints_dropped_;
    return true;
  }
  std::optional<Constraint> constraint;
  if (request.type == ConstraintType::kPrior) {
    constraint = constraint_builder_.makePrior(
        *node, request.T_map_base, request.covariance);
  } else {
    constraint = constraint_builder_.makeAssociatedRelocalization(
        *node, request.T_map_base, request.covariance, request.score,
        request.maximum_time_difference_s);
  }
  if (!constraint) {
    ++global_constraints_dropped_;
    return true;
  }
  return process(std::move(*constraint));
}

bool BackendScheduler::applyOptimizer(
    GraphUpdate update, const std::optional<PoseGraphNode>& parity_node) {
  if (!optimizer_) {
    return true;
  }
  OptimizerOutput output = optimizer_->apply(update);
  ++optimizer_updates_;
  if (output.status != OptimizationStatus::kApplied ||
      !pose_graph_.apply(output.result) ||
      !map_manager_.applyOptimization(output.result)) {
    ++optimizer_failures_;
    return false;
  }

  if (parity_node) {
    for (const auto& optimized : output.result.optimized_nodes) {
      if (optimized.node_id != parity_node->id) {
        continue;
      }
      const auto& expected = parity_node->T_odom_base.T_target_source;
      const auto& actual = optimized.T_map_base.T_target_source;
      maximum_optimizer_translation_delta_m_ = std::max(
          maximum_optimizer_translation_delta_m_,
          (actual.translation - expected.translation).norm());
      const Eigen::Quaterniond delta =
          (actual.rotation * expected.rotation.conjugate()).normalized();
      maximum_optimizer_rotation_delta_rad_ = std::max(
          maximum_optimizer_rotation_delta_rad_,
          std::abs(Eigen::AngleAxisd(delta).angle()));
      break;
    }
  }
  return true;
}

bool BackendScheduler::publishMapSnapshot() {
  auto snapshot = map_manager_.publishSnapshot();
  if (!snapshot || !snapshot->valid()) {
    return false;
  }
  if (snapshot_observer_) {
    // Observer contracts are deliberately limited to O(1), non-blocking
    // handoff (normally an atomic shared_ptr exchange). Consumer work must
    // never execute on the backend single-writer thread.
    try {
      snapshot_observer_(snapshot);
    } catch (...) {
      ++snapshot_observer_failures_;
    }
  }
  return true;
}

}  // namespace adagraph::backend
