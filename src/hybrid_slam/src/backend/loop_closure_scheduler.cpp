#include "hybrid_slam/backend/loop_closure_scheduler.h"

#include <algorithm>
#include <set>
#include <stdexcept>
#include <utility>

namespace hybrid_slam::backend {
namespace {

const hybrid_slam::mapping::KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::common::NodeId node_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [node_id](const auto& keyframe) { return keyframe.node_id == node_id; });
  return found == map.keyframes.end() ? nullptr : &*found;
}

PoseGraphState BuildGraph(const hybrid_slam::mapping::MapSnapshot& map) {
  PoseGraphState graph;
  graph.version = map.version;
  graph.constraints = map.constraints;
  graph.nodes.reserve(map.keyframes.size());
  for (const auto& keyframe : map.keyframes) {
    graph.nodes.push_back({keyframe.node_id, keyframe.keyframe_id,
                           keyframe.trajectory_id, keyframe.session_id,
                           keyframe.T_odom_base, keyframe.T_map_base});
  }
  return graph;
}

}  // namespace

LoopClosureScheduler::LoopClosureScheduler(
    std::unique_ptr<LoopDetector> detector,
    std::unique_ptr<LoopVerifier> verifier,
    ConstraintCallback callback)
    : detector_(std::move(detector)),
      verifier_(std::move(verifier)),
      callback_(std::move(callback)) {
  if (!detector_ || !verifier_ || !callback_) {
    throw std::invalid_argument(
        "LoopClosureScheduler requires detector, verifier and callback");
  }
  worker_ = std::thread(&LoopClosureScheduler::workerLoop, this);
}

LoopClosureScheduler::~LoopClosureScheduler() { finishLatest(); }

LoopClosureSubmissionStatus LoopClosureScheduler::submit(
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map,
    const hybrid_slam::common::NodeId query_node) {
  return submit(std::move(map), std::vector{query_node});
}

LoopClosureSubmissionStatus LoopClosureScheduler::submit(
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map,
    std::vector<hybrid_slam::common::NodeId> query_nodes) {
  // Full snapshot validation and descriptor-index construction belong to the
  // worker. The backend snapshot observer must remain an O(1) handoff.
  query_nodes.erase(
      std::remove_if(query_nodes.begin(), query_nodes.end(),
                     [](const auto node) { return node.value() == 0; }),
      query_nodes.end());
  std::sort(query_nodes.begin(), query_nodes.end(),
            [](const auto lhs, const auto rhs) {
              return lhs.value() < rhs.value();
            });
  query_nodes.erase(
      std::unique(query_nodes.begin(), query_nodes.end()), query_nodes.end());
  if (!map || query_nodes.empty()) {
    return LoopClosureSubmissionStatus::kInvalid;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return LoopClosureSubmissionStatus::kClosed;
  }
  if (pending_) {
    ++stats_.replaced_pending;
  }
  // A completed registration is a useful graph observation even when a newer
  // snapshot has arrived. Coalesce work that has not started yet, but let the
  // active verification reach a publish decision. The application submits at
  // submap-completion cadence, so this cannot turn into a keyframe-rate queue.
  const auto sequence = next_sequence_++;
  latest_sequence_ = sequence;
  pending_ = WorkItem{sequence, std::move(map), std::move(query_nodes)};
  ++stats_.submitted;
  cv_.notify_one();
  return LoopClosureSubmissionStatus::kAccepted;
}

void LoopClosureScheduler::finishLatest() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_) {
      closed_ = true;
    }
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void LoopClosureScheduler::finish() { finishLatest(); }

LoopClosureSchedulerStats LoopClosureScheduler::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void LoopClosureScheduler::workerLoop() {
  for (;;) {
    WorkItem item;
    std::stop_token stop_token;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return closed_ || pending_.has_value(); });
      // Once finishLatest() has closed submissions, process the one retained
      // latest item, if any, then leave after the queue is drained.
      if (!pending_) {
        return;
      }
      item = std::move(*pending_);
      pending_.reset();
      active_stop_source_ = std::stop_source{};
      stop_token = active_stop_source_.get_token();
      active_ = true;
    }

    if (!item.map->valid()) {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = false;
      ++stats_.completed;
      continue;
    }

    auto index = std::make_shared<
        hybrid_slam::mapping::SubmapDescriptorIndex>();
    index->rebuild(*item.map);
    const LoopDetectionSnapshot snapshot{BuildGraph(*item.map), item.map,
                                         std::move(index)};
    verifier_->prepare(snapshot);
    std::vector<Constraint> accepted;
    std::set<std::pair<std::uint64_t, std::uint64_t>> accepted_pairs;
    constexpr std::size_t kMaximumConstraintsPerWorkItem = 4;
    for (const auto query_node : item.query_nodes) {
      if (stop_token.stop_requested()) {
        break;
      }
      const auto* query = FindKeyFrame(*item.map, query_node);
      if (!query || !query->valid()) {
        continue;
      }
      const auto candidates = detector_->detect(snapshot, query_node);
      for (const auto& candidate : candidates) {
        if (stop_token.stop_requested()) {
          break;
        }
        const auto* candidate_query =
            FindKeyFrame(*item.map, candidate.query_node);
        const auto* match = FindKeyFrame(*item.map, candidate.match_node);
        if (!candidate_query || !candidate_query->valid() || !match ||
            candidate.match_node == candidate.query_node || !match->valid()) {
          continue;
        }
        const auto pair_key = std::pair{candidate.match_node.value(),
                                        candidate.query_node.value()};
        if (accepted_pairs.contains(pair_key)) {
          continue;
        }
        const auto initial = match->T_map_base.T_target_source.inverse()
                                 .compose(
                                     candidate_query->T_map_base.T_target_source);
        const LoopVerificationInput input{
            candidate, candidate_query->T_odom_base.timestamp,
            candidate_query->cloud_base,
            match->cloud_base, initial};
        const auto measurement = verifier_->verify(input);
        if (!measurement || measurement->from_node != candidate.match_node ||
            measurement->to_node != candidate.query_node ||
            !measurement->T_from_to.isFinite() ||
            !IsValidCovariance6d(measurement->covariance)) {
          continue;
        }
        Constraint constraint;
        constraint.type = ConstraintType::kLoopClosure;
        constraint.measurement = *measurement;
        if (constraint.valid()) {
          accepted_pairs.insert(pair_key);
          accepted.push_back(std::move(constraint));
          if (accepted.size() >= kMaximumConstraintsPerWorkItem) {
            break;
          }
        }
      }
      if (accepted.size() >= kMaximumConstraintsPerWorkItem) {
        break;
      }
    }

    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = false;
      ++stats_.completed;
      // finishLatest() closes future submission but lets active work commit
      // before the backend is finalized.
      publish = !stop_token.stop_requested();
    }
    if (!publish || accepted.empty()) {
      continue;
    }
    for (auto& constraint : accepted) {
      try {
        callback_(std::move(constraint));
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.constraints_published;
      } catch (...) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.callback_failures;
      }
    }
  }
}

}  // namespace hybrid_slam::backend
