#include "adagraph/mapping/map_manager.h"

#include <algorithm>
#include <functional>
#include <unordered_set>
#include <utility>

namespace adagraph::mapping {

MapManager::MapManager() {
  published_snapshot_.store(std::make_shared<const MapSnapshot>());
}

bool MapManager::insertKeyFrame(
    const KeyFrame& keyframe,
    const adagraph::backend::PoseGraphNode& node,
    adagraph::common::SubmapId submap_id) {
  if (!keyframe.valid() || !node.valid() ||
      keyframe.id() != node.keyframe_id ||
      keyframe.trajectoryId() != node.trajectory_id ||
      keyframe.sessionId() != node.session_id ||
      keyframe.T_odom_base().timestamp != node.T_odom_base.timestamp ||
      keyframe_index_.contains(keyframe.id().value()) ||
      node_index_.contains(node.id.value())) {
    return false;
  }

  const TrajectoryKey trajectory_key{
      keyframe.sessionId(), keyframe.trajectoryId()};
  auto trajectory = trajectory_index_.find(trajectory_key);
  if (trajectory == trajectory_index_.end()) {
    const std::size_t index = trajectories_.size();
    trajectories_.push_back(
        {keyframe.trajectoryId(), keyframe.sessionId(), {}});
    trajectory = trajectory_index_.emplace(trajectory_key, index).first;
  }

  const SubmapKey submap_key{trajectory_key, submap_id};
  auto submap = submap_index_.find(submap_key);
  if (submap == submap_index_.end()) {
    const std::size_t index = submaps_.size();
    submaps_.push_back({submap_id, keyframe.trajectoryId(),
                        keyframe.sessionId(), SubmapState::kActive, {}});
    submap = submap_index_.emplace(submap_key, index).first;
    trajectories_[trajectory->second].submap_ids.push_back(submap_id);
  } else if (submaps_[submap->second].state == SubmapState::kFinished) {
    return false;
  }

  adagraph::geometry::StampedTransform3d T_map_base{
      node.T_odom_base.timestamp,
      adagraph::geometry::FrameId::kMap,
      adagraph::geometry::FrameId::kBase,
      node.T_map_base ? node.T_map_base->T_target_source
                      : node.T_odom_base.T_target_source,
  };
  const std::size_t index = keyframes_.size();
  keyframes_.push_back({
      node.id,
      keyframe.id(),
      keyframe.trajectoryId(),
      keyframe.sessionId(),
      submap_id,
      keyframe.T_odom_base(),
      std::move(T_map_base),
      keyframe.cloudBase(),
      keyframe.quality(),
  });
  keyframe_index_.emplace(keyframe.id().value(), index);
  node_index_.emplace(node.id.value(), index);
  submaps_[submap->second].keyframe_ids.push_back(keyframe.id());
  ++memory_stats_.keyframe_cloud_count;
  const auto& cloud = keyframe.cloudBase();
  const auto* storage = cloud.view().storageIdentity();
  if (storage != nullptr &&
      unique_keyframe_cloud_storage_.insert(storage).second) {
    ++memory_stats_.unique_keyframe_cloud_count;
    memory_stats_.unique_keyframe_cloud_points += cloud.size();
    memory_stats_.unique_keyframe_cloud_logical_bytes +=
        cloud.size() * cloud.view().strideBytes();
    memory_stats_.unique_keyframe_cloud_backing_bytes +=
        cloud.backingBytesForDiagnostics();
  }
  return true;
}

bool MapManager::addConstraint(
    const adagraph::backend::Constraint& constraint) {
  if (!constraint.valid()) {
    return false;
  }
  const auto known = [this](adagraph::common::NodeId id) {
    return node_index_.contains(id.value());
  };
  if (const auto* relative =
          std::get_if<adagraph::backend::RelativePoseMeasurement>(
              &constraint.measurement)) {
    if (!known(relative->from_node) || !known(relative->to_node)) {
      return false;
    }
  } else if (!known(
                 std::get<adagraph::backend::AbsolutePoseMeasurement>(
                     constraint.measurement)
                     .node_id)) {
    return false;
  }
  constraints_.push_back(constraint);
  return true;
}

bool MapManager::applyOptimization(
    const adagraph::backend::OptimizationResult& result) {
  if (!result.valid()) {
    return false;
  }
  std::unordered_set<std::uint64_t> optimized_node_ids;
  optimized_node_ids.reserve(result.optimized_nodes.size());
  for (const auto& optimized : result.optimized_nodes) {
    const auto index = node_index_.find(optimized.node_id.value());
    if (index == node_index_.end() ||
        !optimized_node_ids.insert(optimized.node_id.value()).second ||
        keyframes_[index->second].T_odom_base.timestamp !=
            optimized.T_map_base.timestamp) {
      return false;
    }
  }
  for (const auto& optimized : result.optimized_nodes) {
    keyframes_[node_index_.at(optimized.node_id.value())].T_map_base =
        optimized.T_map_base;
  }
  return true;
}

bool MapManager::finishSubmap(
    adagraph::common::SessionId session_id,
    adagraph::common::TrajectoryId trajectory_id,
    adagraph::common::SubmapId submap_id) {
  const auto found = submap_index_.find(
      {{session_id, trajectory_id}, submap_id});
  if (found == submap_index_.end()) {
    return false;
  }
  submaps_[found->second].state = SubmapState::kFinished;
  return true;
}

std::shared_ptr<const MapSnapshot> MapManager::publishSnapshot() {
  auto next = std::make_shared<MapSnapshot>();
  next->version = next_snapshot_version_++;
  next->trajectories = trajectories_;
  next->submaps = submaps_;
  next->keyframes = keyframes_;
  next->constraints = constraints_;
  std::shared_ptr<const MapSnapshot> immutable = std::move(next);
  published_snapshot_.store(immutable, std::memory_order_release);
  return immutable;
}

std::shared_ptr<const MapSnapshot> MapManager::snapshot() const noexcept {
  return published_snapshot_.load(std::memory_order_acquire);
}

MapMemoryStats MapManager::memoryStats() const noexcept {
  return memory_stats_;
}

std::size_t MapManager::TrajectoryKeyHash::operator()(
    const TrajectoryKey& key) const noexcept {
  const std::size_t first =
      std::hash<std::uint64_t>{}(key.session_id.value());
  const std::size_t second =
      std::hash<std::uint64_t>{}(key.trajectory_id.value());
  return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) +
                  (first >> 2U));
}

std::size_t MapManager::SubmapKeyHash::operator()(
    const SubmapKey& key) const noexcept {
  const std::size_t trajectory = TrajectoryKeyHash{}(key.trajectory);
  const std::size_t submap =
      std::hash<std::uint64_t>{}(key.submap_id.value());
  return trajectory ^
         (submap + 0x9e3779b97f4a7c15ULL + (trajectory << 6U) +
          (trajectory >> 2U));
}

}  // namespace adagraph::mapping
