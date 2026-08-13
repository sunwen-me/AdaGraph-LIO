#ifndef ADAGRAPH_BACKEND_POSE_GRAPH_H_
#define ADAGRAPH_BACKEND_POSE_GRAPH_H_

#include <cstdint>
#include <cmath>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "adagraph/backend/constraint.h"
#include "adagraph/backend/optimization_result.h"
#include "adagraph/common/domain_ids.h"
#include "adagraph/geometry/transform3.h"

namespace adagraph::backend {

struct PoseGraphNode {
  adagraph::common::NodeId id;
  adagraph::common::KeyFrameId keyframe_id;
  adagraph::common::TrajectoryId trajectory_id;
  adagraph::common::SessionId session_id;
  adagraph::geometry::StampedTransform3d T_odom_base;
  std::optional<adagraph::geometry::StampedTransform3d> T_map_base;

  [[nodiscard]] bool valid() const noexcept {
    if (!T_odom_base.valid() ||
        T_odom_base.target_frame != adagraph::geometry::FrameId::kOdom ||
        T_odom_base.source_frame != adagraph::geometry::FrameId::kBase) {
      return false;
    }
    return !T_map_base ||
           (T_map_base->valid() &&
            T_map_base->timestamp == T_odom_base.timestamp &&
            T_map_base->target_frame == adagraph::geometry::FrameId::kMap &&
            T_map_base->source_frame == adagraph::geometry::FrameId::kBase);
  }
};

struct PoseGraphState {
  std::uint64_t version = 0;
  std::vector<PoseGraphNode> nodes;
  std::vector<Constraint> constraints;
};

// Single-writer mutable topology store. It intentionally owns no mutex and
// performs no optimization/registration/ROS work; BackendScheduler supplies
// serialization. buildState() is the explicit immutable-problem copy point.
class PoseGraph {
 public:
  [[nodiscard]] std::optional<adagraph::common::NodeId> addNode(
      PoseGraphNode node) {
    if (!node.valid()) {
      return std::nullopt;
    }
    const adagraph::common::NodeId id(next_node_id_++);
    node.id = id;
    node_index_.emplace(id.value(), nodes_.size());
    nodes_.push_back(std::move(node));
    ++version_;
    return id;
  }

  [[nodiscard]] std::optional<adagraph::common::ConstraintId> addConstraint(
      Constraint constraint) {
    if (!constraint.valid() || !referencesKnownNodes(constraint)) {
      return std::nullopt;
    }
    const adagraph::common::ConstraintId id(next_constraint_id_++);
    constraint.id = id;
    constraints_.push_back(std::move(constraint));
    ++version_;
    return id;
  }

  [[nodiscard]] PoseGraphState buildState() const {
    return {version_, nodes_, constraints_};
  }

  // Transitional timestamp association for asynchronous absolute sensors.
  // It is single-writer and currently O(nodes), matching the legacy backend;
  // Phase 5 replaces this with trajectory/submap indices.
  [[nodiscard]] std::optional<PoseGraphNode> findClosestNode(
      adagraph::common::SessionId session_id,
      adagraph::common::TrajectoryId trajectory_id,
      adagraph::common::Timestamp timestamp,
      double maximum_time_difference_s) const noexcept {
    if (!std::isfinite(maximum_time_difference_s) ||
        maximum_time_difference_s < 0.0) {
      return std::nullopt;
    }
    const PoseGraphNode* closest = nullptr;
    double closest_difference = maximum_time_difference_s;
    for (const auto& node : nodes_) {
      if (node.session_id != session_id ||
          node.trajectory_id != trajectory_id) {
        continue;
      }
      const double difference = std::abs(
          node.T_odom_base.timestamp.seconds() - timestamp.seconds());
      if (difference <= closest_difference) {
        closest = &node;
        closest_difference = difference;
      }
    }
    return closest ? std::optional<PoseGraphNode>(*closest) : std::nullopt;
  }

  [[nodiscard]] bool apply(const OptimizationResult& result) {
    if (!result.valid() || result.input_graph_version != version_) {
      return false;
    }

    std::unordered_set<std::uint64_t> seen;
    for (const auto& optimized : result.optimized_nodes) {
      const auto index = node_index_.find(optimized.node_id.value());
      if (index == node_index_.end() ||
          !seen.insert(optimized.node_id.value()).second ||
          optimized.T_map_base.timestamp !=
              nodes_[index->second].T_odom_base.timestamp) {
        return false;
      }
    }
    for (const auto& optimized : result.optimized_nodes) {
      nodes_[node_index_.at(optimized.node_id.value())].T_map_base =
          optimized.T_map_base;
    }
    if (!result.optimized_nodes.empty()) {
      ++version_;
    }
    return true;
  }

  [[nodiscard]] std::uint64_t version() const noexcept { return version_; }
  [[nodiscard]] std::size_t nodeCount() const noexcept { return nodes_.size(); }
  [[nodiscard]] std::size_t constraintCount() const noexcept {
    return constraints_.size();
  }

 private:
  [[nodiscard]] bool referencesKnownNodes(
      const Constraint& constraint) const noexcept {
    if (const auto* relative =
            std::get_if<RelativePoseMeasurement>(&constraint.measurement)) {
      return node_index_.contains(relative->from_node.value()) &&
             node_index_.contains(relative->to_node.value());
    }
    const auto& absolute =
        std::get<AbsolutePoseMeasurement>(constraint.measurement);
    return node_index_.contains(absolute.node_id.value());
  }

  std::uint64_t version_ = 0;
  std::uint64_t next_node_id_ = 0;
  std::uint64_t next_constraint_id_ = 0;
  std::vector<PoseGraphNode> nodes_;
  std::vector<Constraint> constraints_;
  std::unordered_map<std::uint64_t, std::size_t> node_index_;
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_POSE_GRAPH_H_
