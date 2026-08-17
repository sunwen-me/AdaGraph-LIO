#ifndef HYBRID_SLAM_BACKEND_OPTIMIZER_H_
#define HYBRID_SLAM_BACKEND_OPTIMIZER_H_

#include <cstdint>
#include <vector>

#include "hybrid_slam/backend/optimization_result.h"
#include "hybrid_slam/backend/pose_graph.h"

namespace hybrid_slam::backend {

// Small immutable graph delta produced by the PoseGraph single writer. This
// avoids rebuilding/copying the complete graph at every keyframe while still
// keeping optimizer mutation behind one explicit boundary.
struct GraphUpdate {
  std::uint64_t input_graph_version = 0;
  std::vector<PoseGraphNode> added_nodes;
  std::vector<Constraint> added_constraints;

  [[nodiscard]] bool valid() const noexcept {
    if (added_nodes.empty() && added_constraints.empty()) {
      return false;
    }
    for (const auto& node : added_nodes) {
      if (!node.valid()) {
        return false;
      }
    }
    for (const auto& constraint : added_constraints) {
      if (!constraint.valid()) {
        return false;
      }
    }
    return true;
  }
};

enum class OptimizationStatus {
  kApplied = 0,
  kInvalidInput,
  kFailed,
};

struct OptimizerOutput {
  OptimizationStatus status = OptimizationStatus::kFailed;
  OptimizationResult result;
};

// Coarse keyframe/task-frequency interface. One virtual dispatch per graph
// update is acceptable; no point-level code depends on this interface.
class Optimizer {
 public:
  virtual ~Optimizer() = default;
  [[nodiscard]] virtual OptimizerOutput apply(const GraphUpdate& update) = 0;
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_OPTIMIZER_H_
