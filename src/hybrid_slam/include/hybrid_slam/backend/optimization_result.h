#ifndef HYBRID_SLAM_BACKEND_OPTIMIZATION_RESULT_H_
#define HYBRID_SLAM_BACKEND_OPTIMIZATION_RESULT_H_

#include <cmath>
#include <cstdint>
#include <vector>

#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::backend {

struct OptimizedNodePose {
  hybrid_slam::common::NodeId node_id;
  hybrid_slam::geometry::StampedTransform3d T_map_base;

  [[nodiscard]] bool valid() const noexcept {
    return T_map_base.valid() &&
           T_map_base.target_frame == hybrid_slam::geometry::FrameId::kMap &&
           T_map_base.source_frame == hybrid_slam::geometry::FrameId::kBase;
  }
};

// Post-optimization residual of a non-local relative constraint. Keeping this
// in the optimizer result makes "factor inserted" and "factor satisfied"
// separately observable without exposing GTSAM through the backend boundary.
struct RelativeConstraintResidual {
  hybrid_slam::common::ConstraintId constraint_id;
  hybrid_slam::common::NodeId from_node;
  hybrid_slam::common::NodeId to_node;
  double translation_error_m = 0.0;
  double rotation_error_rad = 0.0;

  [[nodiscard]] bool valid() const noexcept {
    return from_node != to_node && std::isfinite(translation_error_m) &&
           translation_error_m >= 0.0 && std::isfinite(rotation_error_rad) &&
           rotation_error_rad >= 0.0;
  }
};

struct OptimizationResult {
  std::uint64_t input_graph_version = 0;
  std::vector<OptimizedNodePose> optimized_nodes;
  std::vector<RelativeConstraintResidual> relative_constraint_residuals;

  [[nodiscard]] bool valid() const noexcept {
    for (const auto& node : optimized_nodes) {
      if (!node.valid()) {
        return false;
      }
    }
    for (const auto& residual : relative_constraint_residuals) {
      if (!residual.valid()) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_OPTIMIZATION_RESULT_H_
