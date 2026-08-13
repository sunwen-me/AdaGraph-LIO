#ifndef ADAGRAPH_BACKEND_OPTIMIZATION_RESULT_H_
#define ADAGRAPH_BACKEND_OPTIMIZATION_RESULT_H_

#include <cstdint>
#include <vector>

#include "adagraph/common/domain_ids.h"
#include "adagraph/geometry/transform3.h"

namespace adagraph::backend {

struct OptimizedNodePose {
  adagraph::common::NodeId node_id;
  adagraph::geometry::StampedTransform3d T_map_base;

  [[nodiscard]] bool valid() const noexcept {
    return T_map_base.valid() &&
           T_map_base.target_frame == adagraph::geometry::FrameId::kMap &&
           T_map_base.source_frame == adagraph::geometry::FrameId::kBase;
  }
};

struct OptimizationResult {
  std::uint64_t input_graph_version = 0;
  std::vector<OptimizedNodePose> optimized_nodes;

  [[nodiscard]] bool valid() const noexcept {
    for (const auto& node : optimized_nodes) {
      if (!node.valid()) {
        return false;
      }
    }
    return true;
  }
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_OPTIMIZATION_RESULT_H_
