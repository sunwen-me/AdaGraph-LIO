#ifndef ADAGRAPH_BACKEND_LOOP_CLOSURE_H_
#define ADAGRAPH_BACKEND_LOOP_CLOSURE_H_

#include <optional>
#include <vector>

#include "adagraph/backend/constraint.h"
#include "adagraph/backend/pose_graph.h"
#include "adagraph/sensor/immutable_point_cloud.h"

namespace adagraph::backend {

struct LoopCandidate {
  adagraph::common::NodeId query_node;
  adagraph::common::NodeId match_node;
  double retrieval_score = 0.0;
};

// Candidate retrieval only. It cannot mutate PoseGraph or perform geometric
// registration. Calls occur at loop-task frequency outside the frontend.
class LoopDetector {
 public:
  virtual ~LoopDetector() = default;
  [[nodiscard]] virtual std::vector<LoopCandidate> detect(
      const PoseGraphState& snapshot,
      adagraph::common::NodeId query_node) = 0;
};

struct LoopVerificationInput {
  LoopCandidate candidate;
  adagraph::sensor::ImmutablePointCloud query_cloud_base;
  adagraph::sensor::ImmutablePointCloud match_cloud_base;
  adagraph::geometry::RigidTransform3d initial_T_match_query;
};

// Registration verification only. A successful result is a measurement;
// BackendScheduler/ConstraintBuilder remain responsible for graph mutation.
class LoopVerifier {
 public:
  virtual ~LoopVerifier() = default;
  [[nodiscard]] virtual std::optional<RelativePoseMeasurement> verify(
      const LoopVerificationInput& input) = 0;
};

class DisabledLoopDetector final : public LoopDetector {
 public:
  [[nodiscard]] std::vector<LoopCandidate> detect(
      const PoseGraphState&,
      adagraph::common::NodeId) override {
    return {};
  }
};

class DisabledLoopVerifier final : public LoopVerifier {
 public:
  [[nodiscard]] std::optional<RelativePoseMeasurement> verify(
      const LoopVerificationInput&) override {
    return std::nullopt;
  }
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_LOOP_CLOSURE_H_
