#ifndef HYBRID_SLAM_BACKEND_LOOP_CLOSURE_H_
#define HYBRID_SLAM_BACKEND_LOOP_CLOSURE_H_

#include <memory>
#include <optional>
#include <vector>

#include "hybrid_slam/backend/constraint.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/backend/pose_graph.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/mapping/submap_index.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"

namespace hybrid_slam::backend {

struct LoopCandidate {
  hybrid_slam::common::NodeId query_node;
  hybrid_slam::common::NodeId match_node;
  double retrieval_score = 0.0;
  // Scan Context may supply a yaw-only estimate. Learned plugins may instead
  // supply a complete planar relative pose for subsequent 3D refinement.
  std::optional<double> coarse_yaw_rad;
  std::optional<hybrid_slam::geometry::RigidTransform3d>
      coarse_T_match_query;
};

struct LoopDetectionSnapshot {
  PoseGraphState graph;
  std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map;
  std::shared_ptr<const hybrid_slam::mapping::SubmapDescriptorIndex> index;
};

// Candidate retrieval only. It cannot mutate PoseGraph or perform geometric
// registration. Calls occur at loop-task frequency outside the frontend.
class LoopDetector {
 public:
  virtual ~LoopDetector() = default;
  [[nodiscard]] virtual std::vector<LoopCandidate> detect(
      const LoopDetectionSnapshot& snapshot,
      hybrid_slam::common::NodeId query_node) = 0;
};

struct LoopVerificationInput {
  LoopCandidate candidate;
  hybrid_slam::common::Timestamp timestamp;
  hybrid_slam::sensor::ImmutablePointCloud query_cloud_base;
  hybrid_slam::sensor::ImmutablePointCloud match_cloud_base;
  hybrid_slam::geometry::RigidTransform3d initial_T_match_query;
};

// Registration verification only. A successful result is a measurement;
// BackendScheduler/ConstraintBuilder remain responsible for graph mutation.
class LoopVerifier {
 public:
  virtual ~LoopVerifier() = default;
  // Called once per worker item before detection and verification. Adapters
  // may retain the immutable snapshot to materialize richer registration
  // geometry without making the scheduler depend on a point-cloud backend.
  virtual void prepare(const LoopDetectionSnapshot&) {}
  [[nodiscard]] virtual std::optional<RelativePoseMeasurement> verify(
      const LoopVerificationInput& input) = 0;
};

class DisabledLoopDetector final : public LoopDetector {
 public:
  [[nodiscard]] std::vector<LoopCandidate> detect(
      const LoopDetectionSnapshot&,
      hybrid_slam::common::NodeId) override {
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

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_LOOP_CLOSURE_H_
