#ifndef LI2SUP_BEVPLACE2_LOOP_DETECTOR_H_
#define LI2SUP_BEVPLACE2_LOOP_DETECTOR_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hybrid_slam/backend/loop_closure.h"
#include "lio/scan_context_loop_detector.h"

namespace LI2Sup {

// BEVPlace++ is hosted by a local GPU sidecar. Keeping inference outside the
// process prevents Python/CUDA work from sharing the Point-LIO callback path.
struct BevPlace2Config {
  std::string socket_path = "/tmp/adagraph_bevplace2.sock";
  std::size_t image_size = 201;
  double voxel_size_m = 0.4;
  double half_extent_m = 40.0;
  std::size_t descriptor_dimension = 8192;
  int request_timeout_ms = 2000;
  std::size_t minimum_pose_inliers = 16;
  double maximum_pose_residual_m = 1.0;
  double maximum_registration_initial_translation_m = 100.0;
  // A single appearance/geometric match can still be a repeated-road alias.
  // Require the same place relation to recur on neighboring completed
  // submaps before forwarding it to expensive 3D verification.
  std::size_t minimum_sequence_consensus = 2;
  std::size_t maximum_sequence_gap_submaps = 2;
  // A failed or sub-threshold local-feature pose must not break an already
  // established place sequence.  Requiring more evidence than the normal
  // sequence gate lets a descriptor-consistent continuation reach the strict
  // 3D verifier without treating its missing 2D pose as an accepted coarse
  // transform.  The final edge still has to pass every submap-geometry gate.
  std::size_t minimum_pose_free_sequence_consensus = 3;
  // A completed submap is represented by several raw LiDAR sweeps so that a
  // revisit from the opposite lane is not judged from one unlucky viewpoint.
  // The first/middle/last sweep pattern keeps the BEVPlace++ input in the
  // single-sweep domain on which the network was trained.
  std::size_t representative_keyframes_per_submap = 3;
  // Also index the completed submap geometry.  This accumulated view exposes
  // static roadside structure shared by outbound and return lanes even when
  // no individual sweep sees enough of it for reliable retrieval.
  bool use_submap_geometry_descriptor = true;

  [[nodiscard]] std::optional<std::string> validate() const;
};

struct BevPlace2SequenceObservation {
  hybrid_slam::common::SessionId query_session_id;
  hybrid_slam::common::TrajectoryId query_trajectory_id;
  hybrid_slam::common::SubmapId query_submap_id;
  hybrid_slam::common::SessionId match_session_id;
  hybrid_slam::common::TrajectoryId match_trajectory_id;
  hybrid_slam::common::SubmapId match_submap_id;
};

// Returns true when two observations describe adjacent progress through the
// same query/match submap sequences.  The match sequence may run in either
// direction, but its stride must equal the query stride.
[[nodiscard]] bool IsBevPlace2SequenceConsistent(
    const BevPlace2SequenceObservation& current,
    const BevPlace2SequenceObservation& previous,
    std::size_t maximum_gap_submaps);

// Counts the current observation plus distinct recent observations whose
// query and match submap indices advance by the same magnitude. Both
// same-direction and reverse-direction revisits are supported.
[[nodiscard]] std::size_t CountBevPlace2SequenceConsensus(
    const BevPlace2SequenceObservation& current,
    const std::vector<BevPlace2SequenceObservation>& history,
    std::size_t maximum_gap_submaps);

// Returns true only for a descriptor pair that continues enough independently
// pose-qualified neighboring submap observations to be sent to 3D validation
// without a BEV coarse pose.
[[nodiscard]] bool ShouldVerifyBevPlace2WithoutCoarsePose(
    const BevPlace2SequenceObservation& current,
    const std::vector<BevPlace2SequenceObservation>& pose_history,
    std::size_t maximum_gap_submaps,
    std::size_t minimum_pose_free_sequence_consensus);

class BevPlace2LoopDetector final : public hybrid_slam::backend::LoopDetector {
 public:
  struct Config {
    BevPlace2Config bevplace2;
    ScanContextConfig yaw_initializer;
    std::size_t maximum_candidates = 5;
    double minimum_temporal_separation_s = 20.0;
  };

  explicit BevPlace2LoopDetector(Config config);

  [[nodiscard]] std::vector<hybrid_slam::backend::LoopCandidate> detect(
      const hybrid_slam::backend::LoopDetectionSnapshot& snapshot,
      hybrid_slam::common::NodeId query_node) override;

 private:
  struct DescriptorEntry {
    hybrid_slam::common::SessionId session_id;
    hybrid_slam::common::TrajectoryId trajectory_id;
    hybrid_slam::common::SubmapId submap_id;
    // Sweep entries keep BEVPlace++ in its training domain. Geometry entries
    // use the immutable rolling-submap frame and therefore point at that
    // geometry's anchor node.
    hybrid_slam::common::NodeId representative_node;
    bool submap_geometry_view = false;
    double timestamp_s = 0.0;
    std::vector<float> descriptor;
    std::vector<std::uint8_t> bev_image;
  };

  struct PendingSequenceCandidate {
    BevPlace2SequenceObservation observation;
    hybrid_slam::backend::LoopCandidate candidate;
  };

  [[nodiscard]] bool rebuildIfNeeded(
      const hybrid_slam::backend::LoopDetectionSnapshot& snapshot);

  Config config_;
  std::uint64_t cached_map_version_ =
      std::numeric_limits<std::uint64_t>::max();
  std::vector<DescriptorEntry> entries_;
  // Only observations with a BEV local-feature pose that passed its own
  // inlier/residual checks enter this history.  Pose-free continuations cannot
  // recursively confirm more pose-free continuations.
  std::vector<BevPlace2SequenceObservation> sequence_history_;
  std::vector<PendingSequenceCandidate> pending_sequence_candidates_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_BEVPLACE2_LOOP_DETECTOR_H_
