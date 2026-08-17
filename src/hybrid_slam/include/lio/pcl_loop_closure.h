#ifndef LI2SUP_PCL_LOOP_CLOSURE_H_
#define LI2SUP_PCL_LOOP_CLOSURE_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "hybrid_slam/backend/loop_closure.h"
#include "hybrid_slam/localization/relocalization_config.h"
#include "lio/bevplace2_loop_detector.h"
#include "lio/pcl_relocalizer_adapters.h"
#include "lio/scan_context_loop_detector.h"

namespace LI2Sup {

struct PclLoopClosureConfig {
  hybrid_slam::localization::RelocalizationConfig registration;
  std::string plugin = "scan_context";
  ScanContextConfig scan_context;
  BevPlace2Config bevplace2;
  std::size_t maximum_candidates = 5;
  // Zero uses exactly the query and match rolling submaps. Positive values
  // additionally concatenate temporal neighbors and should be reserved for
  // legacy non-overlapping maps; the default rolling geometry already has 50%
  // overlap and does not need that extra, alias-prone support.
  std::size_t neighbor_submaps = 0;
  double registration_crop_radius_m = 50.0;
  double minimum_temporal_separation_s = 20.0;
  // Maximum absolute translation for descriptor-produced initializers.  The
  // graph prediction is not subject to this limit: T_match_query can be large
  // simply because the two valid loop endpoints are spatially separated.
  double maximum_initial_translation_m = 15.0;
  double minimum_inlier_ratio = 0.20;
  double minimum_overlap = 0.10;
  // A loop edge must be supported by completed-submap geometry. These limits
  // prevent a sparse or partially materialized submap from being accepted as
  // a global constraint.
  std::size_t minimum_source_points = 1000;
  std::size_t minimum_target_points = 1000;
  // Loop-specific 3D support gates, evaluated after fine registration in both
  // directions. A flat or spatially tiny match is not enough evidence for a
  // six-degree-of-freedom Pose3 factor.
  double minimum_bidirectional_overlap = 0.25;
  // Mutual nearest-neighbor correspondences reject repeated-road aliases that
  // can have acceptable one-way overlap in both directions but do not establish
  // a sufficiently unique point-to-point association.
  double minimum_mutual_inlier_ratio = 0.30;
  // RMSE is evaluated only over symmetric nearest-neighbor inliers inside the
  // fixed fine-registration correspondence gate. Unlike FastVGICP's full-cloud
  // score, it does not punish legitimate non-overlapping neighborhood edges.
  double maximum_bidirectional_inlier_rmse_m = 0.75;
  double minimum_support_extent_m = 15.0;
  double minimum_support_z_extent_m = 2.0;
  double minimum_support_eigenvalue_ratio = 1e-4;
  std::size_t minimum_support_voxels = 20;
  double support_voxel_size_m = 5.0;
  // Appearance/RANSAC is a coarse initializer only. The final 3D solution must
  // remain consistent with its observable planar components.
  double maximum_coarse_translation_disagreement_m = 5.0;
  double maximum_coarse_yaw_disagreement_rad = 0.35;
  // A BEV sequence continuation without a valid local-feature pose is useful
  // only after earlier loop factors have made the graph prediction local.
  // Prevent graph-only VGICP from jumping to a different repeated-road basin.
  double maximum_pose_free_graph_correction_m = 3.0;
  double maximum_pose_free_graph_correction_rad = 0.15;
  double maximum_relative_z_m = 5.0;
  double maximum_relative_roll_pitch_rad = 0.35;
  double maximum_solution_disagreement_m = 1.0;
  double maximum_solution_disagreement_rad = 0.10;
  // Probe the local objective around an appearance initializer. Repeated
  // corridors often expose several geometry-qualified translations; a loop
  // factor is allowed only when multiple perturbed starts return to one basin.
  double uniqueness_probe_translation_m = 8.0;
  std::size_t minimum_uniqueness_probe_consensus = 1;
  // Systematic-error floors for loop-factor uncertainty.  The statistical
  // component is the bidirectional inlier RMSE reduced by the conservative
  // effective count of occupied 3D support voxels; FastVGICP's full-cloud score
  // remains diagnostic only.
  double translation_sigma_m = 0.35;
  double rotation_sigma_rad = 0.10;
  // Empty disables PCD diagnostics. When set, each attempted initializer saves
  // its voxelized source, target and aligned source for direct inspection.
  std::string debug_directory;

  [[nodiscard]] std::optional<std::string> validate() const;
};

// Descriptor-based completed-submap candidate retrieval. It never runs PCL
// registration and never mutates the graph; the scheduler owns the expensive
// follow-up.
class PclSubmapLoopDetector final : public hybrid_slam::backend::LoopDetector {
 public:
  explicit PclSubmapLoopDetector(PclLoopClosureConfig config);

  [[nodiscard]] std::vector<hybrid_slam::backend::LoopCandidate> detect(
      const hybrid_slam::backend::LoopDetectionSnapshot& snapshot,
      hybrid_slam::common::NodeId query_node) override;

 private:
  PclLoopClosureConfig config_;
};

// Reuses PclRegistrationVerifier by treating the matched keyframe's base
// frame as the temporary registration target frame. The returned measurement
// is T_match_base_query_base and is directly suitable for ConstraintBuilder.
class PclLoopClosureVerifier final : public hybrid_slam::backend::LoopVerifier {
 public:
  explicit PclLoopClosureVerifier(PclLoopClosureConfig config)
      : config_(std::move(config)) {}

  void prepare(const hybrid_slam::backend::LoopDetectionSnapshot& snapshot) override;

  [[nodiscard]] std::optional<hybrid_slam::backend::RelativePoseMeasurement>
  verify(const hybrid_slam::backend::LoopVerificationInput& input) override;

 private:
  PclLoopClosureConfig config_;
  std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_LOOP_CLOSURE_H_
