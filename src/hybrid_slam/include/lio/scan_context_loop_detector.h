#ifndef LI2SUP_SCAN_CONTEXT_LOOP_DETECTOR_H_
#define LI2SUP_SCAN_CONTEXT_LOOP_DETECTOR_H_

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "hybrid_slam/backend/loop_closure.h"

namespace LI2Sup {

struct ScanContextConfig {
  std::size_t ring_count = 20;
  std::size_t sector_count = 60;
  std::size_t ring_key_candidates = 10;
  double max_radius_m = 80.0;
  // Bound the geometric initializer after global place recognition. This is
  // intentionally separate from a local-only plugin's capture domain.
  double maximum_registration_initial_translation_m = 100.0;

  [[nodiscard]] std::optional<std::string> validate() const;
};

// Estimates the planar rotation from query coordinates into candidate
// coordinates. Descriptor retrieval can remain independent from Scan Context
// while reusing this inexpensive geometric yaw initializer before VGICP.
[[nodiscard]] std::optional<double> EstimateScanContextYaw(
    const hybrid_slam::sensor::ImmutablePointCloud& query,
    const hybrid_slam::sensor::ImmutablePointCloud& candidate,
    const ScanContextConfig& config);

// Independent implementation of the Scan Context idea: polar max-height
// descriptors, ring-key shortlist, and circular sector matching for yaw.
class ScanContextLoopDetector final : public hybrid_slam::backend::LoopDetector {
 public:
  struct Config {
    ScanContextConfig scan_context;
    std::size_t maximum_candidates = 5;
    double minimum_temporal_separation_s = 20.0;
  };

  explicit ScanContextLoopDetector(Config config);

  [[nodiscard]] std::vector<hybrid_slam::backend::LoopCandidate> detect(
      const hybrid_slam::backend::LoopDetectionSnapshot& snapshot,
      hybrid_slam::common::NodeId query_node) override;

 private:
  struct DescriptorEntry {
    hybrid_slam::common::SessionId session_id;
    hybrid_slam::common::TrajectoryId trajectory_id;
    hybrid_slam::common::SubmapId submap_id;
    hybrid_slam::common::NodeId anchor_node;
    double timestamp_s = 0.0;
    std::vector<float> descriptor;
    std::vector<float> ring_key;
  };

  void rebuildIfNeeded(const hybrid_slam::backend::LoopDetectionSnapshot& snapshot);

  Config config_;
  std::uint64_t cached_map_version_ =
      std::numeric_limits<std::uint64_t>::max();
  std::vector<DescriptorEntry> entries_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SCAN_CONTEXT_LOOP_DETECTOR_H_
