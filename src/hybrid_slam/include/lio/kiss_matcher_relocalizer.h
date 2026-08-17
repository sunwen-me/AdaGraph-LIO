#ifndef LI2SUP_KISS_MATCHER_RELOCALIZER_H_
#define LI2SUP_KISS_MATCHER_RELOCALIZER_H_

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "basic/Manifold.h"
#include "hybrid_slam/adapter/pcl_types.h"
#include "lio/runtime_memory.h"

namespace LI2Sup {

struct KISSMatcherCandidate {
  BASIC::SE3 target_source;
  bool used_ratio_test = true;
  bool passes_quality_gates = false;
  std::size_t initial_matches = 0;
  std::size_t rotation_inliers = 0;
  std::size_t final_inliers = 0;
};

class KISSMatcherRelocalizer {
public:
  using MemoryCallback = std::function<void(RuntimeMemorySample)>;

  struct Config {
    bool enabled = false;
    float voxel_size = 0.5F;
    bool use_quatro = false;
    bool use_voxel_sampling = true;
    bool use_ratio_test = true;
    // A failed strict ratio test can be retried with the alternate
    // correspondence policy when a deployment explicitly enables it. Both
    // paths still pass through the registration quality gate.
    bool adaptive_ratio_test = false;
    // The vendored parallel voxel sampler is intentionally non-deterministic.
    // Relocalization is a low-frequency background operation, so make its
    // candidate input stable when a deployment explicitly enables it.
    bool deterministic_voxel_sampling = false;
    // KISS is a proposal generator. A handful of descriptor matches is not
    // enough to seed either startup localization or a frontend restart.
    int min_initial_matches = 12;
    int min_rotation_inliers = 8;
    int min_final_inliers = 5;
    int max_source_points = 30'000;
    int max_target_points = 120'000;
    // Run a second coarser proposal pass. Each pass still has independent
    // KISS quality gates and is verified geometrically downstream.
    bool multi_scale = true;
    float coarse_voxel_scale = 1.5F;
    int max_candidates = 4;
  };

  KISSMatcherRelocalizer() = default;
  explicit KISSMatcherRelocalizer(Config config)
      : config_(std::move(config)) {}

  bool estimate(const hybrid_slam::adapter::PclPointCloud::ConstPtr& source,
                const hybrid_slam::adapter::PclPointCloud::ConstPtr& target,
                BASIC::SE3& target_source) const;

  std::vector<KISSMatcherCandidate> estimateCandidates(
      const hybrid_slam::adapter::PclPointCloud::ConstPtr& source,
      const hybrid_slam::adapter::PclPointCloud::ConstPtr& target,
      MemoryCallback memory_callback = {}) const;

  [[nodiscard]] const Config& config() const noexcept { return config_; }

 private:
  Config config_;
};

}  // namespace LI2Sup

#endif
