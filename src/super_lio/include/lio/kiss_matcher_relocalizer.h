#ifndef LI2SUP_KISS_MATCHER_RELOCALIZER_H_
#define LI2SUP_KISS_MATCHER_RELOCALIZER_H_

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "basic/alias.h"
#include "basic/Manifold.h"
#include "lio/runtime_memory.h"

namespace LI2Sup {

struct KISSMatcherCandidate {
  BASIC::SE3 target_source;
  bool used_ratio_test = true;
  std::size_t initial_matches = 0;
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
  // correspondence policy when a deployment explicitly enables it.  Both
  // paths still pass through the unchanged registration quality gate.
  bool adaptive_ratio_test = false;
  // The vendored parallel voxel sampler is intentionally non-deterministic.
  // Relocalization is a low-frequency background operation, so make its
  // candidate input stable when a deployment explicitly enables it.
  bool deterministic_voxel_sampling = false;
    int min_final_inliers = 5;
    int max_source_points = 30'000;
    int max_target_points = 120'000;
  };

  KISSMatcherRelocalizer() = default;
  explicit KISSMatcherRelocalizer(Config config)
      : config_(std::move(config)) {}

  bool estimate(const BASIC::PointCloudType::ConstPtr& source,
                const BASIC::PointCloudType::ConstPtr& target,
                BASIC::SE3& target_source) const;

  std::vector<KISSMatcherCandidate> estimateCandidates(
      const BASIC::PointCloudType::ConstPtr& source,
      const BASIC::PointCloudType::ConstPtr& target,
      MemoryCallback memory_callback = {}) const;

  [[nodiscard]] const Config& config() const noexcept { return config_; }

 private:
  Config config_;
};

}  // namespace LI2Sup

#endif
