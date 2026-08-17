#ifndef LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_
#define LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_

#include <cstddef>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stop_token>
#include <utility>
#include <vector>

#include "hybrid_slam/localization/relocalizer.h"
#include "hybrid_slam/localization/relocalization_config.h"
#include "hybrid_slam/adapter/pcl_types.h"
#include "lio/kiss_matcher_relocalizer.h"

namespace LI2Sup {

// Explicit PCL clouds owned by the adapters while a query is running. This
// excludes allocation internal to PCL/FastVGICP; RSS remains the authority for
// total process memory.
struct PclRelocalizationTemporaryMemory {
  enum class Stage {
    kKiss,
    kRegistration,
  };

  Stage stage = Stage::kRegistration;
  std::size_t source_odom_backing_bytes = 0;
  std::size_t registration_aligned_backing_bytes = 0;
  RuntimeMemorySample runtime;
};

using PclRelocalizationMemoryCallback = std::function<void(
    const PclRelocalizationTemporaryMemory&)>;

// Borrowed, synchronous registration diagnostics. Cloud references passed to
// the callback remain valid only for the duration of the callback. This keeps
// loop-closure debug export out of the generic registration implementation
// without retaining another copy of a large submap cloud.
struct PclRegistrationDiagnostics {
  hybrid_slam::geometry::RigidTransform3d initial_T_target_source;
  hybrid_slam::geometry::RigidTransform3d final_T_target_source;
  std::size_t source_points = 0;
  std::size_t target_points = 0;
  std::size_t coarse_source_points = 0;
  std::size_t coarse_target_points = 0;
  bool coarse_attempted = false;
  bool coarse_converged = false;
  bool converged = false;
  double coarse_fitness = std::numeric_limits<double>::infinity();
  double fitness = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  double overlap = 0.0;
  bool observable = false;
};

using PclRegistrationDiagnosticsCallback = std::function<void(
    const PclRegistrationDiagnostics&,
    const hybrid_slam::adapter::PclPointCloud& source,
    const hybrid_slam::adapter::PclPointCloud& target,
    const hybrid_slam::adapter::PclPointCloud& aligned_source)>;

// Transitional concrete adapters around the existing, validated PCL/KISS
// implementation. They recover immutable PCL owners from the core cloud
// handles and perform no architecture-induced point-cloud copy.
class PclKissCandidateRetriever final
    : public hybrid_slam::localization::CandidateRetriever {
 public:
  PclKissCandidateRetriever() = default;
  explicit PclKissCandidateRetriever(
      KISSMatcherRelocalizer::Config config,
      PclRelocalizationMemoryCallback memory_callback = {})
      : matcher_(std::move(config)),
        memory_callback_(std::move(memory_callback)) {}

  [[nodiscard]] std::vector<hybrid_slam::localization::RelocalizationCandidate>
  Retrieve(const hybrid_slam::localization::RelocalizationQuery& query,
           const hybrid_slam::localization::LocalizationMap& map,
           std::stop_token stop_token) override;

 private:
  KISSMatcherRelocalizer matcher_;
  PclRelocalizationMemoryCallback memory_callback_;
};

class PclRegistrationVerifier final
    : public hybrid_slam::localization::RegistrationVerifier {
 public:
  using TargetSelector = std::function<hybrid_slam::adapter::PclPointCloud::ConstPtr(
      const BASIC::SE3& initial_T_map_base)>;

  PclRegistrationVerifier(
      hybrid_slam::localization::RelocalizationConfig config,
      TargetSelector target_selector,
      PclRelocalizationMemoryCallback memory_callback = {},
      PclRegistrationDiagnosticsCallback diagnostics_callback = {});

  [[nodiscard]] std::optional<
      hybrid_slam::localization::RegistrationVerification>
  Verify(const hybrid_slam::localization::RelocalizationQuery& query,
         const hybrid_slam::localization::LocalizationMap& map,
         const hybrid_slam::localization::RelocalizationCandidate& candidate,
         std::stop_token stop_token) override;

 private:
  hybrid_slam::localization::RelocalizationConfig config_;
  TargetSelector target_selector_;
  PclRelocalizationMemoryCallback memory_callback_;
  PclRegistrationDiagnosticsCallback diagnostics_callback_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_
