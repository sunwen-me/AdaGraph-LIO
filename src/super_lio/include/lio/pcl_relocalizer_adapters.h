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

#include "adagraph/localization/relocalizer.h"
#include "adagraph/localization/relocalization_config.h"
#include "basic/alias.h"
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

// Transitional concrete adapters around the existing, validated PCL/KISS
// implementation. They recover immutable PCL owners from the core cloud
// handles and perform no architecture-induced point-cloud copy.
class PclKissCandidateRetriever final
    : public adagraph::localization::CandidateRetriever {
 public:
  PclKissCandidateRetriever() = default;
  explicit PclKissCandidateRetriever(
      KISSMatcherRelocalizer::Config config,
      PclRelocalizationMemoryCallback memory_callback = {})
      : matcher_(std::move(config)),
        memory_callback_(std::move(memory_callback)) {}

  [[nodiscard]] std::vector<adagraph::localization::RelocalizationCandidate>
  Retrieve(const adagraph::localization::RelocalizationQuery& query,
           const adagraph::localization::LocalizationMap& map,
           std::stop_token stop_token) override;

 private:
  KISSMatcherRelocalizer matcher_;
  PclRelocalizationMemoryCallback memory_callback_;
};

class PclRegistrationVerifier final
    : public adagraph::localization::RegistrationVerifier {
 public:
  using TargetSelector = std::function<BASIC::PointCloudType::ConstPtr(
      const BASIC::SE3& initial_T_map_base)>;

  PclRegistrationVerifier(
      adagraph::localization::RelocalizationConfig config,
      TargetSelector target_selector,
      PclRelocalizationMemoryCallback memory_callback = {});

  [[nodiscard]] std::optional<
      adagraph::localization::RegistrationVerification>
  Verify(const adagraph::localization::RelocalizationQuery& query,
         const adagraph::localization::LocalizationMap& map,
         const adagraph::localization::RelocalizationCandidate& candidate,
         std::stop_token stop_token) override;

 private:
  adagraph::localization::RelocalizationConfig config_;
  TargetSelector target_selector_;
  PclRelocalizationMemoryCallback memory_callback_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_
