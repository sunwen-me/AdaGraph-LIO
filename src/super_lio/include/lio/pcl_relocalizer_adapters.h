#ifndef LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_
#define LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_

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

// Transitional concrete adapters around the existing, validated PCL/KISS
// implementation. They recover immutable PCL owners from the core cloud
// handles and perform no architecture-induced point-cloud copy.
class PclKissCandidateRetriever final
    : public adagraph::localization::CandidateRetriever {
 public:
  PclKissCandidateRetriever() = default;
  explicit PclKissCandidateRetriever(
      KISSMatcherRelocalizer::Config config)
      : matcher_(std::move(config)) {}

  [[nodiscard]] std::vector<adagraph::localization::RelocalizationCandidate>
  Retrieve(const adagraph::localization::RelocalizationQuery& query,
           const adagraph::localization::LocalizationMap& map,
           std::stop_token stop_token) override;

 private:
  KISSMatcherRelocalizer matcher_;
};

class PclRegistrationVerifier final
    : public adagraph::localization::RegistrationVerifier {
 public:
  using TargetSelector = std::function<BASIC::PointCloudType::ConstPtr(
      const BASIC::SE3& initial_T_map_base)>;

  PclRegistrationVerifier(
      adagraph::localization::RelocalizationConfig config,
      TargetSelector target_selector);

  [[nodiscard]] std::optional<
      adagraph::localization::RegistrationVerification>
  Verify(const adagraph::localization::RelocalizationQuery& query,
         const adagraph::localization::LocalizationMap& map,
         const adagraph::localization::RelocalizationCandidate& candidate,
         std::stop_token stop_token) override;

 private:
  adagraph::localization::RelocalizationConfig config_;
  TargetSelector target_selector_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_RELOCALIZER_ADAPTERS_H_
