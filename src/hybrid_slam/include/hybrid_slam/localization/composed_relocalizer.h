#ifndef HYBRID_SLAM_LOCALIZATION_COMPOSED_RELOCALIZER_H_
#define HYBRID_SLAM_LOCALIZATION_COMPOSED_RELOCALIZER_H_

#include <memory>

#include "hybrid_slam/localization/relocalizer.h"

namespace hybrid_slam::localization {

// Small deterministic composition seam: retrieval proposes initial poses;
// verification owns registration/quality gating; this class selects the
// accepted verification with lowest fitness.
class ComposedRelocalizer final : public Relocalizer {
 public:
  ComposedRelocalizer(std::unique_ptr<CandidateRetriever> retriever,
                      std::unique_ptr<RegistrationVerifier> verifier);

  [[nodiscard]] RelocalizationResult Relocalize(
      const RelocalizationQuery& query,
      const LocalizationMap& map,
      std::stop_token stop_token) override;

 private:
  std::unique_ptr<CandidateRetriever> retriever_;
  std::unique_ptr<RegistrationVerifier> verifier_;
};

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_COMPOSED_RELOCALIZER_H_
