#ifndef ADAGRAPH_LOCALIZATION_COMPOSED_RELOCALIZER_H_
#define ADAGRAPH_LOCALIZATION_COMPOSED_RELOCALIZER_H_

#include <memory>

#include "adagraph/localization/relocalizer.h"

namespace adagraph::localization {

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

}  // namespace adagraph::localization

#endif  // ADAGRAPH_LOCALIZATION_COMPOSED_RELOCALIZER_H_
