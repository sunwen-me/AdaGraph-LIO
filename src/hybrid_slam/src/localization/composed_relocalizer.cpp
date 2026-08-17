#include "hybrid_slam/localization/composed_relocalizer.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace hybrid_slam::localization {
namespace {

bool ValidCandidate(const RelocalizationCandidate& candidate,
                    const RelocalizationQuery& query) {
  return candidate.initial_T_map_base.valid() &&
         candidate.initial_T_map_base.timestamp ==
             query.T_odom_base.timestamp &&
         candidate.initial_T_map_base.target_frame ==
             hybrid_slam::geometry::FrameId::kMap &&
         candidate.initial_T_map_base.source_frame ==
             hybrid_slam::geometry::FrameId::kBase &&
         std::isfinite(candidate.retrieval_score);
}

bool ValidVerification(const RegistrationVerification& verification,
                       const RelocalizationQuery& query) {
  return verification.T_map_base.valid() &&
         verification.T_map_base.timestamp == query.T_odom_base.timestamp &&
         verification.T_map_base.target_frame ==
             hybrid_slam::geometry::FrameId::kMap &&
         verification.T_map_base.source_frame ==
             hybrid_slam::geometry::FrameId::kBase &&
         std::isfinite(verification.fitness) &&
         verification.fitness >= 0.0 &&
         std::isfinite(verification.inlier_ratio) &&
         verification.inlier_ratio >= 0.0 &&
         verification.inlier_ratio <= 1.0;
}

}  // namespace

ComposedRelocalizer::ComposedRelocalizer(
    std::unique_ptr<CandidateRetriever> retriever,
    std::unique_ptr<RegistrationVerifier> verifier)
    : retriever_(std::move(retriever)), verifier_(std::move(verifier)) {
  if (!retriever_ || !verifier_) {
    throw std::invalid_argument(
        "ComposedRelocalizer requires retriever and verifier");
  }
}

RelocalizationResult ComposedRelocalizer::Relocalize(
    const RelocalizationQuery& query,
    const LocalizationMap& map,
    std::stop_token stop_token) {
  RelocalizationResult result;
  if (!query.valid() || !map.valid()) {
    result.status = RelocalizationStatus::kInvalidInput;
    return result;
  }
  if (stop_token.stop_requested()) {
    result.status = RelocalizationStatus::kCancelled;
    return result;
  }

  try {
    const auto candidates = retriever_->Retrieve(query, map, stop_token);
    if (stop_token.stop_requested()) {
      result.status = RelocalizationStatus::kCancelled;
      return result;
    }
    bool had_valid_candidate = false;
    std::optional<RegistrationVerification> best;
    for (const auto& candidate : candidates) {
      if (stop_token.stop_requested()) {
        result.status = RelocalizationStatus::kCancelled;
        return result;
      }
      if (!ValidCandidate(candidate, query)) {
        continue;
      }
      had_valid_candidate = true;
      auto verification =
          verifier_->Verify(query, map, candidate, stop_token);
      if (!verification || !ValidVerification(*verification, query)) {
        continue;
      }
      if (!best || verification->fitness < best->fitness) {
        best = std::move(verification);
      }
    }
    if (!best) {
      result.status = had_valid_candidate
                          ? RelocalizationStatus::kRegistrationRejected
                          : RelocalizationStatus::kNoCandidate;
      return result;
    }
    result.status = RelocalizationStatus::kSuccess;
    result.T_map_base = best->T_map_base;
    result.fitness = best->fitness;
    result.inlier_ratio = best->inlier_ratio;
    result.confidence = best->confidence;
    return result;
  } catch (...) {
    result.status = RelocalizationStatus::kInternalError;
    return result;
  }
}

}  // namespace hybrid_slam::localization
