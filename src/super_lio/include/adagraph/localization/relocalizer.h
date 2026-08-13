#ifndef ADAGRAPH_LOCALIZATION_RELOCALIZER_H_
#define ADAGRAPH_LOCALIZATION_RELOCALIZER_H_

#include <optional>
#include <cmath>
#include <stop_token>
#include <vector>

#include "adagraph/geometry/transform3.h"
#include "adagraph/localization/localization_map.h"
#include "adagraph/sensor/immutable_point_cloud.h"

namespace adagraph::localization {

enum class RelocalizationStatus {
  kSuccess = 0,
  kNoCandidate,
  kRegistrationRejected,
  kCancelled,
  kInvalidInput,
  kInternalError,
};

enum class Confidence {
  kUnknown = 0,
  kLow,
  kMedium,
  kHigh,
};

struct RelocalizationQuery {
  adagraph::geometry::StampedTransform3d T_odom_base;
  adagraph::sensor::ImmutablePointCloud current_scan_base;
  std::optional<adagraph::geometry::StampedTransform3d> prior_T_map_base;

  [[nodiscard]] bool valid() const noexcept {
    if (!T_odom_base.valid() ||
        T_odom_base.target_frame != adagraph::geometry::FrameId::kOdom ||
        T_odom_base.source_frame != adagraph::geometry::FrameId::kBase ||
        !current_scan_base.valid() || current_scan_base.empty() ||
        current_scan_base.frame() != adagraph::geometry::FrameId::kBase) {
      return false;
    }
    return !prior_T_map_base ||
           (prior_T_map_base->valid() &&
            prior_T_map_base->timestamp == T_odom_base.timestamp &&
            prior_T_map_base->target_frame ==
                adagraph::geometry::FrameId::kMap &&
            prior_T_map_base->source_frame ==
                adagraph::geometry::FrameId::kBase);
  }
};

struct RelocalizationCandidate {
  adagraph::geometry::StampedTransform3d initial_T_map_base;
  double retrieval_score = 0.0;  // Larger is better; retriever-specific.
};

struct RegistrationVerification {
  adagraph::geometry::StampedTransform3d T_map_base;
  double fitness = 0.0;  // Mean squared correspondence distance; lower is better.
  double inlier_ratio = 0.0;
  Confidence confidence = Confidence::kUnknown;
};

struct RelocalizationResult {
  RelocalizationStatus status = RelocalizationStatus::kInvalidInput;
  std::optional<adagraph::geometry::StampedTransform3d> T_map_base;
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  Confidence confidence = Confidence::kUnknown;

  [[nodiscard]] bool success() const noexcept {
    return status == RelocalizationStatus::kSuccess && T_map_base.has_value();
  }
  [[nodiscard]] bool validFor(
      const RelocalizationQuery& query) const noexcept {
    if (!query.valid()) {
      return false;
    }
    if (status != RelocalizationStatus::kSuccess) {
      return !T_map_base.has_value();
    }
    return success() && T_map_base->valid() &&
           T_map_base->timestamp == query.T_odom_base.timestamp &&
           T_map_base->target_frame == adagraph::geometry::FrameId::kMap &&
           T_map_base->source_frame == adagraph::geometry::FrameId::kBase &&
           std::isfinite(fitness) && fitness >= 0.0 &&
           std::isfinite(inlier_ratio) && inlier_ratio >= 0.0 &&
           inlier_ratio <= 1.0;
  }
};

// Coarse per-query seams only. Concrete KISS/VGICP adapters may remain direct
// composition; there is no runtime plugin registry and no per-point dispatch.
class CandidateRetriever {
 public:
  virtual ~CandidateRetriever() = default;
  [[nodiscard]] virtual std::vector<RelocalizationCandidate> Retrieve(
      const RelocalizationQuery& query,
      const LocalizationMap& map,
      std::stop_token stop_token) = 0;
};

class RegistrationVerifier {
 public:
  virtual ~RegistrationVerifier() = default;
  [[nodiscard]] virtual std::optional<RegistrationVerification> Verify(
      const RelocalizationQuery& query,
      const LocalizationMap& map,
      const RelocalizationCandidate& candidate,
      std::stop_token stop_token) = 0;
};

class Relocalizer {
 public:
  virtual ~Relocalizer() = default;
  [[nodiscard]] virtual RelocalizationResult Relocalize(
      const RelocalizationQuery& query,
      const LocalizationMap& map,
      std::stop_token stop_token) = 0;
};

}  // namespace adagraph::localization

#endif  // ADAGRAPH_LOCALIZATION_RELOCALIZER_H_
