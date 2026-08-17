#ifndef HYBRID_SLAM_LOCALIZATION_RELOCALIZER_H_
#define HYBRID_SLAM_LOCALIZATION_RELOCALIZER_H_

#include <optional>
#include <cmath>
#include <stop_token>
#include <vector>

#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/localization/localization_map.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"

namespace hybrid_slam::localization {

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
  hybrid_slam::geometry::StampedTransform3d T_odom_base;
  hybrid_slam::sensor::ImmutablePointCloud current_scan_base;
  std::optional<hybrid_slam::geometry::StampedTransform3d> prior_T_map_base;

  [[nodiscard]] bool valid() const noexcept {
    if (!T_odom_base.valid() ||
        T_odom_base.target_frame != hybrid_slam::geometry::FrameId::kOdom ||
        T_odom_base.source_frame != hybrid_slam::geometry::FrameId::kBase ||
        !current_scan_base.valid() || current_scan_base.empty() ||
        current_scan_base.frame() != hybrid_slam::geometry::FrameId::kBase) {
      return false;
    }
    return !prior_T_map_base ||
           (prior_T_map_base->valid() &&
            prior_T_map_base->timestamp == T_odom_base.timestamp &&
            prior_T_map_base->target_frame ==
                hybrid_slam::geometry::FrameId::kMap &&
            prior_T_map_base->source_frame ==
                hybrid_slam::geometry::FrameId::kBase);
  }
};

struct RelocalizationCandidate {
  hybrid_slam::geometry::StampedTransform3d initial_T_map_base;
  double retrieval_score = 0.0;  // Larger is better; retriever-specific.
};

struct RegistrationVerification {
  hybrid_slam::geometry::StampedTransform3d T_map_base;
  double fitness = 0.0;  // Mean squared correspondence distance; lower is better.
  double inlier_ratio = 0.0;
  Confidence confidence = Confidence::kUnknown;
  // Optional geometric evidence. Existing generic relocalizers may leave
  // these at their defaults; submap recovery requires both explicitly.
  double overlap = 0.0;
  bool observable = false;
};

struct RelocalizationResult {
  RelocalizationStatus status = RelocalizationStatus::kInvalidInput;
  std::optional<hybrid_slam::geometry::StampedTransform3d> T_map_base;
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
           T_map_base->target_frame == hybrid_slam::geometry::FrameId::kMap &&
           T_map_base->source_frame == hybrid_slam::geometry::FrameId::kBase &&
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

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_RELOCALIZER_H_
