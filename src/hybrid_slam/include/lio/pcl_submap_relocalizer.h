#ifndef LI2SUP_PCL_SUBMAP_RELOCALIZER_H_
#define LI2SUP_PCL_SUBMAP_RELOCALIZER_H_

#include <cstddef>
#include <functional>
#include <optional>
#include <stop_token>
#include <vector>

#include "hybrid_slam/localization/submap_relocalization.h"
#include "hybrid_slam/mapping/submap_index.h"
#include "lio/pcl_relocalizer_adapters.h"

namespace LI2Sup {

// Recovery owns no serialized map-file lifecycle. It searches the immutable
// finished-submap map globally, then associates a verified pose with a local
// submap only for frontend seeding.
struct PclSubmapRelocalizationConfig {
  hybrid_slam::localization::RelocalizationConfig registration;
  KISSMatcherRelocalizer::Config kiss;
  // Number of fresh LiDAR frames accumulated in the IMU-propagated odom
  // frame before starting submap retrieval and geometric verification.
  std::size_t query_window_frames = 4;
  double map_odom_prior_max_age_seconds = 10.0;
  // Include this many neighboring submaps on either side of the KISS-selected
  // submap when building the final registration target.
  std::size_t neighbor_submaps = 1;
  double minimum_inlier_ratio = 0.20;
  double minimum_overlap = 0.10;

  [[nodiscard]] std::optional<std::string> validate() const;
};

// Test seams deliberately operate on the typed generic relocalization
// contracts. Production leaves them empty and uses the PCL/KISS/GICP path.
struct PclSubmapRelocalizerOperations {
  using CandidateRetriever = std::function<std::vector<
      hybrid_slam::localization::RelocalizationCandidate>(
      const hybrid_slam::localization::RelocalizationQuery&,
      const hybrid_slam::localization::LocalizationMap&, std::stop_token)>;
  using RegistrationVerifier = std::function<std::optional<
      hybrid_slam::localization::RegistrationVerification>(
      const hybrid_slam::localization::RelocalizationQuery&,
      const hybrid_slam::localization::LocalizationMap&,
      const hybrid_slam::localization::RelocalizationCandidate&,
      std::stop_token)>;

  CandidateRetriever retrieve;
  RegistrationVerifier verify;
};

// PCL-only recovery implementation. KISS searches the immutable global map;
// VGICP receives a candidate-centered crop and the final pose is associated
// with the nearest finished submap for seeding.
class PclSubmapRelocalizer final {
 public:
  explicit PclSubmapRelocalizer(
      PclSubmapRelocalizationConfig config,
      PclSubmapRelocalizerOperations operations = {});

  [[nodiscard]] std::optional<
      hybrid_slam::localization::SubmapRelocalizationResult>
  Relocalize(const hybrid_slam::localization::RelocalizationQuery& query,
             const hybrid_slam::mapping::MapSnapshot& map,
             const hybrid_slam::mapping::SubmapDescriptorIndex& index,
             std::stop_token stop_token = {}) const;

 private:
  PclSubmapRelocalizationConfig config_;
  PclSubmapRelocalizerOperations operations_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_SUBMAP_RELOCALIZER_H_
