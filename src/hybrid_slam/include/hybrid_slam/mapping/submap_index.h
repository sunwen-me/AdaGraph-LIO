#ifndef HYBRID_SLAM_MAPPING_SUBMAP_INDEX_H_
#define HYBRID_SLAM_MAPPING_SUBMAP_INDEX_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "hybrid_slam/mapping/map_snapshot.h"

namespace hybrid_slam::mapping {

struct SubmapIndexCandidate {
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::common::SubmapId submap_id;
  double score = 0.0;  // cosine similarity; larger is better
};

// Immutable-snapshot descriptor index. It owns only small descriptor metadata;
// clouds remain owned by MapSnapshot and are materialized by registration only
// for returned candidates.
class SubmapDescriptorIndex {
 public:
  void rebuild(const MapSnapshot& snapshot);

  [[nodiscard]] std::uint64_t mapVersion() const noexcept {
    return map_version_;
  }
  [[nodiscard]] std::size_t size() const noexcept { return entries_.size(); }

  [[nodiscard]] std::vector<SubmapIndexCandidate> query(
      const std::vector<float>& descriptor, std::size_t maximum_candidates) const;

  // The initial descriptor is intentionally simple and rotation-independent.
  // It is a coarse shortlist mechanism only; geometric registration must
  // validate every candidate before any recovery action.
  [[nodiscard]] static std::vector<float> buildRadialDescriptor(
      const hybrid_slam::sensor::ImmutablePointCloud& cloud,
      std::size_t radial_bins);

 private:
  struct Entry {
    hybrid_slam::common::SessionId session_id;
    hybrid_slam::common::TrajectoryId trajectory_id;
    hybrid_slam::common::SubmapId submap_id;
    std::vector<float> descriptor;
    double norm = 0.0;
  };

  std::uint64_t map_version_ = 0;
  std::vector<Entry> entries_;
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_SUBMAP_INDEX_H_
