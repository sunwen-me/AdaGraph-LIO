#ifndef HYBRID_SLAM_LOCALIZATION_LOCALIZATION_MAP_H_
#define HYBRID_SLAM_LOCALIZATION_LOCALIZATION_MAP_H_

#include <cstdint>
#include <optional>
#include <utility>

#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"

namespace hybrid_slam::localization {

// Immutable map-frame registration target. It may originate from a loaded
// serialized map or a committed MapSnapshot; relocalizers do not own map IO.
class LocalizationMap {
 public:
  LocalizationMap() = default;
  LocalizationMap(std::uint64_t version,
                  hybrid_slam::sensor::ImmutablePointCloud cloud_map) noexcept
      : version_(version), cloud_map_(std::move(cloud_map)) {}

  [[nodiscard]] bool valid() const noexcept {
    return cloud_map_.valid() && !cloud_map_.empty() &&
           cloud_map_.frame() == hybrid_slam::geometry::FrameId::kMap;
  }
  [[nodiscard]] std::uint64_t version() const noexcept { return version_; }
  [[nodiscard]] const hybrid_slam::sensor::ImmutablePointCloud& cloudMap()
      const noexcept {
    return cloud_map_;
  }

 private:
  std::uint64_t version_ = 0;
  hybrid_slam::sensor::ImmutablePointCloud cloud_map_;
};

// Explicitly materializes a registration map outside realtime paths. The
// resulting point buffer is immutable and shared by candidate/verifier stages.
[[nodiscard]] std::optional<LocalizationMap> BuildLocalizationMap(
    const hybrid_slam::mapping::MapSnapshot& snapshot);

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_LOCALIZATION_MAP_H_
