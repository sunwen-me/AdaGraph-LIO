#ifndef ADAGRAPH_LOCALIZATION_LOCALIZATION_MAP_H_
#define ADAGRAPH_LOCALIZATION_LOCALIZATION_MAP_H_

#include <cstdint>
#include <optional>
#include <utility>

#include "adagraph/mapping/map_snapshot.h"
#include "adagraph/sensor/immutable_point_cloud.h"

namespace adagraph::localization {

// Immutable map-frame registration target. It may originate from a loaded
// serialized map or a committed MapSnapshot; relocalizers do not own map IO.
class LocalizationMap {
 public:
  LocalizationMap() = default;
  LocalizationMap(std::uint64_t version,
                  adagraph::sensor::ImmutablePointCloud cloud_map) noexcept
      : version_(version), cloud_map_(std::move(cloud_map)) {}

  [[nodiscard]] bool valid() const noexcept {
    return cloud_map_.valid() && !cloud_map_.empty() &&
           cloud_map_.frame() == adagraph::geometry::FrameId::kMap;
  }
  [[nodiscard]] std::uint64_t version() const noexcept { return version_; }
  [[nodiscard]] const adagraph::sensor::ImmutablePointCloud& cloudMap()
      const noexcept {
    return cloud_map_;
  }

 private:
  std::uint64_t version_ = 0;
  adagraph::sensor::ImmutablePointCloud cloud_map_;
};

// Explicitly materializes a registration map outside realtime paths. The
// resulting point buffer is immutable and shared by candidate/verifier stages.
[[nodiscard]] std::optional<LocalizationMap> BuildLocalizationMap(
    const adagraph::mapping::MapSnapshot& snapshot);

}  // namespace adagraph::localization

#endif  // ADAGRAPH_LOCALIZATION_LOCALIZATION_MAP_H_
