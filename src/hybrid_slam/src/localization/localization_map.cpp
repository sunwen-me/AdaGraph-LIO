#include "hybrid_slam/localization/localization_map.h"

#include <cstddef>
#include <memory>
#include <utility>

#include "hybrid_slam/mapping/snapshot_products.h"

namespace hybrid_slam::localization {

std::optional<LocalizationMap> BuildLocalizationMap(
    const hybrid_slam::mapping::MapSnapshot& snapshot) {
  auto derived =
      hybrid_slam::mapping::SnapshotProductBuilder::BuildPointCloudMap(snapshot);
  if (!derived || derived->points.empty()) {
    return std::nullopt;
  }

  using Storage = std::vector<hybrid_slam::mapping::MapPoint3f>;
  auto storage = std::make_shared<const Storage>(std::move(derived->points));
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::mapping::MapPoint3f),
      offsetof(hybrid_slam::mapping::MapPoint3f, x),
      offsetof(hybrid_slam::mapping::MapPoint3f, y),
      offsetof(hybrid_slam::mapping::MapPoint3f, z),
      hybrid_slam::geometry::FrameId::kMap);
  LocalizationMap map(
      derived->snapshot_version,
      hybrid_slam::sensor::ImmutablePointCloud::Share(storage, view));
  if (!map.valid()) {
    return std::nullopt;
  }
  return map;
}

}  // namespace hybrid_slam::localization
