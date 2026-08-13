#include "adagraph/localization/localization_map.h"

#include <cstddef>
#include <memory>
#include <utility>

#include "adagraph/mapping/snapshot_products.h"

namespace adagraph::localization {

std::optional<LocalizationMap> BuildLocalizationMap(
    const adagraph::mapping::MapSnapshot& snapshot) {
  auto derived =
      adagraph::mapping::SnapshotProductBuilder::BuildPointCloudMap(snapshot);
  if (!derived || derived->points.empty()) {
    return std::nullopt;
  }

  using Storage = std::vector<adagraph::mapping::MapPoint3f>;
  auto storage = std::make_shared<const Storage>(std::move(derived->points));
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(adagraph::mapping::MapPoint3f),
      offsetof(adagraph::mapping::MapPoint3f, x),
      offsetof(adagraph::mapping::MapPoint3f, y),
      offsetof(adagraph::mapping::MapPoint3f, z),
      adagraph::geometry::FrameId::kMap);
  LocalizationMap map(
      derived->snapshot_version,
      adagraph::sensor::ImmutablePointCloud::Share(storage, view));
  if (!map.valid()) {
    return std::nullopt;
  }
  return map;
}

}  // namespace adagraph::localization
