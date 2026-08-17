#include "hybrid_slam/mapping/submap_geometry.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hybrid_slam::mapping {
namespace {

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey&) const = default;
};

struct VoxelKeyHash {
  [[nodiscard]] std::size_t operator()(const VoxelKey& key) const noexcept {
    const auto mix = [](std::uint64_t value) {
      value += 0x9e3779b97f4a7c15ULL;
      value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
      value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
      return value ^ (value >> 31U);
    };
    return static_cast<std::size_t>(
        mix(static_cast<std::uint64_t>(key.x)) ^
        (mix(static_cast<std::uint64_t>(key.y)) << 1U) ^
        (mix(static_cast<std::uint64_t>(key.z)) << 2U));
  }
};

struct SubmapPoint3f {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct VoxelAccumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  std::size_t count = 0;
};

using VoxelMap = std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash>;

VoxelKey Quantize(const Eigen::Vector3d& point, const double resolution_m) {
  return {
      static_cast<std::int64_t>(std::floor(point.x() / resolution_m)),
      static_cast<std::int64_t>(std::floor(point.y() / resolution_m)),
      static_cast<std::int64_t>(std::floor(point.z() / resolution_m)),
  };
}

void Accumulate(VoxelMap& voxels, const VoxelKey& key,
                const Eigen::Vector3d& point, const std::size_t weight = 1) {
  auto& voxel = voxels[key];
  voxel.sum += static_cast<double>(weight) * point;
  voxel.count += weight;
}

VoxelMap CoarsenVoxels(const VoxelMap& fine_voxels,
                       const double resolution_m) {
  VoxelMap coarse_voxels;
  coarse_voxels.reserve(fine_voxels.size());
  for (const auto& [unused_key, voxel] : fine_voxels) {
    static_cast<void>(unused_key);
    const Eigen::Vector3d centroid =
        voxel.sum / static_cast<double>(voxel.count);
    Accumulate(coarse_voxels, Quantize(centroid, resolution_m), centroid,
               voxel.count);
  }
  return coarse_voxels;
}

std::shared_ptr<const std::vector<SubmapPoint3f>> FreezePoints(
    std::vector<SubmapPoint3f> points) {
  return std::make_shared<const std::vector<SubmapPoint3f>>(
      std::move(points));
}

hybrid_slam::sensor::ImmutablePointCloud MakeCloud(
    const std::shared_ptr<const std::vector<SubmapPoint3f>>& storage) {
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(), sizeof(SubmapPoint3f),
      offsetof(SubmapPoint3f, x), offsetof(SubmapPoint3f, y),
      offsetof(SubmapPoint3f, z),
      hybrid_slam::geometry::FrameId::kSubmap);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(
      storage, view, storage->capacity() * sizeof(SubmapPoint3f));
}

const KeyFrameSnapshot* FindKeyFrame(
    const std::vector<KeyFrameSnapshot>& keyframes,
    hybrid_slam::common::KeyFrameId id,
    const SubmapSnapshot& submap) {
  for (const auto& keyframe : keyframes) {
    if (keyframe.keyframe_id == id &&
        keyframe.session_id == submap.session_id &&
        keyframe.trajectory_id == submap.trajectory_id) {
      return &keyframe;
    }
  }
  return nullptr;
}

std::vector<float> BuildDescriptor(
    const std::vector<SubmapPoint3f>& points,
    std::size_t radial_bins) {
  std::vector<float> descriptor(radial_bins, 0.0F);
  if (points.empty()) {
    return descriptor;
  }

  double max_radius = 0.0;
  for (const auto& point : points) {
    max_radius = std::max(
        max_radius,
        std::hypot(static_cast<double>(point.x),
                   static_cast<double>(point.y)));
  }
  if (!(max_radius > 0.0) || !std::isfinite(max_radius)) {
    descriptor.front() = 1.0F;
    return descriptor;
  }

  for (const auto& point : points) {
    const double radius = std::hypot(
        static_cast<double>(point.x), static_cast<double>(point.y));
    const auto bin = std::min(
        radial_bins - 1,
        static_cast<std::size_t>(radius / max_radius * radial_bins));
    descriptor[bin] += 1.0F;
  }
  const float normalizer = static_cast<float>(points.size());
  for (float& value : descriptor) {
    value /= normalizer;
  }
  return descriptor;
}

}  // namespace

bool SubmapGeometryBuildConfig::valid() const noexcept {
  return std::isfinite(voxel_resolution_m) && voxel_resolution_m > 0.0 &&
         descriptor_radial_bins > 0 && max_points > 0;
}

std::optional<SubmapGeometryBuildResult> BuildSubmapGeometry(
    const std::vector<KeyFrameSnapshot>& keyframes,
    const SubmapSnapshot& submap,
    SubmapGeometryBuildConfig config) {
  if (!config.valid() || submap.keyframe_ids.empty()) {
    return std::nullopt;
  }

  const auto* anchor = FindKeyFrame(
      keyframes, submap.anchor_keyframe_id, submap);
  if (anchor == nullptr || !anchor->valid()) {
    return std::nullopt;
  }

  const auto T_anchor_odom = anchor->T_odom_base.T_target_source.inverse();
  // Cartographer's submaps integrate every accepted range datum into a grid.
  // Do the same essential thing here: visit every member keyframe and compute
  // voxel centroids. The previous implementation stopped as soon as max_points
  // was reached, which systematically discarded the tail of dense submaps and
  // could remove the loop endpoint itself from its support surface.
  VoxelMap fine_voxels;
  fine_voxels.reserve(config.max_points);
  std::size_t input_point_count = 0;

  for (const auto keyframe_id : submap.keyframe_ids) {
    const auto* keyframe = FindKeyFrame(keyframes, keyframe_id, submap);
    if (keyframe == nullptr || !keyframe->valid()) {
      return std::nullopt;
    }
    const auto T_submap_base = T_anchor_odom.compose(
        keyframe->T_odom_base.T_target_source);
    const auto& view = keyframe->cloud_base.view();
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> raw{};
      if (!view.xyz(index, raw)) {
        return std::nullopt;
      }
      const Eigen::Vector3d point = T_submap_base.transformPoint(
          Eigen::Vector3d(raw[0], raw[1], raw[2]));
      if (!point.allFinite()) {
        continue;
      }
      ++input_point_count;
      Accumulate(fine_voxels,
                 Quantize(point, config.voxel_resolution_m), point);
    }
  }

  if (fine_voxels.empty()) {
    return std::nullopt;
  }

  // Adapt the voxel length to the requested point budget instead of clipping
  // a chronological prefix. This is deterministic, covers the complete
  // rolling submap, and mirrors the intent of Cartographer's adaptive voxel
  // filters without importing its HybridGrid implementation.
  double selected_resolution_m = config.voxel_resolution_m;
  VoxelMap selected_voxels;
  if (fine_voxels.size() <= config.max_points) {
    selected_voxels = std::move(fine_voxels);
  } else {
    for (int iteration = 0; iteration < 16; ++iteration) {
      selected_resolution_m *= 2.0;
      selected_voxels = CoarsenVoxels(fine_voxels, selected_resolution_m);
      if (selected_voxels.size() <= config.max_points) {
        break;
      }
    }
  }
  if (selected_voxels.size() > config.max_points) {
    return std::nullopt;
  }

  std::vector<std::pair<VoxelKey, SubmapPoint3f>> ordered_points;
  ordered_points.reserve(selected_voxels.size());
  for (const auto& [key, voxel] : selected_voxels) {
    const Eigen::Vector3d point =
        voxel.sum / static_cast<double>(voxel.count);
    ordered_points.push_back(
        {key, {static_cast<float>(point.x()),
               static_cast<float>(point.y()),
               static_cast<float>(point.z())}});
  }
  std::sort(ordered_points.begin(), ordered_points.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.first.x != rhs.first.x) {
                return lhs.first.x < rhs.first.x;
              }
              if (lhs.first.y != rhs.first.y) {
                return lhs.first.y < rhs.first.y;
              }
              return lhs.first.z < rhs.first.z;
            });

  std::vector<SubmapPoint3f> points;
  points.reserve(ordered_points.size());
  SubmapBounds bounds;
  bounds.minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  bounds.maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
  for (const auto& [unused_key, point] : ordered_points) {
    static_cast<void>(unused_key);
    points.push_back(point);
    const Eigen::Vector3d point_d(point.x, point.y, point.z);
    bounds.minimum = bounds.minimum.cwiseMin(point_d);
    bounds.maximum = bounds.maximum.cwiseMax(point_d);
    bounds.has_points = true;
  }

  const auto storage = FreezePoints(std::move(points));
  SubmapGeometryBuildResult result;
  result.T_map_submap = anchor->T_map_base.T_target_source;
  result.cloud_submap = MakeCloud(storage);
  result.seed_cloud_submap = result.cloud_submap;
  result.bounds = bounds;
  result.descriptor = BuildDescriptor(
      std::vector<SubmapPoint3f>(storage->begin(), storage->end()),
      config.descriptor_radial_bins);
  result.input_point_count = input_point_count;
  result.keyframe_count = submap.keyframe_ids.size();
  result.voxel_resolution_m = selected_resolution_m;
  if (!result.valid()) {
    return std::nullopt;
  }
  return result;
}

}  // namespace hybrid_slam::mapping
