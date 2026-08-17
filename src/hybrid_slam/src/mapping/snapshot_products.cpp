#include "hybrid_slam/mapping/snapshot_products.h"

#include <array>
#include <cmath>
#include <limits>

namespace hybrid_slam::mapping {

std::optional<SnapshotTrajectory> SnapshotProductBuilder::BuildTrajectory(
    const MapSnapshot& snapshot) {
  if (!snapshot.valid()) {
    return std::nullopt;
  }

  SnapshotTrajectory trajectory;
  trajectory.snapshot_version = snapshot.version;
  trajectory.poses.reserve(snapshot.keyframes.size());
  for (const auto& keyframe : snapshot.keyframes) {
    trajectory.poses.push_back({
        keyframe.node_id,
        keyframe.keyframe_id,
        keyframe.trajectory_id,
        keyframe.session_id,
        keyframe.T_map_base,
    });
  }
  return trajectory;
}

std::optional<SnapshotPointCloudMap>
SnapshotProductBuilder::BuildPointCloudMap(const MapSnapshot& snapshot) {
  return BuildPointCloudMap(snapshot, 0, snapshot.keyframes.size());
}

std::optional<SnapshotPointCloudMap>
SnapshotProductBuilder::BuildPointCloudMap(const MapSnapshot& snapshot,
                                           const std::size_t keyframe_begin,
                                           const std::size_t keyframe_end) {
  if (!snapshot.valid()) {
    return std::nullopt;
  }
  if (keyframe_begin >= keyframe_end ||
      keyframe_end > snapshot.keyframes.size()) {
    return std::nullopt;
  }

  std::size_t point_count = 0;
  for (std::size_t index = keyframe_begin; index < keyframe_end; ++index) {
    const auto& keyframe = snapshot.keyframes[index];
    if (keyframe.cloud_base.size() >
        std::numeric_limits<std::size_t>::max() - point_count) {
      return std::nullopt;
    }
    point_count += keyframe.cloud_base.size();
  }

  SnapshotPointCloudMap map;
  map.snapshot_version = snapshot.version;
  map.points.reserve(point_count);
  Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::infinity());
  Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
      -std::numeric_limits<float>::infinity());

  for (std::size_t keyframe_index = keyframe_begin;
       keyframe_index < keyframe_end; ++keyframe_index) {
    const auto& keyframe = snapshot.keyframes[keyframe_index];
    const auto& view = keyframe.cloud_base.view();
    const auto& T_map_base = keyframe.T_map_base.T_target_source;
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> source{};
      if (!view.xyz(index, source)) {
        return std::nullopt;
      }
      const Eigen::Vector3d transformed = T_map_base.transformPoint(
          Eigen::Vector3d(source[0], source[1], source[2]));
      if (!transformed.allFinite()) {
        return std::nullopt;
      }
      const Eigen::Vector3f point = transformed.cast<float>();
      if (!point.allFinite()) {
        return std::nullopt;
      }
      map.points.push_back({point.x(), point.y(), point.z()});
      minimum = minimum.cwiseMin(point);
      maximum = maximum.cwiseMax(point);
    }
  }

  if (!map.points.empty()) {
    map.minimum = minimum;
    map.maximum = maximum;
  }
  return map;
}

}  // namespace hybrid_slam::mapping
