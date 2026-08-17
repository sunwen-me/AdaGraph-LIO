#include "hybrid_slam/mapping/submap_recovery_seed.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace hybrid_slam::mapping {

std::optional<std::vector<Eigen::Vector3f>> BuildRecoverySeedInOdom(
    const SubmapSnapshot& submap,
    const hybrid_slam::geometry::RigidTransform3d& T_map_odom) {
  if (!submap.geometry_ready || !submap.valid() || !T_map_odom.isFinite()) {
    return std::nullopt;
  }
  const auto& cloud = submap.seed_cloud_submap;
  if (!cloud.valid() || cloud.empty() ||
      cloud.frame() != hybrid_slam::geometry::FrameId::kSubmap) {
    return std::nullopt;
  }

  const auto T_odom_submap = T_map_odom.inverse().compose(
      submap.T_map_submap);
  std::vector<Eigen::Vector3f> seed;
  seed.reserve(cloud.size());
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::array<float, 3> raw{};
    if (!cloud.view().xyz(index, raw)) {
      return std::nullopt;
    }
    const Eigen::Vector3d transformed = T_odom_submap.transformPoint(
        Eigen::Vector3d(raw[0], raw[1], raw[2]));
    if (!transformed.allFinite() ||
        transformed.x() > std::numeric_limits<float>::max() ||
        transformed.x() < -std::numeric_limits<float>::max() ||
        transformed.y() > std::numeric_limits<float>::max() ||
        transformed.y() < -std::numeric_limits<float>::max() ||
        transformed.z() > std::numeric_limits<float>::max() ||
        transformed.z() < -std::numeric_limits<float>::max()) {
      return std::nullopt;
    }
    seed.push_back(transformed.cast<float>());
  }
  if (seed.empty()) {
    return std::nullopt;
  }
  return seed;
}

std::optional<std::vector<Eigen::Vector3f>> BuildRecoverySeedInOdom(
    const SubmapSnapshot& submap,
    const std::vector<KeyFrameSnapshot>& keyframes,
    const hybrid_slam::geometry::RigidTransform3d& T_map_odom) {
  if (!T_map_odom.isFinite() || !submap.valid()) {
    return std::nullopt;
  }
  if (submap.geometry_ready) {
    return BuildRecoverySeedInOdom(submap, T_map_odom);
  }

  const auto T_odom_map = T_map_odom.inverse();
  std::vector<Eigen::Vector3f> seed;
  for (const auto keyframe_id : submap.keyframe_ids) {
    const auto found = std::find_if(
        keyframes.begin(), keyframes.end(), [&](const KeyFrameSnapshot& keyframe) {
          return keyframe.keyframe_id == keyframe_id &&
                 keyframe.session_id == submap.session_id &&
                 keyframe.trajectory_id == submap.trajectory_id &&
                 keyframe.submap_id == submap.id;
        });
    if (found == keyframes.end() || !found->valid()) {
      return std::nullopt;
    }
    const auto T_odom_base = T_odom_map.compose(found->T_map_base.T_target_source);
    const auto& view = found->cloud_base.view();
    seed.reserve(seed.size() + view.size());
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> raw{};
      if (!view.xyz(index, raw)) {
        return std::nullopt;
      }
      const Eigen::Vector3d transformed = T_odom_base.transformPoint(
          Eigen::Vector3d(raw[0], raw[1], raw[2]));
      if (!transformed.allFinite() ||
          (transformed.array().abs() >
           Eigen::Vector3d::Constant(std::numeric_limits<float>::max())
               .array()).any()) {
        return std::nullopt;
      }
      seed.push_back(transformed.cast<float>());
    }
  }
  if (seed.empty()) {
    return std::nullopt;
  }
  return seed;
}

}  // namespace hybrid_slam::mapping
