#include "lio/pcl_loop_closure.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Eigenvalues>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {
namespace {

using hybrid_slam::adapter::PclPoint;
using hybrid_slam::adapter::PclPointCloud;
using hybrid_slam::backend::LoopCandidate;
using hybrid_slam::backend::LoopDetectionSnapshot;
using hybrid_slam::backend::LoopVerificationInput;
using hybrid_slam::backend::RelativePoseMeasurement;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::localization::LocalizationMap;

struct CloudBounds {
  Eigen::Vector3d minimum =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  std::size_t finite_points = 0;

  [[nodiscard]] bool valid() const noexcept { return finite_points > 0; }
};

CloudBounds ComputeBounds(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
  CloudBounds bounds;
  const auto& view = cloud.view();
  for (std::size_t index = 0; index < view.size(); ++index) {
    std::array<float, 3> raw{};
    if (!view.xyz(index, raw)) {
      continue;
    }
    const Eigen::Vector3d point(raw[0], raw[1], raw[2]);
    if (!point.allFinite()) {
      continue;
    }
    bounds.minimum = bounds.minimum.cwiseMin(point);
    bounds.maximum = bounds.maximum.cwiseMax(point);
    ++bounds.finite_points;
  }
  return bounds;
}

CloudBounds ComputeBounds(const PclPointCloud& cloud) {
  CloudBounds bounds;
  for (const auto& raw : cloud.points) {
    const Eigen::Vector3d point(raw.x, raw.y, raw.z);
    if (!point.allFinite()) {
      continue;
    }
    bounds.minimum = bounds.minimum.cwiseMin(point);
    bounds.maximum = bounds.maximum.cwiseMax(point);
    ++bounds.finite_points;
  }
  return bounds;
}

std::string BoundsString(const CloudBounds& bounds) {
  if (!bounds.valid()) {
    return "empty";
  }
  std::ostringstream output;
  output << std::setprecision(6) << "min=[" << bounds.minimum.x() << ','
         << bounds.minimum.y() << ',' << bounds.minimum.z() << "] max=["
         << bounds.maximum.x() << ',' << bounds.maximum.y() << ','
         << bounds.maximum.z() << "] extent=["
         << (bounds.maximum - bounds.minimum).transpose() << ']';
  return output.str();
}

Eigen::Vector3d RollPitchYaw(
    const hybrid_slam::geometry::RigidTransform3d& pose) {
  const Eigen::Matrix3d rotation = pose.rotation.normalized().toRotationMatrix();
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) < 1e-9) {
    return {std::atan2(-rotation(0, 1), rotation(1, 1)), pitch, 0.0};
  }
  return {std::atan2(rotation(2, 1), rotation(2, 2)), pitch,
          std::atan2(rotation(1, 0), rotation(0, 0))};
}

std::string PoseString(
    const hybrid_slam::geometry::RigidTransform3d& pose) {
  const Eigen::Vector3d rpy = RollPitchYaw(pose);
  std::ostringstream output;
  output << std::setprecision(8) << "xyz=[" << pose.translation.x() << ','
         << pose.translation.y() << ',' << pose.translation.z() << "] rpy=["
         << rpy.x() << ',' << rpy.y() << ',' << rpy.z() << ']';
  return output.str();
}

double RotationDistance(
    const hybrid_slam::geometry::RigidTransform3d& lhs,
    const hybrid_slam::geometry::RigidTransform3d& rhs) {
  return Eigen::AngleAxisd(
             (lhs.rotation.inverse() * rhs.rotation).normalized())
      .angle();
}

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;
  bool operator==(const VoxelKey&) const = default;
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey& key) const noexcept {
    const auto hash = std::hash<std::int64_t>{};
    const std::size_t first = hash(key.x);
    const std::size_t second = hash(key.y);
    const std::size_t third = hash(key.z);
    return first ^ (second + 0x9e3779b9U + (first << 6U) + (first >> 2U)) ^
           (third + 0x85ebca6bU + (second << 6U) + (second >> 2U));
  }
};

struct LoopGeometryEvidence {
  double source_inlier_ratio = 0.0;
  double target_inlier_ratio = 0.0;
  double mutual_inlier_ratio = 0.0;
  double bidirectional_overlap = 0.0;
  double source_inlier_rmse_m = std::numeric_limits<double>::infinity();
  double target_inlier_rmse_m = std::numeric_limits<double>::infinity();
  double bidirectional_inlier_rmse_m =
      std::numeric_limits<double>::infinity();
  double support_extent_m = 0.0;
  double support_z_extent_m = 0.0;
  double eigenvalue_ratio = 0.0;
  std::size_t support_voxels = 0;
  bool observable_6dof = false;
};

LoopGeometryEvidence EvaluateLoopGeometry(
    const PclPointCloud& aligned_source, const PclPointCloud& target,
    const double correspondence_distance, const double support_voxel_size_m,
    const double minimum_support_z_extent_m,
    const double minimum_support_eigenvalue_ratio) {
  LoopGeometryEvidence evidence;
  if (aligned_source.empty() || target.empty() ||
      !(correspondence_distance > 0.0) || !(support_voxel_size_m > 0.0)) {
    return evidence;
  }

  const auto aligned_owner = std::make_shared<const PclPointCloud>(aligned_source);
  const auto target_owner = std::make_shared<const PclPointCloud>(target);
  pcl::KdTreeFLANN<PclPoint> target_tree;
  pcl::KdTreeFLANN<PclPoint> source_tree;
  target_tree.setInputCloud(target_owner);
  source_tree.setInputCloud(aligned_owner);
  const float squared_gate = static_cast<float>(
      correspondence_distance * correspondence_distance);
  std::vector<int> source_to_target(aligned_source.size(), -1);
  std::vector<int> target_to_source(target.size(), -1);
  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(aligned_source.size());
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  double source_squared_error = 0.0;

  for (std::size_t index = 0; index < aligned_source.size(); ++index) {
    const auto& point = aligned_source.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    std::vector<int> nearest(1);
    std::vector<float> squared_distance(1);
    if (target_tree.nearestKSearch(point, 1, nearest, squared_distance) != 1 ||
        squared_distance.front() > squared_gate) {
      continue;
    }
    source_to_target[index] = nearest.front();
    source_squared_error += squared_distance.front();
    const Eigen::Vector3d inlier(point.x, point.y, point.z);
    inliers.push_back(inlier);
    occupied.insert({
        static_cast<std::int64_t>(std::floor(inlier.x() /
                                             support_voxel_size_m)),
        static_cast<std::int64_t>(std::floor(inlier.y() /
                                             support_voxel_size_m)),
        static_cast<std::int64_t>(std::floor(inlier.z() /
                                             support_voxel_size_m))});
  }
  for (std::size_t index = 0; index < target.size(); ++index) {
    const auto& point = target.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    std::vector<int> nearest(1);
    std::vector<float> squared_distance(1);
    if (source_tree.nearestKSearch(point, 1, nearest, squared_distance) == 1 &&
        squared_distance.front() <= squared_gate) {
      target_to_source[index] = nearest.front();
    }
  }

  std::size_t target_inliers = 0;
  double target_squared_error = 0.0;
  for (std::size_t index = 0; index < target_to_source.size(); ++index) {
    if (target_to_source[index] < 0) {
      continue;
    }
    ++target_inliers;
    const auto& point = target.points[index];
    const auto& nearest =
        aligned_source.points[static_cast<std::size_t>(target_to_source[index])];
    const double dx = static_cast<double>(point.x) - nearest.x;
    const double dy = static_cast<double>(point.y) - nearest.y;
    const double dz = static_cast<double>(point.z) - nearest.z;
    target_squared_error += dx * dx + dy * dy + dz * dz;
  }
  std::size_t mutual_inliers = 0;
  for (std::size_t source_index = 0; source_index < source_to_target.size();
       ++source_index) {
    const int target_index = source_to_target[source_index];
    if (target_index >= 0 &&
        target_to_source[static_cast<std::size_t>(target_index)] ==
            static_cast<int>(source_index)) {
      ++mutual_inliers;
    }
  }
  evidence.source_inlier_ratio = static_cast<double>(inliers.size()) /
                                 static_cast<double>(aligned_source.size());
  evidence.target_inlier_ratio = static_cast<double>(target_inliers) /
                                 static_cast<double>(target.size());
  evidence.mutual_inlier_ratio = static_cast<double>(mutual_inliers) /
                                 static_cast<double>(std::min(
                                     aligned_source.size(), target.size()));
  evidence.bidirectional_overlap =
      std::min(evidence.source_inlier_ratio, evidence.target_inlier_ratio);
  if (!inliers.empty()) {
    evidence.source_inlier_rmse_m =
        std::sqrt(source_squared_error / static_cast<double>(inliers.size()));
  }
  if (target_inliers > 0) {
    evidence.target_inlier_rmse_m = std::sqrt(
        target_squared_error / static_cast<double>(target_inliers));
  }
  evidence.bidirectional_inlier_rmse_m =
      std::max(evidence.source_inlier_rmse_m,
               evidence.target_inlier_rmse_m);
  evidence.support_voxels = occupied.size();
  if (inliers.size() < 3) {
    return evidence;
  }

  Eigen::Vector3d minimum =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum =
      Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : inliers) {
    minimum = minimum.cwiseMin(point);
    maximum = maximum.cwiseMax(point);
    mean += point;
  }
  mean /= static_cast<double>(inliers.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : inliers) {
    const Eigen::Vector3d centered = point - mean;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(inliers.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite() ||
      solver.eigenvalues()[2] <= 1e-9) {
    return evidence;
  }
  evidence.support_extent_m = (maximum - minimum).head<2>().norm();
  evidence.support_z_extent_m = maximum.z() - minimum.z();
  evidence.eigenvalue_ratio =
      solver.eigenvalues()[0] / solver.eigenvalues()[2];
  evidence.observable_6dof =
      evidence.support_z_extent_m >= minimum_support_z_extent_m &&
      evidence.eigenvalue_ratio >= minimum_support_eigenvalue_ratio;
  return evidence;
}

void SaveRegistrationDebug(
    const std::string& directory, const std::string& prefix,
    const PclPointCloud& source, const PclPointCloud& target,
    const PclPointCloud& aligned) {
  if (directory.empty()) {
    return;
  }
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    LOG(WARNING) << "[LoopClosure]: cannot create debug directory="
                 << directory << " error=" << error.message();
    return;
  }
  const std::filesystem::path base =
      std::filesystem::path(directory) / prefix;
  const int source_status =
      pcl::io::savePCDFileBinary(base.string() + "_source.pcd", source);
  const int target_status =
      pcl::io::savePCDFileBinary(base.string() + "_target.pcd", target);
  const int aligned_status = aligned.empty()
                                 ? -1
                                 : pcl::io::savePCDFileBinary(
                                       base.string() + "_aligned.pcd", aligned);
  LOG(INFO) << "[LoopClosure]: debug_clouds prefix=" << base.string()
            << " save_status=" << source_status << ',' << target_status << ','
            << aligned_status;
}

const hybrid_slam::mapping::KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::common::NodeId node_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [node_id](const auto& keyframe) { return keyframe.node_id == node_id; });
  return found == map.keyframes.end() ? nullptr : &*found;
}

const hybrid_slam::mapping::KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::common::KeyFrameId keyframe_id,
    const hybrid_slam::common::SessionId session_id,
    const hybrid_slam::common::TrajectoryId trajectory_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [keyframe_id, session_id, trajectory_id](const auto& keyframe) {
        return keyframe.keyframe_id == keyframe_id &&
               keyframe.session_id == session_id &&
               keyframe.trajectory_id == trajectory_id;
      });
  return found == map.keyframes.end() ? nullptr : &*found;
}

std::optional<LocalizationMap> AsMapTarget(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
  const auto owner = cloud.ownerAs<PclPointCloud>();
  if (!owner || owner->empty()) {
    return std::nullopt;
  }
  const auto& points = owner->points;
  const auto* origin = reinterpret_cast<const std::byte*>(points.data());
  const auto& first = points.front();
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      owner->data(), owner->size(), sizeof(PclPoint),
      static_cast<std::size_t>(reinterpret_cast<const std::byte*>(&first.x) -
                               origin),
      static_cast<std::size_t>(reinterpret_cast<const std::byte*>(&first.y) -
                               origin),
      static_cast<std::size_t>(reinterpret_cast<const std::byte*>(&first.z) -
                               origin),
      FrameId::kMap);
  LocalizationMap target(
      0, hybrid_slam::sensor::ImmutablePointCloud::Share(
             owner, view, points.capacity() * sizeof(PclPoint)));
  return target.valid() ? std::optional<LocalizationMap>(std::move(target))
                        : std::nullopt;
}

const hybrid_slam::mapping::SubmapSnapshot* FindFinishedSubmap(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::mapping::KeyFrameSnapshot& keyframe) {
  const auto found = std::find_if(
      map.submaps.begin(), map.submaps.end(), [&keyframe](const auto& submap) {
        return submap.id == keyframe.submap_id &&
               submap.session_id == keyframe.session_id &&
               submap.trajectory_id == keyframe.trajectory_id &&
               submap.state == hybrid_slam::mapping::SubmapState::kFinished &&
               submap.geometry_ready && submap.valid();
      });
  return found == map.submaps.end() ? nullptr : &*found;
}

void AppendSubmapInBaseFrame(
    const hybrid_slam::mapping::SubmapSnapshot& submap,
    const hybrid_slam::mapping::KeyFrameSnapshot& submap_anchor,
    const hybrid_slam::mapping::KeyFrameSnapshot& base,
    const double crop_radius_m, PclPointCloud& output) {
  // A submap frame is the immutable base frame of its anchor keyframe. Build
  // each local registration neighborhood from odometry-relative anchor poses,
  // not optimized map poses. Otherwise an earlier loop correction warps the
  // support cloud used to measure the next loop, making the measurement depend
  // circularly on the graph it is supposed to constrain.
  const auto T_base_submap =
      base.T_odom_base.T_target_source.inverse().compose(
          submap_anchor.T_odom_base.T_target_source);
  const auto& view = submap.cloud_submap.view();
  output.points.reserve(output.size() + view.size());
  for (std::size_t index = 0; index < view.size(); ++index) {
    std::array<float, 3> raw{};
    if (!view.xyz(index, raw)) {
      continue;
    }
    const Eigen::Vector3d raw_point(raw[0], raw[1], raw[2]);
    const Eigen::Vector3d point = T_base_submap.transformPoint(raw_point);
    if (point.allFinite() &&
        point.squaredNorm() <= crop_radius_m * crop_radius_m) {
      output.points.push_back({static_cast<float>(point.x()),
                               static_cast<float>(point.y()),
                               static_cast<float>(point.z()), 0.0F});
    }
  }
}

hybrid_slam::sensor::ImmutablePointCloud FreezeBaseCloud(
    std::shared_ptr<PclPointCloud> cloud) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  std::shared_ptr<const PclPointCloud> frozen = std::move(cloud);
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      frozen->data(), frozen->size(), sizeof(PclPoint), offsetof(PclPoint, x),
      offsetof(PclPoint, y), offsetof(PclPoint, z), FrameId::kBase);
  const auto bytes = frozen->points.capacity() * sizeof(PclPoint);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(
      std::move(frozen), view, bytes);
}

std::optional<std::pair<hybrid_slam::sensor::ImmutablePointCloud,
                        hybrid_slam::sensor::ImmutablePointCloud>>
BuildSubmapRegistrationClouds(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::mapping::KeyFrameSnapshot& query,
    const hybrid_slam::mapping::KeyFrameSnapshot& match,
    const std::size_t neighbor_submaps, const double crop_radius_m) {
  const auto* query_submap = FindFinishedSubmap(map, query);
  const auto* match_submap = FindFinishedSubmap(map, match);
  if (!query_submap || !match_submap) {
    return std::nullopt;
  }

  auto source = std::make_shared<PclPointCloud>();
  auto target = std::make_shared<PclPointCloud>();
  // Build each side independently. Session/trajectory filtering by the match
  // side for both neighborhoods silently broke cross-session verification.
  // Spatial cropping is centered on the graph endpoints used by T_match_query;
  // it retains local 3D support while excluding non-overlapping route tails.
  for (const auto& submap : map.submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid()) {
      continue;
    }
    const auto id = submap.id.value();
    const auto query_id = query_submap->id.value();
    const auto match_id = match_submap->id.value();
    const auto query_distance =
        id >= query_id ? id - query_id : query_id - id;
    const auto match_distance =
        id >= match_id ? id - match_id : match_id - id;
    if (submap.session_id == query.session_id &&
        submap.trajectory_id == query.trajectory_id &&
        query_distance <= neighbor_submaps) {
      const auto geometry_anchor = submap.geometry_anchor_keyframe_id.value_or(
          submap.anchor_keyframe_id);
      const auto* anchor = FindKeyFrame(
          map, geometry_anchor, submap.session_id,
          submap.trajectory_id);
      if (anchor) {
        AppendSubmapInBaseFrame(submap, *anchor, query, crop_radius_m, *source);
      }
    }
    if (submap.session_id == match.session_id &&
        submap.trajectory_id == match.trajectory_id &&
        match_distance <= neighbor_submaps) {
      const auto geometry_anchor = submap.geometry_anchor_keyframe_id.value_or(
          submap.anchor_keyframe_id);
      const auto* anchor = FindKeyFrame(
          map, geometry_anchor, submap.session_id,
          submap.trajectory_id);
      if (anchor) {
        AppendSubmapInBaseFrame(submap, *anchor, match, crop_radius_m, *target);
      }
    }
  }
  auto source_cloud = FreezeBaseCloud(std::move(source));
  auto target_cloud = FreezeBaseCloud(std::move(target));
  if (!source_cloud.valid() || !target_cloud.valid()) {
    return std::nullopt;
  }
  return std::make_pair(std::move(source_cloud), std::move(target_cloud));
}

}  // namespace

std::optional<std::string> PclLoopClosureConfig::validate() const {
  auto registration_config = this->registration;
  registration_config.enabled = false;
  if (const auto error = registration_config.validate()) {
    return error;
  }
  if (plugin != "scan_context" && plugin != "bevplace2") {
    return "unsupported loop closure plugin: " + plugin;
  }
  if (const auto error = scan_context.validate()) {
    return error;
  }
  if (plugin == "bevplace2") {
    if (const auto error = bevplace2.validate()) {
      return error;
    }
  }
  if (maximum_candidates == 0 ||
      !std::isfinite(registration_crop_radius_m) ||
      registration_crop_radius_m <= 0.0 ||
      !std::isfinite(minimum_temporal_separation_s) ||
      minimum_temporal_separation_s < 0.0 ||
      !std::isfinite(maximum_initial_translation_m) ||
      maximum_initial_translation_m <= 0.0 ||
      !std::isfinite(minimum_inlier_ratio) || minimum_inlier_ratio <= 0.0 ||
      minimum_inlier_ratio > 1.0 || !std::isfinite(minimum_overlap) ||
      minimum_overlap <= 0.0 || minimum_overlap > 1.0 ||
      minimum_source_points == 0 || minimum_target_points == 0 ||
      !std::isfinite(minimum_bidirectional_overlap) ||
      minimum_bidirectional_overlap <= 0.0 ||
      minimum_bidirectional_overlap > 1.0 ||
      !std::isfinite(minimum_mutual_inlier_ratio) ||
      minimum_mutual_inlier_ratio <= 0.0 ||
      minimum_mutual_inlier_ratio > 1.0 ||
      !std::isfinite(maximum_bidirectional_inlier_rmse_m) ||
      maximum_bidirectional_inlier_rmse_m <= 0.0 ||
      !std::isfinite(minimum_support_extent_m) ||
      minimum_support_extent_m <= 0.0 ||
      !std::isfinite(minimum_support_z_extent_m) ||
      minimum_support_z_extent_m <= 0.0 ||
      !std::isfinite(minimum_support_eigenvalue_ratio) ||
      minimum_support_eigenvalue_ratio <= 0.0 ||
      minimum_support_eigenvalue_ratio > 1.0 ||
      minimum_support_voxels == 0 ||
      !std::isfinite(support_voxel_size_m) || support_voxel_size_m <= 0.0 ||
      !std::isfinite(maximum_coarse_translation_disagreement_m) ||
      maximum_coarse_translation_disagreement_m <= 0.0 ||
      !std::isfinite(maximum_coarse_yaw_disagreement_rad) ||
      maximum_coarse_yaw_disagreement_rad <= 0.0 ||
      !std::isfinite(maximum_pose_free_graph_correction_m) ||
      maximum_pose_free_graph_correction_m <= 0.0 ||
      !std::isfinite(maximum_pose_free_graph_correction_rad) ||
      maximum_pose_free_graph_correction_rad <= 0.0 ||
      !std::isfinite(maximum_relative_z_m) || maximum_relative_z_m <= 0.0 ||
      !std::isfinite(maximum_relative_roll_pitch_rad) ||
      maximum_relative_roll_pitch_rad <= 0.0 ||
      !std::isfinite(maximum_solution_disagreement_m) ||
      maximum_solution_disagreement_m <= 0.0 ||
      !std::isfinite(maximum_solution_disagreement_rad) ||
      maximum_solution_disagreement_rad <= 0.0 ||
      !std::isfinite(uniqueness_probe_translation_m) ||
      uniqueness_probe_translation_m <= 0.0 ||
      minimum_uniqueness_probe_consensus == 0 ||
      minimum_uniqueness_probe_consensus > 4 ||
      !std::isfinite(translation_sigma_m) || translation_sigma_m <= 0.0 ||
      !std::isfinite(rotation_sigma_rad) || rotation_sigma_rad <= 0.0) {
    return "invalid loop closure candidate or quality gates";
  }
  return std::nullopt;
}

PclSubmapLoopDetector::PclSubmapLoopDetector(PclLoopClosureConfig config)
    : config_(std::move(config)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid PCL loop detector config: " + *error);
  }
}

std::vector<LoopCandidate> PclSubmapLoopDetector::detect(
    const LoopDetectionSnapshot& snapshot,
    const hybrid_slam::common::NodeId query_node) {
  std::vector<LoopCandidate> result;
  if (!snapshot.map || !snapshot.index || !snapshot.map->consistent() ||
      snapshot.index->mapVersion() != snapshot.map->version) {
    return result;
  }
  const auto* query = FindKeyFrame(*snapshot.map, query_node);
  if (!query || !query->valid()) {
    return result;
  }
  const auto* query_submap = FindFinishedSubmap(*snapshot.map, *query);
  if (!query_submap || query_submap->descriptor.empty()) {
    return result;
  }
  const auto candidates = snapshot.index->query(
      query_submap->descriptor, config_.maximum_candidates);
  for (const auto& candidate : candidates) {
    const auto found_submap = std::find_if(
        snapshot.map->submaps.begin(), snapshot.map->submaps.end(),
        [&candidate](const auto& submap) {
          return submap.id == candidate.submap_id &&
                 submap.session_id == candidate.session_id &&
                 submap.trajectory_id == candidate.trajectory_id &&
                 submap.state == hybrid_slam::mapping::SubmapState::kFinished;
        });
    if (found_submap == snapshot.map->submaps.end()) {
      continue;
    }
    // Matching the same submap measures local overlap, not a revisit. The
    // scheduler may call this detector for several representative queries from
    // the same completed submap within one batch.
    if (found_submap->id == query->submap_id &&
        found_submap->session_id == query->session_id &&
        found_submap->trajectory_id == query->trajectory_id) {
      continue;
    }
    // Geometry and descriptor are expressed in the immutable anchor frame.
    // Use that same node as the graph endpoint of the loop measurement.
    const auto* match = FindKeyFrame(
        *snapshot.map, found_submap->anchor_keyframe_id,
        found_submap->session_id, found_submap->trajectory_id);
    if (!match || match->node_id == query->node_id) {
      continue;
    }
    const double time_delta = std::abs(
        match->T_odom_base.timestamp.seconds() -
        query->T_odom_base.timestamp.seconds());
    if (time_delta < config_.minimum_temporal_separation_s) {
      continue;
    }
    result.push_back({query->node_id, match->node_id, candidate.score});
  }
  return result;
}

void PclLoopClosureVerifier::prepare(const LoopDetectionSnapshot& snapshot) {
  map_ = snapshot.map;
}

std::optional<RelativePoseMeasurement> PclLoopClosureVerifier::verify(
    const LoopVerificationInput& input) {
  if (!map_ || !input.query_cloud_base.valid() ||
      !input.match_cloud_base.valid() ||
      input.query_cloud_base.frame() != FrameId::kBase ||
      input.match_cloud_base.frame() != FrameId::kBase ||
      !input.initial_T_match_query.isFinite()) {
    return std::nullopt;
  }
  const double graph_initial_translation_m =
      input.initial_T_match_query.translation.norm();
  const auto* query_keyframe = FindKeyFrame(*map_, input.candidate.query_node);
  const auto* match_keyframe = FindKeyFrame(*map_, input.candidate.match_node);
  if (!query_keyframe || !match_keyframe) {
    return std::nullopt;
  }
  const auto* query_submap = FindFinishedSubmap(*map_, *query_keyframe);
  const auto* match_submap = FindFinishedSubmap(*map_, *match_keyframe);
  // There is deliberately no raw-keyframe or single-submap acceptance path.
  // The only support is a spatially cropped neighborhood of completed
  // submaps on both sides of the candidate.
  const auto clouds = BuildSubmapRegistrationClouds(
      *map_, *query_keyframe, *match_keyframe, config_.neighbor_submaps,
      config_.registration_crop_radius_m);
  if (!clouds) {
    LOG(INFO) << "[LoopClosure]: rejected query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value()
              << " reason=no_finished_submap_support";
    return std::nullopt;
  }
  const auto& source_cloud = clouds->first;
  const auto& target_cloud_base = clouds->second;
  if (source_cloud.size() < config_.minimum_source_points ||
      target_cloud_base.size() < config_.minimum_target_points) {
    LOG(INFO) << "[LoopClosure]: rejected query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value()
              << " support=joint_submaps reason=insufficient_submap_support"
              << " source_points=" << source_cloud.size()
              << " target_points=" << target_cloud_base.size();
    return std::nullopt;
  }
  LOG(INFO) << "[LoopClosure]: evaluating query_node="
            << input.candidate.query_node.value() << " match_node="
            << input.candidate.match_node.value()
            << " support=joint_submaps crop_radius="
            << config_.registration_crop_radius_m
            << " query_submap=" << query_submap->id.value()
            << " query_geometry_keyframes="
            << query_submap->geometry_keyframe_count
            << " query_geometry_points=" << query_submap->cloud_submap.size()
            << " query_geometry_input_points="
            << query_submap->geometry_input_point_count
            << " query_geometry_voxel="
            << query_submap->geometry_voxel_resolution_m
            << " match_submap=" << match_submap->id.value()
            << " match_geometry_keyframes="
            << match_submap->geometry_keyframe_count
            << " match_geometry_points=" << match_submap->cloud_submap.size()
            << " match_geometry_input_points="
            << match_submap->geometry_input_point_count
            << " match_geometry_voxel="
            << match_submap->geometry_voxel_resolution_m
            << " source_points=" << source_cloud.size()
            << " source_aabb={" << BoundsString(ComputeBounds(source_cloud))
            << "} target_points=" << target_cloud_base.size()
            << " target_aabb={"
            << BoundsString(ComputeBounds(target_cloud_base)) << '}';

  struct RegistrationHypothesis {
    const char* name;
    hybrid_slam::geometry::RigidTransform3d initial;
    double maximum_initial_translation_m;
    bool limit_absolute_translation;
    bool uniqueness_probe;
  };
  std::vector<RegistrationHypothesis> hypotheses;
  hypotheses.push_back({"graph", input.initial_T_match_query,
                        config_.maximum_initial_translation_m, false, false});
  if (input.candidate.coarse_yaw_rad) {
    auto yaw_with_graph_translation = input.initial_T_match_query;
    yaw_with_graph_translation.rotation = Eigen::AngleAxisd(
        *input.candidate.coarse_yaw_rad, Eigen::Vector3d::UnitZ());
    hypotheses.push_back({"descriptor_yaw_graph_translation",
                          yaw_with_graph_translation,
                          config_.maximum_initial_translation_m, false, false});
  }
  if (input.candidate.coarse_T_match_query) {
    hypotheses.push_back({"bevplace2_ransac",
                          *input.candidate.coarse_T_match_query,
                          config_.bevplace2
                              .maximum_registration_initial_translation_m,
                          true, false});
    const std::array<std::pair<const char*, Eigen::Vector3d>, 4> probes{{
        {"bevplace2_probe_pos_x",
         {config_.uniqueness_probe_translation_m, 0.0, 0.0}},
        {"bevplace2_probe_neg_x",
         {-config_.uniqueness_probe_translation_m, 0.0, 0.0}},
        {"bevplace2_probe_pos_y",
         {0.0, config_.uniqueness_probe_translation_m, 0.0}},
        {"bevplace2_probe_neg_y",
         {0.0, -config_.uniqueness_probe_translation_m, 0.0}},
    }};
    for (const auto& [name, offset] : probes) {
      auto perturbed = *input.candidate.coarse_T_match_query;
      perturbed.translation += offset;
      hypotheses.push_back({
          name, perturbed,
          config_.bevplace2.maximum_registration_initial_translation_m,
          true, true});
    }
  }

  hybrid_slam::localization::RelocalizationConfig registration =
      config_.registration;
  registration.enabled = false;
  const auto target = AsMapTarget(target_cloud_base);
  if (!target) {
    return std::nullopt;
  }
  const auto target_cloud = target->cloudMap().ownerAs<PclPointCloud>();
  if (!target_cloud) {
    return std::nullopt;
  }
  const hybrid_slam::localization::RelocalizationQuery query{
      {input.timestamp, FrameId::kOdom, FrameId::kBase,
       hybrid_slam::geometry::RigidTransform3d::Identity()},
      source_cloud,
      std::nullopt};

  struct AcceptedAttempt {
    const char* hypothesis = nullptr;
    hybrid_slam::geometry::RigidTransform3d initial;
    hybrid_slam::localization::RegistrationVerification verification;
    LoopGeometryEvidence evidence;
  };
  std::vector<hybrid_slam::geometry::RigidTransform3d> attempted_initials;
  std::vector<AcceptedAttempt> accepted_attempts;
  std::vector<AcceptedAttempt> accepted_uniqueness_probes;
  for (const auto& hypothesis : hypotheses) {
    const double initial_translation_m = hypothesis.initial.translation.norm();
    LOG(INFO) << "[LoopClosure]: initializer query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value() << " name="
              << hypothesis.name << ' ' << PoseString(hypothesis.initial);
    if (hypothesis.limit_absolute_translation &&
        initial_translation_m > hypothesis.maximum_initial_translation_m) {
      LOG(INFO) << "[LoopClosure]: skipped query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " initializer=" << hypothesis.name
                << " reason=outside_capture_domain"
                << " initial_translation=" << initial_translation_m
                << " m max_initial_translation="
                << hypothesis.maximum_initial_translation_m << " m";
      continue;
    }
    if (!hypothesis.uniqueness_probe) {
      attempted_initials.push_back(hypothesis.initial);
    }
    const hybrid_slam::localization::RelocalizationCandidate candidate{
        {input.timestamp, FrameId::kMap, FrameId::kBase, hypothesis.initial},
        input.candidate.retrieval_score};
    std::optional<PclRegistrationDiagnostics> diagnostics;
    std::optional<LoopGeometryEvidence> strict_evidence;
    std::ostringstream debug_prefix;
    debug_prefix << "q" << input.candidate.query_node.value() << "_m"
                 << input.candidate.match_node.value()
                 << "_joint_submaps_" << hypothesis.name;
    PclRegistrationVerifier verifier(
        registration,
        [target_cloud](const BASIC::SE3&) { return target_cloud; }, {},
        [&](const PclRegistrationDiagnostics& update,
            const PclPointCloud& prepared_source,
            const PclPointCloud& prepared_target,
            const PclPointCloud& aligned_source) {
          diagnostics = update;
          strict_evidence = EvaluateLoopGeometry(
              aligned_source, prepared_target,
              std::max(0.1, registration.source_voxel_size_m * 2.0),
              config_.support_voxel_size_m,
              config_.minimum_support_z_extent_m,
              config_.minimum_support_eigenvalue_ratio);
          LOG(INFO) << "[LoopClosure]: voxelized_geometry query_node="
                    << input.candidate.query_node.value() << " match_node="
                    << input.candidate.match_node.value() << " initializer="
                    << hypothesis.name << " source_aabb={"
                    << BoundsString(ComputeBounds(prepared_source))
                    << "} target_aabb={"
                    << BoundsString(ComputeBounds(prepared_target))
                    << "} aligned_aabb={"
                    << BoundsString(ComputeBounds(aligned_source)) << '}';
          SaveRegistrationDebug(config_.debug_directory, debug_prefix.str(),
                                prepared_source, prepared_target,
                                aligned_source);
        });
    const auto verification = verifier.Verify(query, *target, candidate, {});
    if (diagnostics) {
      LOG(INFO) << "[LoopClosure]: registration_result query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " support=joint_submaps initializer=" << hypothesis.name
                << " coarse_points=" << diagnostics->coarse_source_points
                << '/' << diagnostics->coarse_target_points
                << " fine_points=" << diagnostics->source_points << '/'
                << diagnostics->target_points
                << " coarse_converged=" << diagnostics->coarse_converged
                << " coarse_fitness=" << diagnostics->coarse_fitness
                << " converged=" << diagnostics->converged
                << " fitness=" << diagnostics->fitness << " final_"
                << PoseString(diagnostics->final_T_target_source);
    }
    if (!verification || !strict_evidence) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " support=joint_submaps initializer=" << hypothesis.name
                << " reason=registration_failed initial_translation="
                << initial_translation_m << " m";
      continue;
    }
    const auto& evidence = *strict_evidence;
    LOG(INFO) << "[LoopClosure]: geometry_evidence query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value() << " initializer="
              << hypothesis.name
              << " source_inlier=" << evidence.source_inlier_ratio
              << " target_inlier=" << evidence.target_inlier_ratio
              << " mutual_inlier=" << evidence.mutual_inlier_ratio
              << " bidirectional_overlap=" << evidence.bidirectional_overlap
              << " bidirectional_inlier_rmse="
              << evidence.bidirectional_inlier_rmse_m
              << " support_extent=" << evidence.support_extent_m
              << " support_z_extent=" << evidence.support_z_extent_m
              << " eigenvalue_ratio=" << evidence.eigenvalue_ratio
              << " support_voxels=" << evidence.support_voxels
              << " observable_6dof=" << evidence.observable_6dof;
    if (verification->inlier_ratio < config_.minimum_inlier_ratio ||
        verification->overlap < config_.minimum_overlap ||
        !verification->observable ||
        evidence.bidirectional_overlap <
            config_.minimum_bidirectional_overlap ||
        evidence.mutual_inlier_ratio < config_.minimum_mutual_inlier_ratio ||
        evidence.bidirectional_inlier_rmse_m >
            config_.maximum_bidirectional_inlier_rmse_m ||
        evidence.support_extent_m < config_.minimum_support_extent_m ||
        evidence.support_z_extent_m < config_.minimum_support_z_extent_m ||
        evidence.eigenvalue_ratio <
            config_.minimum_support_eigenvalue_ratio ||
        evidence.support_voxels < config_.minimum_support_voxels ||
        !evidence.observable_6dof) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " support=joint_submaps initializer=" << hypothesis.name
                << " reason=loop_geometry_gate";
      continue;
    }

    const auto& final = verification->T_map_base.T_target_source;
    const Eigen::Vector3d final_rpy = RollPitchYaw(final);
    if (std::abs(final.translation.z()) > config_.maximum_relative_z_m ||
        std::abs(final_rpy.x()) > config_.maximum_relative_roll_pitch_rad ||
        std::abs(final_rpy.y()) > config_.maximum_relative_roll_pitch_rad) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " initializer=" << hypothesis.name
                << " reason=relative_pose_sanity final_" << PoseString(final);
      continue;
    }
    if (input.candidate.coarse_T_match_query &&
        !hypothesis.uniqueness_probe) {
      const auto coarse_delta = input.candidate.coarse_T_match_query->inverse()
                                    .compose(final);
      const double planar_disagreement =
          coarse_delta.translation.head<2>().norm();
      const double yaw_disagreement =
          std::abs(RollPitchYaw(coarse_delta).z());
      if (planar_disagreement >
              config_.maximum_coarse_translation_disagreement_m ||
          yaw_disagreement > config_.maximum_coarse_yaw_disagreement_rad) {
        LOG(INFO) << "[LoopClosure]: rejected query_node="
                  << input.candidate.query_node.value() << " match_node="
                  << input.candidate.match_node.value()
                  << " initializer=" << hypothesis.name
                  << " reason=coarse_3d_disagreement planar="
                  << planar_disagreement << " yaw=" << yaw_disagreement;
        continue;
      }
    }
    if (hypothesis.uniqueness_probe) {
      accepted_uniqueness_probes.push_back(
          {hypothesis.name, hypothesis.initial, *verification, evidence});
    } else {
      accepted_attempts.push_back(
          {hypothesis.name, hypothesis.initial, *verification, evidence});
    }
  }
  if (accepted_attempts.empty()) {
    // FastVGICP occasionally reports hasConverged=false for the unperturbed
    // BEV start even though two independently perturbed starts converge to the
    // same strict-geometry solution.  In that case the probes are stronger
    // evidence than insisting that the badly drifted graph start also work.
    // Promote one probe only when enough other accepted probes remain to pass
    // the normal uniqueness consensus below.
    if (!input.candidate.coarse_T_match_query ||
        accepted_uniqueness_probes.size() <=
            config_.minimum_uniqueness_probe_consensus) {
      return std::nullopt;
    }
    const auto promoted = std::min_element(
        accepted_uniqueness_probes.begin(),
        accepted_uniqueness_probes.end(),
        [](const auto& lhs, const auto& rhs) {
          return lhs.verification.fitness < rhs.verification.fitness;
        });
    LOG(INFO) << "[LoopClosure]: promoted_probe query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value() << " initializer="
              << promoted->hypothesis << " remaining_probe_consensus="
              << accepted_uniqueness_probes.size() - 1;
    accepted_attempts.push_back(*promoted);
    accepted_uniqueness_probes.erase(promoted);
  }

  const auto best = std::min_element(
      accepted_attempts.begin(), accepted_attempts.end(),
      [](const auto& lhs, const auto& rhs) {
        return lhs.verification.fitness < rhs.verification.fitness;
      });
  bool distinct_initializer_attempted = false;
  for (const auto& initial : attempted_initials) {
    if ((initial.translation - best->initial.translation).norm() >
            config_.maximum_solution_disagreement_m ||
        RotationDistance(initial, best->initial) >
            config_.maximum_solution_disagreement_rad) {
      distinct_initializer_attempted = true;
      break;
    }
  }
  if (distinct_initializer_attempted) {
    bool consensus = false;
    const auto& best_pose = best->verification.T_map_base.T_target_source;
    for (const auto& attempt : accepted_attempts) {
      const bool distinct_initial =
          (attempt.initial.translation - best->initial.translation).norm() >
              config_.maximum_solution_disagreement_m ||
          RotationDistance(attempt.initial, best->initial) >
              config_.maximum_solution_disagreement_rad;
      if (!distinct_initial) {
        continue;
      }
      const auto& pose = attempt.verification.T_map_base.T_target_source;
      if ((pose.translation - best_pose.translation).norm() <=
              config_.maximum_solution_disagreement_m &&
          RotationDistance(pose, best_pose) <=
              config_.maximum_solution_disagreement_rad) {
        consensus = true;
        break;
      }
    }
    if (!consensus) {
      for (const auto& probe : accepted_uniqueness_probes) {
        const bool distinct_initial =
            (probe.initial.translation - best->initial.translation).norm() >
                config_.maximum_solution_disagreement_m ||
            RotationDistance(probe.initial, best->initial) >
                config_.maximum_solution_disagreement_rad;
        if (!distinct_initial) {
          continue;
        }
        const auto& pose = probe.verification.T_map_base.T_target_source;
        if ((pose.translation - best_pose.translation).norm() <=
                config_.maximum_solution_disagreement_m &&
            RotationDistance(pose, best_pose) <=
                config_.maximum_solution_disagreement_rad) {
          consensus = true;
          break;
        }
      }
    }
    if (!consensus) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " reason=initializer_consensus_failed accepted="
                << accepted_attempts.size();
      return std::nullopt;
    }
  }

  if (input.candidate.coarse_T_match_query) {
    const auto& best_pose = best->verification.T_map_base.T_target_source;
    const auto coarse_delta =
        input.candidate.coarse_T_match_query->inverse().compose(best_pose);
    const double planar_disagreement =
        coarse_delta.translation.head<2>().norm();
    const double yaw_disagreement = std::abs(RollPitchYaw(coarse_delta).z());
    if (planar_disagreement >
            config_.maximum_coarse_translation_disagreement_m ||
        yaw_disagreement > config_.maximum_coarse_yaw_disagreement_rad) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " reason=coarse_3d_disagreement_best planar="
                << planar_disagreement << " yaw=" << yaw_disagreement;
      return std::nullopt;
    }
    std::size_t agreeing_probes = 0;
    std::size_t ambiguous_probes = 0;
    for (const auto& probe : accepted_uniqueness_probes) {
      const auto& pose = probe.verification.T_map_base.T_target_source;
      if ((pose.translation - best_pose.translation).norm() <=
              config_.maximum_solution_disagreement_m &&
          RotationDistance(pose, best_pose) <=
              config_.maximum_solution_disagreement_rad) {
        ++agreeing_probes;
      } else {
        ++ambiguous_probes;
      }
    }
    if (ambiguous_probes > 0 ||
        agreeing_probes < config_.minimum_uniqueness_probe_consensus) {
      LOG(INFO) << "[LoopClosure]: rejected query_node="
                << input.candidate.query_node.value() << " match_node="
                << input.candidate.match_node.value()
                << " reason=non_unique_registration_basin"
                << " agreeing_probes=" << agreeing_probes
                << " ambiguous_probes=" << ambiguous_probes
                << " required="
                << config_.minimum_uniqueness_probe_consensus;
      return std::nullopt;
    }
  }

  const auto T_initial_verified = input.initial_T_match_query.inverse().compose(
      best->verification.T_map_base.T_target_source);
  const double graph_correction_translation_m =
      T_initial_verified.translation.norm();
  const double graph_correction_rotation_rad =
      Eigen::AngleAxisd(T_initial_verified.rotation.normalized()).angle();
  if (config_.plugin == "bevplace2" &&
      !input.candidate.coarse_T_match_query &&
      (graph_correction_translation_m >
           config_.maximum_pose_free_graph_correction_m ||
       graph_correction_rotation_rad >
           config_.maximum_pose_free_graph_correction_rad)) {
    LOG(INFO) << "[LoopClosure]: rejected query_node="
              << input.candidate.query_node.value() << " match_node="
              << input.candidate.match_node.value()
              << " reason=pose_free_graph_disagreement translation="
              << graph_correction_translation_m << " m rotation="
              << graph_correction_rotation_rad << " rad limits="
              << config_.maximum_pose_free_graph_correction_m << " m/"
              << config_.maximum_pose_free_graph_correction_rad << " rad";
    return std::nullopt;
  }

  hybrid_slam::backend::Covariance6d covariance =
      hybrid_slam::backend::Covariance6d::Zero();
  const double residual_rmse = best->evidence.bidirectional_inlier_rmse_m;
  // The inlier RMSE measures individual surface correspondences, not the
  // uncertainty of the rigid pose estimated from the whole support volume.
  // Treat only sqrt(number of occupied 3D support voxels) as independent to
  // remain conservative under spatial correlation; the standard error then
  // falls with the fourth root of the voxel count.  A configurable systematic
  // floor still covers calibration, deskew and local-map deformation.
  const double correlated_support = std::sqrt(
      static_cast<double>(std::max<std::size_t>(
          1, best->evidence.support_voxels)));
  const double pose_translation_standard_error =
      residual_rmse / std::sqrt(correlated_support);
  const double translation_sigma_m = std::clamp(
      pose_translation_standard_error, config_.translation_sigma_m, 2.0);
  const double rotation_sigma_rad = std::clamp(
      std::atan2(translation_sigma_m,
                 std::max(1.0, best->evidence.support_extent_m * 0.5)),
      config_.rotation_sigma_rad, 0.35);
  covariance.diagonal().head<3>().setConstant(
      rotation_sigma_rad * rotation_sigma_rad);
  covariance.diagonal().tail<3>().setConstant(
      translation_sigma_m * translation_sigma_m);
  const Eigen::Vector3d verified_rpy =
      RollPitchYaw(best->verification.T_map_base.T_target_source);
  const Eigen::Vector3d initial_rpy =
      RollPitchYaw(input.initial_T_match_query);
  LOG(INFO) << "[LoopClosure]: accepted query_node="
            << input.candidate.query_node.value() << " match_node="
            << input.candidate.match_node.value() << " initializer="
            << best->hypothesis << " support=joint_submaps"
            << " fitness=" << best->verification.fitness
            << " verified_translation=["
            << best->verification.T_map_base.T_target_source.translation.x()
            << ","
            << best->verification.T_map_base.T_target_source.translation.y()
            << ","
            << best->verification.T_map_base.T_target_source.translation.z()
            << "] verified_rpy=[" << verified_rpy.x() << ','
            << verified_rpy.y() << ',' << verified_rpy.z() << ']'
            << " graph_initial_translation="
            << graph_initial_translation_m << " m initial_to_verified="
            << T_initial_verified.translation.x() << ","
            << T_initial_verified.translation.y() << ","
            << T_initial_verified.translation.z() << " m delta_rpy=["
            << (verified_rpy.x() - initial_rpy.x()) << ','
            << (verified_rpy.y() - initial_rpy.y()) << ','
            << (verified_rpy.z() - initial_rpy.z()) << "] rad norm="
            << graph_correction_rotation_rad << " rad covariance_sigma="
            << translation_sigma_m << " m/" << rotation_sigma_rad << " rad"
            << " point_rmse=" << residual_rmse
            << " m support_voxels=" << best->evidence.support_voxels
            << " pose_standard_error="
            << pose_translation_standard_error << " m";
  return RelativePoseMeasurement{
      input.candidate.match_node,
      input.candidate.query_node,
      input.timestamp,
      best->verification.T_map_base.T_target_source,
      covariance,
  };
}

}  // namespace LI2Sup
