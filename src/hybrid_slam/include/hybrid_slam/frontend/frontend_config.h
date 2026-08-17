#ifndef HYBRID_SLAM_FRONTEND_FRONTEND_CONFIG_H_
#define HYBRID_SLAM_FRONTEND_FRONTEND_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::frontend {

// Project-owned frontend configuration. Values here describe shared physical
// input and map semantics; an algorithm adapter may add only its own tuning
// fields and converts this value exactly once to its vendor representation.
struct FrontendConfig {
  double min_range_m = 0.5;
  double max_range_m = 1000.0;
  std::size_t point_filter_stride = 1;
  double voxel_filter_size_m = 0.5;
  double local_map_resolution_m = 0.5;
  Eigen::Vector3d gravity_mps2{0.0, 0.0, -9.81};
  hybrid_slam::geometry::StaticTransform3d T_imu_lidar{
      hybrid_slam::geometry::FrameId::kImu,
      hybrid_slam::geometry::FrameId::kLidar,
      hybrid_slam::geometry::RigidTransform3d::Identity(),
  };
  hybrid_slam::geometry::StaticTransform3d T_base_imu{
      hybrid_slam::geometry::FrameId::kBase,
      hybrid_slam::geometry::FrameId::kImu,
      hybrid_slam::geometry::RigidTransform3d::Identity(),
  };

  // Validation runs during construction/configuration, outside the realtime
  // path. A string is allocated only when reporting an invalid config.
  [[nodiscard]] std::optional<std::string> validate() const {
    if (!std::isfinite(min_range_m) || min_range_m < 0.0) {
      return "frontend.min_range_m must be finite and non-negative";
    }
    if (!std::isfinite(max_range_m) || max_range_m <= min_range_m) {
      return "frontend.max_range_m must be finite and greater than min_range_m";
    }
    if (point_filter_stride == 0) {
      return "frontend.point_filter_stride must be greater than zero";
    }
    if (!std::isfinite(voxel_filter_size_m) || voxel_filter_size_m <= 0.0) {
      return "frontend.voxel_filter_size_m must be finite and positive";
    }
    if (!std::isfinite(local_map_resolution_m) ||
        local_map_resolution_m <= 0.0) {
      return "frontend.local_map_resolution_m must be finite and positive";
    }
    if (!gravity_mps2.allFinite() || gravity_mps2.norm() <= 0.0) {
      return "frontend.gravity_mps2 must be finite and non-zero";
    }
    if (!T_imu_lidar.valid() ||
        T_imu_lidar.target_frame != hybrid_slam::geometry::FrameId::kImu ||
        T_imu_lidar.source_frame != hybrid_slam::geometry::FrameId::kLidar) {
      return "frontend.T_imu_lidar must be a finite lidar-to-imu transform";
    }
    if (!T_base_imu.valid() ||
        T_base_imu.target_frame != hybrid_slam::geometry::FrameId::kBase ||
        T_base_imu.source_frame != hybrid_slam::geometry::FrameId::kImu) {
      return "frontend.T_base_imu must be a finite imu-to-base transform";
    }
    return std::nullopt;
  }
};

}  // namespace hybrid_slam::frontend

#endif  // HYBRID_SLAM_FRONTEND_FRONTEND_CONFIG_H_
