#ifndef ADAGRAPH_FRONTEND_FRONTEND_CONFIG_H_
#define ADAGRAPH_FRONTEND_FRONTEND_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

#include "adagraph/geometry/transform3.h"

namespace adagraph::frontend {

// Algorithm-independent construction values shared by frontend adapters.
// Frontend-specific tuning remains in the concrete implementation config.
struct FrontendConfig {
  double min_range_m = 0.1;
  double max_range_m = 200.0;
  std::size_t point_filter_stride = 1;
  double voxel_filter_size_m = 0.2;
  double local_map_resolution_m = 0.5;
  std::size_t local_map_capacity = 100'000;
  double gravity_mps2 = 9.81;
  adagraph::geometry::StaticTransform3d T_imu_lidar{
      adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar,
      adagraph::geometry::RigidTransform3d::Identity(),
  };
  adagraph::geometry::StaticTransform3d T_base_imu{
      adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu,
      adagraph::geometry::RigidTransform3d::Identity(),
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
    if (local_map_capacity == 0) {
      return "frontend.local_map_capacity must be greater than zero";
    }
    if (!std::isfinite(gravity_mps2) || gravity_mps2 <= 0.0) {
      return "frontend.gravity_mps2 must be finite and positive";
    }
    if (!T_imu_lidar.valid() ||
        T_imu_lidar.target_frame != adagraph::geometry::FrameId::kImu ||
        T_imu_lidar.source_frame != adagraph::geometry::FrameId::kLidar) {
      return "frontend.T_imu_lidar must be a finite lidar-to-imu transform";
    }
    if (!T_base_imu.valid() ||
        T_base_imu.target_frame != adagraph::geometry::FrameId::kBase ||
        T_base_imu.source_frame != adagraph::geometry::FrameId::kImu) {
      return "frontend.T_base_imu must be a finite imu-to-base transform";
    }
    return std::nullopt;
  }
};

}  // namespace adagraph::frontend

#endif  // ADAGRAPH_FRONTEND_FRONTEND_CONFIG_H_
