#ifndef ADAGRAPH_FRONTEND_POINT_LIO_CONFIG_H_
#define ADAGRAPH_FRONTEND_POINT_LIO_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Core>

#include "adagraph/geometry/transform3.h"

namespace adagraph::frontend {

// Project-owned configuration for the Point-LIO adapter.  It deliberately
// describes physical values (metres, radians and covariances) rather than the
// vendored small_point_lio::Parameters representation.  Conversion to that
// representation happens exactly once inside point_lio_adapter.
struct PointLIOConfig {
  std::size_t point_filter_stride = 1;
  double min_range_m = 0.5;
  double max_range_m = 1000.0;
  bool space_downsample = true;
  double space_downsample_leaf_size_m = 0.5;

  Eigen::Vector3d gravity_mps2{0.0, 0.0, -9.81};
  bool check_saturation = true;
  bool fix_gravity_direction = true;
  double saturation_acceleration_mps2 = 2.97;
  double saturation_angular_velocity_radps = 34.65;
  double acceleration_norm_mps2 = 9.81;

  double local_map_resolution_m = 0.5;
  std::size_t initial_map_size = 10;

  bool estimate_extrinsic = false;
  adagraph::geometry::StaticTransform3d T_imu_lidar{
      adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar,
      adagraph::geometry::RigidTransform3d::Identity(),
  };

  double lidar_point_covariance = 0.01;
  double imu_acceleration_measurement_covariance = 0.01;
  double imu_angular_velocity_measurement_covariance = 0.01;
  double velocity_covariance = 20.0;
  double acceleration_covariance = 500.0;
  double angular_velocity_covariance = 1000.0;
  double accelerometer_bias_covariance = 0.0001;
  double gyroscope_bias_covariance = 0.0001;
  double plane_threshold_m = 0.1;
  double matching_distance_squared_m2 = 81.0;
  bool publish_odometry_without_downsample = false;

  [[nodiscard]] std::optional<std::string> validate() const {
    const auto finite = [](const double value) { return std::isfinite(value); };
    if (point_filter_stride == 0 ||
        point_filter_stride >
            static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      return "point_lio.point_filter_stride must fit a positive int";
    }
    if (!finite(min_range_m) || min_range_m < 0.0 || !finite(max_range_m) ||
        max_range_m <= min_range_m) {
      return "point_lio range bounds must be finite and ordered";
    }
    if (space_downsample &&
        (!finite(space_downsample_leaf_size_m) ||
         space_downsample_leaf_size_m <= 0.0)) {
      return "point_lio.space_downsample_leaf_size_m must be positive";
    }
    if (!gravity_mps2.allFinite() || gravity_mps2.norm() <= 0.0 ||
        !finite(acceleration_norm_mps2) || acceleration_norm_mps2 <= 0.0) {
      return "point_lio gravity and acceleration norm must be finite and positive";
    }
    if (!finite(saturation_acceleration_mps2) ||
        !finite(saturation_angular_velocity_radps) ||
        !finite(local_map_resolution_m) || local_map_resolution_m <= 0.0 ||
        initial_map_size == 0) {
      return "point_lio saturation and map initialization values are invalid";
    }
    if (!T_imu_lidar.valid() ||
        T_imu_lidar.target_frame != adagraph::geometry::FrameId::kImu ||
        T_imu_lidar.source_frame != adagraph::geometry::FrameId::kLidar) {
      return "point_lio.T_imu_lidar must be a finite lidar-to-imu transform";
    }
    for (const double value : {
             lidar_point_covariance,
             imu_acceleration_measurement_covariance,
             imu_angular_velocity_measurement_covariance,
             velocity_covariance,
             acceleration_covariance,
             angular_velocity_covariance,
             accelerometer_bias_covariance,
             gyroscope_bias_covariance,
             plane_threshold_m,
             matching_distance_squared_m2,
         }) {
      if (!finite(value)) {
        return "point_lio covariance and matching values must be finite";
      }
    }
    if (lidar_point_covariance <= 0.0 || plane_threshold_m <= 0.0 ||
        matching_distance_squared_m2 <= 0.0) {
      return "point_lio lidar covariance and matching thresholds must be positive";
    }
    return std::nullopt;
  }
};

}  // namespace adagraph::frontend

#endif  // ADAGRAPH_FRONTEND_POINT_LIO_CONFIG_H_
