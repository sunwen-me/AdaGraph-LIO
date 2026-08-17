#ifndef HYBRID_SLAM_FRONTEND_POINT_LIO_CONFIG_H_
#define HYBRID_SLAM_FRONTEND_POINT_LIO_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>

#include <Eigen/Core>

namespace hybrid_slam::frontend {

// Point-LIO-specific tuning. Shared range/filtering, gravity, local-map
// resolution and lidar extrinsics live in FrontendConfig, so this type cannot
// drift from the downstream frontend contract.
struct PointLIOConfig {
  bool space_downsample = true;

  bool check_saturation = true;
  bool fix_gravity_direction = true;
  double saturation_acceleration_mps2 = 2.97;
  double saturation_angular_velocity_radps = 34.65;
  double acceleration_norm_mps2 = 9.81;

  std::size_t initial_map_size = 10;

  bool estimate_extrinsic = false;

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
    if (!finite(acceleration_norm_mps2) || acceleration_norm_mps2 <= 0.0) {
      return "point_lio.acceleration_norm_mps2 must be finite and positive";
    }
    if (!finite(saturation_acceleration_mps2) ||
        !finite(saturation_angular_velocity_radps) || initial_map_size == 0) {
      return "point_lio saturation and map initialization values are invalid";
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

}  // namespace hybrid_slam::frontend

#endif  // HYBRID_SLAM_FRONTEND_POINT_LIO_CONFIG_H_
