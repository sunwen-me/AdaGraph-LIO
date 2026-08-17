#ifndef HYBRID_SLAM_SENSOR_IMU_DATA_H_
#define HYBRID_SLAM_SENSOR_IMU_DATA_H_

#include <Eigen/Core>

#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::sensor {

struct ImuData {
  hybrid_slam::common::Timestamp timestamp;
  hybrid_slam::geometry::FrameId frame = hybrid_slam::geometry::FrameId::kImu;
  Eigen::Vector3d linear_acceleration_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_radps = Eigen::Vector3d::Zero();

  [[nodiscard]] bool valid() const noexcept {
    return frame != hybrid_slam::geometry::FrameId::kUnknown &&
           linear_acceleration_mps2.allFinite() &&
           angular_velocity_radps.allFinite();
  }
};

}  // namespace hybrid_slam::sensor

#endif  // HYBRID_SLAM_SENSOR_IMU_DATA_H_
