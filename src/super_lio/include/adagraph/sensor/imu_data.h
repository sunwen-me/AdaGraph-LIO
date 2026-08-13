#ifndef ADAGRAPH_SENSOR_IMU_DATA_H_
#define ADAGRAPH_SENSOR_IMU_DATA_H_

#include <Eigen/Core>

#include "adagraph/common/timestamp.h"
#include "adagraph/geometry/transform3.h"

namespace adagraph::sensor {

struct ImuData {
  adagraph::common::Timestamp timestamp;
  adagraph::geometry::FrameId frame = adagraph::geometry::FrameId::kImu;
  Eigen::Vector3d linear_acceleration_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity_radps = Eigen::Vector3d::Zero();

  [[nodiscard]] bool valid() const noexcept {
    return frame != adagraph::geometry::FrameId::kUnknown &&
           linear_acceleration_mps2.allFinite() &&
           angular_velocity_radps.allFinite();
  }
};

}  // namespace adagraph::sensor

#endif  // ADAGRAPH_SENSOR_IMU_DATA_H_
