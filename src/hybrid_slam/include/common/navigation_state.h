#ifndef LI2SUP_NAVIGATION_STATE_H_
#define LI2SUP_NAVIGATION_STATE_H_

#include <Eigen/Core>

#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/sensor/imu_data.h"

namespace LI2Sup {

// Value types shared by the Point-LIO adapter and pipeline. They intentionally
// contain no PCL, logging or vendor math types.
struct NavState {
  double timestamp = 0;
  hybrid_slam::geometry::RigidTransform3d T_odom_imu;
  Eigen::Vector3d velocity_odom = Eigen::Vector3d::Zero();
};

using IMUData = hybrid_slam::sensor::ImuData;

}  // namespace LI2Sup

#endif  // LI2SUP_NAVIGATION_STATE_H_
