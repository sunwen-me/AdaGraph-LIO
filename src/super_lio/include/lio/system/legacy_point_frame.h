#ifndef LI2SUP_SYSTEM_LEGACY_POINT_FRAME_H_
#define LI2SUP_SYSTEM_LEGACY_POINT_FRAME_H_

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include "common/legacy_state.h"

namespace LI2Sup {

// Transitional owning output of the Point-LIO adapter. The point vectors are
// moved from the vendor frame, so the system layer does not depend on vendor
// types and the transfer performs no per-point copy.
struct LegacyPointFrame {
  NavState state;
  std::vector<Eigen::Vector3f> dense_points_odom;
  std::vector<Eigen::Vector3f> matching_points_odom;
  Eigen::Matrix<double, 6, 6> lidar_hessian =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> lidar_gradient =
      Eigen::Matrix<double, 6, 1>::Zero();
  std::size_t attempted_point_updates = 0;
  std::size_t effective_point_updates = 0;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_LEGACY_POINT_FRAME_H_
