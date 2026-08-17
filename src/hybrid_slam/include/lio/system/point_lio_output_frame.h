#ifndef LI2SUP_SYSTEM_POINT_LIO_OUTPUT_FRAME_H_
#define LI2SUP_SYSTEM_POINT_LIO_OUTPUT_FRAME_H_

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "common/navigation_state.h"
#include "hybrid_slam/geometry/transform3.h"

namespace LI2Sup {

// A raw LiDAR frame placed only by the IMU propagation captured after the
// frontend entered LOST.  It deliberately does not reuse the vendor's
// scan-to-map corrected pose: that state is precisely what recovery is meant
// to replace.
struct RecoveryQueryFrame {
  hybrid_slam::geometry::StampedTransform3d T_odom_base;
  std::vector<Eigen::Vector3f> points_odom;

  [[nodiscard]] bool valid() const noexcept {
    return T_odom_base.valid() &&
           T_odom_base.target_frame == hybrid_slam::geometry::FrameId::kOdom &&
           T_odom_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           !points_odom.empty();
  }
};

// Owning output of the Point-LIO adapter. The point vectors are
// moved from the vendor frame, so the system layer does not depend on vendor
// types and the transfer performs no per-point copy.
struct PointLIOOutputFrame {
  NavState state;
  std::vector<Eigen::Vector3f> dense_points_odom;
  std::vector<Eigen::Vector3f> matching_points_odom;
  std::optional<RecoveryQueryFrame> recovery_query;
  Eigen::Matrix<double, 6, 6> lidar_hessian =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> lidar_gradient =
      Eigen::Matrix<double, 6, 1>::Zero();
  std::size_t attempted_point_updates = 0;
  std::size_t effective_point_updates = 0;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_POINT_LIO_OUTPUT_FRAME_H_
