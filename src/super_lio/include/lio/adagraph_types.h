#ifndef ADAGRAPH_TYPES_H_
#define ADAGRAPH_TYPES_H_

#include <cstddef>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "basic/Manifold.h"
#include "basic/alias.h"

namespace LI2Sup {

struct FeatureFrame {
  // Relative motion since the last keyframe, both expressed in the last
  // keyframe body frame: translation R_last^T * (t_cur - t_last) [m],
  // rotation Log(R_last^T * R_cur) axis-angle [rad].
  BASIC::V3 relative_translation = BASIC::V3::Zero();
  BASIC::V3 relative_rotation = BASIC::V3::Zero();

  // Eigen-structure of the accumulated 6x6 scan-matching Hessian
  // H = sum_i J_i * w * J_i^T with tangent ordering [rot(0..2), trans(3..5)].
  // H carries the frontend observation weight w = 1/sigma^2 ~= 1000, so
  // informative directions over thousands of points reach ~1e5..1e7.
  double hessian_eig[6] = {0.0};  // ascending eigenvalues of H
  double hessian_trace = 0.0;
  double condition_number = 1.0;  // max/min eigenvalue of H
  double eigenvalue_ratio = 1.0;  // min/max eigenvalue of H
  double effective_rank = 0.0;    // eigenvalue count above weight-scaled floor

  // Norm of the accumulated weighted gradient H^T V r. The frontend exposes
  // only this single aggregated 6-vector, not per-point residuals, so no
  // residual distribution statistics can be computed here.
  double gradient_norm = 0.0;

  // Spatial scatter of the downsampled scan: eigenvalues of the 3x3
  // point-coordinate covariance [m^2]. Describes scene shape/extent, not
  // scan-matching observability.
  double scene_scatter_eig[3] = {0.0};  // ascending
  double scene_scatter_trace = 0.0;
  double scene_scatter_condition_number = 1.0;
  double scene_scatter_eigenvalue_ratio = 1.0;
  double scene_scatter_effective_rank = 0.0;

  int effective_point_number = 0;
  double inlier_ratio = 0.0;

  BASIC::V3 imu_angular_velocity = BASIC::V3::Zero();
  BASIC::V3 imu_linear_acceleration = BASIC::V3::Zero();
  double angular_velocity_norm = 0.0;
  double acceleration_norm = 0.0;
};

// One accepted VGICP registration against the prior map: absolute body pose
// in the map frame plus the registration fitness (mean squared correspondence
// distance [m^2], lower is better).
struct MapObservation {
  double timestamp = 0.0;
  BASIC::SE3 map_lidar;
  double fitness = 0.0;
};

struct BackendResult {
  bool optimized = false;
  BASIC::SE3 optimized_pose;
  Eigen::Matrix<double, 6, 1> sigmas = Eigen::Matrix<double, 6, 1>::Zero();
  std::size_t keyframe_count = 0;
  // Optimized map-frame keyframe trajectory {timestamp [s], pose}; filled
  // only when localization factors are enabled.
  std::vector<std::pair<double, BASIC::SE3>> optimized_trajectory;
};

}  // namespace LI2Sup

#endif
