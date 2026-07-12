#ifndef ADAGRAPH_TYPES_H_
#define ADAGRAPH_TYPES_H_

#include <cstddef>

#include <Eigen/Core>

#include "basic/Manifold.h"
#include "basic/alias.h"

namespace LI2Sup {

struct FeatureFrame {
  BASIC::V3 relative_translation;
  BASIC::V3 relative_rotation;
  double scan_matching_residual = 0.0;

  double hessian_eig[6] = {0.0};
  double hessian_trace = 0.0;
  double condition_number = 1.0;
  double eigenvalue_ratio = 1.0;
  double effective_rank = 0.0;

  double residual_mean = 0.0;
  double residual_std = 0.0;
  double residual_median = 0.0;
  double residual_mad = 0.0;
  double residual_max = 0.0;
  double residual_p90 = 0.0;
  double residual_p95 = 0.0;
  double huber_loss_mean = 0.0;

  int effective_point_number = 0;
  double inlier_ratio = 0.0;
  int residual_cluster_count = 0;
  double normal_entropy = 0.0;

  double local_map_density = 0.0;
  double voxel_occupancy = 0.0;
  double point_age_mean = 0.0;

  BASIC::V3 imu_angular_velocity = BASIC::V3::Zero();
  BASIC::V3 imu_linear_acceleration = BASIC::V3::Zero();
  double angular_velocity_norm = 0.0;
  double acceleration_norm = 0.0;
  double motion_distortion_score = 0.0;

  double innovation_norm = 0.0;
  int eskf_iteration_count = 0;
  double convergence_state = 0.0;
  double prediction_error = 0.0;
};

struct BackendResult {
  bool optimized = false;
  BASIC::SE3 optimized_pose;
  Eigen::Matrix<double, 6, 1> sigmas = Eigen::Matrix<double, 6, 1>::Zero();
  std::size_t keyframe_count = 0;
};

}  // namespace LI2Sup

#endif
