#include "lio/adagraph_feature_extractor.h"

#include <algorithm>
#include <cmath>

namespace LI2Sup {

FeatureFrame AdaGraphFeatureExtractor::build(const Input& input) {
  FeatureFrame frame;
  fillRelativePose(input, frame);
  fillResidualStats(input, frame);
  fillGeometryStats(input, frame);
  fillMatchAndMotionStats(input, frame);
  return frame;
}

void AdaGraphFeatureExtractor::fillRelativePose(const Input& input, FeatureFrame& frame) {
  frame.relative_translation = input.current_pose.t_ - input.last_keyframe_pose.t_;

  BASIC::M3 rel_R = input.last_keyframe_pose.R_.transpose() * input.current_pose.R_;
  Eigen::AngleAxisd aa(rel_R.cast<double>());
  frame.relative_rotation = (aa.axis() * aa.angle()).cast<BASIC::scalar>();
  frame.scan_matching_residual = 0.0;
  frame.innovation_norm = frame.relative_translation.norm() + frame.relative_rotation.norm();
}

void AdaGraphFeatureExtractor::fillResidualStats(const Input& input, FeatureFrame& frame) {
  if (input.hessian_terms.empty()) return;

  double sum_residual = 0.0;
  double sum_residual_sq = 0.0;
  double max_residual = 0.0;
  std::vector<double> residuals;
  residuals.reserve(input.hessian_terms.size());

  for (const auto& term : input.hessian_terms) {
    double r = term.second.norm();
    sum_residual += r;
    sum_residual_sq += r * r;
    max_residual = std::max(max_residual, r);
    residuals.push_back(r);
  }

  const int n = static_cast<int>(residuals.size());
  frame.residual_mean = sum_residual / n;
  double variance = sum_residual_sq / n - frame.residual_mean * frame.residual_mean;
  frame.residual_std = std::sqrt(std::max(0.0, variance));
  frame.residual_max = max_residual;

  std::sort(residuals.begin(), residuals.end());
  frame.residual_median = residuals[n / 2];
  frame.residual_p90 = residuals[std::min(n - 1, static_cast<int>(n * 0.9))];
  frame.residual_p95 = residuals[std::min(n - 1, static_cast<int>(n * 0.95))];

  std::vector<double> abs_dev;
  abs_dev.reserve(residuals.size());
  for (double r : residuals) {
    abs_dev.push_back(std::abs(r - frame.residual_median));
  }
  std::sort(abs_dev.begin(), abs_dev.end());
  frame.residual_mad = abs_dev[n / 2];

  constexpr double huber_delta = 1.345;
  double huber_sum = 0.0;
  for (double r : residuals) {
    if (std::abs(r) <= huber_delta) {
      huber_sum += 0.5 * r * r;
    } else {
      huber_sum += huber_delta * (std::abs(r) - 0.5 * huber_delta);
    }
  }
  frame.huber_loss_mean = huber_sum / n;
}

void AdaGraphFeatureExtractor::fillGeometryStats(const Input& input, FeatureFrame& frame) {
  if (!input.downsampled_cloud || input.downsampled_cloud->size() <= 10) return;

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();

  for (const auto& pt : input.downsampled_cloud->points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    mean += p;
  }
  mean /= input.downsampled_cloud->size();

  for (const auto& pt : input.downsampled_cloud->points) {
    Eigen::Vector3d p(pt.x, pt.y, pt.z);
    Eigen::Vector3d d = p - mean;
    scatter += d * d.transpose();
  }
  scatter /= input.downsampled_cloud->size();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(scatter);
  auto eigenvalues = solver.eigenvalues();

  frame.hessian_eig[0] = eigenvalues(0);
  frame.hessian_eig[1] = eigenvalues(1);
  frame.hessian_eig[2] = eigenvalues(2);
  frame.hessian_eig[3] = 0.0;
  frame.hessian_eig[4] = 0.0;
  frame.hessian_eig[5] = 0.0;
  frame.hessian_trace = eigenvalues.sum();
  frame.condition_number = eigenvalues.maxCoeff() / (eigenvalues.minCoeff() + 1e-6);
  frame.eigenvalue_ratio = eigenvalues.minCoeff() / (eigenvalues.maxCoeff() + 1e-6);
  frame.effective_rank = (eigenvalues.array() > 0.01).count();
}

void AdaGraphFeatureExtractor::fillMatchAndMotionStats(const Input& input, FeatureFrame& frame) {
  frame.effective_point_number = input.downsampled_cloud ? input.downsampled_cloud->size() : 0;
  frame.inlier_ratio = (frame.effective_point_number > 0)
                           ? static_cast<double>(input.effective_knn_num) / frame.effective_point_number
                           : 0.0;
  frame.residual_cluster_count = input.hessian_terms.empty() ? 0 : 1;
  frame.normal_entropy = 0.0;

  frame.local_map_density = 0.0;
  frame.voxel_occupancy = 0.0;
  frame.point_age_mean = 0.0;

  frame.imu_angular_velocity = input.latest_imu.gyr;
  frame.imu_linear_acceleration = input.latest_imu.acc;
  frame.angular_velocity_norm = input.latest_imu.gyr.norm();
  frame.acceleration_norm = input.latest_imu.acc.norm();
  frame.motion_distortion_score = 0.0;

  frame.eskf_iteration_count = 0;
  frame.convergence_state = 0.0;
  frame.prediction_error = 0.0;
}

}  // namespace LI2Sup
