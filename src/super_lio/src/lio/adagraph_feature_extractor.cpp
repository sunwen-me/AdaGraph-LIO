#include "lio/adagraph_feature_extractor.h"

#include <algorithm>
#include <cmath>

namespace LI2Sup {

FeatureFrame AdaGraphFeatureExtractor::build(const Input& input) {
  FeatureFrame frame;
  fillRelativePose(input, frame);
  fillGradientStats(input, frame);
  fillGeometryStats(input, frame);
  fillMatchAndMotionStats(input, frame);
  return frame;
}

void AdaGraphFeatureExtractor::fillRelativePose(const Input& input, FeatureFrame& frame) {
  frame.relative_translation =
      input.last_keyframe_pose.R_.transpose() * (input.current_pose.t_ - input.last_keyframe_pose.t_);

  BASIC::M3 rel_R = input.last_keyframe_pose.R_.transpose() * input.current_pose.R_;
  Eigen::AngleAxisd aa(rel_R.cast<double>());
  frame.relative_rotation = (aa.axis() * aa.angle()).cast<BASIC::scalar>();
}

void AdaGraphFeatureExtractor::fillGradientStats(const Input& input, FeatureFrame& frame) {
  // The frontend aggregates all point observations into a single
  // (H, H^T V r) pair, so only the norm of the accumulated weighted
  // gradient is available; per-point residuals are not.
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto& term : input.hessian_terms) {
    gradient += term.second.cast<double>();
  }
  frame.gradient_norm = gradient.norm();
}

void AdaGraphFeatureExtractor::fillGeometryStats(const Input& input, FeatureFrame& frame) {
  if (!input.hessian_terms.empty()) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
    for (const auto& term : input.hessian_terms) {
      H += term.first.cast<double>();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> h_solver(H);
    const auto& h_eig = h_solver.eigenvalues();

    for (int i = 0; i < 6; ++i) {
      frame.hessian_eig[i] = h_eig(i);
    }
    frame.hessian_trace = h_eig.sum();
    frame.condition_number = h_eig(5) / std::max(h_eig(0), 1e-12);
    frame.eigenvalue_ratio = h_eig(0) / std::max(h_eig(5), 1e-12);
    // A direction counts as observed above this floor; H carries the frontend
    // 1/sigma^2 ~= 1000 observation weight, so one well-constrained point
    // already contributes ~1e3 (uncalibrated engineering initial value).
    constexpr double kRankEigMin = 1e3;
    frame.effective_rank = (h_eig.array() > kRankEigMin).count();
  }

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

  frame.scene_scatter_eig[0] = eigenvalues(0);
  frame.scene_scatter_eig[1] = eigenvalues(1);
  frame.scene_scatter_eig[2] = eigenvalues(2);
  frame.scene_scatter_trace = eigenvalues.sum();
  frame.scene_scatter_condition_number = eigenvalues.maxCoeff() / (eigenvalues.minCoeff() + 1e-6);
  frame.scene_scatter_eigenvalue_ratio = eigenvalues.minCoeff() / (eigenvalues.maxCoeff() + 1e-6);
  frame.scene_scatter_effective_rank = (eigenvalues.array() > 0.01).count();
}

void AdaGraphFeatureExtractor::fillMatchAndMotionStats(const Input& input, FeatureFrame& frame) {
  frame.effective_point_number = input.downsampled_cloud ? input.downsampled_cloud->size() : 0;
  frame.inlier_ratio = (frame.effective_point_number > 0)
                           ? static_cast<double>(input.effective_knn_num) / frame.effective_point_number
                           : 0.0;

  frame.imu_angular_velocity = input.latest_imu.gyr;
  frame.imu_linear_acceleration = input.latest_imu.acc;
  frame.angular_velocity_norm = input.latest_imu.gyr.norm();
  frame.acceleration_norm = input.latest_imu.acc.norm();
}

}  // namespace LI2Sup
