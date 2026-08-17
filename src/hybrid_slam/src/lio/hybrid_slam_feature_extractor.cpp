#include "lio/hybrid_slam_feature_extractor.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace LI2Sup {

FeatureFrame HybridSLAMFeatureExtractor::build(const Input& input) {
  FeatureFrame frame;
  fillRelativePose(input, frame);
  fillGradientStats(input, frame);
  fillGeometryStats(input, frame);
  fillMatchAndMotionStats(input, frame);
  return frame;
}

hybrid_slam::backend::OdometryUncertaintyInput
HybridSLAMFeatureExtractor::buildUncertaintyInput(
    const FeatureFrame& frame,
    const std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                                Eigen::Matrix<double, 6, 1>>>& hessian_terms) {
  hybrid_slam::backend::OdometryUncertaintyInput input;
  input.inlier_ratio = frame.inlier_ratio;
  input.condition_number = frame.condition_number;
  input.hessian_trace = frame.hessian_trace;
  input.effective_point_count = frame.effective_point_number;
  input.angular_velocity_norm = frame.angular_velocity_norm;
  input.has_information = !hessian_terms.empty();
  for (const auto& term : hessian_terms) {
    input.information += term.first;
  }
  return input;
}

void HybridSLAMFeatureExtractor::fillRelativePose(const Input& input, FeatureFrame& frame) {
  frame.relative_translation =
      input.last_keyframe_pose.rotation.conjugate() *
      (input.current_pose.translation - input.last_keyframe_pose.translation);

  const Eigen::Quaterniond relative_rotation =
      input.last_keyframe_pose.rotation.conjugate() *
      input.current_pose.rotation;
  const Eigen::AngleAxisd aa(relative_rotation.normalized());
  frame.relative_rotation = aa.axis() * aa.angle();
}

void HybridSLAMFeatureExtractor::fillGradientStats(const Input& input, FeatureFrame& frame) {
  // The frontend aggregates all point observations into a single
  // (H, H^T V r) pair, so only the norm of the accumulated weighted
  // gradient is available; per-point residuals are not.
  Eigen::Matrix<double, 6, 1> gradient = Eigen::Matrix<double, 6, 1>::Zero();
  for (const auto& term : input.hessian_terms) {
    gradient += term.second;
  }
  frame.gradient_norm = gradient.norm();
}

void HybridSLAMFeatureExtractor::fillGeometryStats(const Input& input, FeatureFrame& frame) {
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

  if (!input.downsampled_cloud.valid() || input.downsampled_cloud.size() <= 10) {
    return;
  }

  Eigen::Matrix3d scatter = Eigen::Matrix3d::Zero();
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  std::array<float, 3> raw_point{};
  std::size_t point_count = 0;

  for (std::size_t index = 0; index < input.downsampled_cloud.size(); ++index) {
    if (!input.downsampled_cloud.xyz(index, raw_point)) {
      continue;
    }
    mean += Eigen::Vector3d(raw_point[0], raw_point[1], raw_point[2]);
    ++point_count;
  }
  if (point_count <= 10) {
    return;
  }
  mean /= static_cast<double>(point_count);

  for (std::size_t index = 0; index < input.downsampled_cloud.size(); ++index) {
    if (!input.downsampled_cloud.xyz(index, raw_point)) {
      continue;
    }
    const Eigen::Vector3d d(
        raw_point[0] - mean.x(), raw_point[1] - mean.y(), raw_point[2] - mean.z());
    scatter += d * d.transpose();
  }
  scatter /= static_cast<double>(point_count);

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

void HybridSLAMFeatureExtractor::fillMatchAndMotionStats(const Input& input, FeatureFrame& frame) {
  frame.effective_point_number = input.downsampled_cloud.valid()
                                     ? static_cast<int>(input.downsampled_cloud.size())
                                     : 0;
  frame.inlier_ratio = (frame.effective_point_number > 0)
                           ? static_cast<double>(input.effective_knn_num) / frame.effective_point_number
                           : 0.0;

  frame.imu_angular_velocity = input.latest_imu.angular_velocity_radps;
  frame.imu_linear_acceleration = input.latest_imu.linear_acceleration_mps2;
  frame.angular_velocity_norm = input.latest_imu.angular_velocity_radps.norm();
  frame.acceleration_norm = input.latest_imu.linear_acceleration_mps2.norm();
}

}  // namespace LI2Sup
