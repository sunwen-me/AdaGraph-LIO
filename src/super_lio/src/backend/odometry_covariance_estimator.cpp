#include "adagraph/backend/odometry_covariance_estimator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Eigenvalues>

namespace adagraph::backend {

bool OdometryUncertaintyInput::valid() const noexcept {
  return std::isfinite(inlier_ratio) && inlier_ratio >= 0.0 &&
         std::isfinite(condition_number) && condition_number >= 0.0 &&
         std::isfinite(hessian_trace) &&
         std::isfinite(angular_velocity_norm) &&
         angular_velocity_norm >= 0.0 &&
         (!has_information || information.allFinite());
}

OdometryCovarianceEstimator::OdometryCovarianceEstimator(
    const BackendConfig& config)
    : base_translation_sigma_m_(config.odometry_translation_sigma_m),
      base_rotation_sigma_rad_(config.odometry_rotation_sigma_rad) {
  if (const auto error = config.validate()) {
    throw std::invalid_argument("invalid covariance estimator config: " +
                                *error);
  }
}

Covariance6d OdometryCovarianceEstimator::estimate(
    const OdometryUncertaintyInput& input) const noexcept {
  double translation_scale = 1.0;
  double rotation_scale = 1.0;

  if (input.inlier_ratio < 0.3) {
    translation_scale *= 3.0;
    rotation_scale *= 3.0;
  } else if (input.inlier_ratio < 0.5) {
    translation_scale *= 2.0;
    rotation_scale *= 2.0;
  } else if (input.inlier_ratio < 0.7) {
    translation_scale *= 1.3;
    rotation_scale *= 1.3;
  }

  if (input.condition_number > 1e4) {
    translation_scale *= 3.0;
    rotation_scale *= 3.0;
  } else if (input.condition_number > 1e3) {
    translation_scale *= 2.0;
    rotation_scale *= 2.0;
  }
  if (input.hessian_trace < 1e5) {
    translation_scale *= 1.5;
    rotation_scale *= 1.5;
  }
  if (input.effective_point_count < 1500) {
    translation_scale *= 1.5;
    rotation_scale *= 1.5;
  }
  if (input.angular_velocity_norm > 1.0) {
    rotation_scale *= 2.0;
    translation_scale *= 1.5;
  }

  translation_scale = std::clamp(translation_scale, 0.5, 10.0);
  rotation_scale = std::clamp(rotation_scale, 0.5, 10.0);
  const double rotation_sigma =
      base_rotation_sigma_rad_ * rotation_scale;
  const double translation_sigma =
      base_translation_sigma_m_ * translation_scale;
  Covariance6d rule_covariance = Covariance6d::Zero();
  rule_covariance.diagonal().head<3>().setConstant(
      rotation_sigma * rotation_sigma);
  rule_covariance.diagonal().tail<3>().setConstant(
      translation_sigma * translation_sigma);

  if (!input.valid() || !input.has_information) {
    return rule_covariance;
  }
  Eigen::SelfAdjointEigenSolver<Covariance6d> solver(input.information);
  if (solver.info() != Eigen::Success ||
      solver.eigenvalues().maxCoeff() < hessian_eigenvalue_minimum_) {
    return rule_covariance;
  }

  Covariance6d blended = Covariance6d::Zero();
  for (int index = 0; index < 6; ++index) {
    const Eigen::Matrix<double, 6, 1> direction =
        solver.eigenvectors().col(index);
    const double eigenvalue = solver.eigenvalues()(index);
    const double variance =
        eigenvalue >= hessian_eigenvalue_minimum_
            ? 1.0 / eigenvalue
            : direction.dot(rule_covariance * direction);
    blended += variance * direction * direction.transpose();
  }
  return blended;
}

}  // namespace adagraph::backend
