#ifndef HYBRID_SLAM_BACKEND_ODOMETRY_COVARIANCE_ESTIMATOR_H_
#define HYBRID_SLAM_BACKEND_ODOMETRY_COVARIANCE_ESTIMATOR_H_

#include "hybrid_slam/backend/backend_config.h"
#include "hybrid_slam/backend/constraint.h"

namespace hybrid_slam::backend {

struct OdometryUncertaintyInput {
  double inlier_ratio = 0.0;
  double condition_number = 1.0;
  double hessian_trace = 0.0;
  std::size_t effective_point_count = 0;
  double angular_velocity_norm = 0.0;
  Covariance6d information = Covariance6d::Zero();
  bool has_information = false;

  [[nodiscard]] bool valid() const noexcept;
};

// Exact extraction of the current HybridSLAM rule/Hessian covariance policy.
// It is deterministic, allocation-free and runs only at keyframe frequency.
class OdometryCovarianceEstimator {
 public:
  explicit OdometryCovarianceEstimator(const BackendConfig& config);

  [[nodiscard]] Covariance6d estimate(
      const OdometryUncertaintyInput& input) const noexcept;

 private:
  double base_translation_sigma_m_ = 0.035;
  double base_rotation_sigma_rad_ = 0.012;
  double hessian_eigenvalue_minimum_ = 1e3;
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_ODOMETRY_COVARIANCE_ESTIMATOR_H_
