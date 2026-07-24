#include "lio/adagraph_backend.h"

#include <algorithm>
#include <cmath>

#include "lio/adagraph_log.h"

namespace LI2Sup {

void AdaGraphBackend::init() {
  if (initialized_) return;

  ADAGRAPH_LOG_INFO("Initializing backend...");

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(params);

  last_keyframe_pose_ = BASIC::SE3();

  initialized_ = true;
  ADAGRAPH_LOG_INFO("Backend initialized.");
}

bool AdaGraphBackend::isKeyframe(const BASIC::SE3& current_pose) const {
  BASIC::V3 translation_diff = current_pose.t_ - last_keyframe_pose_.t_;
  double translation_norm = translation_diff.norm();

  BASIC::M3 rotation_diff = current_pose.R_ * last_keyframe_pose_.R_.transpose();
  Eigen::AngleAxisd aa(rotation_diff.cast<double>());
  double rotation_angle = std::abs(aa.angle());

  return (translation_norm > kf_translation_thresh_ ||
          rotation_angle > kf_rotation_thresh_ ||
          frames_since_last_keyframe_ >= kf_max_frames_);
}

void AdaGraphBackend::noteNonKeyframe() {
  ++frames_since_last_keyframe_;
}

BackendResult AdaGraphBackend::processKeyframe(
    const FeatureFrame& frame,
    const BASIC::SE3& pose,
    const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms,
    double timestamp) {
  BackendResult result;
  if (!initialized_) return result;

  SE3Covariance cov = predictSE3Covariance(frame, hessian_terms);
  gtsam::Pose3 optimized_pose = addGTSAMFactorWithCovariance(pose, cov, timestamp);

  last_keyframe_pose_ = pose;
  frames_since_last_keyframe_ = 0;

  result.optimized = true;
  result.optimized_pose = toBasicPose(optimized_pose);
  result.sigmas = cov.getSigmas();
  result.keyframe_count = keyframe_poses_.size();

  ADAGRAPH_LOG_INFO("Keyframe " << result.keyframe_count
                    << ", sigmas: [" << result.sigmas.transpose() << "]");
  return result;
}

AdaGraphBackend::ScaleFactors AdaGraphBackend::computeRuleBasedScales(const FeatureFrame& frame) const {
  double s_t = 1.0, s_r = 1.0;

  if (frame.inlier_ratio < 0.3) {
    s_t *= 3.0;
    s_r *= 3.0;
  } else if (frame.inlier_ratio < 0.5) {
    s_t *= 2.0;
    s_r *= 2.0;
  } else if (frame.inlier_ratio < 0.7) {
    s_t *= 1.3;
    s_r *= 1.3;
  }

  if (frame.residual_mean > 0.1) {
    s_t *= 2.0;
    s_r *= 2.0;
  } else if (frame.residual_mean > 0.05) {
    s_t *= 1.5;
    s_r *= 1.5;
  }

  if (frame.condition_number > 100.0) {
    s_t *= 3.0;
    s_r *= 3.0;
  } else if (frame.condition_number > 50.0) {
    s_t *= 2.0;
    s_r *= 2.0;
  }

  if (frame.hessian_trace < 100.0) {
    s_t *= 1.5;
    s_r *= 1.5;
  }

  if (frame.effective_point_number < 20000) {
    s_t *= 1.5;
    s_r *= 1.5;
  }

  if (frame.angular_velocity_norm > 1.0) {
    s_r *= 2.0;
    s_t *= 1.5;
  }

  if (frame.motion_distortion_score > 0.5) {
    s_t *= 2.0;
    s_r *= 2.0;
  }

  s_t = std::clamp(s_t, 0.5, 10.0);
  s_r = std::clamp(s_r, 0.5, 10.0);

  return {s_t, s_r};
}

Eigen::Matrix<double, 6, 1> AdaGraphBackend::SE3Covariance::getSigmas() const {
  Eigen::Matrix<double, 6, 1> sigmas;
  for (int i = 0; i < 6; ++i) {
    sigmas(i) = std::sqrt(std::max(cov(i, i), 1e-10));
  }
  return sigmas;
}

gtsam::noiseModel::Gaussian::shared_ptr AdaGraphBackend::SE3Covariance::toGTSAMNoise() const {
  return gtsam::noiseModel::Gaussian::Covariance(cov);
}

AdaGraphBackend::SE3Covariance AdaGraphBackend::predictSE3Covariance(
    const FeatureFrame& frame,
    const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms) const {
  SE3Covariance result;

  if (!hessian_terms.empty()) {
    Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();

    for (const auto& hr : hessian_terms) {
      H += hr.first.cast<double>();
    }

    H /= hessian_terms.size();
    H += 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();
    result.cov = H.inverse();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(result.cov);
    auto eigenvalues = solver.eigenvalues();

    if (eigenvalues.minCoeff() < 1e-6) {
      ScaleFactors scales = computeRuleBasedScales(frame);
      Eigen::Matrix<double, 6, 6> diag_cov = Eigen::Matrix<double, 6, 6>::Zero();
      double sigma_r = base_sigma_r_ * scales.s_r;
      double sigma_t = base_sigma_t_ * scales.s_t;
      diag_cov(0, 0) = sigma_r * sigma_r;
      diag_cov(1, 1) = sigma_r * sigma_r;
      diag_cov(2, 2) = sigma_r * sigma_r;
      diag_cov(3, 3) = sigma_t * sigma_t;
      diag_cov(4, 4) = sigma_t * sigma_t;
      diag_cov(5, 5) = sigma_t * sigma_t;
      result.cov = diag_cov;
    }
  } else {
    ScaleFactors scales = computeRuleBasedScales(frame);
    Eigen::Matrix<double, 6, 6> diag_cov = Eigen::Matrix<double, 6, 6>::Zero();
    double sigma_r = base_sigma_r_ * scales.s_r;
    double sigma_t = base_sigma_t_ * scales.s_t;
    diag_cov(0, 0) = sigma_r * sigma_r;
    diag_cov(1, 1) = sigma_r * sigma_r;
    diag_cov(2, 2) = sigma_r * sigma_r;
    diag_cov(3, 3) = sigma_t * sigma_t;
    diag_cov(4, 4) = sigma_t * sigma_t;
    diag_cov(5, 5) = sigma_t * sigma_t;
    result.cov = diag_cov;
  }

  return result;
}

gtsam::Pose3 AdaGraphBackend::addGTSAMFactorWithCovariance(
    const BASIC::SE3& pose, const SE3Covariance& cov, double /*timestamp*/) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  gtsam::Pose3 gtsam_pose = toGtsamPose(pose);

  int kf_id = keyframe_poses_.size();
  keyframe_poses_.push_back(gtsam_pose);

  if (kf_id == 0) {
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 0), gtsam_pose, prior_noise));
    new_values.insert(gtsam::Symbol('x', 0), gtsam_pose);
    isam2_.update(new_factors, new_values);
    return gtsam_pose;
  }

  gtsam::Pose3 prev_pose = keyframe_poses_[kf_id - 1];
  gtsam::Pose3 relative_pose = prev_pose.inverse() * gtsam_pose;

  auto gaussian_noise = cov.toGTSAMNoise();
  auto robust_noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_noise);

  new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('x', kf_id - 1), gtsam::Symbol('x', kf_id), relative_pose, robust_noise));

  new_values.insert(gtsam::Symbol('x', kf_id), gtsam_pose);
  isam2_.update(new_factors, new_values);

  gtsam::Values result = isam2_.calculateEstimate();
  return result.at<gtsam::Pose3>(gtsam::Symbol('x', kf_id));
}

gtsam::Pose3 AdaGraphBackend::toGtsamPose(const BASIC::SE3& pose) {
  gtsam::Rot3 rotation(pose.R_(0, 0), pose.R_(0, 1), pose.R_(0, 2),
                       pose.R_(1, 0), pose.R_(1, 1), pose.R_(1, 2),
                       pose.R_(2, 0), pose.R_(2, 1), pose.R_(2, 2));
  gtsam::Point3 translation(pose.t_.x(), pose.t_.y(), pose.t_.z());
  return gtsam::Pose3(rotation, translation);
}

BASIC::SE3 AdaGraphBackend::toBasicPose(const gtsam::Pose3& pose) {
  return BASIC::SE3(BASIC::SO3(pose.rotation().matrix().cast<BASIC::scalar>()),
                    BASIC::V3(pose.x(), pose.y(), pose.z()));
}

}  // namespace LI2Sup
