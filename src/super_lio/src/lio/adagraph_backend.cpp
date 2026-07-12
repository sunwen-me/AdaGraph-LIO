#include "lio/adagraph_backend.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "lio/adagraph_log.h"

namespace LI2Sup {

AdaGraphBackend::~AdaGraphBackend() {
  closeFeatureLog();
}

void AdaGraphBackend::init() {
  if (initialized_) return;

  ADAGRAPH_LOG_INFO("Initializing backend...");

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(params);

  std::string oracle_path = "oracle_scales.txt";
  if (std::filesystem::exists(oracle_path)) {
    loadOracleScales(oracle_path);
    use_oracle_scales_ = true;
    ADAGRAPH_LOG_INFO("Loaded oracle scales from " << oracle_path);
  }

  std::string model_path = "models/covariance_estimator.pt";
  ADAGRAPH_LOG_INFO("Looking for model at: " << model_path);

  if (std::filesystem::exists(model_path)) {
    try {
      kalman_net_model_ = torch::jit::load(model_path);
      kalman_net_model_.eval();
      kalman_net_loaded_ = true;
      ADAGRAPH_LOG_INFO("KalmanNet model loaded.");
    } catch (const c10::Error&) {
      ADAGRAPH_LOG_WARN("Failed to load KalmanNet model, using default scales.");
    }
  } else {
    ADAGRAPH_LOG_INFO("No KalmanNet model found, using default scales.");
  }

  last_keyframe_pose_ = BASIC::SE3();
  openFeatureLog("/tmp/adagraph_features.csv");

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

  logFeatureFrame(frame, pose, timestamp);

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

void AdaGraphBackend::loadOracleScales(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    ADAGRAPH_LOG_WARN("Failed to open oracle scales file: " << file_path);
    return;
  }

  oracle_scales_.clear();
  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    double s_t, s_r;
    if (iss >> s_t >> s_r) {
      oracle_scales_.push_back({s_t, s_r});
    }
  }

  ADAGRAPH_LOG_INFO("Loaded " << oracle_scales_.size() << " oracle scales.");
}

void AdaGraphBackend::openFeatureLog(const std::string& path) {
  feature_log_file_.open(path);
  if (feature_log_file_.is_open()) {
    feature_log_file_ << "timestamp,"
                      << "rel_tx,rel_ty,rel_tz,"
                      << "rel_rx,rel_ry,rel_rz,"
                      << "residual_mean,residual_std,residual_max,"
                      << "condition_number,hessian_trace,eigenvalue_ratio,"
                      << "innovation_norm,"
                      << "effective_points,knn_ratio,"
                      << "range_mean,range_std,"
                      << "motion_consistency,"
                      << "pose_x,pose_y,pose_z,"
                      << "pose_qw,pose_qx,pose_qy,pose_qz" << std::endl;
    ADAGRAPH_LOG_INFO("Feature log opened: " << path);
  }
}

void AdaGraphBackend::closeFeatureLog() {
  if (feature_log_file_.is_open()) {
    feature_log_file_.close();
    ADAGRAPH_LOG_INFO("Feature log closed.");
  }
}

void AdaGraphBackend::logFeatureFrame(const FeatureFrame& frame, const BASIC::SE3& pose, double timestamp) {
  if (!feature_log_file_.is_open()) return;

  Eigen::Quaterniond q(pose.R_.cast<double>());

  feature_log_file_ << std::fixed << std::setprecision(9) << timestamp << ","
                    << frame.relative_translation.x() << ","
                    << frame.relative_translation.y() << ","
                    << frame.relative_translation.z() << ","
                    << frame.relative_rotation.x() << ","
                    << frame.relative_rotation.y() << ","
                    << frame.relative_rotation.z() << ","
                    << frame.residual_mean << ","
                    << frame.residual_std << ","
                    << frame.residual_max << ","
                    << frame.condition_number << ","
                    << frame.hessian_trace << ","
                    << frame.eigenvalue_ratio << ","
                    << frame.innovation_norm << ","
                    << frame.effective_point_number << ","
                    << frame.inlier_ratio << ","
                    << frame.angular_velocity_norm << ","
                    << frame.acceleration_norm << ","
                    << frame.motion_distortion_score << ","
                    << pose.t_.x() << ","
                    << pose.t_.y() << ","
                    << pose.t_.z() << ","
                    << q.w() << ","
                    << q.x() << ","
                    << q.y() << ","
                    << q.z() << std::endl;
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

AdaGraphBackend::ScaleFactors AdaGraphBackend::predictScaleFactors(const FeatureFrame& frame) {
  ScaleFactors scales{1.0, 1.0};

  if (use_oracle_scales_ && oracle_index_ < static_cast<int>(oracle_scales_.size())) {
    scales = oracle_scales_[oracle_index_];
    oracle_index_++;
    ADAGRAPH_LOG_INFO("Using oracle scale: [" << scales.s_t << ", " << scales.s_r << "]");
    return scales;
  }

  if (!kalman_net_loaded_) {
    return computeRuleBasedScales(frame);
  }

  feature_buffer_.push_back(frame);
  while (feature_buffer_.size() > static_cast<size_t>(kalman_net_seq_len_)) {
    feature_buffer_.pop_front();
  }

  if (feature_buffer_.size() < static_cast<size_t>(kalman_net_seq_len_)) {
    return computeRuleBasedScales(frame);
  }

  ScaleFactors rule_scales = computeRuleBasedScales(frame);
  double log_s_rule_t = std::log(rule_scales.s_t);

  std::vector<float> data;
  data.reserve(kalman_net_seq_len_ * 18);
  for (const auto& f : feature_buffer_) {
    data.push_back(static_cast<float>(f.relative_translation.x()));
    data.push_back(static_cast<float>(f.relative_translation.y()));
    data.push_back(static_cast<float>(f.relative_translation.z()));
    data.push_back(static_cast<float>(f.relative_rotation.x()));
    data.push_back(static_cast<float>(f.relative_rotation.y()));
    data.push_back(static_cast<float>(f.relative_rotation.z()));
    data.push_back(static_cast<float>(f.residual_mean));
    data.push_back(static_cast<float>(f.residual_std));
    data.push_back(static_cast<float>(f.residual_max));
    data.push_back(static_cast<float>(f.condition_number));
    data.push_back(static_cast<float>(f.hessian_trace));
    data.push_back(static_cast<float>(f.eigenvalue_ratio));
    data.push_back(static_cast<float>(f.innovation_norm));
    data.push_back(static_cast<float>(f.effective_point_number) / 10000.0f);
    data.push_back(static_cast<float>(f.inlier_ratio));
    data.push_back(static_cast<float>(f.angular_velocity_norm));
    data.push_back(static_cast<float>(f.acceleration_norm));
    data.push_back(static_cast<float>(f.motion_distortion_score));
  }

  auto options = torch::TensorOptions().dtype(torch::kFloat32);
  auto tensor = torch::from_blob(data.data(), {1, kalman_net_seq_len_, 18}, options).clone();

  double nis_t = frame.innovation_norm * frame.innovation_norm / (base_sigma_t_ * base_sigma_t_);
  double log_nis = std::log(nis_t / 3.0 + 1e-6);

  auto log_s_rule_tensor = torch::tensor({{log_s_rule_t}}, options);
  auto log_nis_tensor = torch::tensor({{static_cast<float>(log_nis)}}, options);

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(tensor);
  inputs.push_back(log_s_rule_tensor);
  inputs.push_back(log_nis_tensor);
  auto output = kalman_net_model_.forward(inputs).toTensor();

  double log_s_t = output[0][0].item<double>();
  scales.s_t = std::exp(log_s_t);
  scales.s_r = 1.0;

  return scales;
}

AdaGraphBackend::ConfidenceLevel AdaGraphBackend::getConfidenceLevel(const ScaleFactors& scales) const {
  double max_scale = std::max(scales.s_t, scales.s_r);
  if (max_scale < 5.0) return ConfidenceLevel::NORMAL;
  if (max_scale < 9.0) return ConfidenceLevel::LOW;
  return ConfidenceLevel::VERY_LOW;
}

gtsam::Pose3 AdaGraphBackend::addGTSAMFactor(
    const BASIC::SE3& pose, const ScaleFactors& scales, double /*timestamp*/) {
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

  gtsam::Vector6 sigmas;
  sigmas << base_sigma_r_ * scales.s_r,
            base_sigma_r_ * scales.s_r,
            base_sigma_r_ * scales.s_r,
            base_sigma_t_ * scales.s_t,
            base_sigma_t_ * scales.s_t,
            base_sigma_t_ * scales.s_t;
  auto adaptive_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  gtsam::Pose3 prev_pose = keyframe_poses_[kf_id - 1];
  gtsam::Pose3 relative_pose = prev_pose.inverse() * gtsam_pose;

  ConfidenceLevel level = getConfidenceLevel(scales);
  if (level == ConfidenceLevel::VERY_LOW) {
    ADAGRAPH_LOG_WARN("Skipping very low confidence factor, but still adding keyframe.");
    new_values.insert(gtsam::Symbol('x', kf_id), gtsam_pose);
    isam2_.update(new_factors, new_values);
    gtsam::Values result = isam2_.calculateEstimate();
    return result.at<gtsam::Pose3>(gtsam::Symbol('x', kf_id));
  }

  if (level == ConfidenceLevel::LOW) {
    auto robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), adaptive_noise);
    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', kf_id - 1), gtsam::Symbol('x', kf_id), relative_pose, robust_noise));
  } else {
    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', kf_id - 1), gtsam::Symbol('x', kf_id), relative_pose, adaptive_noise));
  }

  new_values.insert(gtsam::Symbol('x', kf_id), gtsam_pose);
  isam2_.update(new_factors, new_values);

  gtsam::Values result = isam2_.calculateEstimate();
  return result.at<gtsam::Pose3>(gtsam::Symbol('x', kf_id));
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
