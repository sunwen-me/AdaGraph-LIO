#include "lio/adagraph_backend.h"

#include <algorithm>
#include <cmath>

#include "lio/adagraph_log.h"
#include "lio/params.h"

namespace LI2Sup {

void AdaGraphBackend::init() {
  if (initialized_) return;

  ADAGRAPH_LOG_INFO("Initializing backend...");

  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(params);

  last_keyframe_pose_ = BASIC::SE3();

  use_localization_factor_ = g_backend_use_localization_factor;
  loc_sigma_t_ = g_backend_loc_factor_sigma_t;
  loc_sigma_r_ = g_backend_loc_factor_sigma_r;
  loc_fitness_ref_ = g_backend_loc_factor_fitness_ref;
  loc_max_time_diff_ = g_backend_loc_factor_max_time_diff;
  if (use_localization_factor_) {
    ADAGRAPH_LOG_WARN("Prior-map localization factors enabled"
                      " (experimental, not yet validated on a real robot).");
  }

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
    double timestamp,
    const std::vector<MapObservation>& map_observations,
    const BASIC::SE3* map_to_odom) {
  BackendResult result;
  if (!initialized_) return result;

  SE3Covariance cov = predictSE3Covariance(frame, hessian_terms);

  last_keyframe_pose_ = pose;
  frames_since_last_keyframe_ = 0;

  result.optimized_pose = pose;
  result.sigmas = cov.getSigmas();
  result.keyframe_count = keyframe_poses_.size();

  if (!isam_healthy_) {
    return result;
  }

  if (!addGTSAMFactorWithCovariance(pose, cov, timestamp, map_to_odom)) {
    return result;
  }

  if (use_localization_factor_) {
    addLocalizationFactors(map_observations);
  }

  const gtsam::Symbol current_key('x', keyframe_poses_.size() - 1);
  try {
    result.optimized_pose =
        toBasicPose(isam2_.calculateEstimate<gtsam::Pose3>(current_key));
  } catch (const std::exception& e) {
    LOG(ERROR) << " ---> [AdaGraph]: calculateEstimate failed: " << e.what();
    result.optimized_pose =
        (use_localization_factor_ && map_to_odom) ? *map_to_odom * pose : pose;
  }

  // A pure odometry chain reproduces its input exactly, so optimization only
  // genuinely moves the estimate once localization factors carry information
  // independent of the chain.
  result.optimized = use_localization_factor_ && loc_factor_count_ > 0;
  result.keyframe_count = keyframe_poses_.size();

  if (use_localization_factor_) {
    fillOptimizedTrajectory(result);
  }

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

  // Condition number of the true 6x6 Hessian. Well-conditioned scans land
  // around 1e2..1e3 (the rotation/translation blocks carry different units);
  // directional degeneracy pushes it to 1e4+. Thresholds are uncalibrated
  // engineering initial values.
  if (frame.condition_number > 1e4) {
    s_t *= 3.0;
    s_r *= 3.0;
  } else if (frame.condition_number > 1e3) {
    s_t *= 2.0;
    s_r *= 2.0;
  }

  // Trace of H with its 1/sigma^2 ~= 1000 observation weight is ~1e7 for a
  // typical scan; below 1e5 the total observation strength is unusually weak
  // (uncalibrated engineering initial value).
  if (frame.hessian_trace < 1e5) {
    s_t *= 1.5;
    s_r *= 1.5;
  }

  // Downsampled scans (0.5 m voxel) typically hold a few thousand points
  // (frontend buffers cap at 20000); below ~1500 the scan is unusually
  // sparse (uncalibrated engineering initial value).
  if (frame.effective_point_number < 1500) {
    s_t *= 1.5;
    s_r *= 1.5;
  }

  if (frame.angular_velocity_norm > 1.0) {
    s_r *= 2.0;
    s_t *= 1.5;
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

  ScaleFactors scales = computeRuleBasedScales(frame);
  double sigma_r = base_sigma_r_ * scales.s_r;
  double sigma_t = base_sigma_t_ * scales.s_t;
  Eigen::Matrix<double, 6, 6> rule_cov = Eigen::Matrix<double, 6, 6>::Zero();
  rule_cov.diagonal().head<3>().setConstant(sigma_r * sigma_r);
  rule_cov.diagonal().tail<3>().setConstant(sigma_t * sigma_t);

  if (hessian_terms.empty()) {
    result.cov = rule_cov;
    return result;
  }

  // Information is additive: sum (not average) the per-term Hessians.
  Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
  for (const auto& hr : hessian_terms) {
    H += hr.first.cast<double>();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
  const auto& eigenvalues = solver.eigenvalues();
  const auto& eigenvectors = solver.eigenvectors();

  if (eigenvalues.maxCoeff() < hessian_eig_min_) {
    result.cov = rule_cov;
    return result;
  }

  // Per-direction blend: directions the Hessian observes (eigenvalue above
  // hessian_eig_min_) take their variance from the information matrix; for
  // degenerate directions the rule-based variance is injected, projected onto
  // that eigen-direction.
  Eigen::Matrix<double, 6, 6> blended = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 6; ++i) {
    Eigen::Matrix<double, 6, 1> v = eigenvectors.col(i);
    double variance = (eigenvalues(i) >= hessian_eig_min_)
                          ? 1.0 / eigenvalues(i)
                          : v.dot(rule_cov * v);
    blended += variance * v * v.transpose();
  }
  result.cov = blended;

  return result;
}

bool AdaGraphBackend::addGTSAMFactorWithCovariance(
    const BASIC::SE3& pose, const SE3Covariance& cov, double timestamp,
    const BASIC::SE3* map_to_odom) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  gtsam::Pose3 odom_pose = toGtsamPose(pose);
  // Between measurements are relative poses taken in the odom frame (relative
  // motion is frame-independent); with localization factors the state itself
  // lives in the map frame, so the initial value is lifted by the current
  // map->odom estimate (identity until localization first converges).
  gtsam::Pose3 initial_pose = (use_localization_factor_ && map_to_odom)
                                  ? toGtsamPose(*map_to_odom * pose)
                                  : odom_pose;

  int kf_id = keyframe_poses_.size();

  if (kf_id == 0) {
    // With localization factors the map frame must be defined by the absolute
    // observations, so the first keyframe only gets a weak gauge prior; the
    // pure chain keeps the original hard anchor.
    auto prior_noise =
        use_localization_factor_
            ? gtsam::noiseModel::Diagonal::Sigmas(
                  (gtsam::Vector6() << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0).finished())
            : gtsam::noiseModel::Diagonal::Sigmas(
                  (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', 0), initial_pose, prior_noise));
    new_values.insert(gtsam::Symbol('x', 0), initial_pose);
  } else {
    gtsam::Pose3 prev_pose = keyframe_poses_[kf_id - 1];
    gtsam::Pose3 relative_pose = prev_pose.inverse() * odom_pose;

    auto gaussian_noise = cov.toGTSAMNoise();
    auto robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian_noise);

    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        gtsam::Symbol('x', kf_id - 1), gtsam::Symbol('x', kf_id), relative_pose, robust_noise));
    new_values.insert(gtsam::Symbol('x', kf_id), initial_pose);
  }

  try {
    isam2_.update(new_factors, new_values);
  } catch (const std::exception& e) {
    // The solver state after a thrown update is undefined; go inert instead
    // of risking a crash on the next update. The frontend keeps running.
    isam_healthy_ = false;
    LOG(ERROR) << " ---> [AdaGraph]: ISAM2 chain update failed, backend"
                  " disabled for the rest of the run: " << e.what();
    return false;
  }

  keyframe_poses_.push_back(odom_pose);
  keyframe_timestamps_.push_back(timestamp);
  return true;
}

void AdaGraphBackend::addLocalizationFactors(
    const std::vector<MapObservation>& observations) {
  if (observations.empty() || keyframe_timestamps_.empty()) return;

  gtsam::NonlinearFactorGraph unary_factors;
  for (const auto& obs : observations) {
    // Nearest keyframe by timestamp; both sides carry the lidar end-of-scan
    // time, so plain nearest-neighbor association is enough. The observation
    // pose is NOT motion-compensated to the keyframe time: within the
    // loc_max_time_diff_ gate the pose error stays around motion * gate and
    // is absorbed by the factor sigmas.
    auto it = std::lower_bound(keyframe_timestamps_.begin(),
                               keyframe_timestamps_.end(), obs.timestamp);
    std::size_t idx;
    if (it == keyframe_timestamps_.begin()) {
      idx = 0;
    } else if (it == keyframe_timestamps_.end()) {
      idx = keyframe_timestamps_.size() - 1;
    } else {
      const std::size_t hi = it - keyframe_timestamps_.begin();
      idx = (obs.timestamp - keyframe_timestamps_[hi - 1] <=
             keyframe_timestamps_[hi] - obs.timestamp)
                ? hi - 1
                : hi;
    }

    const double dt = std::abs(keyframe_timestamps_[idx] - obs.timestamp);
    if (dt > loc_max_time_diff_) {
      ++loc_factor_dropped_;
      ADAGRAPH_LOG_WARN("Localization factor dropped, nearest keyframe is "
                        << dt << " s away (total dropped: "
                        << loc_factor_dropped_ << ").");
      continue;
    }

    unary_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', idx), toGtsamPose(obs.map_lidar),
        localizationNoise(obs.fitness)));
  }

  if (unary_factors.empty()) return;

  try {
    isam2_.update(unary_factors, gtsam::Values());
    loc_factor_count_ += unary_factors.size();
  } catch (const std::exception& e) {
    // Skip this batch and fall back to the plain chain; the next chain
    // update decides (via its own catch) whether the solver is still usable.
    LOG(ERROR) << " ---> [AdaGraph]: ISAM2 localization-factor update failed,"
                  " dropping the batch: " << e.what();
  }
}

gtsam::SharedNoiseModel AdaGraphBackend::localizationNoise(double fitness) const {
  // sigma = base * (1 + fitness / fitness_ref): a registration at the
  // reference fitness (mean squared correspondence distance [m^2]) doubles
  // the base sigmas, and the mapping stays linear and monotone above it.
  // This is an empirical confidence mapping, not a covariance estimated by
  // the registration itself.
  const double f =
      std::isfinite(fitness) ? std::max(0.0, fitness) : 10.0 * loc_fitness_ref_;
  const double scale = 1.0 + f / std::max(1e-6, loc_fitness_ref_);
  const double sigma_r = loc_sigma_r_ * scale;
  const double sigma_t = loc_sigma_t_ * scale;
  auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << sigma_r, sigma_r, sigma_r, sigma_t, sigma_t, sigma_t)
          .finished());
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345), gaussian);
}

void AdaGraphBackend::fillOptimizedTrajectory(BackendResult& result) const {
  // Full back-substitution over all keyframes: O(n) per keyframe, acceptable
  // at keyframe cadence but worth revisiting for multi-hour runs.
  gtsam::Values estimate;
  try {
    estimate = isam2_.calculateEstimate();
  } catch (const std::exception& e) {
    LOG(ERROR) << " ---> [AdaGraph]: full calculateEstimate failed: " << e.what();
    return;
  }

  result.optimized_trajectory.reserve(keyframe_timestamps_.size());
  for (std::size_t i = 0; i < keyframe_timestamps_.size(); ++i) {
    const gtsam::Symbol key('x', i);
    if (!estimate.exists(key)) continue;
    result.optimized_trajectory.emplace_back(
        keyframe_timestamps_[i], toBasicPose(estimate.at<gtsam::Pose3>(key)));
  }
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
