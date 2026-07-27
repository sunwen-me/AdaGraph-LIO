#ifndef ADAGRAPH_BACKEND_H_
#define ADAGRAPH_BACKEND_H_

#include <deque>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "lio/adagraph_types.h"

namespace LI2Sup {

class AdaGraphBackend {
public:
  AdaGraphBackend() = default;
  ~AdaGraphBackend() = default;

  void init();
  bool initialized() const { return initialized_; }

  const BASIC::SE3& lastKeyframePose() const { return last_keyframe_pose_; }
  bool isKeyframe(const BASIC::SE3& current_pose) const;
  void noteNonKeyframe();

  // map_observations: accepted prior-map registrations to attach as unary
  // factors (used only when lio.backend.use_localization_factor is set).
  // map_to_odom: current map->odom estimate for the map-frame initial value,
  // nullptr while prior-map localization has not initialized yet.
  BackendResult processKeyframe(
      const FeatureFrame& frame,
      const BASIC::SE3& pose,
      const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms,
      double timestamp,
      const std::vector<MapObservation>& map_observations = {},
      const BASIC::SE3* map_to_odom = nullptr);

private:
  struct ScaleFactors {
    double s_t = 1.0;
    double s_r = 1.0;
  };

  struct SE3Covariance {
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 1> getSigmas() const;
    gtsam::noiseModel::Gaussian::shared_ptr toGTSAMNoise() const;
  };

  ScaleFactors computeRuleBasedScales(const FeatureFrame& frame) const;
  SE3Covariance predictSE3Covariance(
      const FeatureFrame& frame,
      const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms) const;

  bool addGTSAMFactorWithCovariance(const BASIC::SE3& pose,
                                    const SE3Covariance& cov,
                                    double timestamp,
                                    const BASIC::SE3* map_to_odom);
  void addLocalizationFactors(const std::vector<MapObservation>& observations);
  gtsam::SharedNoiseModel localizationNoise(double fitness) const;
  void fillOptimizedTrajectory(BackendResult& result) const;

  static gtsam::Pose3 toGtsamPose(const BASIC::SE3& pose);
  static BASIC::SE3 toBasicPose(const gtsam::Pose3& pose);

  double base_sigma_t_ = 0.035;
  double base_sigma_r_ = 0.012;

  // Minimum eigenvalue of the accumulated Hessian H = sum_i J_i * w * J_i^T
  // for a direction to be treated as observed. H carries the frontend
  // observation weight w = 1/sigma^2 ~= 1000, so a single well-constrained
  // point already contributes ~1e3 and informative directions over thousands
  // of points reach ~1e5..1e7 (uncalibrated engineering initial value).
  double hessian_eig_min_ = 1e3;

  gtsam::ISAM2 isam2_;
  // Raw frontend pose per keyframe, needed to form the between measurement of
  // the next keyframe. Grows unbounded: one entry per keyframe, at least one
  // keyframe every kf_max_frames_ scans (~1 per 3 s at 10 Hz even when
  // stationary), i.e. tens of thousands of entries over a multi-hour run.
  std::deque<gtsam::Pose3> keyframe_poses_;
  // Keyframe timestamps [s], parallel to keyframe_poses_ (trajectory export).
  std::vector<double> keyframe_timestamps_;
  BASIC::SE3 last_keyframe_pose_;
  int frames_since_last_keyframe_ = 0;
  double kf_translation_thresh_ = 0.5;
  double kf_rotation_thresh_ = 0.1;
  int kf_max_frames_ = 30;
  bool initialized_ = false;

  // Prior-map localization results as unary factors (experimental, not yet
  // validated on a real robot). Loaded from lio.backend.* params in init().
  bool use_localization_factor_ = false;
  double loc_sigma_t_ = 0.1;        // base translation sigma [m]
  double loc_sigma_r_ = 0.02;       // base rotation sigma [rad]
  double loc_fitness_ref_ = 0.1;    // fitness that doubles the sigmas [m^2]
  double loc_max_time_diff_ = 0.15; // observation-to-keyframe gate [s]
  std::size_t loc_factor_count_ = 0;
  std::size_t loc_factor_dropped_ = 0;
  // A thrown ISAM2::update can leave the solver in an unusable state; once a
  // chain update fails, stop feeding it instead of crashing the frontend.
  bool isam_healthy_ = true;
};

}  // namespace LI2Sup

#endif
