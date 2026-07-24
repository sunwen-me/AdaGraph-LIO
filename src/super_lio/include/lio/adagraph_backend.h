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

  BackendResult processKeyframe(
      const FeatureFrame& frame,
      const BASIC::SE3& pose,
      const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms,
      double timestamp);

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

  gtsam::Pose3 addGTSAMFactorWithCovariance(const BASIC::SE3& pose, const SE3Covariance& cov, double timestamp);

  static gtsam::Pose3 toGtsamPose(const BASIC::SE3& pose);
  static BASIC::SE3 toBasicPose(const gtsam::Pose3& pose);

  double base_sigma_t_ = 0.035;
  double base_sigma_r_ = 0.012;

  gtsam::ISAM2 isam2_;
  std::deque<gtsam::Pose3> keyframe_poses_;
  BASIC::SE3 last_keyframe_pose_;
  int frames_since_last_keyframe_ = 0;
  double kf_translation_thresh_ = 0.5;
  double kf_rotation_thresh_ = 0.1;
  int kf_max_frames_ = 30;
  bool initialized_ = false;
};

}  // namespace LI2Sup

#endif
