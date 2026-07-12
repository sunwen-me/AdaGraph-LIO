#ifndef ADAGRAPH_BACKEND_H_
#define ADAGRAPH_BACKEND_H_

#include <deque>
#include <fstream>
#include <string>
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
#include <torch/script.h>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "lio/adagraph_types.h"

namespace LI2Sup {

class AdaGraphBackend {
public:
  AdaGraphBackend() = default;
  ~AdaGraphBackend();

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

  enum class ConfidenceLevel { NORMAL, LOW, VERY_LOW };

  struct SE3Covariance {
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();

    Eigen::Matrix<double, 6, 1> getSigmas() const;
    gtsam::noiseModel::Gaussian::shared_ptr toGTSAMNoise() const;
  };

  ScaleFactors predictScaleFactors(const FeatureFrame& frame);
  ScaleFactors computeRuleBasedScales(const FeatureFrame& frame) const;
  SE3Covariance predictSE3Covariance(
      const FeatureFrame& frame,
      const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms) const;
  ConfidenceLevel getConfidenceLevel(const ScaleFactors& scales) const;

  gtsam::Pose3 addGTSAMFactor(const BASIC::SE3& pose, const ScaleFactors& scales, double timestamp);
  gtsam::Pose3 addGTSAMFactorWithCovariance(const BASIC::SE3& pose, const SE3Covariance& cov, double timestamp);

  void loadOracleScales(const std::string& file_path);
  void openFeatureLog(const std::string& path);
  void closeFeatureLog();
  void logFeatureFrame(const FeatureFrame& frame, const BASIC::SE3& pose, double timestamp);

  static gtsam::Pose3 toGtsamPose(const BASIC::SE3& pose);
  static BASIC::SE3 toBasicPose(const gtsam::Pose3& pose);

  torch::jit::script::Module kalman_net_model_;
  bool kalman_net_loaded_ = false;
  std::deque<FeatureFrame> feature_buffer_;
  int kalman_net_seq_len_ = 10;
  double base_sigma_t_ = 0.035;
  double base_sigma_r_ = 0.012;

  std::vector<ScaleFactors> oracle_scales_;
  bool use_oracle_scales_ = false;
  int oracle_index_ = 0;

  gtsam::ISAM2 isam2_;
  std::deque<gtsam::Pose3> keyframe_poses_;
  BASIC::SE3 last_keyframe_pose_;
  int frames_since_last_keyframe_ = 0;
  double kf_translation_thresh_ = 0.5;
  double kf_rotation_thresh_ = 0.1;
  int kf_max_frames_ = 30;
  bool initialized_ = false;

  std::ofstream feature_log_file_;
};

}  // namespace LI2Sup

#endif
