#ifndef ADAGRAPH_BACKEND_H_
#define ADAGRAPH_BACKEND_H_

#include <deque>
#include <memory>
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
#include "adagraph/backend/backend_config.h"
#include "adagraph/backend/odometry_covariance_estimator.h"
#include "lio/adagraph_types.h"

namespace LI2Sup {

class AdaGraphBackend {
public:
  AdaGraphBackend() = default;
  explicit AdaGraphBackend(adagraph::backend::BackendConfig config)
      : config_(std::move(config)), has_explicit_config_(true) {}
  ~AdaGraphBackend() = default;

  // Compatibility entry point.  Its legacy global-parameter adapter is
  // implemented in adagraph_legacy_compat; core users should call
  // initWithConfig() so the algorithm target remains instance-configured.
  void init();
  void initWithConfig();
  bool initialized() const { return initialized_; }
  [[nodiscard]] const adagraph::backend::BackendConfig& config() const noexcept {
    return config_;
  }

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
  bool addGTSAMFactorWithCovariance(const BASIC::SE3& pose,
                                    const adagraph::backend::Covariance6d& cov,
                                    double timestamp,
                                    const BASIC::SE3* map_to_odom);
  void addLocalizationFactors(const std::vector<MapObservation>& observations);
  gtsam::SharedNoiseModel localizationNoise(double fitness) const;

  static gtsam::Pose3 toGtsamPose(const BASIC::SE3& pose);
  static BASIC::SE3 toBasicPose(const gtsam::Pose3& pose);

  gtsam::ISAM2 isam2_;
  // Raw frontend pose per keyframe, needed to form the between measurement of
  // the next keyframe. Grows unbounded: one entry per keyframe, at least one
  // keyframe every kf_max_frames_ scans (~1 per 3 s at 10 Hz even when
  // stationary), i.e. tens of thousands of entries over a multi-hour run.
  std::deque<gtsam::Pose3> keyframe_poses_;
  // Keyframe timestamps [s], parallel to keyframe_poses_ for associating
  // asynchronous localization observations during rollback operation.
  std::vector<double> keyframe_timestamps_;
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
  adagraph::backend::BackendConfig config_;
  std::unique_ptr<adagraph::backend::OdometryCovarianceEstimator>
      covariance_estimator_;
  bool has_explicit_config_ = false;
};

}  // namespace LI2Sup

#endif
