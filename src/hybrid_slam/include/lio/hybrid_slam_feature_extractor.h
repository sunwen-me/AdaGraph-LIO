#ifndef HYBRID_SLAM_FEATURE_EXTRACTOR_H_
#define HYBRID_SLAM_FEATURE_EXTRACTOR_H_

#include <utility>
#include <vector>

#include "hybrid_slam/backend/odometry_covariance_estimator.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/sensor/point_cloud_view.h"
#include "hybrid_slam/sensor/imu_data.h"
#include "lio/hybrid_slam_types.h"

namespace LI2Sup {

class HybridSLAMFeatureExtractor {
public:
  struct Input {
    hybrid_slam::geometry::RigidTransform3d current_pose;
    hybrid_slam::geometry::RigidTransform3d last_keyframe_pose;
    std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                          Eigen::Matrix<double, 6, 1>>>
        hessian_terms;
    hybrid_slam::sensor::PointCloudView downsampled_cloud;
    std::size_t effective_knn_num = 0;
    hybrid_slam::sensor::ImuData latest_imu;
  };

  static FeatureFrame build(const Input& input);
  static hybrid_slam::backend::OdometryUncertaintyInput buildUncertaintyInput(
      const FeatureFrame& frame,
      const std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                                  Eigen::Matrix<double, 6, 1>>>&
          hessian_terms);

private:
  static void fillRelativePose(const Input& input, FeatureFrame& frame);
  static void fillGradientStats(const Input& input, FeatureFrame& frame);
  static void fillGeometryStats(const Input& input, FeatureFrame& frame);
  static void fillMatchAndMotionStats(const Input& input, FeatureFrame& frame);
};

}  // namespace LI2Sup

#endif
