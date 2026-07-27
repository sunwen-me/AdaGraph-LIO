#ifndef ADAGRAPH_FEATURE_EXTRACTOR_H_
#define ADAGRAPH_FEATURE_EXTRACTOR_H_

#include <utility>
#include <vector>

#include "basic/alias.h"
#include "common/ds.h"
#include "lio/adagraph_types.h"

namespace LI2Sup {

class AdaGraphFeatureExtractor {
public:
  struct Input {
    BASIC::SE3 current_pose;
    BASIC::SE3 last_keyframe_pose;
    std::vector<std::pair<BASIC::M6, BASIC::V6>> hessian_terms;
    BASIC::CloudPtr downsampled_cloud;
    std::size_t effective_knn_num = 0;
    IMUData latest_imu;
  };

  static FeatureFrame build(const Input& input);

private:
  static void fillRelativePose(const Input& input, FeatureFrame& frame);
  static void fillGradientStats(const Input& input, FeatureFrame& frame);
  static void fillGeometryStats(const Input& input, FeatureFrame& frame);
  static void fillMatchAndMotionStats(const Input& input, FeatureFrame& frame);
};

}  // namespace LI2Sup

#endif
