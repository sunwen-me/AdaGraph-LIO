#ifndef HYBRID_SLAM_LIO_FRONTEND_FRONTEND_RESULT_H_
#define HYBRID_SLAM_LIO_FRONTEND_FRONTEND_RESULT_H_

#include <cstddef>
#include <functional>

#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/sensor/point_cloud_view.h"

namespace hybrid_slam::frontend {

enum class Status {
  kInitializing = 0,
  kTracking,
  kDegraded,
  kLost,
};

// Compatibility alias for the Phase 1 callback API. The reusable view now
// belongs to the sensor module. Callback lifetime remains unchanged.
using PointCloudView = hybrid_slam::sensor::PointCloudView;

struct MatchingSummary {
  std::size_t input_point_count = 0;
  std::size_t registered_point_count = 0;
  std::size_t matching_point_count = 0;
  std::size_t effective_constraint_count = 0;
};

struct FrontendResult {
  Status status = Status::kInitializing;
  hybrid_slam::geometry::StampedTransform3d local_pose;
  PointCloudView registered_cloud;
  MatchingSummary matching;
};

// The callback is synchronous, non-owning and configured before processing.
// It must not block, throw, or retain registered_cloud.
using FrontendResultCallback = std::function<void(const FrontendResult&)>;

}  // namespace hybrid_slam::frontend

#endif  // HYBRID_SLAM_LIO_FRONTEND_FRONTEND_RESULT_H_
