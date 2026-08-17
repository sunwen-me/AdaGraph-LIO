#ifndef LI2SUP_POINT_LIO_RECOVERY_COORDINATOR_H_
#define LI2SUP_POINT_LIO_RECOVERY_COORDINATOR_H_

#include "hybrid_slam/localization/submap_relocalization.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "lio/point_lio_frontend.h"
#include "lio/system/point_lio_pipeline.h"

namespace LI2Sup {

struct PointLIORecoveryRequest {
  hybrid_slam::localization::SubmapRelocalizationResult relocalization;
  hybrid_slam::common::TrajectoryId new_trajectory_id;
  PointLIOFrontend::RecoveryState frontend_state;

  [[nodiscard]] bool valid() const noexcept {
    return relocalization.valid() && new_trajectory_id.value() != 0 &&
           frontend_state.valid() &&
           std::abs(frontend_state.timestamp_seconds -
                    relocalization.T_map_base.timestamp.seconds()) <= 1e-9;
  }
};

// Application-level transaction coordinator. The caller must stop external
// ingress (for example SensorBridge) before calling Recover(), then resume it
// after a successful result. This class never exposes PCL/ROS/vendor types to
// the map or localization core.
class PointLIORecoveryCoordinator {
 public:
  [[nodiscard]] static bool Recover(
      const PointLIORecoveryRequest& request,
      const hybrid_slam::mapping::MapSnapshot& map,
      PointLIOFrontend& frontend,
      PointLIOPipeline& pipeline);
};

}  // namespace LI2Sup

#endif  // LI2SUP_POINT_LIO_RECOVERY_COORDINATOR_H_
