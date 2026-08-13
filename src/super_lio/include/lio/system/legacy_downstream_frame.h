#ifndef LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_FRAME_H_
#define LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_FRAME_H_

#include <utility>
#include <vector>

#include "common/ds.h"
#include "lio/system/legacy_point_frame.h"

namespace LI2Sup {

struct LegacySuperBuffers {
  BASIC::CloudPtr full_body_cloud;
  BASIC::CloudPtr downsampled_body_cloud;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> observation_terms;
};

// Move-only transitional Super-LIO output. The system queue exclusively owns
// these mutable buffers until processing completes, then recycles them to the
// frontend. This provides cross-thread lifetime without a per-scan cloud copy.
struct LegacySuperFrame {
  LegacySuperFrame() = default;
  LegacySuperFrame(const LegacySuperFrame&) = delete;
  LegacySuperFrame& operator=(const LegacySuperFrame&) = delete;
  LegacySuperFrame(LegacySuperFrame&&) noexcept = default;
  LegacySuperFrame& operator=(LegacySuperFrame&&) noexcept = default;

  NavState state;
  BASIC::SE3 odom_pose;
  BASIC::V3 angular_velocity = BASIC::V3::Zero();
  BASIC::CloudPtr full_body_cloud;
  BASIC::CloudPtr downsampled_body_cloud;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> observation_terms;
  std::size_t effective_constraint_count = 0;
  IMUData latest_imu;

  [[nodiscard]] bool valid() const noexcept {
    return full_body_cloud && downsampled_body_cloud;
  }
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_FRAME_H_
