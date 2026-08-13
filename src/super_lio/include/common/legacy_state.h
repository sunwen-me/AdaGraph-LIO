#ifndef LI2SUP_LEGACY_STATE_H_
#define LI2SUP_LEGACY_STATE_H_

#include <ostream>

#include "basic/Manifold.h"

namespace LI2Sup {

// Transitional value types shared by the legacy algorithm adapters.  They
// intentionally contain no PCL/ROS types so Point-LIO can use them without
// acquiring the legacy point-cloud compatibility surface.
struct SysState {
  SysState() = default;
  explicit SysState(double time, const BASIC::SO3& rotation = BASIC::SO3(),
                    const BASIC::V3& translation = BASIC::V3::Zero(),
                    const BASIC::V3& velocity = BASIC::V3::Zero(),
                    const BASIC::V3& gyro_bias = BASIC::V3::Zero(),
                    const BASIC::V3& acceleration_bias = BASIC::V3::Zero())
      : timestamp(time), R(rotation), p(translation), v(velocity),
        bg(gyro_bias), ba(acceleration_bias) {}
  SysState(double time, const BASIC::SE3& pose,
           const BASIC::V3& velocity = BASIC::V3::Zero())
      : timestamp(time), R(pose.R()), p(pose.t()), v(velocity) {}

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }

  friend std::ostream& operator<<(std::ostream& os, const SysState& state) {
    return os << "p: " << state.p.transpose() << ", v: "
              << state.v.transpose() << ", q: "
              << state.R.coeffs().transpose() << ", bg: "
              << state.bg.transpose() << ", ba: "
              << state.ba.transpose();
  }

  double timestamp = 0;
  BASIC::SO3 R;
  BASIC::V3 p = BASIC::V3::Zero();
  BASIC::V3 v = BASIC::V3::Zero();
  BASIC::V3 bg = BASIC::V3::Zero();
  BASIC::V3 ba = BASIC::V3::Zero();
};

struct NavState {
  NavState() = default;
  explicit NavState(double time, const BASIC::SO3& rotation = BASIC::SO3(),
                    const BASIC::V3& translation = BASIC::V3::Zero(),
                    const BASIC::V3& velocity = BASIC::V3::Zero())
      : timestamp(time), R(rotation), p(translation), v(velocity) {}

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }
  double timestamp = 0;
  BASIC::SO3 R = BASIC::Eye3;
  BASIC::V3 p = BASIC::V3::Zero();
  BASIC::V3 v = BASIC::V3::Zero();
};

struct DynamicState {
  DynamicState() = default;
  explicit DynamicState(double timestamp_seconds, const BASIC::M3& rotation,
                        const BASIC::V3& translation,
                        const BASIC::V3& velocity,
                        const BASIC::V3& angular_velocity,
                        const BASIC::V3& acceleration)
      : time(timestamp_seconds), R(rotation), p(translation), v(velocity),
        w(angular_velocity), a(acceleration) {}

  double time = 0;
  BASIC::M3 R = BASIC::M3::Identity();
  BASIC::V3 p = BASIC::V3::Zero();
  BASIC::V3 v = BASIC::V3::Zero();
  BASIC::V3 w = BASIC::V3::Zero();
  BASIC::V3 a = BASIC::V3::Zero();
};

struct Pose_t {
  Pose_t() = default;
  explicit Pose_t(double time, const BASIC::SO3& rotation = BASIC::SO3(),
                  const BASIC::V3& translation = BASIC::V3::Zero())
      : timestamp(time), R(rotation), p(translation) {}

  BASIC::SE3 GetSE3() const { return BASIC::SE3(R, p); }
  double timestamp = 0;
  BASIC::SO3 R = BASIC::Eye3;
  BASIC::V3 p = BASIC::V3::Zero();
};

struct IMUData {
  double secs = 0.0;
  BASIC::V3 acc = BASIC::V3::Zero();
  BASIC::V3 gyr = BASIC::V3::Zero();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace LI2Sup

#endif  // LI2SUP_LEGACY_STATE_H_
