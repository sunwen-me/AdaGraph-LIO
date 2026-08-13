#ifndef ADAGRAPH_GEOMETRY_TRANSFORM3_H_
#define ADAGRAPH_GEOMETRY_TRANSFORM3_H_

#include <cmath>
#include <cstdint>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "adagraph/common/timestamp.h"

namespace adagraph::geometry {

enum class FrameId : std::uint8_t {
  kUnknown = 0,
  kMap,
  kOdom,
  kBase,
  kImu,
  kLidar,
};

// Rigid transform value without frame semantics. Public module APIs should
// expose it through StampedTransform3d, whose name and fields encode the
// transform direction.
struct RigidTransform3d {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  [[nodiscard]] static RigidTransform3d Identity() noexcept { return {}; }

  [[nodiscard]] bool isFinite() const noexcept {
    return rotation.coeffs().allFinite() && translation.allFinite() &&
           std::abs(rotation.squaredNorm() - 1.0) <= 1e-6;
  }

  [[nodiscard]] RigidTransform3d inverse() const noexcept {
    const Eigen::Quaterniond inverse_rotation = rotation.inverse();
    return {inverse_rotation, -(inverse_rotation * translation)};
  }

  // Returns (*this) * T_source_child.
  [[nodiscard]] RigidTransform3d compose(
      const RigidTransform3d& T_source_child) const noexcept {
    return {
        rotation * T_source_child.rotation,
        translation + rotation * T_source_child.translation,
    };
  }

  [[nodiscard]] Eigen::Vector3d transformPoint(
      const Eigen::Vector3d& point_source) const noexcept {
    return rotation * point_source + translation;
  }
};

// Time-invariant calibrated transform. Keeping this distinct from a stamped
// pose prevents callers from inventing timestamp zero for sensor extrinsics.
struct StaticTransform3d {
  FrameId target_frame = FrameId::kUnknown;
  FrameId source_frame = FrameId::kUnknown;
  RigidTransform3d T_target_source;

  [[nodiscard]] bool valid() const noexcept {
    return target_frame != FrameId::kUnknown &&
           source_frame != FrameId::kUnknown &&
           T_target_source.isFinite();
  }

  [[nodiscard]] StaticTransform3d inverse() const noexcept {
    return {source_frame, target_frame, T_target_source.inverse()};
  }
};

struct StampedTransform3d {
  adagraph::common::Timestamp timestamp;
  FrameId target_frame = FrameId::kUnknown;
  FrameId source_frame = FrameId::kUnknown;
  // T_target_source maps source-frame coordinates into target-frame
  // coordinates.
  RigidTransform3d T_target_source;

  [[nodiscard]] bool valid() const noexcept {
    return target_frame != FrameId::kUnknown &&
           source_frame != FrameId::kUnknown &&
           T_target_source.isFinite();
  }

  [[nodiscard]] StampedTransform3d inverse() const noexcept {
    return {
        timestamp,
        source_frame,
        target_frame,
        T_target_source.inverse(),
    };
  }

  // Composition is intentionally strict: both transforms must describe the
  // same instant and frame directions must join. Callers handling interpolated
  // or static transforms must first produce values at an explicit timestamp.
  [[nodiscard]] std::optional<StampedTransform3d> compose(
      const StampedTransform3d& T_source_child) const noexcept {
    if (!valid() || !T_source_child.valid() ||
        source_frame != T_source_child.target_frame ||
        timestamp != T_source_child.timestamp) {
      return std::nullopt;
    }
    return StampedTransform3d{
        timestamp,
        target_frame,
        T_source_child.source_frame,
        T_target_source.compose(T_source_child.T_target_source),
    };
  }

  // Compose a calibrated static extrinsic at this pose's timestamp.
  [[nodiscard]] std::optional<StampedTransform3d> compose(
      const StaticTransform3d& T_source_child) const noexcept {
    if (!valid() || !T_source_child.valid() ||
        source_frame != T_source_child.target_frame) {
      return std::nullopt;
    }
    return StampedTransform3d{
        timestamp,
        target_frame,
        T_source_child.source_frame,
        T_target_source.compose(T_source_child.T_target_source),
    };
  }
};

}  // namespace adagraph::geometry

#endif  // ADAGRAPH_GEOMETRY_TRANSFORM3_H_
