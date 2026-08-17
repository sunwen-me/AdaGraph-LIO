#ifndef HYBRID_SLAM_BACKEND_CONSTRAINT_BUILDER_H_
#define HYBRID_SLAM_BACKEND_CONSTRAINT_BUILDER_H_

#include <cmath>
#include <optional>

#include "hybrid_slam/backend/constraint.h"
#include "hybrid_slam/backend/pose_graph.h"

namespace hybrid_slam::backend {

// Stateless, ROS/GTSAM-independent conversion from validated domain poses to
// typed graph constraints. The builder performs no allocation beyond the
// returned value and owns no graph/optimizer state.
class ConstraintBuilder {
 public:
  [[nodiscard]] std::optional<Constraint> makeOdometry(
      const PoseGraphNode& from, const PoseGraphNode& to,
      const Covariance6d& covariance) const noexcept {
    if (!compatibleRelativeNodes(from, to)) {
      return std::nullopt;
    }

    Constraint constraint;
    constraint.type = ConstraintType::kOdometry;
    constraint.measurement = RelativePoseMeasurement{
        from.id,
        to.id,
        to.T_odom_base.timestamp,
        from.T_odom_base.T_target_source.inverse().compose(
            to.T_odom_base.T_target_source),
        covariance,
    };
    return constraint.valid() ? std::optional<Constraint>(constraint)
                              : std::nullopt;
  }

  [[nodiscard]] std::optional<Constraint> makeLoopClosure(
      const PoseGraphNode& from, const PoseGraphNode& to,
      const hybrid_slam::geometry::RigidTransform3d& T_from_to,
      const Covariance6d& covariance) const noexcept {
    if (!compatibleLoopNodes(from, to)) {
      return std::nullopt;
    }

    Constraint constraint;
    constraint.type = ConstraintType::kLoopClosure;
    constraint.measurement = RelativePoseMeasurement{
        from.id, to.id, to.T_odom_base.timestamp, T_from_to, covariance};
    return constraint.valid() ? std::optional<Constraint>(constraint)
                              : std::nullopt;
  }

  [[nodiscard]] std::optional<Constraint> makePrior(
      const PoseGraphNode& node,
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const Covariance6d& covariance) const noexcept {
    return makeAbsolute(ConstraintType::kPrior, node, T_map_base, covariance,
                        0.0);
  }

  [[nodiscard]] std::optional<Constraint> makeRelocalization(
      const PoseGraphNode& node,
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const Covariance6d& covariance, double score) const noexcept {
    return makeAbsolute(ConstraintType::kRelocalization, node, T_map_base,
                        covariance, score);
  }

  // Explicit nearest-time association for asynchronous registration/GNSS
  // measurements. The measurement keeps its sensor timestamp; max time error
  // is a caller policy rather than a hidden retimestamp.
  [[nodiscard]] std::optional<Constraint> makeAssociatedRelocalization(
      const PoseGraphNode& node,
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const Covariance6d& covariance, double score,
      double maximum_time_difference_s) const noexcept {
    if (!std::isfinite(maximum_time_difference_s) ||
        maximum_time_difference_s < 0.0 ||
        std::abs(T_map_base.timestamp.seconds() -
                 node.T_odom_base.timestamp.seconds()) >
            maximum_time_difference_s) {
      return std::nullopt;
    }
    return makeAbsolute(ConstraintType::kRelocalization, node, T_map_base,
                        covariance, score, false);
  }

 private:
  [[nodiscard]] static bool compatibleRelativeNodes(
      const PoseGraphNode& from, const PoseGraphNode& to) noexcept {
    return from.valid() && to.valid() && from.id != to.id &&
           from.trajectory_id == to.trajectory_id &&
           from.session_id == to.session_id &&
           from.T_odom_base.timestamp < to.T_odom_base.timestamp;
  }

  [[nodiscard]] static bool compatibleLoopNodes(
      const PoseGraphNode& from, const PoseGraphNode& to) noexcept {
    return from.valid() && to.valid() && from.id != to.id &&
           from.session_id == to.session_id;
  }

  [[nodiscard]] static std::optional<Constraint> makeAbsolute(
      ConstraintType type, const PoseGraphNode& node,
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const Covariance6d& covariance, double score,
      bool require_exact_timestamp = true) noexcept {
    if (!node.valid() ||
        (require_exact_timestamp &&
         T_map_base.timestamp != node.T_odom_base.timestamp)) {
      return std::nullopt;
    }

    Constraint constraint;
    constraint.type = type;
    constraint.measurement =
        AbsolutePoseMeasurement{node.id, T_map_base, covariance, score};
    return constraint.valid() ? std::optional<Constraint>(constraint)
                              : std::nullopt;
  }
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_CONSTRAINT_BUILDER_H_
