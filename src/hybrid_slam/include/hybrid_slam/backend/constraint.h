#ifndef HYBRID_SLAM_BACKEND_CONSTRAINT_H_
#define HYBRID_SLAM_BACKEND_CONSTRAINT_H_

#include <cmath>
#include <variant>

#include <Eigen/Core>

#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::backend {

using Covariance6d = Eigen::Matrix<double, 6, 6>;

[[nodiscard]] inline bool IsValidCovariance6d(
    const Covariance6d& covariance) noexcept {
  return covariance.allFinite() &&
         covariance.isApprox(covariance.transpose(), 1e-10) &&
         (covariance.diagonal().array() > 0.0).all();
}

enum class ConstraintType {
  kOdometry = 0,
  kLoopClosure,
  kPrior,
  kRelocalization,
};

// T_from_to = inverse(T_world_from) * T_world_to. Node identity supplies the
// otherwise node-specific source/target frames; tangent ordering is
// [rotation xyz, translation xyz].
struct RelativePoseMeasurement {
  hybrid_slam::common::NodeId from_node;
  hybrid_slam::common::NodeId to_node;
  hybrid_slam::common::Timestamp timestamp;
  hybrid_slam::geometry::RigidTransform3d T_from_to;
  Covariance6d covariance = Covariance6d::Identity();
};

struct AbsolutePoseMeasurement {
  hybrid_slam::common::NodeId node_id;
  hybrid_slam::geometry::StampedTransform3d T_map_base;
  Covariance6d covariance = Covariance6d::Identity();
  double score = 0.0;
};

using ConstraintMeasurement =
    std::variant<RelativePoseMeasurement, AbsolutePoseMeasurement>;

struct Constraint {
  hybrid_slam::common::ConstraintId id;
  ConstraintType type = ConstraintType::kOdometry;
  ConstraintMeasurement measurement = RelativePoseMeasurement{};

  [[nodiscard]] bool valid() const noexcept {
    if (const auto* relative =
            std::get_if<RelativePoseMeasurement>(&measurement)) {
      return (type == ConstraintType::kOdometry ||
              type == ConstraintType::kLoopClosure) &&
             relative->from_node != relative->to_node &&
             relative->T_from_to.isFinite() &&
             IsValidCovariance6d(relative->covariance);
    }

    const auto& absolute = std::get<AbsolutePoseMeasurement>(measurement);
    return (type == ConstraintType::kPrior ||
            type == ConstraintType::kRelocalization) &&
           absolute.T_map_base.valid() &&
           absolute.T_map_base.target_frame ==
               hybrid_slam::geometry::FrameId::kMap &&
           absolute.T_map_base.source_frame ==
               hybrid_slam::geometry::FrameId::kBase &&
           IsValidCovariance6d(absolute.covariance) &&
           std::isfinite(absolute.score);
  }
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_CONSTRAINT_H_
