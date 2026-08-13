#ifndef ADAGRAPH_BACKEND_CONSTRAINT_H_
#define ADAGRAPH_BACKEND_CONSTRAINT_H_

#include <cmath>
#include <variant>

#include <Eigen/Core>

#include "adagraph/common/domain_ids.h"
#include "adagraph/common/timestamp.h"
#include "adagraph/geometry/transform3.h"

namespace adagraph::backend {

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
  adagraph::common::NodeId from_node;
  adagraph::common::NodeId to_node;
  adagraph::common::Timestamp timestamp;
  adagraph::geometry::RigidTransform3d T_from_to;
  Covariance6d covariance = Covariance6d::Identity();
};

struct AbsolutePoseMeasurement {
  adagraph::common::NodeId node_id;
  adagraph::geometry::StampedTransform3d T_map_base;
  Covariance6d covariance = Covariance6d::Identity();
  double score = 0.0;
};

using ConstraintMeasurement =
    std::variant<RelativePoseMeasurement, AbsolutePoseMeasurement>;

struct Constraint {
  adagraph::common::ConstraintId id;
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
               adagraph::geometry::FrameId::kMap &&
           absolute.T_map_base.source_frame ==
               adagraph::geometry::FrameId::kBase &&
           IsValidCovariance6d(absolute.covariance) &&
           std::isfinite(absolute.score);
  }
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_CONSTRAINT_H_
