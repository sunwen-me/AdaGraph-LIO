#include "adagraph/backend/isam2_optimizer.h"

#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <Eigen/Cholesky>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace adagraph::backend {
namespace {

gtsam::SharedNoiseModel GaussianNoise(const Covariance6d& covariance) {
  return gtsam::noiseModel::Gaussian::Covariance(covariance);
}

gtsam::SharedNoiseModel RobustNoise(const Covariance6d& covariance) {
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.345),
      GaussianNoise(covariance));
}

bool IsPositiveDefinite(const Covariance6d& covariance) {
  Eigen::LLT<Covariance6d> decomposition(covariance);
  return decomposition.info() == Eigen::Success;
}

}  // namespace

Isam2Optimizer::Isam2Optimizer(BackendConfig config)
    : config_(std::move(config)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid ISAM2 optimizer config: " + *error);
  }
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam2_ = gtsam::ISAM2(params);
}

OptimizerOutput Isam2Optimizer::apply(const GraphUpdate& update) {
  OptimizerOutput output;
  output.result.input_graph_version = update.input_graph_version;
  if (!healthy_) {
    output.status = OptimizationStatus::kFailed;
    return output;
  }
  if (!update.valid()) {
    output.status = OptimizationStatus::kInvalidInput;
    return output;
  }

  std::unordered_set<std::uint64_t> new_node_ids;
  for (const auto& node : update.added_nodes) {
    if (nodes_.contains(node.id.value()) ||
        !new_node_ids.insert(node.id.value()).second) {
      output.status = OptimizationStatus::kInvalidInput;
      return output;
    }
  }
  const auto node_exists = [this, &new_node_ids](
                               adagraph::common::NodeId id) {
    return nodes_.contains(id.value()) || new_node_ids.contains(id.value());
  };
  for (const auto& constraint : update.added_constraints) {
    const Covariance6d* covariance = nullptr;
    if (const auto* relative =
            std::get_if<RelativePoseMeasurement>(&constraint.measurement)) {
      if (!node_exists(relative->from_node) ||
          !node_exists(relative->to_node)) {
        output.status = OptimizationStatus::kInvalidInput;
        return output;
      }
      covariance = &relative->covariance;
    } else {
      const auto& absolute =
          std::get<AbsolutePoseMeasurement>(constraint.measurement);
      if (!node_exists(absolute.node_id)) {
        output.status = OptimizationStatus::kInvalidInput;
        return output;
      }
      covariance = &absolute.covariance;
    }
    if (!IsPositiveDefinite(*covariance)) {
      output.status = OptimizationStatus::kInvalidInput;
      return output;
    }
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  std::size_t gauge_factor_count = 0;
  std::unordered_set<TrajectoryKey, TrajectoryKeyHash> new_anchors;
  for (const auto& node : update.added_nodes) {
    const gtsam::Symbol symbol('x', node.id.value());
    const auto& initial = node.T_map_base
                              ? node.T_map_base->T_target_source
                              : node.T_odom_base.T_target_source;
    new_values.insert(symbol, toGtsamPose(initial));

    const TrajectoryKey trajectory_key{node.session_id, node.trajectory_id};
    if (!anchored_trajectories_.contains(trajectory_key) &&
        new_anchors.insert(trajectory_key).second) {
      const gtsam::Vector6 sigmas =
          config_.use_localization_factor
              ? (gtsam::Vector6() << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0)
                    .finished()
              : (gtsam::Vector6() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01)
                    .finished();
      new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
          symbol, toGtsamPose(initial),
          gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
      ++gauge_factor_count;
    }
  }

  bool absolute_update = false;
  for (const auto& constraint : update.added_constraints) {
    if (const auto* relative =
            std::get_if<RelativePoseMeasurement>(&constraint.measurement)) {
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
          gtsam::Symbol('x', relative->from_node.value()),
          gtsam::Symbol('x', relative->to_node.value()),
          toGtsamPose(relative->T_from_to),
          RobustNoise(relative->covariance)));
      continue;
    }

    const auto& absolute =
        std::get<AbsolutePoseMeasurement>(constraint.measurement);
    const auto noise = constraint.type == ConstraintType::kPrior
                           ? GaussianNoise(absolute.covariance)
                           : RobustNoise(absolute.covariance);
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
        gtsam::Symbol('x', absolute.node_id.value()),
        toGtsamPose(absolute.T_map_base.T_target_source), noise));
    absolute_update = true;
  }

  try {
    isam2_.update(new_factors, new_values);
  } catch (const std::exception&) {
    healthy_ = false;
    output.status = OptimizationStatus::kFailed;
    return output;
  }

  for (const auto& node : update.added_nodes) {
    nodes_.emplace(node.id.value(), node);
    node_order_.push_back(node.id.value());
  }
  for (const auto& key : new_anchors) {
    anchored_trajectories_.emplace(key, true);
  }
  factor_count_ += gauge_factor_count + update.added_constraints.size();
  ++update_count_;

  const auto append_estimate = [this, &output](const PoseGraphNode& node) {
    const auto estimate = isam2_.calculateEstimate<gtsam::Pose3>(
        gtsam::Symbol('x', node.id.value()));
    output.result.optimized_nodes.push_back({
        node.id,
        {node.T_odom_base.timestamp,
         adagraph::geometry::FrameId::kMap,
         adagraph::geometry::FrameId::kBase,
         fromGtsamPose(estimate)},
    });
  };

  try {
    if (absolute_update) {
      output.result.optimized_nodes.reserve(nodes_.size());
      for (const std::uint64_t id : node_order_) {
        append_estimate(nodes_.at(id));
      }
    } else {
      output.result.optimized_nodes.reserve(update.added_nodes.size());
      for (const auto& node : update.added_nodes) {
        append_estimate(node);
      }
    }
  } catch (const std::exception&) {
    healthy_ = false;
    output.result.optimized_nodes.clear();
    output.status = OptimizationStatus::kFailed;
    return output;
  }

  output.status = OptimizationStatus::kApplied;
  return output;
}

Isam2OptimizerStats Isam2Optimizer::stats() const noexcept {
  return {nodes_.size(), factor_count_, update_count_, healthy_};
}

std::size_t Isam2Optimizer::TrajectoryKeyHash::operator()(
    const TrajectoryKey& key) const noexcept {
  const std::size_t first =
      std::hash<std::uint64_t>{}(key.session_id.value());
  const std::size_t second =
      std::hash<std::uint64_t>{}(key.trajectory_id.value());
  return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) +
                  (first >> 2U));
}

gtsam::Pose3 Isam2Optimizer::toGtsamPose(
    const adagraph::geometry::RigidTransform3d& pose) {
  return {gtsam::Rot3(pose.rotation.toRotationMatrix()),
          gtsam::Point3(pose.translation)};
}

adagraph::geometry::RigidTransform3d Isam2Optimizer::fromGtsamPose(
    const gtsam::Pose3& pose) {
  return {
      Eigen::Quaterniond(pose.rotation().matrix()).normalized(),
      Eigen::Vector3d(pose.x(), pose.y(), pose.z()),
  };
}

}  // namespace adagraph::backend
