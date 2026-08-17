#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "hybrid_slam/backend/constraint_builder.h"
#include "hybrid_slam/backend/isam2_optimizer.h"

namespace {

using hybrid_slam::backend::Constraint;
using hybrid_slam::backend::ConstraintBuilder;
using hybrid_slam::backend::Covariance6d;
using hybrid_slam::backend::GraphUpdate;
using hybrid_slam::backend::OptimizationStatus;
using hybrid_slam::backend::PoseGraphNode;
using hybrid_slam::common::KeyFrameId;
using hybrid_slam::common::NodeId;
using hybrid_slam::common::SessionId;
using hybrid_slam::common::Timestamp;
using hybrid_slam::common::TrajectoryId;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::geometry::StampedTransform3d;

PoseGraphNode MakeNode(std::uint64_t id, std::int64_t timestamp_ns,
                       double x, std::uint64_t trajectory_id = 1) {
  PoseGraphNode node;
  node.id = NodeId(id);
  node.keyframe_id = KeyFrameId(id);
  node.trajectory_id = TrajectoryId(trajectory_id);
  node.session_id = SessionId(1);
  node.T_odom_base = {
      Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
      FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0.0, 0.0)}};
  return node;
}

PoseGraphNode MakeLoopReplayNode(std::uint64_t id,
                                 std::size_t revisit_period) {
  constexpr double kPi = 3.14159265358979323846;
  const double phase =
      2.0 * kPi * static_cast<double>(id % revisit_period) /
      static_cast<double>(revisit_period);
  const double laps =
      static_cast<double>(id) / static_cast<double>(revisit_period);
  PoseGraphNode node = MakeNode(id, 100 + static_cast<std::int64_t>(id), 0.0);
  node.T_odom_base.T_target_source.translation =
      Eigen::Vector3d(30.0 * std::cos(phase), 30.0 * std::sin(phase),
                      10.0 * laps);
  node.T_odom_base.T_target_source.rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(phase + 0.04 * laps, Eigen::Vector3d::UnitZ()));
  return node;
}

struct LoopReplayResult {
  double maximum_translation_residual_m = 0.0;
  double maximum_rotation_residual_rad = 0.0;
};

LoopReplayResult RunConsistentFourLoopReplay(const double huber_delta) {
  hybrid_slam::backend::BackendConfig config;
  config.loop_closure_huber_delta = huber_delta;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;

  constexpr std::size_t kRevisitPeriod = 4200;
  constexpr std::size_t kNodeCount = 4483;
  GraphUpdate odometry_update;
  odometry_update.input_graph_version = 2 * kNodeCount - 1;
  odometry_update.added_nodes.reserve(kNodeCount);
  odometry_update.added_constraints.reserve(kNodeCount - 1);
  for (std::size_t index = 0; index < kNodeCount; ++index) {
    odometry_update.added_nodes.push_back(
        MakeLoopReplayNode(index, kRevisitPeriod));
  }

  Covariance6d odometry_covariance = Covariance6d::Zero();
  odometry_covariance.diagonal().head<3>().setConstant(0.012 * 0.012);
  odometry_covariance.diagonal().tail<3>().setConstant(0.035 * 0.035);
  for (std::size_t index = 1; index < kNodeCount; ++index) {
    const auto odometry = builder.makeOdometry(
        odometry_update.added_nodes[index - 1],
        odometry_update.added_nodes[index], odometry_covariance);
    EXPECT_TRUE(odometry);
    if (odometry) {
      odometry_update.added_constraints.push_back(*odometry);
    }
  }
  EXPECT_EQ(optimizer.apply(odometry_update).status,
            OptimizationStatus::kApplied);

  Covariance6d loop_covariance = Covariance6d::Zero();
  loop_covariance.diagonal().head<3>().setConstant(0.04 * 0.04);
  loop_covariance.diagonal().tail<3>().setConstant(0.25 * 0.25);
  constexpr std::array<std::pair<std::size_t, std::size_t>, 4> kLoopPairs{{
      {104, 4304},
      {139, 4339},
      {174, 4374},
      {244, 4444},
  }};
  hybrid_slam::backend::OptimizerOutput output;
  std::uint64_t version = odometry_update.input_graph_version + 2;
  for (const auto& [match, query] : kLoopPairs) {
    const auto loop = builder.makeLoopClosure(
        odometry_update.added_nodes[match],
        odometry_update.added_nodes[query], RigidTransform3d::Identity(),
        loop_covariance);
    EXPECT_TRUE(loop);
    if (!loop) {
      continue;
    }
    output = optimizer.apply(GraphUpdate{version, {}, {*loop}});
    EXPECT_EQ(output.status, OptimizationStatus::kApplied);
    version += 2;
  }

  LoopReplayResult result;
  for (const auto& residual : output.result.relative_constraint_residuals) {
    result.maximum_translation_residual_m =
        std::max(result.maximum_translation_residual_m,
                 residual.translation_error_m);
    result.maximum_rotation_residual_rad =
        std::max(result.maximum_rotation_residual_rad,
                 residual.rotation_error_rad);
  }
  return result;
}

TEST(Isam2Optimizer, ReproducesTypedPureOdometryChainIncrementally) {
  hybrid_slam::backend::BackendConfig config;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;
  const Covariance6d covariance = Covariance6d::Identity() * 0.01;

  const PoseGraphNode first = MakeNode(0, 100, 0.0);
  auto output = optimizer.apply(GraphUpdate{1, {first}, {}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  ASSERT_EQ(output.result.optimized_nodes.size(), 1U);
  EXPECT_TRUE(output.result.optimized_nodes.front()
                  .T_map_base.T_target_source.translation.isZero(1e-10));

  const PoseGraphNode second = MakeNode(1, 200, 1.25);
  const auto first_between = builder.makeOdometry(first, second, covariance);
  ASSERT_TRUE(first_between);
  output = optimizer.apply(GraphUpdate{3, {second}, {*first_between}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  ASSERT_EQ(output.result.optimized_nodes.size(), 1U);
  EXPECT_NEAR(output.result.optimized_nodes.front()
                  .T_map_base.T_target_source.translation.x(),
              1.25, 1e-8);

  const PoseGraphNode third = MakeNode(2, 300, 3.0);
  const auto second_between = builder.makeOdometry(second, third, covariance);
  ASSERT_TRUE(second_between);
  output = optimizer.apply(GraphUpdate{5, {third}, {*second_between}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  EXPECT_NEAR(output.result.optimized_nodes.front()
                  .T_map_base.T_target_source.translation.x(),
              3.0, 1e-8);

  const auto stats = optimizer.stats();
  EXPECT_EQ(stats.node_count, 3U);
  EXPECT_EQ(stats.factor_count, 3U);  // one gauge prior + two between factors
  EXPECT_EQ(stats.update_count, 3U);
  EXPECT_TRUE(stats.healthy);
}

TEST(Isam2Optimizer, AcceptsTypedAbsoluteConstraintAndRejectsBadCovariance) {
  hybrid_slam::backend::BackendConfig config;
  config.use_localization_factor = true;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;
  const PoseGraphNode node = MakeNode(0, 100, 0.0);
  ASSERT_EQ(optimizer.apply(GraphUpdate{1, {node}, {}}).status,
            OptimizationStatus::kApplied);

  const StampedTransform3d T_map_base{
      node.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0)}};
  const auto relocalization = builder.makeRelocalization(
      node, T_map_base, Covariance6d::Identity() * 1e-4, 0.9);
  ASSERT_TRUE(relocalization);
  const auto localized =
      optimizer.apply(GraphUpdate{2, {}, {*relocalization}});
  ASSERT_EQ(localized.status, OptimizationStatus::kApplied);
  ASSERT_EQ(localized.result.optimized_nodes.size(), 1U);
  EXPECT_GT(localized.result.optimized_nodes.front()
                .T_map_base.T_target_source.translation.x(),
            1.0);

  Constraint invalid = *relocalization;
  auto& covariance =
      std::get<hybrid_slam::backend::AbsolutePoseMeasurement>(
          invalid.measurement)
          .covariance;
  covariance(0, 1) = covariance(1, 0) = 1.0;
  EXPECT_EQ(optimizer.apply(GraphUpdate{3, {}, {invalid}}).status,
            OptimizationStatus::kInvalidInput);
  EXPECT_TRUE(optimizer.stats().healthy);
}

TEST(Isam2Optimizer, PropagatesAllNodesForLoopClosureWithoutNewNode) {
  hybrid_slam::backend::BackendConfig config;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;
  const Covariance6d covariance = Covariance6d::Identity() * 0.01;
  const PoseGraphNode first = MakeNode(0, 100, 0.0);
  const PoseGraphNode second = MakeNode(1, 200, 1.0);

  ASSERT_EQ(optimizer.apply(GraphUpdate{1, {first}, {}}).status,
            OptimizationStatus::kApplied);
  const auto odometry = builder.makeOdometry(first, second, covariance);
  ASSERT_TRUE(odometry);
  ASSERT_EQ(optimizer.apply(GraphUpdate{3, {second}, {*odometry}}).status,
            OptimizationStatus::kApplied);

  const auto loop = builder.makeLoopClosure(
      first, second,
      first.T_odom_base.T_target_source.inverse().compose(
          second.T_odom_base.T_target_source),
      covariance);
  ASSERT_TRUE(loop);
  const auto output = optimizer.apply(GraphUpdate{5, {}, {*loop}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  EXPECT_EQ(output.result.optimized_nodes.size(), 2U);
}

TEST(Isam2Optimizer, RefreshesLoopResidualsAfterFollowingOdometryUpdate) {
  hybrid_slam::backend::BackendConfig config;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;
  const Covariance6d covariance = Covariance6d::Identity() * 0.01;
  const PoseGraphNode first = MakeNode(0, 100, 0.0);
  const PoseGraphNode second = MakeNode(1, 200, 1.0);
  const PoseGraphNode third = MakeNode(2, 300, 2.0);

  ASSERT_EQ(optimizer.apply(GraphUpdate{1, {first}, {}}).status,
            OptimizationStatus::kApplied);
  const auto first_odometry =
      builder.makeOdometry(first, second, covariance);
  ASSERT_TRUE(first_odometry);
  ASSERT_EQ(optimizer.apply(GraphUpdate{2, {second}, {*first_odometry}}).status,
            OptimizationStatus::kApplied);
  const auto loop = builder.makeLoopClosure(
      first, second,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.9, 0.0, 0.0)},
      covariance);
  ASSERT_TRUE(loop);
  ASSERT_EQ(optimizer.apply(GraphUpdate{3, {}, {*loop}}).status,
            OptimizationStatus::kApplied);

  const auto second_odometry =
      builder.makeOdometry(second, third, covariance);
  ASSERT_TRUE(second_odometry);
  const auto output =
      optimizer.apply(GraphUpdate{4, {third}, {*second_odometry}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  ASSERT_EQ(output.result.relative_constraint_residuals.size(), 1U);
  EXPECT_EQ(output.result.relative_constraint_residuals.front().from_node,
            first.id);
  EXPECT_EQ(output.result.relative_constraint_residuals.front().to_node,
            second.id);
}

TEST(Isam2Optimizer,
     PropagatesAllTrajectoriesForCrossTrajectoryReconnect) {
  hybrid_slam::backend::BackendConfig config;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;
  const Covariance6d covariance = Covariance6d::Identity() * 0.01;
  const PoseGraphNode historical_first = MakeNode(0, 100, 0.0, 1);
  const PoseGraphNode historical_second = MakeNode(1, 200, 1.0, 1);
  const PoseGraphNode recovered_first = MakeNode(2, 300, 100.0, 2);

  ASSERT_EQ(optimizer.apply(GraphUpdate{1, {historical_first}, {}}).status,
            OptimizationStatus::kApplied);
  const auto historical_odometry = builder.makeOdometry(
      historical_first, historical_second, covariance);
  ASSERT_TRUE(historical_odometry);
  ASSERT_EQ(optimizer
                .apply(GraphUpdate{3, {historical_second},
                                   {*historical_odometry}})
                .status,
            OptimizationStatus::kApplied);
  ASSERT_EQ(optimizer.apply(GraphUpdate{5, {recovered_first}, {}}).status,
            OptimizationStatus::kApplied);

  const auto reconnect = builder.makeLoopClosure(
      historical_second, recovered_first,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)},
      Covariance6d::Identity() * 1e-6);
  ASSERT_TRUE(reconnect);
  const auto output = optimizer.apply(GraphUpdate{7, {}, {*reconnect}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  EXPECT_EQ(output.result.optimized_nodes.size(), 3U);
}

TEST(Isam2Optimizer, ConvergesLargeDriftAfterVerifiedLoopClosure) {
  hybrid_slam::backend::BackendConfig config;
  hybrid_slam::backend::Isam2Optimizer optimizer(config);
  const ConstraintBuilder builder;

  constexpr std::size_t kEdgeCount = 4200;
  constexpr double kAccumulatedZDriftM = 10.0;
  GraphUpdate odometry_update;
  odometry_update.input_graph_version = 2 * kEdgeCount + 1;
  odometry_update.added_nodes.reserve(kEdgeCount + 1);
  odometry_update.added_constraints.reserve(kEdgeCount);
  for (std::size_t index = 0; index <= kEdgeCount; ++index) {
    PoseGraphNode node = MakeNode(index, 100 + index, 0.0);
    node.T_odom_base.T_target_source.translation.z() =
        kAccumulatedZDriftM * static_cast<double>(index) /
        static_cast<double>(kEdgeCount);
    odometry_update.added_nodes.push_back(std::move(node));
  }

  Covariance6d odometry_covariance = Covariance6d::Zero();
  odometry_covariance.diagonal().head<3>().setConstant(0.012 * 0.012);
  odometry_covariance.diagonal().tail<3>().setConstant(0.035 * 0.035);
  for (std::size_t index = 1; index <= kEdgeCount; ++index) {
    const auto odometry = builder.makeOdometry(
        odometry_update.added_nodes[index - 1],
        odometry_update.added_nodes[index], odometry_covariance);
    ASSERT_TRUE(odometry);
    odometry_update.added_constraints.push_back(*odometry);
  }
  ASSERT_EQ(optimizer.apply(odometry_update).status,
            OptimizationStatus::kApplied);

  Covariance6d loop_covariance = Covariance6d::Zero();
  // A submap pose is estimated from hundreds of spatial support voxels.  Its
  // uncertainty is not the per-point registration residual used by the old
  // 0.50 m / 0.10 rad model.  Keep a conservative systematic-error floor while
  // still requiring a verified loop to dominate accumulated odometry drift.
  loop_covariance.diagonal().head<3>().setConstant(0.04 * 0.04);
  loop_covariance.diagonal().tail<3>().setConstant(0.25 * 0.25);
  const auto loop = builder.makeLoopClosure(
      odometry_update.added_nodes.front(),
      odometry_update.added_nodes.back(), RigidTransform3d::Identity(),
      loop_covariance);
  ASSERT_TRUE(loop);
  const auto output = optimizer.apply(
      GraphUpdate{odometry_update.input_graph_version + 2, {}, {*loop}});
  ASSERT_EQ(output.status, OptimizationStatus::kApplied);
  ASSERT_EQ(output.result.optimized_nodes.size(), kEdgeCount + 1);
  ASSERT_EQ(output.result.relative_constraint_residuals.size(), 1U);
  const auto& endpoint = output.result.optimized_nodes.back()
                             .T_map_base.T_target_source.translation;
  EXPECT_LT(std::abs(endpoint.z()), 0.30)
      << "a verified loop must not remain visually layered";
  EXPECT_LT(output.result.relative_constraint_residuals.front()
                .translation_error_m,
            0.30);
}

TEST(Isam2Optimizer,
     KeepsMultipleVerifiedClosuresInsideTheQuadraticRobustBasin) {
  const LoopReplayResult ordinary_huber =
      RunConsistentFourLoopReplay(/*huber_delta=*/1.345);
  const LoopReplayResult verified_loop_huber =
      RunConsistentFourLoopReplay(/*huber_delta=*/3.0);

  EXPECT_LT(verified_loop_huber.maximum_translation_residual_m, 0.30);
  EXPECT_LT(verified_loop_huber.maximum_rotation_residual_rad, 0.02);
  EXPECT_LT(verified_loop_huber.maximum_translation_residual_m, 0.05);
  EXPECT_LT(ordinary_huber.maximum_translation_residual_m, 0.05);
}

}  // namespace
