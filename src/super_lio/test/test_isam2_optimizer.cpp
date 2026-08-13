#include <cstddef>
#include <memory>

#include <gtest/gtest.h>

#include "adagraph/backend/constraint_builder.h"
#include "adagraph/backend/isam2_optimizer.h"

namespace {

using adagraph::backend::Constraint;
using adagraph::backend::ConstraintBuilder;
using adagraph::backend::Covariance6d;
using adagraph::backend::GraphUpdate;
using adagraph::backend::OptimizationStatus;
using adagraph::backend::PoseGraphNode;
using adagraph::common::KeyFrameId;
using adagraph::common::NodeId;
using adagraph::common::SessionId;
using adagraph::common::Timestamp;
using adagraph::common::TrajectoryId;
using adagraph::geometry::FrameId;
using adagraph::geometry::RigidTransform3d;
using adagraph::geometry::StampedTransform3d;

PoseGraphNode MakeNode(std::uint64_t id, std::int64_t timestamp_ns,
                       double x) {
  PoseGraphNode node;
  node.id = NodeId(id);
  node.keyframe_id = KeyFrameId(id);
  node.trajectory_id = TrajectoryId(1);
  node.session_id = SessionId(1);
  node.T_odom_base = {
      Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
      FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0.0, 0.0)}};
  return node;
}

TEST(Isam2Optimizer, ReproducesTypedPureOdometryChainIncrementally) {
  adagraph::backend::BackendConfig config;
  adagraph::backend::Isam2Optimizer optimizer(config);
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
  adagraph::backend::BackendConfig config;
  config.use_localization_factor = true;
  adagraph::backend::Isam2Optimizer optimizer(config);
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
      std::get<adagraph::backend::AbsolutePoseMeasurement>(
          invalid.measurement)
          .covariance;
  covariance(0, 1) = covariance(1, 0) = 1.0;
  EXPECT_EQ(optimizer.apply(GraphUpdate{3, {}, {invalid}}).status,
            OptimizationStatus::kInvalidInput);
  EXPECT_TRUE(optimizer.stats().healthy);
}

}  // namespace
