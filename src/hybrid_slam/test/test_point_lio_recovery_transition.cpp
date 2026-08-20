#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/frontend/frontend_config.h"
#include "hybrid_slam/localization/submap_relocalization.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "lio/point_lio_recovery_coordinator.h"
#include "lio/system/point_lio_pipeline.h"

namespace {

using hybrid_slam::common::SessionId;
using hybrid_slam::common::Timestamp;
using hybrid_slam::common::TrajectoryId;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::geometry::StampedTransform3d;

TEST(PointLioRecoveryTransition,
     PreservesPipelineAndPublishesNewTrajectoryAlignment) {
  bool local_map_write_enabled = true;
  RigidTransform3d published_map_odom;
  bool published_alignment = false;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.frontend.set_frontend_local_map_write_enabled =
      [&local_map_write_enabled](const bool enabled) {
        local_map_write_enabled = enabled;
      };
  callbacks.alignment.set_map_to_odom =
      [&published_map_odom, &published_alignment](
          const RigidTransform3d& transform) {
        published_map_odom = transform;
        published_alignment = true;
      };

  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  pipeline.init(hybrid_slam::frontend::FrontendConfig{},
                LI2Sup::PointLIORuntimeConfig{});

  const auto timestamp = Timestamp::FromSeconds(10.0);
  ASSERT_TRUE(timestamp.has_value());
  LI2Sup::RecoveredTrajectoryStart start;
  start.session_id = SessionId(1);
  start.trajectory_id = TrajectoryId(2);
  start.T_map_base = {*timestamp, FrameId::kMap, FrameId::kBase,
                      {Eigen::Quaterniond::Identity(),
                       Eigen::Vector3d(5.0, -2.0, 1.0)}};
  start.T_odom_base = {*timestamp, FrameId::kOdom, FrameId::kBase,
                       {Eigen::Quaterniond::Identity(),
                        Eigen::Vector3d(1.0, 3.0, 0.5)}};

  ASSERT_TRUE(pipeline.beginRecoveredTrajectory(start));
  EXPECT_FALSE(local_map_write_enabled);
  ASSERT_TRUE(published_alignment);
  EXPECT_TRUE(published_map_odom.translation.isApprox(
      Eigen::Vector3d(4.0, -5.0, 0.5)));

  const auto alignment = pipeline.alignmentSnapshot();
  ASSERT_TRUE(alignment);
  EXPECT_TRUE(alignment->valid());
  EXPECT_TRUE(alignment->T_map_odom.T_target_source.translation.isApprox(
      published_map_odom.translation));
  pipeline.finish();
}

TEST(PointLioRecoveryPipeline,
     ClearsProvisionalOnlineLayerAfterVerificationFailure) {
  std::size_t clear_count = 0;
  bool local_map_write_enabled = true;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.frontend.set_frontend_local_map_write_enabled =
      [&local_map_write_enabled](const bool enabled) {
        local_map_write_enabled = enabled;
      };
  callbacks.frontend.clear_frontend_online_local_map = [&clear_count] {
    ++clear_count;
  };

  LI2Sup::PointLIORuntimeConfig runtime;
  runtime.frontend_health.enabled = true;
  runtime.frontend_health.suspect_bad_frames = 1;
  runtime.frontend_health.lost_bad_frames = 1;
  runtime.frontend_health.recover_good_frames = 2;
  runtime.frontend_health.verification_grace_frames = 0;
  runtime.frontend_health.min_effective_points = 1;
  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  pipeline.init(hybrid_slam::frontend::FrontendConfig{}, runtime);

  const auto timestamp = Timestamp::FromSeconds(10.0);
  ASSERT_TRUE(timestamp.has_value());
  LI2Sup::RecoveredTrajectoryStart start;
  start.session_id = SessionId(1);
  start.trajectory_id = TrajectoryId(2);
  start.T_map_base = {*timestamp, FrameId::kMap, FrameId::kBase,
                      RigidTransform3d::Identity()};
  start.T_odom_base = {*timestamp, FrameId::kOdom, FrameId::kBase,
                       RigidTransform3d::Identity()};
  ASSERT_TRUE(pipeline.beginRecoveredTrajectory(start));
  EXPECT_FALSE(local_map_write_enabled);

  LI2Sup::PointLIOOutputFrame bad_frame;
  bad_frame.state.timestamp = 10.1;
  bad_frame.state.T_odom_imu = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
  bad_frame.dense_points_odom = {Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
  bad_frame.matching_points_odom = bad_frame.dense_points_odom;
  bad_frame.attempted_point_updates = 1;
  LI2Sup::IMUData imu;
  imu.timestamp = *Timestamp::FromSeconds(10.1);
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);

  pipeline.processPointFrame(std::move(bad_frame), imu);
  EXPECT_EQ(clear_count, 1U);
  EXPECT_FALSE(local_map_write_enabled);
  pipeline.finish();
}

TEST(PointLioRecoveryPipeline,
     RecoveryCommitStartsANewWatchdogWindowBeforeVerification) {
  std::size_t clear_count = 0;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.frontend.clear_frontend_online_local_map = [&clear_count] { ++clear_count; };

  LI2Sup::PointLIORuntimeConfig runtime;
  runtime.frontend_health.enabled = true;
  runtime.frontend_health.suspect_bad_frames = 1;
  runtime.frontend_health.lost_bad_frames = 1;
  runtime.frontend_health.max_timestamp_gap_s = 0.001;
  runtime.frontend_health.min_effective_points = 1;
  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  pipeline.init(hybrid_slam::frontend::FrontendConfig{}, runtime);

  LI2Sup::PointLIOOutputFrame frame;
  frame.state.timestamp = 10.0;
  frame.state.T_odom_imu = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()};
  frame.dense_points_odom = {Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
  frame.matching_points_odom = frame.dense_points_odom;
  frame.attempted_point_updates = 10;
  frame.effective_point_updates = 10;
  frame.lidar_hessian = Eigen::Matrix<double, 6, 6>::Identity() * 2000.0;
  LI2Sup::IMUData imu;
  imu.timestamp = *Timestamp::FromSeconds(10.0);
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
  pipeline.processPointFrame(std::move(frame), imu);

  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  const auto timestamp = Timestamp::FromSeconds(10.1);
  ASSERT_TRUE(timestamp);
  LI2Sup::RecoveredTrajectoryStart start;
  start.session_id = SessionId(1);
  start.trajectory_id = TrajectoryId(2);
  start.T_map_base = {*timestamp, FrameId::kMap, FrameId::kBase,
                      RigidTransform3d::Identity()};
  start.T_odom_base = {*timestamp, FrameId::kOdom, FrameId::kBase,
                       RigidTransform3d::Identity()};
  ASSERT_TRUE(pipeline.beginRecoveredTrajectory(start));

  // The old frame has already exceeded max_timestamp_gap_s. The first
  // watchdog observation after commit must use the new verification window,
  // not immediately clear the provisional online layer.
  pipeline.observeFrontendTimeout();
  EXPECT_EQ(clear_count, 0U);
  pipeline.finish();
}

TEST(PointLioRecoveryCoordinator,
     SeedsNewOdomFrameFromMatchedFinishedSubmap) {
  using Buffer = hybrid_slam::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(
      Buffer{{1.0F, 0.0F, 0.0F, 0.0F, 0}});
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::sensor::PointXYZIT),
      offsetof(hybrid_slam::sensor::PointXYZIT, x),
      offsetof(hybrid_slam::sensor::PointXYZIT, y),
      offsetof(hybrid_slam::sensor::PointXYZIT, z), FrameId::kSubmap);

  hybrid_slam::mapping::MapSnapshot map;
  hybrid_slam::mapping::SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(0);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = hybrid_slam::mapping::SubmapState::kFinished;
  submap.T_map_submap = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)};
  submap.cloud_submap = hybrid_slam::sensor::ImmutablePointCloud::Share(
      storage, view);
  submap.seed_cloud_submap = submap.cloud_submap;
  submap.descriptor = {1.0F};
  submap.geometry_ready = true;
  ASSERT_TRUE(submap.valid());
  map.submaps.push_back(submap);
  ASSERT_TRUE(map.valid());

  LI2Sup::PointLIOFrontend::Config frontend_config;
  LI2Sup::PointLIOFrontend frontend(std::move(frontend_config));
  frontend.setDownstreamFrameCallback(
      [](LI2Sup::PointLIOOutputFrame&&, const LI2Sup::IMUData&) {});
  frontend.setOdometryCallback(
      [](const LI2Sup::NavState&, const Eigen::Vector3d&) {});
  frontend.init();
  frontend.setInputEnabled(false);

  LI2Sup::PointLIOPipeline pipeline;
  LI2Sup::PointLIORuntimeConfig runtime;
  // Recovery verification is explicit even when normal health hysteresis is
  // disabled. Keep the test window short while preserving the production
  // ordering: verify, recover, cool down, then admit the first keyframe.
  runtime.frontend_health.recover_good_frames = 2;
  runtime.frontend_health.recovery_cooldown_frames = 1;
  pipeline.init(frontend.frontendConfig(), runtime);
  const auto timestamp = Timestamp::FromSeconds(20.0);
  ASSERT_TRUE(timestamp.has_value());
  LI2Sup::PointLIORecoveryRequest request;
  request.new_trajectory_id = TrajectoryId(2);
  request.frontend_state.timestamp_seconds = timestamp->seconds();
  request.frontend_state.T_odom_base = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)};
  request.relocalization.T_map_base = {
      *timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}};
  request.relocalization.session_id = SessionId(1);
  request.relocalization.trajectory_id = TrajectoryId(1);
  request.relocalization.submap_id = hybrid_slam::common::SubmapId(0);
  request.relocalization.fitness = 0.1;
  request.relocalization.inlier_ratio = 0.9;
  request.relocalization.overlap = 0.8;
  request.relocalization.observable = true;

  ASSERT_TRUE(LI2Sup::PointLIORecoveryCoordinator::Recover(
      request, map, frontend, pipeline));
  EXPECT_FALSE(frontend.inputEnabled());
  EXPECT_EQ(frontend.memoryStats().prior_ivox_entries, 1U);
  const auto alignment = pipeline.alignmentSnapshot();
  ASSERT_TRUE(alignment);
  EXPECT_TRUE(alignment->T_map_odom.T_target_source.translation.isApprox(
      Eigen::Vector3d(4.0, 0.0, 0.0)));

  frontend.setInputEnabled(true);
  LI2Sup::IMUData stale_imu;
  stale_imu.timestamp = *Timestamp::FromSeconds(20.0);
  stale_imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
  EXPECT_FALSE(frontend.enqueueImu(stale_imu));
  LI2Sup::IMUData fresh_imu = stale_imu;
  fresh_imu.timestamp = *Timestamp::FromSeconds(20.1);
  EXPECT_TRUE(frontend.enqueueImu(fresh_imu));
  EXPECT_FALSE(frontend.enqueueImu(stale_imu));

  const auto make_input_cloud = [](double seconds) {
    const auto timestamp = Timestamp::FromSeconds(seconds);
    EXPECT_TRUE(timestamp);
    return hybrid_slam::sensor::PointCloudFrame(
        *timestamp, *timestamp, FrameId::kLidar,
        hybrid_slam::sensor::PointCloudBuffer{
            {1.0F, 0.0F, 0.0F, 0.0F, 0}});
  };
  EXPECT_FALSE(frontend.enqueuePointCloud(make_input_cloud(20.0)));
  EXPECT_TRUE(frontend.enqueuePointCloud(make_input_cloud(20.1)));
  EXPECT_FALSE(frontend.enqueuePointCloud(make_input_cloud(20.05)));
  (void)frontend.discardQueuedInput();
  frontend.setInputEnabled(false);

  LI2Sup::IMUData recovered_imu;
  recovered_imu.timestamp = *Timestamp::FromSeconds(20.1);
  recovered_imu.linear_acceleration_mps2 =
      Eigen::Vector3d(0.0, 0.0, 9.81);
  for (int index = 1; index <= 3; ++index) {
    LI2Sup::PointLIOOutputFrame recovered_frame;
    recovered_frame.state.timestamp = 20.0 + 0.1 * index;
    recovered_frame.state.T_odom_imu = {
        Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)};
    recovered_frame.dense_points_odom = {
        Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
    recovered_frame.matching_points_odom = recovered_frame.dense_points_odom;
    recovered_frame.lidar_hessian =
        Eigen::Matrix<double, 6, 6>::Identity() * 2000.0;
    recovered_frame.effective_point_updates = 10;
    recovered_frame.attempted_point_updates = 10;
    pipeline.processPointFrame(std::move(recovered_frame), recovered_imu);
  }
  pipeline.finish();
  const auto recovered_snapshot = pipeline.mapSnapshot();
  ASSERT_TRUE(recovered_snapshot);
  ASSERT_EQ(recovered_snapshot->keyframes.size(), 1U);
  ASSERT_EQ(recovered_snapshot->constraints.size(), 1U);
  EXPECT_EQ(recovered_snapshot->constraints.front().type,
            hybrid_slam::backend::ConstraintType::kRelocalization);
  frontend.finish();
}

TEST(PointLioRecoveryCoordinator,
     FrontendResetFailureDoesNotCommitPipelineTransition) {
  using Buffer = hybrid_slam::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(
      Buffer{{1.0F, 0.0F, 0.0F, 0.0F, 0}});
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::sensor::PointXYZIT),
      offsetof(hybrid_slam::sensor::PointXYZIT, x),
      offsetof(hybrid_slam::sensor::PointXYZIT, y),
      offsetof(hybrid_slam::sensor::PointXYZIT, z), FrameId::kSubmap);

  hybrid_slam::mapping::MapSnapshot map;
  hybrid_slam::mapping::SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(0);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = hybrid_slam::mapping::SubmapState::kFinished;
  submap.T_map_submap = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)};
  submap.cloud_submap = hybrid_slam::sensor::ImmutablePointCloud::Share(
      storage, view);
  submap.seed_cloud_submap = submap.cloud_submap;
  submap.descriptor = {1.0F};
  submap.geometry_ready = true;
  map.submaps.push_back(submap);
  ASSERT_TRUE(map.valid());

  LI2Sup::PointLIOFrontend frontend(LI2Sup::PointLIOFrontend::Config{});
  // Keep ingress disabled so the coordinator reaches resetAndSeed(). The
  // frontend is intentionally not initialized, which injects the reset
  // failure after pipeline preparation but before pipeline commit.
  frontend.setInputEnabled(false);

  LI2Sup::PointLIOPipeline pipeline;
  pipeline.init(hybrid_slam::frontend::FrontendConfig{},
                LI2Sup::PointLIORuntimeConfig{});
  const auto timestamp = Timestamp::FromSeconds(30.0);
  ASSERT_TRUE(timestamp.has_value());
  LI2Sup::PointLIORecoveryRequest request;
  request.new_trajectory_id = TrajectoryId(2);
  request.frontend_state.timestamp_seconds = timestamp->seconds();
  request.relocalization.T_map_base = {
      *timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}};
  request.relocalization.session_id = SessionId(1);
  request.relocalization.trajectory_id = TrajectoryId(1);
  request.relocalization.submap_id = hybrid_slam::common::SubmapId(0);
  request.relocalization.fitness = 0.1;
  request.relocalization.inlier_ratio = 0.9;
  request.relocalization.overlap = 0.8;
  request.relocalization.observable = true;

  EXPECT_FALSE(LI2Sup::PointLIORecoveryCoordinator::Recover(
      request, map, frontend, pipeline));
  EXPECT_FALSE(pipeline.alignmentSnapshot());
  EXPECT_EQ(pipeline.mapSnapshot()->version, 0U);
  pipeline.finish();
}

TEST(PointLioRecoveryCoordinator, RejectsUnknownSubmapBeforePreparingPipeline) {
  LI2Sup::PointLIOFrontend frontend(LI2Sup::PointLIOFrontend::Config{});
  frontend.setInputEnabled(false);
  LI2Sup::PointLIOPipeline pipeline;
  pipeline.init(hybrid_slam::frontend::FrontendConfig{},
                LI2Sup::PointLIORuntimeConfig{});

  const auto timestamp = Timestamp::FromSeconds(40.0);
  ASSERT_TRUE(timestamp.has_value());
  LI2Sup::PointLIORecoveryRequest request;
  request.new_trajectory_id = TrajectoryId(2);
  request.frontend_state.timestamp_seconds = timestamp->seconds();
  request.relocalization.T_map_base = {
      *timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}};
  request.relocalization.session_id = SessionId(1);
  request.relocalization.trajectory_id = TrajectoryId(1);
  request.relocalization.submap_id = hybrid_slam::common::SubmapId(99);
  request.relocalization.fitness = 0.1;
  request.relocalization.inlier_ratio = 0.9;
  request.relocalization.overlap = 0.8;
  request.relocalization.observable = true;

  const hybrid_slam::mapping::MapSnapshot empty_map;
  EXPECT_FALSE(LI2Sup::PointLIORecoveryCoordinator::Recover(
      request, empty_map, frontend, pipeline));
  EXPECT_FALSE(pipeline.alignmentSnapshot());
  pipeline.finish();
}

TEST(PointLioRecoveryPipeline, LostFramesDoNotAdmitDurableKeyframes) {
  std::size_t recovery_queries = 0;
  bool local_map_write_enabled = true;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.recovery.request_submap_recovery =
      [&recovery_queries](
          const hybrid_slam::localization::RelocalizationQuery& query,
          std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) {
        ASSERT_TRUE(query.valid());
        ASSERT_TRUE(snapshot);
        ++recovery_queries;
      };
  callbacks.frontend.set_frontend_local_map_write_enabled =
      [&local_map_write_enabled](const bool enabled) {
        local_map_write_enabled = enabled;
      };

  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  LI2Sup::PointLIORuntimeConfig runtime;
  runtime.frontend_health.enabled = true;
  runtime.frontend_health.suspect_bad_frames = 1;
  runtime.frontend_health.lost_bad_frames = 2;
  runtime.frontend_health.recover_good_frames = 2;
  runtime.frontend_health.recovery_cooldown_frames = 1;
  runtime.frontend_health.min_effective_points = 1;
  runtime.submap_recovery.query_window_frames = 1;
  pipeline.init(hybrid_slam::frontend::FrontendConfig{}, runtime);

  const auto imu_timestamp = Timestamp::FromSeconds(50.0);
  ASSERT_TRUE(imu_timestamp);
  LI2Sup::IMUData imu;
  imu.timestamp = *imu_timestamp;
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);

  const auto make_frame = [](double timestamp, bool good) {
    LI2Sup::PointLIOOutputFrame frame;
    frame.state.timestamp = timestamp;
    frame.state.T_odom_imu =
        {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)};
    frame.dense_points_odom = {Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
    frame.matching_points_odom = frame.dense_points_odom;
    frame.effective_point_updates = good ? 10U : 0U;
    frame.attempted_point_updates = 10U;
    if (good) {
      frame.lidar_hessian = Eigen::Matrix<double, 6, 6>::Identity() * 2000.0;
    }
    return frame;
  };

  pipeline.processPointFrame(make_frame(50.0, true), imu);
  pipeline.processPointFrame(make_frame(50.1, false), imu);
  pipeline.processPointFrame(make_frame(50.2, false), imu);
  EXPECT_FALSE(local_map_write_enabled);
  EXPECT_EQ(recovery_queries, 1U);

  // A failed scheduler result returns the lifecycle to LOST, allowing the
  // next newer query to start another explicit relocalization attempt after
  // the configured retry cooldown.
  pipeline.markRecoveryAttemptFailed();
  pipeline.processPointFrame(make_frame(50.3, false), imu);
  EXPECT_EQ(recovery_queries, 1U);
  pipeline.processPointFrame(make_frame(50.4, false), imu);
  EXPECT_EQ(recovery_queries, 2U);

  pipeline.finish();
  const auto snapshot = pipeline.mapSnapshot();
  ASSERT_TRUE(snapshot);
  // The first healthy frame is the only durable keyframe. Both bad frames
  // reached LOST and were rejected before global keyframe admission.
  EXPECT_EQ(snapshot->keyframes.size(), 1U);
}

TEST(PointLioRecoveryPipeline, LidarWatchdogDetectsDropoutWithoutNewFrames) {
  std::size_t recovery_queries = 0;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.recovery.request_submap_recovery =
      [&recovery_queries](
          const hybrid_slam::localization::RelocalizationQuery& query,
          std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) {
        ASSERT_TRUE(query.valid());
        ASSERT_TRUE(snapshot);
        ++recovery_queries;
      };

  LI2Sup::PointLIORuntimeConfig runtime;
  runtime.frontend_health.enabled = true;
  runtime.frontend_health.suspect_bad_frames = 1;
  // This deliberately exceeds the number of watchdog observations. A LiDAR
  // silence must enter LOST by elapsed wall time rather than frame hysteresis.
  runtime.frontend_health.lost_bad_frames = 10;
  runtime.frontend_health.recover_good_frames = 2;
  runtime.frontend_health.recovery_cooldown_frames = 1;
  runtime.frontend_health.max_timestamp_gap_s = 0.001;
  runtime.frontend_health.lidar_dropout_lost_timeout_s = 0.002;
  runtime.frontend_health.min_effective_points = 1;
  runtime.submap_recovery.query_window_frames = 1;

  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  pipeline.init(hybrid_slam::frontend::FrontendConfig{}, runtime);

  LI2Sup::PointLIOOutputFrame frame;
  frame.state.timestamp = 60.0;
  frame.state.T_odom_imu = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)};
  frame.dense_points_odom = {Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
  frame.matching_points_odom = frame.dense_points_odom;
  frame.effective_point_updates = 10;
  frame.attempted_point_updates = 10;
  frame.lidar_hessian = Eigen::Matrix<double, 6, 6>::Identity() * 2000.0;
  LI2Sup::IMUData imu;
  imu.timestamp = *Timestamp::FromSeconds(60.0);
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);

  pipeline.processPointFrame(std::move(frame), imu);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  pipeline.observeFrontendTimeout();
  EXPECT_EQ(recovery_queries, 0U);
  std::this_thread::sleep_for(std::chrono::milliseconds(3));
  pipeline.observeFrontendTimeout();

  // A watchdog loss freezes the lifecycle but must not reuse a pre-dropout
  // scan as a recovery query. The next post-dropout LiDAR frame owns the
  // relocalization timestamp and geometry.
  EXPECT_EQ(recovery_queries, 0U);
  LI2Sup::PointLIOOutputFrame recovered_frame;
  recovered_frame.state.timestamp = 60.1;
  recovered_frame.state.T_odom_imu = {
      Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)};
  recovered_frame.dense_points_odom = {Eigen::Vector3f(1.0F, 0.0F, 0.0F)};
  recovered_frame.matching_points_odom = recovered_frame.dense_points_odom;
  recovered_frame.effective_point_updates = 10;
  recovered_frame.attempted_point_updates = 10;
  recovered_frame.lidar_hessian =
      Eigen::Matrix<double, 6, 6>::Identity() * 2000.0;
  imu.timestamp = *Timestamp::FromSeconds(60.1);
  pipeline.processPointFrame(std::move(recovered_frame), imu);
  EXPECT_EQ(recovery_queries, 1U);
  pipeline.finish();
}

TEST(PointLioRecoveryPipeline,
     RecoveryQueryAccumulatesFramesInLatestImuPropagatedBaseFrame) {
  std::size_t query_count = 0;
  std::size_t query_points = 0;
  float first_query_x = 0.0F;
  float last_query_x = 0.0F;
  LI2Sup::PointLIOPipelineCallbacks callbacks;
  callbacks.recovery.request_submap_recovery =
      [&query_count, &query_points, &first_query_x, &last_query_x](
          const hybrid_slam::localization::RelocalizationQuery& query,
          std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) {
        ASSERT_TRUE(query.valid());
        ASSERT_TRUE(snapshot);
        ++query_count;
        query_points = query.current_scan_base.size();
        std::array<float, 3> first{};
        std::array<float, 3> last{};
        ASSERT_TRUE(query.current_scan_base.view().xyz(0, first));
        ASSERT_TRUE(query.current_scan_base.view().xyz(query_points - 1, last));
        first_query_x = first[0];
        last_query_x = last[0];
      };

  LI2Sup::PointLIORuntimeConfig runtime;
  runtime.frontend_health.enabled = true;
  runtime.frontend_health.suspect_bad_frames = 1;
  runtime.frontend_health.lost_bad_frames = 1;
  runtime.frontend_health.recover_good_frames = 2;
  runtime.frontend_health.min_effective_points = 1;
  runtime.submap_recovery.query_window_frames = 3;

  LI2Sup::PointLIOPipeline pipeline(std::move(callbacks));
  pipeline.init(hybrid_slam::frontend::FrontendConfig{}, runtime);

  LI2Sup::IMUData imu;
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
  const auto make_frame = [&imu](double timestamp, double odom_x) {
    LI2Sup::PointLIOOutputFrame frame;
    frame.state.timestamp = timestamp;
    frame.state.T_odom_imu = {
        Eigen::Quaterniond::Identity(), Eigen::Vector3d(odom_x, 0.0, 0.0)};
    // These emulate scan-to-map corrupted output. The recovery query below
    // instead carries the raw scan placed by the independent IMU propagation.
    frame.dense_points_odom = {
        Eigen::Vector3f(static_cast<float>(odom_x + 100.0), 0.0F, 0.0F)};
    frame.matching_points_odom = frame.dense_points_odom;
    const auto typed_timestamp = Timestamp::FromSeconds(timestamp);
    EXPECT_TRUE(typed_timestamp);
    frame.recovery_query = LI2Sup::RecoveryQueryFrame{
        {*typed_timestamp, FrameId::kOdom, FrameId::kBase,
         {Eigen::Quaterniond::Identity(), Eigen::Vector3d(odom_x, 0.0, 0.0)}},
        {Eigen::Vector3f(static_cast<float>(odom_x + 1.0), 0.0F, 0.0F)}};
    frame.attempted_point_updates = 10;
    frame.effective_point_updates = 0;
    imu.timestamp = *Timestamp::FromSeconds(timestamp);
    return frame;
  };

  pipeline.processPointFrame(make_frame(70.0, 0.0), imu);
  pipeline.processPointFrame(make_frame(70.1, 1.0), imu);
  pipeline.processPointFrame(make_frame(70.2, 2.0), imu);
  EXPECT_EQ(query_count, 0U);
  pipeline.processPointFrame(make_frame(70.3, 3.0), imu);

  EXPECT_EQ(query_count, 1U);
  EXPECT_EQ(query_points, 3U);
  EXPECT_NEAR(first_query_x, -1.0F, 1e-5F);
  EXPECT_NEAR(last_query_x, 1.0F, 1e-5F);
  pipeline.finish();
}

}  // namespace
