#include <future>
#include <memory>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "adagraph/system/slam_system.h"

namespace {

using adagraph::common::Timestamp;
using adagraph::geometry::FrameId;
using adagraph::sensor::ImuData;
using adagraph::sensor::PointCloudBuffer;
using adagraph::sensor::PointCloudFrame;
using adagraph::sensor::PointXYZIT;
using adagraph::system::LifecycleState;
using adagraph::system::MappingOptions;
using adagraph::system::SlamSystem;
using adagraph::system::SystemStatusCode;

PointCloudFrame MakeCloud() {
  PointCloudBuffer points{{1.0F, 0.0F, 0.0F, 0.0F, 0}};
  return PointCloudFrame(Timestamp::FromNanoseconds(10),
                         Timestamp::FromNanoseconds(20), FrameId::kLidar,
                         std::move(points));
}

TEST(SlamSystem, EnforcesLifecycleAndMovesSensorValuesToConfiguredSinks) {
  SlamSystem system;
  int imu_count = 0;
  int cloud_count = 0;
  system.setImuSink([&imu_count](ImuData imu) {
    ++imu_count;
    return imu.valid() ? adagraph::system::SystemStatus::Ok()
                       : adagraph::system::SystemStatus::Error(
                             SystemStatusCode::kRejected, "invalid");
  });
  system.setPointCloudSink([&cloud_count](PointCloudFrame&& cloud) {
    ++cloud_count;
    return cloud.valid() && !cloud.points().empty()
               ? adagraph::system::SystemStatus::Ok()
               : adagraph::system::SystemStatus::Error(
                     SystemStatusCode::kRejected, "invalid");
  });

  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(1);
  ASSERT_FALSE(system.AddImu(imu).ok());
  ASSERT_TRUE(system.StartMapping().ok());
  EXPECT_EQ(system.state(), LifecycleState::kMapping);
  EXPECT_TRUE(system.AddImu(imu).ok());
  EXPECT_TRUE(system.AddPointCloud(MakeCloud()).ok());
  EXPECT_EQ(imu_count, 1);
  EXPECT_EQ(cloud_count, 1);

  EXPECT_TRUE(system.FinishTrajectory().ok());
  EXPECT_EQ(system.state(), LifecycleState::kIdle);
  EXPECT_FALSE(system.AddPointCloud(MakeCloud()).ok());
}

TEST(SlamSystem, SupportsDeferredLocalizationAndStopReset) {
  SlamSystem system;
  adagraph::system::LocalizationOptions options;
  EXPECT_FALSE(system.StartLocalization(options).ok());
  options.allow_deferred_map = true;
  ASSERT_TRUE(system.StartLocalization(options).ok());
  EXPECT_EQ(system.state(), LifecycleState::kLocalization);
  EXPECT_FALSE(system.StartMapping().ok());
  EXPECT_TRUE(system.Stop().ok());
  EXPECT_EQ(system.state(), LifecycleState::kStopped);
  EXPECT_FALSE(system.StartMapping().ok());
  EXPECT_TRUE(system.Reset().ok());
  EXPECT_EQ(system.state(), LifecycleState::kIdle);
  EXPECT_TRUE(system.StartMapping(MappingOptions{}).ok());
}

TEST(SlamSystem, ReturnsReadyFailureForUnconfiguredMapSave) {
  SlamSystem system;
  adagraph::io::MapSaveRequest request;
  auto result = system.SaveMapAsync(std::move(request)).get();
  EXPECT_EQ(result.code, SystemStatusCode::kInvalidArgument);
}

TEST(SlamSystem, LifecycleHandlersRunBeforeInputIsAcceptedOrClosed) {
  SlamSystem system;
  int starts = 0;
  int finishes = 0;
  int imu_count = 0;
  system.setMappingStartHandler([&](const MappingOptions&) {
    ++starts;
    return adagraph::system::SystemStatus::Ok();
  });
  system.setFinishTrajectoryHandler([&] {
    ++finishes;
    return adagraph::system::SystemStatus::Ok();
  });
  system.setImuSink([&](ImuData) {
    ++imu_count;
    return adagraph::system::SystemStatus::Ok();
  });

  EXPECT_TRUE(system.StartMapping().ok());
  EXPECT_EQ(starts, 1);
  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(10);
  EXPECT_TRUE(system.AddImu(imu).ok());
  EXPECT_TRUE(system.FinishTrajectory().ok());
  EXPECT_EQ(finishes, 1);
  EXPECT_EQ(imu_count, 1);
  EXPECT_FALSE(system.AddImu(imu).ok());
}

TEST(SlamSystem, ProcessingPumpUsesTheLifecycleBoundary) {
  SlamSystem system;
  int processed = 0;
  system.setProcessHandler([&] {
    ++processed;
    return adagraph::system::SystemStatus::Ok();
  });

  EXPECT_EQ(system.ProcessPending().code, SystemStatusCode::kInvalidState);
  ASSERT_TRUE(system.StartMapping().ok());
  EXPECT_TRUE(system.hasProcessHandler());
  EXPECT_TRUE(system.ProcessPending().ok());
  EXPECT_EQ(processed, 1);
  ASSERT_TRUE(system.FinishTrajectory().ok());
  EXPECT_EQ(system.ProcessPending().code, SystemStatusCode::kInvalidState);
}

TEST(SlamSystem, ResetHandlerReturnsToIdleAndAllowsAnotherSession) {
  SlamSystem system;
  int reset_count = 0;
  system.setResetHandler([&] {
    ++reset_count;
    return adagraph::system::SystemStatus::Ok();
  });

  ASSERT_TRUE(system.StartMapping().ok());
  ASSERT_TRUE(system.Reset().ok());
  EXPECT_EQ(reset_count, 1);
  EXPECT_EQ(system.state(), LifecycleState::kIdle);
  EXPECT_TRUE(system.StartMapping().ok());
}

TEST(SlamSystem, LifecycleTransitionWaitsForInFlightSensorSubmission) {
  SlamSystem system;
  std::promise<void> entered;
  auto entered_future = entered.get_future();
  std::promise<void> release;
  auto release_future = release.get_future().share();

  system.setImuSink([&](ImuData imu) {
    if (!imu.valid()) {
      return adagraph::system::SystemStatus::Error(
          SystemStatusCode::kInvalidArgument, "invalid imu");
    }
    entered.set_value();
    release_future.wait();
    return adagraph::system::SystemStatus::Ok();
  });
  ASSERT_TRUE(system.StartMapping().ok());

  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(10);
  std::thread input_thread([&] {
    EXPECT_TRUE(system.AddImu(std::move(imu)).ok());
  });
  ASSERT_EQ(entered_future.wait_for(std::chrono::seconds(1)),
            std::future_status::ready);

  std::promise<adagraph::system::SystemStatus> finish_result;
  auto finish_future = finish_result.get_future();
  std::thread finish_thread([&] {
    finish_result.set_value(system.FinishTrajectory());
  });
  EXPECT_EQ(finish_future.wait_for(std::chrono::milliseconds(50)),
            std::future_status::timeout);

  release.set_value();
  input_thread.join();
  finish_thread.join();
  EXPECT_TRUE(finish_future.get().ok());
  EXPECT_EQ(system.state(), LifecycleState::kIdle);
}

}  // namespace
