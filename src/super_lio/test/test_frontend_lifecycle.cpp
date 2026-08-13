#include <cstddef>
#include <utility>

#include <gtest/gtest.h>

#include "adagraph/common/timestamp.h"
#include "adagraph/geometry/transform3.h"
#include "adagraph/sensor/imu_data.h"
#include "adagraph/sensor/point_cloud_frame.h"
#include "lio/point_lio_frontend.h"
#include "lio/super_lio.h"

namespace {

using adagraph::common::Timestamp;
using adagraph::geometry::FrameId;
using adagraph::sensor::ImuData;
using adagraph::sensor::PointCloudBuffer;
using adagraph::sensor::PointCloudFrame;
using adagraph::sensor::PointXYZIT;

ImuData MakeImu(const std::int64_t nanoseconds) {
  ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(nanoseconds);
  imu.frame = FrameId::kImu;
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, -9.81);
  imu.angular_velocity_radps = Eigen::Vector3d::Zero();
  return imu;
}

PointCloudFrame MakeCloud(const std::int64_t start_nanoseconds) {
  PointCloudBuffer points{{1.0F, 0.0F, 0.0F, 0.0F, 0}};
  return PointCloudFrame(
      Timestamp::FromNanoseconds(start_nanoseconds),
      Timestamp::FromNanoseconds(start_nanoseconds + 10), FrameId::kLidar,
      std::move(points));
}

TEST(FrontendLifecycle, SuperLioResetAllowsNewTypedSession) {
  LI2Sup::SuperLIO frontend;
  frontend.init();
  frontend.setTypedInputEnabled(true);
  ASSERT_TRUE(frontend.enqueueImu(MakeImu(10)));
  ASSERT_TRUE(frontend.enqueuePointCloud(MakeCloud(10)));

  frontend.reset();
  EXPECT_FALSE(frontend.typedInputEnabled());

  frontend.setTypedInputEnabled(true);
  EXPECT_TRUE(frontend.enqueueImu(MakeImu(20)));
  EXPECT_TRUE(frontend.enqueuePointCloud(MakeCloud(20)));
  const auto discarded = frontend.discardTypedInput();
  EXPECT_EQ(discarded.first, 1U);
  EXPECT_EQ(discarded.second, 1U);
}

TEST(FrontendLifecycle, PointLioResetRestartsDownstreamWorker) {
  LI2Sup::PointLIOFrontend::Config config;
  config.input_queue_capacity = 4;
  config.downstream_queue_capacity = 2;
  LI2Sup::PointLIOFrontend frontend(std::move(config));
  frontend.setDownstreamFrameCallback(
      [](LI2Sup::LegacyPointFrame&&, const LI2Sup::IMUData&) {});
  frontend.setOdometryCallback(
      [](const LI2Sup::NavState&, const BASIC::V3&) {});
  frontend.init();

  frontend.setInputEnabled(true);
  ASSERT_TRUE(frontend.enqueueImu(MakeImu(10)));
  ASSERT_TRUE(frontend.enqueuePointCloud(MakeCloud(10)));

  frontend.reset();
  EXPECT_FALSE(frontend.inputEnabled());

  frontend.setInputEnabled(true);
  EXPECT_TRUE(frontend.enqueueImu(MakeImu(20)));
  EXPECT_TRUE(frontend.enqueuePointCloud(MakeCloud(20)));
  frontend.finish();
}

TEST(FrontendLifecycle, PointLioInstancesKeepIndependentTypedConfig) {
  LI2Sup::PointLIOFrontend::Config first_config;
  first_config.point_lio.min_range_m = 0.1;
  first_config.point_lio.max_range_m = 40.0;
  first_config.point_lio.local_map_resolution_m = 0.2;
  first_config.input_queue_capacity = 2;

  LI2Sup::PointLIOFrontend::Config second_config;
  second_config.point_lio.min_range_m = 0.8;
  second_config.point_lio.max_range_m = 80.0;
  second_config.point_lio.local_map_resolution_m = 0.7;
  second_config.input_queue_capacity = 5;

  LI2Sup::PointLIOFrontend first(std::move(first_config));
  LI2Sup::PointLIOFrontend second(std::move(second_config));
  const auto sink = [](LI2Sup::LegacyPointFrame&&,
                       const LI2Sup::IMUData&) {};
  const auto odometry = [](const LI2Sup::NavState&, const BASIC::V3&) {};
  first.setDownstreamFrameCallback(sink);
  first.setOdometryCallback(odometry);
  second.setDownstreamFrameCallback(sink);
  second.setOdometryCallback(odometry);

  ASSERT_NO_THROW(first.init());
  ASSERT_NO_THROW(second.init());
  EXPECT_DOUBLE_EQ(first.frontendConfig().min_range_m, 0.1);
  EXPECT_DOUBLE_EQ(first.frontendConfig().max_range_m, 40.0);
  EXPECT_DOUBLE_EQ(first.frontendConfig().local_map_resolution_m, 0.2);
  EXPECT_DOUBLE_EQ(second.frontendConfig().min_range_m, 0.8);
  EXPECT_DOUBLE_EQ(second.frontendConfig().max_range_m, 80.0);
  EXPECT_DOUBLE_EQ(second.frontendConfig().local_map_resolution_m, 0.7);

  first.reset();
  EXPECT_FALSE(first.inputEnabled());
  EXPECT_TRUE(second.inputEnabled());
  EXPECT_DOUBLE_EQ(second.frontendConfig().min_range_m, 0.8);
  EXPECT_DOUBLE_EQ(second.frontendConfig().local_map_resolution_m, 0.7);
  second.finish();
}

}  // namespace
