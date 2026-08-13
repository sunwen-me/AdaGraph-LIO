#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "ros/service_bridge.h"

namespace {

TEST(ServiceBridgeConfig, AcceptsDistinctAbsoluteServiceNames) {
  LI2Sup::ServiceBridge::Config config;
  ASSERT_FALSE(config.validate());
}

TEST(ServiceBridgeConfig, RejectsDuplicateOrRelativeServiceNames) {
  LI2Sup::ServiceBridge::Config config;
  config.reset_service = config.stop_service;
  ASSERT_TRUE(config.validate());

  config = {};
  config.save_map_service = "lio/save_map";
  ASSERT_TRUE(config.validate());
}

TEST(ServiceBridgeConfig, RejectsInvalidPollingPeriodAndIds) {
  LI2Sup::ServiceBridge::Config config;
  config.save_poll_period_sec = 0.0;
  ASSERT_TRUE(config.validate());

  config = {};
  config.save_poll_period_sec =
      std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(config.validate());

  config = {};
  config.session_id = adagraph::common::SessionId{};
  ASSERT_TRUE(config.validate());

  config = {};
  config.trajectory_id = adagraph::common::TrajectoryId{};
  ASSERT_TRUE(config.validate());
}

TEST(ServiceBridgeConfig, AcceptsOptionalSavePathAndDeferredLocalization) {
  LI2Sup::ServiceBridge::Config config;
  config.save_map_path = "/tmp/adagraph-lio/map.pcd";
  config.allow_deferred_localization = false;
  EXPECT_FALSE(config.validate());
}

TEST(ServiceBridgeConfig, ManualRelocalizationUsesDedicatedServiceName) {
  LI2Sup::ServiceBridge::Config config;
  EXPECT_EQ(config.manual_relocalization_service,
            "/lio/relocalization/trigger");
  config.manual_relocalization_service = "/lio/slam/reset";
  EXPECT_TRUE(config.validate());
}

}  // namespace
