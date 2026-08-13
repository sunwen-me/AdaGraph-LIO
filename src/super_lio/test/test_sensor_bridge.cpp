#include <gtest/gtest.h>

#include <type_traits>

#include "ros/legacy_input_bridge.h"
#include "ros/sensor_bridge.h"

namespace {

TEST(SensorBridgeConfig, ValidatesTransportAndSensorBoundaries) {
  LI2Sup::SensorBridge::Config config;
  config.imu_topic = "/imu";
  config.lidar_topic = "/points";
  config.lidar_type = LI2Sup::LID_TYPE::SIM_XYZI;
  config.blind_squared = 0.04;
  config.max_range_squared = 100.0;
  config.filter_rate = 2;
  ASSERT_FALSE(config.validate());

  config.filter_rate = 0;
  EXPECT_TRUE(config.validate());
  config.filter_rate = 1;
  config.lidar_type = LI2Sup::LID_TYPE::LS16;
  EXPECT_TRUE(config.validate());
}

TEST(SensorBridgeConfig, KeepsLegacyLidarTypeNamesAndSupportSet) {
  EXPECT_TRUE(LI2Sup::SensorBridge::isSupportedLidarType(
      LI2Sup::LID_TYPE::LIVOX));
  EXPECT_TRUE(LI2Sup::SensorBridge::isSupportedLidarType(
      LI2Sup::LID_TYPE::LIVOX_POINTCLOUD2));
  EXPECT_FALSE(LI2Sup::SensorBridge::isSupportedLidarType(
      LI2Sup::LID_TYPE::LS16));
  EXPECT_EQ(LI2Sup::SensorBridge::lidarTypeName(LI2Sup::LID_TYPE::VELO16),
            "VELO16");
  EXPECT_EQ(LI2Sup::SensorBridge::lidarTypeName(999), "UNKNOWN");
}

TEST(SensorBridgeConfig, MapsAllStandardPointCloud2Adapters) {
  EXPECT_EQ(LI2Sup::SensorBridge::lidarTypeFromName("custom_mid360_driver"),
            LI2Sup::LID_TYPE::CUSTOM_MID360);
  EXPECT_EQ(LI2Sup::SensorBridge::lidarTypeFromName("unilidar"),
            LI2Sup::LID_TYPE::UNILIDAR);
  EXPECT_TRUE(LI2Sup::SensorBridge::isSupportedLidarType(
      LI2Sup::LID_TYPE::CUSTOM_MID360));
  EXPECT_TRUE(LI2Sup::SensorBridge::isSupportedLidarType(
      LI2Sup::LID_TYPE::UNILIDAR));
}

TEST(LegacyInputBridgeConfig, ValidatesCompatibilityBuffersAndNames) {
  LI2Sup::LegacyInputBridge::Config config;
  config.sensor.imu_topic = "/imu";
  config.sensor.lidar_topic = "/points";
  config.sensor.lidar_type = LI2Sup::LID_TYPE::SIM_XYZI;
  config.initial_pose_topic = "/initialpose";
  config.manual_relocalization_service = "/lio/relocalization/trigger";
  ASSERT_FALSE(config.validate());

  config.imu_buffer_capacity = 0;
  EXPECT_TRUE(config.validate());
  config.imu_buffer_capacity = 1;
  config.initial_pose_topic = "initialpose";
  EXPECT_TRUE(config.validate());
}

TEST(LegacyInputBridgeConfig, PredictionPumpIsExplicitlyExposed) {
  // Keep the scheduling contract visible at the adapter boundary: input
  // buffering and prediction execution are separate operations. This test is
  // intentionally compile-time/API-level because ROS callbacks require a
  // running executor and are covered by the application integration path.
  using Predictor = LI2Sup::LegacyInputBridge::LegacyImuPredictor;
  using Pump = decltype(&LI2Sup::LegacyInputBridge::processPendingImuPredictions);
  static_assert(std::is_same_v<Pump, std::size_t (LI2Sup::LegacyInputBridge::*)()>);
  static_assert(std::is_default_constructible_v<Predictor>);
  SUCCEED();
}

}  // namespace
