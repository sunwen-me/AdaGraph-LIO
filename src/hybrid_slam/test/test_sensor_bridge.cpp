#include <gtest/gtest.h>

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

TEST(SensorBridgeConfig, KeepsSupportedLidarTypeNamesAndSupportSet) {
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

}  // namespace
