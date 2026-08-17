#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "ros/ground_segmentation_filter.h"

namespace {

class GroundSegmentationFilterTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      char** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  LI2Sup::GroundSegmentationFilter makeFilter(int min_input_points = 1) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("ground.enable", true),
        rclcpp::Parameter("ground.cell_size_x", 1.0),
        rclcpp::Parameter("ground.cell_size_y", 1.0),
        rclcpp::Parameter("ground.ground_inlier_threshold", 0.125),
        rclcpp::Parameter("ground.centroid_search_radius", 5.0),
        rclcpp::Parameter("ground.auto_ground_quantile", 0.1),
        rclcpp::Parameter("ground.min_input_points", min_input_points),
    });
    node_ = std::make_shared<rclcpp::Node>(
        "ground_segmentation_filter_test", options);
    LI2Sup::GroundSegmentationFilter filter;
    filter.setup(*node_, "ground");
    return filter;
  }

  rclcpp::Node::SharedPtr node_;
};

TEST_F(GroundSegmentationFilterTest, KeepsObstacleOnlyCell) {
  auto filter = makeFilter();
  hybrid_slam::adapter::PclCloudPtr cloud(new hybrid_slam::adapter::PclPointCloud());

  for (int cell = 0; cell < 3; ++cell) {
    for (int i = 0; i < 20; ++i) {
      hybrid_slam::adapter::PclPoint point;
      point.x = static_cast<float>(cell) + 0.1f + 0.02f * i;
      point.y = 0.2f;
      point.z = 0.002f * static_cast<float>(i % 3);
      cloud->push_back(point);
    }
  }
  for (int i = 0; i < 20; ++i) {
    hybrid_slam::adapter::PclPoint table;
    table.x = 3.1f + 0.02f * i;
    table.y = 0.2f;
    table.z = 0.8f;
    cloud->push_back(table);
  }

  const auto result = filter.segmentWorldMap(cloud, 2.0);
  ASSERT_TRUE(result.ground);
  ASSERT_TRUE(result.non_ground);

  std::size_t table_points = 0;
  for (const auto& point : result.non_ground->points) {
    if (point.z > 0.7f) {
      ++table_points;
    }
  }
  EXPECT_EQ(table_points, 20u);
  EXPECT_GE(result.ground->size(), 55u);
}

TEST_F(GroundSegmentationFilterTest, RemovesGroundButKeepsWallAboveIt) {
  auto filter = makeFilter();
  hybrid_slam::adapter::PclCloudPtr cloud(new hybrid_slam::adapter::PclPointCloud());
  for (int i = 0; i < 40; ++i) {
    hybrid_slam::adapter::PclPoint floor;
    floor.x = 0.1f + 0.015f * i;
    floor.y = 0.2f;
    floor.z = 0.0f;
    cloud->push_back(floor);
  }
  for (int i = 0; i < 20; ++i) {
    hybrid_slam::adapter::PclPoint wall;
    wall.x = 0.5f;
    wall.y = 0.2f;
    wall.z = 0.2f + 0.05f * i;
    cloud->push_back(wall);
  }

  const auto result = filter.segmentWorldMap(cloud, 2.0);
  EXPECT_GE(result.ground->size(), 40u);
  EXPECT_EQ(result.non_ground->size(), 20u);
}

TEST_F(GroundSegmentationFilterTest, SmallPriorMapStillRemovesGround) {
  auto filter = makeFilter(30);
  hybrid_slam::adapter::PclCloudPtr cloud(new hybrid_slam::adapter::PclPointCloud());
  for (int i = 0; i < 10; ++i) {
    hybrid_slam::adapter::PclPoint floor;
    floor.x = 0.1f + 0.02f * i;
    floor.y = 0.2f;
    floor.z = 0.0f;
    cloud->push_back(floor);

    hybrid_slam::adapter::PclPoint obstacle = floor;
    obstacle.z = 0.5f;
    cloud->push_back(obstacle);
  }

  ASSERT_LT(cloud->size(), 30u);
  const auto result = filter.segmentWorldMap(cloud, 2.0);
  EXPECT_EQ(result.ground->size(), 10u);
  EXPECT_EQ(result.non_ground->size(), 10u);
}

TEST_F(GroundSegmentationFilterTest, DropsPointsAboveNavigationHeight) {
  auto filter = makeFilter();
  hybrid_slam::adapter::PclCloudPtr cloud(new hybrid_slam::adapter::PclPointCloud());
  for (int i = 0; i < 40; ++i) {
    hybrid_slam::adapter::PclPoint floor;
    floor.x = 0.1f + 0.015f * i;
    floor.y = 0.2f;
    floor.z = 0.0f;
    cloud->push_back(floor);
  }

  hybrid_slam::adapter::PclPoint local_ceiling;
  local_ceiling.x = 0.5f;
  local_ceiling.y = 0.2f;
  local_ceiling.z = 3.0f;
  cloud->push_back(local_ceiling);

  hybrid_slam::adapter::PclPoint isolated_ceiling = local_ceiling;
  isolated_ceiling.x = 10.0f;
  cloud->push_back(isolated_ceiling);

  const auto result = filter.segmentWorldMap(cloud, 2.0);
  EXPECT_EQ(result.ground->size(), 40u);
  EXPECT_TRUE(result.non_ground->empty());
}

}  // namespace
