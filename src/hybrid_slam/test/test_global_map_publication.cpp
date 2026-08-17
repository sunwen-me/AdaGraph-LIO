#include <array>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "ros/publication_bridge.h"

namespace {

using namespace std::chrono_literals;
using hybrid_slam::common::Timestamp;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;

hybrid_slam::sensor::ImmutablePointCloud MakeCloudBase() {
  using Buffer = hybrid_slam::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{
      {1.00F, 0.0F, 0.0F, 1.0F, 0},
      {1.10F, 0.0F, 0.0F, 1.0F, 0},
  });
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::sensor::PointXYZIT),
      offsetof(hybrid_slam::sensor::PointXYZIT, x),
      offsetof(hybrid_slam::sensor::PointXYZIT, y),
      offsetof(hybrid_slam::sensor::PointXYZIT, z), FrameId::kBase);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(storage, view);
}

hybrid_slam::sensor::ImmutablePointCloud MakeCloudSubmap() {
  using Buffer = hybrid_slam::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{
      {1.00F, 0.0F, 0.0F, 1.0F, 0},
      {1.10F, 0.0F, 0.0F, 1.0F, 0},
  });
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::sensor::PointXYZIT),
      offsetof(hybrid_slam::sensor::PointXYZIT, x),
      offsetof(hybrid_slam::sensor::PointXYZIT, y),
      offsetof(hybrid_slam::sensor::PointXYZIT, z), FrameId::kSubmap);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(storage, view);
}

std::shared_ptr<hybrid_slam::mapping::MapSnapshot> MakeSnapshot(
    const double map_x) {
  auto snapshot = std::make_shared<hybrid_slam::mapping::MapSnapshot>();
  snapshot->version = 1;
  const auto timestamp = Timestamp::FromNanoseconds(1);
  snapshot->keyframes.push_back({
      hybrid_slam::common::NodeId(1),
      hybrid_slam::common::KeyFrameId(1),
      hybrid_slam::common::TrajectoryId(1),
      hybrid_slam::common::SessionId(1),
      hybrid_slam::common::SubmapId(1),
      {timestamp, FrameId::kOdom, FrameId::kBase,
       RigidTransform3d::Identity()},
      {timestamp, FrameId::kMap, FrameId::kBase,
       {Eigen::Quaterniond::Identity(), Eigen::Vector3d(map_x, 0.0, 0.0)}},
      MakeCloudBase(),
      {},
  });
  const auto submap_cloud = MakeCloudSubmap();
  snapshot->submaps.push_back({
      hybrid_slam::common::SubmapId(1),
      hybrid_slam::common::TrajectoryId(1),
      hybrid_slam::common::SessionId(1),
      hybrid_slam::mapping::SubmapState::kFinished,
      {hybrid_slam::common::KeyFrameId(1)},
      hybrid_slam::common::KeyFrameId(1),
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(map_x, 0.0, 0.0)},
      submap_cloud,
      submap_cloud,
      {Eigen::Vector3d(1.0, 0.0, 0.0),
       Eigen::Vector3d(1.1, 0.0, 0.0), true},
      {1.0F},
      true,
  });
  return snapshot;
}

std::shared_ptr<hybrid_slam::mapping::MapSnapshot> MakeWideSnapshot() {
  auto snapshot = MakeSnapshot(0.0);
  auto second = snapshot->keyframes.front();
  second.node_id = hybrid_slam::common::NodeId(2);
  second.keyframe_id = hybrid_slam::common::KeyFrameId(2);
  second.submap_id = hybrid_slam::common::SubmapId(2);
  second.T_map_base.T_target_source.translation.x() = 1'000'000.0;
  snapshot->keyframes.push_back(std::move(second));
  auto second_submap = snapshot->submaps.front();
  second_submap.id = hybrid_slam::common::SubmapId(2);
  second_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(2);
  second_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(2)};
  second_submap.T_map_submap.translation.x() = 1'000'000.0;
  snapshot->submaps.push_back(std::move(second_submap));
  return snapshot;
}

TEST(GlobalMapPublication, PublishesLatestOptimizedSnapshotAsVoxelizedMap) {
  const bool owns_rclcpp_context = !rclcpp::ok();
  if (owns_rclcpp_context) {
    rclcpp::init(0, nullptr);
  }

  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("lio.nav2_tf.enable", true);
    options.append_parameter_override("lio.nav2_tf.map_frame", "map");
    options.append_parameter_override("lio.output.global_map.enable", true);
    options.append_parameter_override("lio.output.global_map_voxel_size_m", 0.5);
    auto node = std::make_shared<rclcpp::Node>(
        "global_map_publication_test", options);
    auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    LI2Sup::PublicationBridge publication(
        *node, group, LI2Sup::PublicationBridge::Config::FromNode(*node));
    publication.setup();

    std::mutex mutex;
    std::condition_variable received_cv;
    sensor_msgs::msg::PointCloud2::SharedPtr received;
    const auto subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/lio/global_map", rclcpp::SensorDataQoS().keep_last(1),
        [&mutex, &received_cv, &received](
            sensor_msgs::msg::PointCloud2::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex);
          received = std::move(message);
          received_cv.notify_all();
        });

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    std::jthread spinner([&executor] { executor.spin(); });

    publication.setBackendMapSnapshot(MakeSnapshot(5.0));
    {
      std::unique_lock<std::mutex> lock(mutex);
      ASSERT_TRUE(received_cv.wait_for(lock, 3s, [&received] {
        return static_cast<bool>(received);
      }));
    }

    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "map");
    EXPECT_EQ(received->width, 1U);
    EXPECT_EQ(received->height, 1U);

    publication.resetSession();
    {
      std::lock_guard<std::mutex> lock(mutex);
      received.reset();
    }
    publication.setBackendMapSnapshot(MakeWideSnapshot());
    {
      std::unique_lock<std::mutex> lock(mutex);
      ASSERT_TRUE(received_cv.wait_for(lock, 3s, [&received] {
        return static_cast<bool>(received);
      }));
    }
    ASSERT_TRUE(received);
    EXPECT_EQ(received->header.frame_id, "map");
    EXPECT_EQ(received->width, 2U);

    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    (void)subscription;
  }

  if (owns_rclcpp_context) {
    rclcpp::shutdown();
  }
}

}  // namespace
