#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hybrid_slam/adapter/pcl_types.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "ros/occupancy_grid_publisher.h"

namespace {

using namespace std::chrono_literals;

hybrid_slam::sensor::ImmutablePointCloud MakePclBackedScan() {
  hybrid_slam::adapter::PclCloudPtr cloud(new hybrid_slam::adapter::PclPointCloud());
  constexpr int kRows = 16;
  constexpr int kColumns = 24;
  cloud->reserve(kRows * kColumns);
  for (int row = 0; row < kRows; ++row) {
    for (int column = 0; column < kColumns; ++column) {
      const float x = 2.0F + 0.18F * static_cast<float>(column);
      const float y = -2.0F + 0.25F * static_cast<float>(row);
      const float z = 0.15F + 0.02F * static_cast<float>((row + column) % 4);
      cloud->emplace_back(x, y, z, 1.0F);
    }
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;

  std::shared_ptr<const hybrid_slam::adapter::PclPointCloud> storage = cloud;
  const auto& first = storage->front();
  const auto* origin = reinterpret_cast<const std::byte*>(storage->data());
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(), sizeof(hybrid_slam::adapter::PclPoint),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      hybrid_slam::geometry::FrameId::kBase);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(
      std::move(storage), view, cloud->points.capacity() * sizeof(hybrid_slam::adapter::PclPoint));
}

void PrintSummary(const char* name, const LI2Sup::OccupancyLatencySummary& value) {
  std::cout << name << " samples=" << value.sample_count
            << " mean_us=" << value.mean_us
            << " p50_us<=" << value.p50_us
            << " p95_us<=" << value.p95_us
            << " p99_us<=" << value.p99_us
            << " max_us=" << value.max_us << '\n';
}

TEST(OnlineOccupancyTiming, MeasuresContentionAndTailLatency) {
  const bool owns_rclcpp_context = !rclcpp::ok();
  if (owns_rclcpp_context) {
    rclcpp::init(0, nullptr);
  }

  {
    rclcpp::NodeOptions options;
    options.append_parameter_override("lio.occupancy_grid.enable", true);
    options.append_parameter_override("lio.occupancy_grid.source", "online");
    options.append_parameter_override("lio.occupancy_grid.update_policy", "lazy");
    options.append_parameter_override("lio.occupancy_grid.publish_period", 0.002);
    options.append_parameter_override("lio.occupancy_grid.resolution", 0.1);
    options.append_parameter_override("lio.occupancy_grid.max_range", 15.0);
    options.append_parameter_override("lio.occupancy_grid.max_extent", 100.0);
    options.append_parameter_override("lio.occupancy_grid.raycast.enable", true);
    options.append_parameter_override(
        "lio.occupancy_grid.raycast.max_ray_length", 8.0);
    options.append_parameter_override(
        "lio.occupancy_grid.ground_filter.enable", false);
    auto node = std::make_shared<rclcpp::Node>(
        "online_occupancy_timing", options);
    auto group = node->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);

    LI2Sup::OccupancyGridPublisher publisher;
    publisher.setup(*node, group);
    ASSERT_TRUE(publisher.enabled());

    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    std::jthread spinner([&executor] { executor.spin(); });

    const auto scan = MakePclBackedScan();
    ASSERT_TRUE(scan.valid());
    constexpr std::size_t kUpdates = 20000;
    for (std::size_t index = 0; index < kUpdates; ++index) {
      Eigen::Matrix4f T_odom_base = Eigen::Matrix4f::Identity();
      T_odom_base(0, 3) = static_cast<float>(index % 200) * 0.02F;
      T_odom_base(1, 3) = static_cast<float>((index / 200) % 10) * 0.05F;
      publisher.insertBodyCloud(
          scan, T_odom_base, Eigen::Quaterniond::Identity(),
          rclcpp::Time(static_cast<std::int64_t>(index) * 10'000'000LL));
    }
    std::this_thread::sleep_for(50ms);

    executor.cancel();
    spinner.join();
    executor.remove_node(node);

    const auto stats = publisher.onlineTimingStats();
    PrintSummary("online_update_lock_wait", stats.update_lock_wait);
    PrintSummary("online_update_lock_hold", stats.update_lock_hold);
    PrintSummary("online_update_end_to_end", stats.update_end_to_end);
    PrintSummary("online_publish_lock_wait", stats.publish_lock_wait);
    PrintSummary("online_publish_lock_hold", stats.publish_lock_hold);
    PrintSummary("online_publish_end_to_end", stats.publish_end_to_end);

    EXPECT_EQ(stats.update_lock_wait.sample_count, kUpdates);
    EXPECT_EQ(stats.update_lock_hold.sample_count, kUpdates);
    EXPECT_EQ(stats.update_end_to_end.sample_count, kUpdates);
    EXPECT_GT(stats.publish_lock_hold.sample_count, 0U);
    EXPECT_LE(stats.update_lock_wait.p99_us, stats.update_lock_wait.max_us);
    EXPECT_LE(stats.update_lock_hold.p99_us, stats.update_lock_hold.max_us);
    EXPECT_LE(stats.update_end_to_end.p99_us, stats.update_end_to_end.max_us);
    EXPECT_LE(stats.publish_lock_hold.p99_us, stats.publish_lock_hold.max_us);
  }

  if (owns_rclcpp_context) {
    rclcpp::shutdown();
  }
}

}  // namespace
