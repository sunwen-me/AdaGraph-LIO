#include "ros/occupancy_grid_publisher.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace LI2Sup {

void OccupancyGridPublisher::setup(
    rclcpp::Node& node,
    const rclcpp::CallbackGroup::SharedPtr& callback_group) {
  node.declare_parameter<bool>("lio.occupancy_grid.enable", false);
  node.get_parameter("lio.occupancy_grid.enable", enabled_);

  node.declare_parameter<std::string>("lio.occupancy_grid.topic", "/map");
  node.get_parameter("lio.occupancy_grid.topic", topic_);

  node.declare_parameter<std::string>("lio.occupancy_grid.frame_id", "world");
  node.get_parameter("lio.occupancy_grid.frame_id", frame_id_);

  node.declare_parameter<double>("lio.occupancy_grid.resolution", 0.1);
  node.get_parameter("lio.occupancy_grid.resolution", resolution_);
  resolution_ = std::max(0.01, resolution_);

  node.declare_parameter<double>("lio.occupancy_grid.z_min", -0.3);
  node.get_parameter("lio.occupancy_grid.z_min", z_min_);

  node.declare_parameter<double>("lio.occupancy_grid.z_max", 2.0);
  node.get_parameter("lio.occupancy_grid.z_max", z_max_);
  if (z_min_ > z_max_) {
    std::swap(z_min_, z_max_);
  }

  node.declare_parameter<double>("lio.occupancy_grid.publish_period", 1.0);
  node.get_parameter("lio.occupancy_grid.publish_period", publish_period_);
  publish_period_ = std::max(0.1, publish_period_);

  node.declare_parameter<double>("lio.occupancy_grid.map_padding", 1.0);
  node.get_parameter("lio.occupancy_grid.map_padding", map_padding_);
  map_padding_ = std::max(0.0, map_padding_);

  node.declare_parameter<double>("lio.occupancy_grid.inflation_radius", 0.0);
  node.get_parameter("lio.occupancy_grid.inflation_radius", inflation_radius_);
  inflation_radius_ = std::max(0.0, inflation_radius_);

  node.declare_parameter<bool>("lio.occupancy_grid.use_unknown", false);
  node.get_parameter("lio.occupancy_grid.use_unknown", use_unknown_);

  node.declare_parameter<bool>("lio.occupancy_grid.reset_on_global_map", true);
  node.get_parameter("lio.occupancy_grid.reset_on_global_map", reset_on_global_map_);

  node.declare_parameter<int>("lio.occupancy_grid.occupied_value", 100);
  node.get_parameter("lio.occupancy_grid.occupied_value", occupied_value_);
  occupied_value_ = std::clamp(occupied_value_, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.free_value", 0);
  node.get_parameter("lio.occupancy_grid.free_value", free_value_);
  free_value_ = std::clamp(free_value_, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.unknown_value", -1);
  node.get_parameter("lio.occupancy_grid.unknown_value", unknown_value_);
  unknown_value_ = std::clamp(unknown_value_, -1, 100);

  ground_filter_.setup(node, "lio.occupancy_grid.ground_filter");

  node.declare_parameter<bool>(
      "lio.occupancy_grid.ground_filter.publish_debug_clouds", false);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.publish_debug_clouds",
      publish_ground_debug_);

  node.declare_parameter<std::string>(
      "lio.occupancy_grid.ground_filter.ground_topic", ground_debug_topic_);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.ground_topic", ground_debug_topic_);

  node.declare_parameter<std::string>(
      "lio.occupancy_grid.ground_filter.non_ground_topic", non_ground_debug_topic_);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.non_ground_topic",
      non_ground_debug_topic_);

  if (!enabled_) {
    RCLCPP_INFO(node.get_logger(), "Occupancy grid publisher disabled.");
    return;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);
  if (publish_ground_debug_) {
    auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    ground_debug_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            ground_debug_topic_, debug_qos);
    non_ground_debug_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            non_ground_debug_topic_, debug_qos);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_));
  timer_ = node.create_wall_timer(
      period,
      [this]() { publish(); },
      callback_group);

  RCLCPP_INFO(node.get_logger(),
              "Occupancy grid enabled: topic=%s frame=%s resolution=%.3f z=[%.2f, %.2f]",
              topic_.c_str(), frame_id_.c_str(), resolution_, z_min_, z_max_);
}

void OccupancyGridPublisher::insertCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !cloud || cloud->empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  insertCloudLocked(cloud);
  latest_stamp_ = stamp;
}

void OccupancyGridPublisher::insertBodyCloud(
    const BASIC::CloudPtr& body_cloud,
    const Eigen::Matrix4f& body_to_world,
    const Eigen::Quaterniond& body_to_world_rotation,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !body_cloud || body_cloud->empty()) {
    return;
  }

  const auto segmented =
      ground_filter_.segment(body_cloud, body_to_world_rotation);
  publishDebugClouds(segmented, body_to_world, stamp);

  if (!segmented.non_ground || segmented.non_ground->empty()) {
    return;
  }

  BASIC::CloudPtr non_ground_world(new BASIC::PointCloudType());
  pcl::transformPointCloud(*segmented.non_ground, *non_ground_world, body_to_world);
  insertCloud(non_ground_world, stamp);
}

void OccupancyGridPublisher::resetFromCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !cloud) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (reset_on_global_map_) {
    occupied_cells_.clear();
    has_bounds_ = false;
  }
  insertCloudLocked(cloud);
  latest_stamp_ = stamp;
}

void OccupancyGridPublisher::publish() {
  if (!enabled_ || !publisher_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (occupied_cells_.empty() || !has_bounds_) {
      return;
    }
    msg = buildMessageLocked(latest_stamp_);
  }

  publisher_->publish(msg);
}

void OccupancyGridPublisher::publishDebugClouds(
    const GroundSegmentationFilter::Result& result,
    const Eigen::Matrix4f& body_to_world,
    const rclcpp::Time& stamp) {
  if (!publish_ground_debug_ || !ground_debug_pub_ || !non_ground_debug_pub_) {
    return;
  }

  auto publish_cloud =
      [&](const BASIC::CloudPtr& body_cloud,
          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
        if (!body_cloud) {
          return;
        }

        BASIC::CloudPtr world_cloud(new BASIC::PointCloudType());
        pcl::transformPointCloud(*body_cloud, *world_cloud, body_to_world);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*world_cloud, msg);
        msg.header.frame_id = frame_id_;
        msg.header.stamp = stamp;
        pub->publish(msg);
      };

  publish_cloud(result.ground, ground_debug_pub_);
  publish_cloud(result.non_ground, non_ground_debug_pub_);
}

void OccupancyGridPublisher::insertCloudLocked(const BASIC::CloudPtr& cloud) {
  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& point : cloud->points) {
    if (!pointPassesHeightFilter(point)) {
      continue;
    }

    const auto cell_x = static_cast<std::int32_t>(std::floor(point.x / resolution_));
    const auto cell_y = static_cast<std::int32_t>(std::floor(point.y / resolution_));

    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
          continue;
        }

        const std::int32_t x = cell_x + dx;
        const std::int32_t y = cell_y + dy;
        occupied_cells_.insert(cellKey(x, y));

        if (!has_bounds_) {
          min_x_ = max_x_ = x;
          min_y_ = max_y_ = y;
          has_bounds_ = true;
        } else {
          min_x_ = std::min(min_x_, x);
          max_x_ = std::max(max_x_, x);
          min_y_ = std::min(min_y_, y);
          max_y_ = std::max(max_y_, y);
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid OccupancyGridPublisher::buildMessageLocked(
    const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = stamp.nanoseconds() > 0 ? stamp : rclcpp::Clock().now();
  msg.info.resolution = static_cast<float>(resolution_);

  const int padding_cells = static_cast<int>(std::ceil(map_padding_ / resolution_));
  const std::int32_t origin_x = min_x_ - padding_cells;
  const std::int32_t origin_y = min_y_ - padding_cells;
  const std::int32_t end_x = max_x_ + padding_cells;
  const std::int32_t end_y = max_y_ + padding_cells;

  const auto width64 = static_cast<std::int64_t>(end_x) - origin_x + 1;
  const auto height64 = static_cast<std::int64_t>(end_y) - origin_y + 1;
  if (width64 <= 0 || height64 <= 0 ||
      width64 > std::numeric_limits<std::uint32_t>::max() ||
      height64 > std::numeric_limits<std::uint32_t>::max()) {
    return msg;
  }

  msg.info.width = static_cast<std::uint32_t>(width64);
  msg.info.height = static_cast<std::uint32_t>(height64);
  msg.info.origin.position.x = static_cast<double>(origin_x) * resolution_;
  msg.info.origin.position.y = static_cast<double>(origin_y) * resolution_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  const int8_t default_value =
      static_cast<int8_t>(use_unknown_ ? unknown_value_ : free_value_);
  msg.data.assign(static_cast<std::size_t>(msg.info.width) * msg.info.height,
                  default_value);

  for (const auto key : occupied_cells_) {
    const auto x = keyX(key) - origin_x;
    const auto y = keyY(key) - origin_y;
    if (x < 0 || y < 0 ||
        x >= static_cast<std::int32_t>(msg.info.width) ||
        y >= static_cast<std::int32_t>(msg.info.height)) {
      continue;
    }

    const auto index = static_cast<std::size_t>(y) * msg.info.width +
                       static_cast<std::size_t>(x);
    msg.data[index] = static_cast<int8_t>(occupied_value_);
  }

  return msg;
}

std::int64_t OccupancyGridPublisher::cellKey(
    std::int32_t x,
    std::int32_t y) {
  const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
  return static_cast<std::int64_t>((ux << 32) | uy);
}

std::int32_t OccupancyGridPublisher::keyX(std::int64_t key) {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey >> 32));
}

std::int32_t OccupancyGridPublisher::keyY(std::int64_t key) {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey & 0xffffffff));
}

bool OccupancyGridPublisher::pointPassesHeightFilter(
    const BASIC::PointType& point) const {
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z) &&
         point.z >= z_min_ &&
         point.z <= z_max_;
}

}  // namespace LI2Sup
