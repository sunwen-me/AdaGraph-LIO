/**
 * This file is part of the AdaGraph-LIO integration of Small Point-LIO.
 * Licensed under the MIT License. See LICENSE.txt in the vendor root.
 */

#pragma once

#include "base_lidar.h"

#include <cmath>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace small_point_lio {

/**
 * Adapter for the conventional Velodyne PointXYZIRT layout used by M2DGR.
 * The float32 `time` field is a per-point offset in seconds relative to the
 * PointCloud2 header timestamp and may be negative when the header marks the
 * end of a scan.
 */
class VelodynePointCloud2Adapter final : public LidarAdapterBase {
 public:
  void setup_subscription(
      rclcpp::Node* node,
      const std::string& topic,
      std::function<void(const std::vector<common::Point>&)> callback,
      const rclcpp::QoS& qos,
      const rclcpp::SubscriptionOptions& options) override {
    subscription_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        qos,
        [callback = std::move(callback)](
            const sensor_msgs::msg::PointCloud2& msg) {
          sensor_msgs::PointCloud2ConstIterator<float> out_x(msg, "x");
          sensor_msgs::PointCloud2ConstIterator<float> out_y(msg, "y");
          sensor_msgs::PointCloud2ConstIterator<float> out_z(msg, "z");
          sensor_msgs::PointCloud2ConstIterator<float> out_time(msg, "time");

          const double scan_stamp =
              static_cast<double>(msg.header.stamp.sec) +
              static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
          const std::size_t size =
              static_cast<std::size_t>(msg.width) * msg.height;

          std::vector<common::Point> pointcloud;
          pointcloud.reserve(size);
          for (std::size_t i = 0; i < size; ++i) {
            if (std::isfinite(*out_x) && std::isfinite(*out_y) &&
                std::isfinite(*out_z) && std::isfinite(*out_time)) {
              common::Point point;
              point.position << *out_x, *out_y, *out_z;
              point.timestamp = scan_stamp + static_cast<double>(*out_time);
              pointcloud.push_back(point);
            }
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_time;
          }
          callback(pointcloud);
        },
        options);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace small_point_lio
