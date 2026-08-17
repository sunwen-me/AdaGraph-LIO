#ifndef LI2SUP_SENSOR_BRIDGE_H_
#define LI2SUP_SENSOR_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "common/ds.h"
#include "hybrid_slam/sensor/imu_data.h"
#include "hybrid_slam/sensor/point_cloud_frame.h"

namespace LI2Sup {

// ROS-facing sensor seam. It owns subscriptions and message/layout
// conversion only, then emits ROS-independent values. Buffering, frontend
// scheduling and all output publication belong to the application/system
// composition that consumes these callbacks.
class SensorBridge final {
 public:
  struct Config {
    std::string imu_topic;
    std::string lidar_topic;
    int lidar_type = LID_TYPE::SIM_XYZI;
    double blind_squared = 0.0;
    double max_range_squared = 100.0 * 100.0;
    int filter_rate = 1;
    Eigen::Vector3d imu_acc_scale = Eigen::Vector3d::Ones();
    Eigen::Vector3d imu_gyro_scale = Eigen::Vector3d::Ones();
    double imu_time_offset = 0.0;
    std::size_t imu_queue_depth = 500;
    std::size_t lidar_queue_depth = 20;
    bool reliable = false;

    [[nodiscard]] std::optional<std::string> validate() const;
  };

  using ImuCallback =
      std::function<void(hybrid_slam::sensor::ImuData)>;
  using LidarCallback =
      std::function<void(hybrid_slam::sensor::PointCloudFrame&&)>;

  SensorBridge(rclcpp::Node& node,
               rclcpp::CallbackGroup::SharedPtr callback_group,
               Config config,
               ImuCallback imu_callback,
               LidarCallback lidar_callback);

  [[nodiscard]] static Config FromNode(rclcpp::Node& node);

  SensorBridge(const SensorBridge&) = delete;
  SensorBridge& operator=(const SensorBridge&) = delete;

  // Subscription creation is explicit so the owner can finish its parameter
  // setup before any callback can arrive.
  void start();

  // Subscriptions remain installed across lifecycle transitions, but disabled
  // input is discarded before it reaches the system boundary.
  void setInputEnabled(bool enabled) noexcept {
    input_enabled_.store(enabled, std::memory_order_release);
  }
  [[nodiscard]] bool inputEnabled() const noexcept {
    return input_enabled_.load(std::memory_order_acquire);
  }

  [[nodiscard]] static bool isSupportedLidarType(int lidar_type);
  [[nodiscard]] static std::string lidarTypeName(int lidar_type);
  [[nodiscard]] static int lidarTypeFromName(std::string_view name);

 private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  struct ParsedPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
    float intensity = 0.0F;
    double offset_seconds = 0.0;
  };

  void emitLidar(std::vector<ParsedPoint>&& points,
                 double header_time_seconds);
  [[nodiscard]] bool validPoint(double x, double y, double z) const;
  [[nodiscard]] static double stampToSec(
      const builtin_interfaces::msg::Time& stamp);
  [[nodiscard]] static const sensor_msgs::msg::PointField* findPointField(
      const sensor_msgs::msg::PointCloud2& msg, const std::string& name);
  [[nodiscard]] static bool hasLivoxPointCloud2Layout(
      const sensor_msgs::msg::PointCloud2& msg, std::string& error);

  rclcpp::Node& node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Config config_;
  ImuCallback imu_callback_;
  LidarCallback lidar_callback_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      livox_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      pointcloud_subscription_;
  std::atomic<bool> input_enabled_{true};
  bool started_ = false;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SENSOR_BRIDGE_H_
