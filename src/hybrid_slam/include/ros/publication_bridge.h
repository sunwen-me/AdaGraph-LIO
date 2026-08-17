#ifndef LI2SUP_PUBLICATION_BRIDGE_H_
#define LI2SUP_PUBLICATION_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "common/navigation_state.h"
#include "common/ds.h"
#include "ros/odometry_rate_limiter.h"
#include "ros/occupancy_grid_publisher.h"
#include "ros/tf_bridge.h"

namespace LI2Sup {

// ROS-only output adapter for the Point-LIO composition. It owns publishers,
// timers, TF and occupancy materialization
// but has no sensor subscriptions or SLAM state ownership.
class PublicationBridge final {
 public:
  struct Config {
    bool nav2_tf_enable = false;
    bool nav2_publish_map_to_odom = false;
    bool ground_robot_planar_constraint = false;
    double ground_robot_base_z = 0.0;
    bool publish_robot_pose = false;
    hybrid_slam::geometry::StaticTransform3d T_base_imu{
        hybrid_slam::geometry::FrameId::kBase,
        hybrid_slam::geometry::FrameId::kImu,
        hybrid_slam::geometry::RigidTransform3d::Identity()};
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string imu_frame = "imu";
    std::size_t path_max_poses = 5000;
    // Zero preserves every frontend output. A positive value caps the ROS
    // odom/TF/path output rate without changing estimator update frequency.
    double odometry_publish_rate_hz = 0.0;
    bool publish_global_map = true;
    double global_map_voxel_size_m = 0.20;
    double global_map_pub_period = 10.0;
    double backend_path_pub_period = 0.25;

    bool odom_twist_filter_enabled = false;
    bool odom_twist_filter_zero_lateral_velocity = false;
    double odom_twist_filter_linear_time_constant = 0.08;
    double odom_twist_filter_angular_time_constant = 0.06;
    double odom_twist_filter_max_linear_speed = 2.0;
    double odom_twist_filter_max_angular_speed = 4.0;
    double odom_twist_filter_max_linear_acceleration = 4.0;
    double odom_twist_filter_max_angular_acceleration = 8.0;

    [[nodiscard]] static Config FromNode(rclcpp::Node& node);
  };

  PublicationBridge(rclcpp::Node& node,
                    rclcpp::CallbackGroup::SharedPtr callback_group,
                    Config config);
  ~PublicationBridge();

  PublicationBridge(const PublicationBridge&) = delete;
  PublicationBridge& operator=(const PublicationBridge&) = delete;

  void setup();
  // Must run before rclcpp::shutdown() when a map build may still be active.
  void shutdown();
  void resetSession();

  [[nodiscard]] bool occupancyGridEnabled() const noexcept {
    return occupancy_grid_publisher_ && occupancy_grid_publisher_->enabled();
  }

  void setPriorMapLocalizationActive(bool active);
  void setMapToOdom(
      const hybrid_slam::geometry::RigidTransform3d& map_to_odom);
  [[nodiscard]] bool getMapToOdom(
      hybrid_slam::geometry::RigidTransform3d& map_to_odom) const;
  [[nodiscard]] TfBridge::PublicationStats transformPublicationStats() const noexcept;
  void setBackendMapSnapshot(
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) noexcept;
  [[nodiscard]] std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>
  backendMapSnapshot() const noexcept {
    return pending_backend_snapshot_.load(std::memory_order_acquire);
  }

  void publishOdom(const NavState& state,
                   const Eigen::Vector3d& angular_velocity);
  void publishCloudWorld(
      const hybrid_slam::sensor::ImmutablePointCloud& cloud,
      double timestamp);
  void updateOccupancyGridFromBodyCloud(
      const hybrid_slam::sensor::ImmutablePointCloud& cloud,
      const NavState& state);
  void setGlobalMap(const hybrid_slam::sensor::ImmutablePointCloud& global_map);

 private:
  static builtin_interfaces::msg::Time toRosTime(double seconds);
  static geometry_msgs::msg::Pose toPose(const Eigen::Vector3d& position,
                                         const Eigen::Matrix3d& rotation);
  void filterOdomTwist(const rclcpp::Time& stamp,
                       Eigen::Vector3d& linear_velocity,
                       Eigen::Vector3d& angular_velocity);
  void publishBackendPath();
  void globalMapWorker(std::stop_token stop_token);
  void publishGlobalMapMessage(sensor_msgs::msg::PointCloud2 message);

  rclcpp::Node& node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Config config_;
  bool prior_map_localization_active_ = false;
  mutable std::mutex map_to_odom_mutex_;
  hybrid_slam::geometry::RigidTransform3d map_to_odom_;
  bool has_map_to_odom_ = false;
  bool odom_twist_filter_initialized_ = false;
  double odom_twist_filter_last_stamp_ = -1.0;
  Eigen::Vector3d filtered_odom_linear_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d filtered_odom_angular_velocity_ = Eigen::Vector3d::Zero();
  OdometryRateLimiter odometry_rate_limiter_;

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  mutable std::mutex global_map_mutex_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  bool global_map_dirty_ = false;
  std::size_t global_map_sub_count_ = 0;
  std::uint64_t global_map_generation_ = 0;
  Eigen::Vector3d last_path_point_ = Eigen::Vector3d(0, 0, -100);

  std::atomic<std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>>
      pending_backend_snapshot_{nullptr};
  std::uint64_t published_backend_snapshot_version_ = 0;
  std::mutex global_map_work_mutex_;
  std::condition_variable global_map_work_cv_;
  bool global_map_build_requested_ = false;
  std::size_t requested_finished_submaps_ = 0;
  std::size_t requested_global_constraints_ = 0;
  bool requested_snapshot_without_submaps_ = false;
  std::jthread global_map_worker_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      pub_backend_optimized_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_cloud_world_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_msg2uav_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_;

  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::TimerBase::SharedPtr backend_path_timer_;
  std::unique_ptr<TfBridge> tf_bridge_;
  std::unique_ptr<OccupancyGridPublisher> occupancy_grid_publisher_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PUBLICATION_BRIDGE_H_
