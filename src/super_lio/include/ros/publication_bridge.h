#ifndef LI2SUP_PUBLICATION_BRIDGE_H_
#define LI2SUP_PUBLICATION_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adagraph/mapping/map_snapshot.h"
#include "adagraph/geometry/transform3.h"
#include "basic/alias.h"
#include "common/ds.h"
#include "ros/occupancy_grid_publisher.h"
#include "ros/tf_bridge.h"
#include "super_lio/msg/cloud_pose.hpp"

namespace LI2Sup {

// ROS-only output adapter used by the standard Super-LIO and Point-LIO
// compositions.  It owns publishers, timers, TF and occupancy materialization
// but has no sensor subscriptions or SLAM state ownership.
class PublicationBridge final {
 public:
  struct Config {
    bool nav2_tf_enable = false;
    bool nav2_publish_map_to_odom = false;
    bool ground_robot_planar_constraint = false;
    double ground_robot_base_z = 0.0;
    bool publish_robot_pose = false;
    adagraph::geometry::StaticTransform3d T_base_imu{
        adagraph::geometry::FrameId::kBase,
        adagraph::geometry::FrameId::kImu,
        adagraph::geometry::RigidTransform3d::Identity()};
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string imu_frame = "imu";
    std::size_t path_max_poses = 5000;
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
  void resetSession();

  [[nodiscard]] bool occupancyGridEnabled() const noexcept {
    return occupancy_grid_publisher_ && occupancy_grid_publisher_->enabled();
  }

  void setPriorMapLocalizationActive(bool active);
  void setMapToOdom(const BASIC::SE3& map_to_odom);
  [[nodiscard]] bool getMapToOdom(BASIC::SE3& map_to_odom) const;
  void setBackendMapSnapshot(
      std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept;
  [[nodiscard]] std::shared_ptr<const adagraph::mapping::MapSnapshot>
  backendMapSnapshot() const noexcept {
    return pending_backend_snapshot_.load(std::memory_order_acquire);
  }

  void publishOdom(const NavState& state,
                   const BASIC::V3& angular_velocity);
  void publishLegacyImuPrediction(
      const builtin_interfaces::msg::Time& stamp,
      const DynamicState& imu_state,
      const DynamicState& robot_state);
  void publishCloudWorld(const BASIC::CloudPtr& cloud, double timestamp);
  void updateOccupancyGridFromBodyCloud(const BASIC::CloudPtr& cloud,
                                        const NavState& state);
  void setGlobalMap(const BASIC::CloudPtr& global_map);

  // Compatibility output methods retained for external callers while
  // ownership is moved out of ROSWrapper.
  void publishCloud2Planner(const BASIC::CloudPtr& cloud, double timestamp);
  void publishCloudBodyPose(const BASIC::CloudPtr& cloud,
                            const NavState& state);
  void publishCloudWorldPose(const BASIC::CloudPtr& cloud,
                             const NavState& state);
  void publishProcessingTime(double timestamp, double current_time,
                             double mean_time, double std_time);

 private:
  static builtin_interfaces::msg::Time toRosTime(double seconds);
  static geometry_msgs::msg::Pose toPose(const BASIC::V3& position,
                                         const BASIC::M3& rotation);
  void filterOdomTwist(const rclcpp::Time& stamp,
                       BASIC::V3& linear_velocity,
                       BASIC::V3& angular_velocity);
  void publishBackendPath();

  rclcpp::Node& node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Config config_;
  bool prior_map_localization_active_ = false;
  mutable std::mutex map_to_odom_mutex_;
  BASIC::SE3 map_to_odom_;
  bool has_map_to_odom_ = false;
  bool odom_twist_filter_initialized_ = false;
  double odom_twist_filter_last_stamp_ = -1.0;
  BASIC::V3 filtered_odom_linear_velocity_ = BASIC::V3::Zero();
  BASIC::V3 filtered_odom_angular_velocity_ = BASIC::V3::Zero();

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  bool global_map_dirty_ = false;
  std::size_t global_map_sub_count_ = 0;
  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

  std::atomic<std::shared_ptr<const adagraph::mapping::MapSnapshot>>
      pending_backend_snapshot_{nullptr};
  std::uint64_t published_backend_snapshot_version_ = 0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      pub_backend_optimized_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_cloud_world_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_msg2uav_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_cloud2robot_;
  rclcpp::Publisher<super_lio::msg::CloudPose>::SharedPtr
      pub_cloud_body_pose_;
  rclcpp::Publisher<super_lio::msg::CloudPose>::SharedPtr
      pub_cloud_world_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      pub_processing_time_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_;

  rclcpp::TimerBase::SharedPtr global_map_timer_;
  rclcpp::TimerBase::SharedPtr backend_path_timer_;
  std::unique_ptr<TfBridge> tf_bridge_;
  std::unique_ptr<OccupancyGridPublisher> occupancy_grid_publisher_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PUBLICATION_BRIDGE_H_
