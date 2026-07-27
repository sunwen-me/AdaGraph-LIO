
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <execution>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

/// msgs
#include "super_lio/msg/cloud_pose.hpp"
#include "super_lio/msg/cloud_pose2.hpp"


#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "ros/occupancy_grid_publisher.h"


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper(){};
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}

  // 获取最新IMU数据（用于AdaGraph特征提取）
  IMUData getLatestImuData() const {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (imu_buffer_.empty()) {
      return IMUData();
    }
    return imu_buffer_.back();
  }

  void clear(){
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    lidar_buffer_.clear();
    imu_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
  }

  void pub_odom(const NavState&);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void update_occupancy_grid_from_body_cloud(const BASIC::CloudPtr& pc,
                                             const NavState& state);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc,
                           const NavState& state);
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);

  void set_global_map(const BASIC::CloudPtr& global_map);
  void set_prior_map_localization_active(bool active);
  void set_map_to_odom(const BASIC::SE3& map_to_odom);
  bool get_map_to_odom(BASIC::SE3& map_to_odom) const;
  bool consume_manual_relocalization_request();

  /// Optimized map-frame keyframe trajectory {timestamp [s], pose} from the
  /// AdaGraph backend (recording/evaluation output, not used for control).
  void pub_backend_optimized_path(
      const std::vector<std::pair<double, BASIC::SE3>>& trajectory);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void pushLidarData(LidarData& lidar_data);
  void publishNav2Tf(const rclcpp::Time& stamp,
                     const BASIC::V3& odom_robo_position,
                     const BASIC::M3& odom_robo_rotation);

  void setupParams();
  void setupIO();

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_manual_relocalization_;

  mutable std::mutex buffer_mutex_;
  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;

  double imu_time_offset_ = 0.0;
  std::size_t imu_buffer_capacity_ = 2000;
  std::size_t lidar_buffer_capacity_ = 100;
  std::size_t path_max_poses_ = 5000;
  double global_map_pub_period_ = 10.0;

  ESKF::Ptr eskf_{nullptr};

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  bool global_map_dirty_ = false;
  std::size_t global_map_sub_count_ = 0;
  rclcpp::TimerBase::SharedPtr global_map_timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<OccupancyGridPublisher> occupancy_grid_publisher_;
  bool prior_map_localization_active_ = false;
  mutable std::mutex map_to_odom_mutex_;
  BASIC::SE3 map_to_odom_;
  bool has_map_to_odom_ = false;
  mutable std::mutex manual_relocalization_mutex_;
  bool manual_relocalization_requested_ = false;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// output.
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_msg2uav_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud2robot_;
  rclcpp::Publisher<super_lio::msg::CloudPose>::SharedPtr pub_cloud_body_pose_;
  rclcpp::Publisher<super_lio::msg::CloudPose>::SharedPtr pub_cloud_world_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_processing_time_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_map_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_backend_optimized_path_;
};

} // namespace END.

#endif
