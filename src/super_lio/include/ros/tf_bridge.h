#ifndef LI2SUP_TF_BRIDGE_H_
#define LI2SUP_TF_BRIDGE_H_

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "adagraph/geometry/transform3.h"
#include "basic/alias.h"

namespace LI2Sup {

// ROS-only transform publisher. It consumes value poses and an immutable
// map->odom provider; it does not own or mutate global alignment semantics.
class TfBridge final {
 public:
  struct Config {
    bool nav2_enabled = false;
    bool publish_map_to_odom = false;
    bool planar_constraint = false;
    double planar_base_z = 0.0;
    std::string map_frame = "map";
    std::string odom_frame = "odom";
    std::string base_frame = "base_link";
    std::string imu_frame = "imu";
  };

  using MapToOdomProvider = std::function<std::optional<
      adagraph::geometry::RigidTransform3d>()>;

  TfBridge(rclcpp::Node& node, Config config,
           MapToOdomProvider map_to_odom_provider = {});

  void publishOdomBase(const rclcpp::Time& stamp,
                       const BASIC::V3& position,
                       const BASIC::M3& rotation);
  void publishWorldImu(const rclcpp::Time& stamp,
                       const BASIC::V3& position,
                       const BASIC::M3& rotation);

 private:
  static geometry_msgs::msg::Transform toMessage(
      const BASIC::V3& position, const BASIC::M3& rotation);
  static geometry_msgs::msg::Transform toMessage(
      const adagraph::geometry::RigidTransform3d& transform);

  rclcpp::Node& node_;
  Config config_;
  MapToOdomProvider map_to_odom_provider_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_TF_BRIDGE_H_
