#ifndef LI2SUP_TF_BRIDGE_H_
#define LI2SUP_TF_BRIDGE_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "hybrid_slam/geometry/transform3.h"

namespace LI2Sup {

// ROS-only transform publisher. It consumes value poses and an immutable
// map->odom provider; it does not own or mutate global alignment semantics.
class TfBridge final {
 public:
  struct PublicationStats {
    std::uint64_t odom_base_published = 0;
    std::uint64_t map_odom_published = 0;
    std::uint64_t nonmonotonic_odom_base_stamps = 0;
    std::uint64_t nonmonotonic_map_odom_stamps = 0;
  };
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
      hybrid_slam::geometry::RigidTransform3d>()>;

  TfBridge(rclcpp::Node& node, Config config,
           MapToOdomProvider map_to_odom_provider = {});

  void publishOdomBase(
      const rclcpp::Time& stamp,
      const hybrid_slam::geometry::RigidTransform3d& pose);
  void publishWorldImu(
      const rclcpp::Time& stamp,
      const hybrid_slam::geometry::RigidTransform3d& pose);
  [[nodiscard]] PublicationStats publicationStats() const noexcept;

 private:
  static geometry_msgs::msg::Transform toMessage(
      const hybrid_slam::geometry::RigidTransform3d& transform);

  rclcpp::Node& node_;
  Config config_;
  MapToOdomProvider map_to_odom_provider_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  std::atomic<std::uint64_t> odom_base_published_{0};
  std::atomic<std::uint64_t> map_odom_published_{0};
  std::atomic<std::uint64_t> nonmonotonic_odom_base_stamps_{0};
  std::atomic<std::uint64_t> nonmonotonic_map_odom_stamps_{0};
  std::atomic<std::int64_t> last_odom_base_stamp_ns_{std::numeric_limits<std::int64_t>::min()};
  std::atomic<std::int64_t> last_map_odom_stamp_ns_{std::numeric_limits<std::int64_t>::min()};
};

}  // namespace LI2Sup

#endif  // LI2SUP_TF_BRIDGE_H_
