#include "ros/tf_bridge.h"

#include <cmath>

namespace LI2Sup {

TfBridge::TfBridge(rclcpp::Node& node, Config config,
                   MapToOdomProvider map_to_odom_provider)
    : node_(node),
      config_(std::move(config)),
      map_to_odom_provider_(std::move(map_to_odom_provider)),
      broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(&node_)) {}

geometry_msgs::msg::Transform TfBridge::toMessage(
    const hybrid_slam::geometry::RigidTransform3d& transform) {
  geometry_msgs::msg::Transform message;
  message.translation.x = transform.translation.x();
  message.translation.y = transform.translation.y();
  message.translation.z = transform.translation.z();
  const auto quaternion = transform.rotation.normalized();
  message.rotation.x = quaternion.x();
  message.rotation.y = quaternion.y();
  message.rotation.z = quaternion.z();
  message.rotation.w = quaternion.w();
  return message;
}

void TfBridge::publishOdomBase(const rclcpp::Time& stamp,
                               const hybrid_slam::geometry::RigidTransform3d& pose) {
  if (!config_.nav2_enabled || !broadcaster_) {
    return;
  }

  auto planar_pose = pose;
  if (config_.planar_constraint) {
    const double yaw = std::atan2(
        pose.rotation.toRotationMatrix()(1, 0),
        pose.rotation.toRotationMatrix()(0, 0));
    planar_pose.rotation = Eigen::Quaterniond(
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    planar_pose.translation.z() = config_.planar_base_z;
  }

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = stamp;
  odom_tf.header.frame_id = config_.odom_frame;
  odom_tf.child_frame_id = config_.base_frame;
  odom_tf.transform = toMessage(planar_pose);
  broadcaster_->sendTransform(odom_tf);
  const auto odom_stamp_ns = stamp.nanoseconds();
  const auto previous_odom_stamp =
      last_odom_base_stamp_ns_.exchange(odom_stamp_ns, std::memory_order_relaxed);
  if (previous_odom_stamp != std::numeric_limits<std::int64_t>::min() &&
      odom_stamp_ns <= previous_odom_stamp) {
    nonmonotonic_odom_base_stamps_.fetch_add(1, std::memory_order_relaxed);
  }
  odom_base_published_.fetch_add(1, std::memory_order_relaxed);

  if (!config_.publish_map_to_odom || !map_to_odom_provider_) {
    return;
  }
  const auto map_to_odom = map_to_odom_provider_();
  if (!map_to_odom) {
    return;
  }

  geometry_msgs::msg::TransformStamped map_tf;
  map_tf.header.stamp = stamp;
  map_tf.header.frame_id = config_.map_frame;
  map_tf.child_frame_id = config_.odom_frame;
  map_tf.transform = toMessage(*map_to_odom);
  broadcaster_->sendTransform(map_tf);
  const auto previous_map_stamp =
      last_map_odom_stamp_ns_.exchange(odom_stamp_ns, std::memory_order_relaxed);
  if (previous_map_stamp != std::numeric_limits<std::int64_t>::min() &&
      odom_stamp_ns <= previous_map_stamp) {
    nonmonotonic_map_odom_stamps_.fetch_add(1, std::memory_order_relaxed);
  }
  map_odom_published_.fetch_add(1, std::memory_order_relaxed);
}

TfBridge::PublicationStats TfBridge::publicationStats() const noexcept {
  return {
      odom_base_published_.load(std::memory_order_relaxed),
      map_odom_published_.load(std::memory_order_relaxed),
      nonmonotonic_odom_base_stamps_.load(std::memory_order_relaxed),
      nonmonotonic_map_odom_stamps_.load(std::memory_order_relaxed),
  };
}

void TfBridge::publishWorldImu(const rclcpp::Time& stamp,
                               const hybrid_slam::geometry::RigidTransform3d& pose) {
  if (!broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "world";
  tf.child_frame_id = config_.imu_frame;
  tf.transform = toMessage(pose);
  broadcaster_->sendTransform(tf);
}

}  // namespace LI2Sup
