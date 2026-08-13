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
    const BASIC::V3& position, const BASIC::M3& rotation) {
  geometry_msgs::msg::Transform message;
  message.translation.x = position.x();
  message.translation.y = position.y();
  message.translation.z = position.z();
  BASIC::Quat quaternion(rotation);
  quaternion.normalize();
  message.rotation.x = quaternion.x();
  message.rotation.y = quaternion.y();
  message.rotation.z = quaternion.z();
  message.rotation.w = quaternion.w();
  return message;
}

geometry_msgs::msg::Transform TfBridge::toMessage(
    const adagraph::geometry::RigidTransform3d& transform) {
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
                               const BASIC::V3& position,
                               const BASIC::M3& rotation) {
  if (!config_.nav2_enabled || !broadcaster_) {
    return;
  }

  BASIC::V3 tf_position = position;
  BASIC::M3 tf_rotation = rotation;
  if (config_.planar_constraint) {
    const BASIC::scalar yaw =
        std::atan2(tf_rotation(1, 0), tf_rotation(0, 0));
    tf_rotation = Eigen::AngleAxis<BASIC::scalar>(
                      yaw, BASIC::V3::UnitZ())
                      .toRotationMatrix();
    tf_position.z() = static_cast<BASIC::scalar>(config_.planar_base_z);
  }

  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = stamp;
  odom_tf.header.frame_id = config_.odom_frame;
  odom_tf.child_frame_id = config_.base_frame;
  odom_tf.transform = toMessage(tf_position, tf_rotation);
  broadcaster_->sendTransform(odom_tf);

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
}

void TfBridge::publishWorldImu(const rclcpp::Time& stamp,
                               const BASIC::V3& position,
                               const BASIC::M3& rotation) {
  if (!broadcaster_) {
    return;
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "world";
  tf.child_frame_id = config_.imu_frame;
  tf.transform = toMessage(position, rotation);
  broadcaster_->sendTransform(tf);
}

}  // namespace LI2Sup
