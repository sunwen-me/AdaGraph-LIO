#include "ros/publication_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include "adagraph/mapping/snapshot_products.h"
#include "ros/ground_segmentation_filter.h"

namespace LI2Sup {

namespace {

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, value);
  }
  node.get_parameter(name, value);
  return value;
}

}  // namespace

PublicationBridge::Config PublicationBridge::Config::FromNode(
    rclcpp::Node& node) {
  Config config;
  config.nav2_tf_enable =
      ReadParameter<bool>(node, "lio.nav2_tf.enable", false);
  config.nav2_publish_map_to_odom = ReadParameter<bool>(
      node, "lio.nav2_tf.publish_map_to_odom", false);
  config.ground_robot_planar_constraint = ReadParameter<bool>(
      node, "lio.ground_robot.planar_constraint",
      false);
  config.ground_robot_base_z = ReadParameter<double>(
      node, "lio.ground_robot.base_z", 0.0);
  config.publish_robot_pose =
      ReadParameter<bool>(node, "lio.output.robot", false);
  config.map_frame =
      ReadParameter<std::string>(node, "lio.nav2_tf.map_frame", "map");
  config.odom_frame = ReadParameter<std::string>(
      node, "lio.nav2_tf.odom_frame", "odom");
  config.base_frame = ReadParameter<std::string>(
      node, "lio.nav2_tf.base_frame", "base_link");
  config.imu_frame = ReadParameter<std::string>(
      node, "lio.nav2_tf.imu_frame", "imu");

  const auto extrinsic = ReadParameter<std::vector<double>>(
      node, "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  if (extrinsic.size() != 6) {
    throw std::invalid_argument(
        "lio.extrinsic.odom_robo must contain exactly six values");
  }
  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(extrinsic[5] * kPi / 180.0,
                         Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(extrinsic[4] * kPi / 180.0,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(extrinsic[3] * kPi / 180.0,
                         Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  config.T_base_imu = {
      adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu,
      {Eigen::Quaterniond(rotation).normalized(),
       Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2])}};

  const auto path_max_poses = ReadParameter<int>(
      node, "lio.output.path_max_poses", 5000);
  config.path_max_poses = static_cast<std::size_t>(std::max(1, path_max_poses));
  config.global_map_pub_period = std::max(
      0.1, ReadParameter<double>(node, "lio.output.global_map_pub_period", 10.0));
  config.backend_path_pub_period = std::max(
      0.05,
      ReadParameter<double>(node, "lio.output.backend_path_pub_period", 0.25));

  config.odom_twist_filter_enabled = ReadParameter<bool>(
      node, "lio.output.odom_twist_filter.enable", false);
  config.odom_twist_filter_zero_lateral_velocity = ReadParameter<bool>(
      node, "lio.output.odom_twist_filter.zero_lateral_velocity", false);
  config.odom_twist_filter_linear_time_constant = std::max(
      0.0, ReadParameter<double>(
               node, "lio.output.odom_twist_filter.linear_time_constant", 0.08));
  config.odom_twist_filter_angular_time_constant = std::max(
      0.0, ReadParameter<double>(
               node, "lio.output.odom_twist_filter.angular_time_constant", 0.06));
  config.odom_twist_filter_max_linear_speed = ReadParameter<double>(
      node, "lio.output.odom_twist_filter.max_linear_speed", 2.0);
  config.odom_twist_filter_max_angular_speed = ReadParameter<double>(
      node, "lio.output.odom_twist_filter.max_angular_speed", 4.0);
  config.odom_twist_filter_max_linear_acceleration = ReadParameter<double>(
      node, "lio.output.odom_twist_filter.max_linear_acceleration", 4.0);
  config.odom_twist_filter_max_angular_acceleration = ReadParameter<double>(
      node, "lio.output.odom_twist_filter.max_angular_acceleration", 8.0);

  if (!std::isfinite(config.odom_twist_filter_max_linear_speed) ||
      config.odom_twist_filter_max_linear_speed <= 0.0) {
    config.odom_twist_filter_max_linear_speed = 2.0;
  }
  if (!std::isfinite(config.odom_twist_filter_max_angular_speed) ||
      config.odom_twist_filter_max_angular_speed <= 0.0) {
    config.odom_twist_filter_max_angular_speed = 4.0;
  }
  if (!std::isfinite(config.odom_twist_filter_max_linear_acceleration) ||
      config.odom_twist_filter_max_linear_acceleration <= 0.0) {
    config.odom_twist_filter_max_linear_acceleration = 4.0;
  }
  if (!std::isfinite(config.odom_twist_filter_max_angular_acceleration) ||
      config.odom_twist_filter_max_angular_acceleration <= 0.0) {
    config.odom_twist_filter_max_angular_acceleration = 8.0;
  }
  return config;
}

PublicationBridge::PublicationBridge(
    rclcpp::Node& node, rclcpp::CallbackGroup::SharedPtr callback_group,
    Config config)
    : node_(node),
      callback_group_(std::move(callback_group)),
      config_(std::move(config)) {}

PublicationBridge::~PublicationBridge() = default;

builtin_interfaces::msg::Time PublicationBridge::toRosTime(
    const double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::max(0.0, seconds - static_cast<double>(time.sec)) * 1e9);
  return time;
}

geometry_msgs::msg::Pose PublicationBridge::toPose(
    const BASIC::V3& position, const BASIC::M3& rotation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  BASIC::Quat quaternion(rotation);
  quaternion.normalize();
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

void PublicationBridge::setup() {
  path_.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  msg2uav_.header.frame_id = path_.header.frame_id;

  pub_odom_ = node_.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);
  pub_imu_odom_ = node_.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);
  pub_robo_odom_ = node_.create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);
  pub_path_ = node_.create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);
  pub_backend_optimized_path_ = node_.create_publisher<nav_msgs::msg::Path>(
      "/lio/backend/optimized_path", 10);
  pub_cloud_world_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_world", 10);
  if (config_.publish_robot_pose) {
    pub_msg2uav_ = node_.create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
  }

  occupancy_grid_publisher_ = std::make_unique<OccupancyGridPublisher>();
  occupancy_grid_publisher_->setup(node_, callback_group_);
  occupancy_grid_publisher_->setFrameMode(
      false, config_.nav2_tf_enable ? config_.odom_frame : "world");

  TfBridge::Config tf_config;
  tf_config.nav2_enabled = config_.nav2_tf_enable;
  tf_config.publish_map_to_odom = config_.nav2_publish_map_to_odom;
  tf_config.planar_constraint = config_.ground_robot_planar_constraint;
  tf_config.planar_base_z = config_.ground_robot_base_z;
  tf_config.map_frame = config_.map_frame;
  tf_config.odom_frame = config_.odom_frame;
  tf_config.base_frame = config_.base_frame;
  tf_config.imu_frame = config_.imu_frame;
  tf_bridge_ = std::make_unique<TfBridge>(
      node_, std::move(tf_config),
      [this]()
          -> std::optional<adagraph::geometry::RigidTransform3d> {
        BASIC::SE3 legacy;
        if (!getMapToOdom(legacy)) {
          return std::nullopt;
        }
        return adagraph::geometry::RigidTransform3d{
            Eigen::Quaterniond(legacy.R_.cast<double>()).normalized(),
            legacy.t_.cast<double>()};
      });

  backend_path_timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(config_.backend_path_pub_period),
      [this] { publishBackendPath(); }, callback_group_);
  if (config_.global_map_pub_period > 0.0) {
    // The timer is created lazily by setGlobalMap so that idle nodes do not
    // retain a periodic callback for a product that was never requested.
  }
}

void PublicationBridge::resetSession() {
  path_.poses.clear();
  path_.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  path_.header.stamp = builtin_interfaces::msg::Time{};
  msg2uav_.header.frame_id = path_.header.frame_id;
  global_map_msg_ = sensor_msgs::msg::PointCloud2{};
  global_map_dirty_ = false;
  global_map_sub_count_ = 0;
  last_path_point_ = BASIC::V3(0, 0, -100);
  pending_backend_snapshot_.store(nullptr, std::memory_order_release);
  published_backend_snapshot_version_ = 0;
  prior_map_localization_active_ = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    map_to_odom_ = BASIC::SE3{};
    has_map_to_odom_ = false;
  }
  odom_twist_filter_initialized_ = false;
  odom_twist_filter_last_stamp_ = -1.0;
  filtered_odom_linear_velocity_.setZero();
  filtered_odom_angular_velocity_.setZero();
  if (occupancy_grid_publisher_) {
    occupancy_grid_publisher_->setFrameMode(
        false, config_.nav2_tf_enable ? config_.odom_frame : "world");
    occupancy_grid_publisher_->resetSession();
  }
}

void PublicationBridge::setPriorMapLocalizationActive(const bool active) {
  prior_map_localization_active_ = active;
  if (occupancy_grid_publisher_) {
    occupancy_grid_publisher_->setFrameMode(
        active, config_.nav2_tf_enable ? config_.odom_frame : "world");
  }
}

void PublicationBridge::setMapToOdom(const BASIC::SE3& map_to_odom) {
  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  map_to_odom_ = map_to_odom;
  has_map_to_odom_ = true;
}

bool PublicationBridge::getMapToOdom(BASIC::SE3& map_to_odom) const {
  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  if (!has_map_to_odom_) {
    return false;
  }
  map_to_odom = map_to_odom_;
  return true;
}

void PublicationBridge::setBackendMapSnapshot(
    std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept {
  if (!snapshot || !snapshot->valid()) {
    return;
  }
  pending_backend_snapshot_.store(std::move(snapshot),
                                  std::memory_order_release);
  const auto latest = pending_backend_snapshot_.load(
      std::memory_order_acquire);
  if (occupancy_grid_publisher_) {
    occupancy_grid_publisher_->setMapSnapshot(latest);
  }
}

void PublicationBridge::filterOdomTwist(
    const rclcpp::Time& stamp, BASIC::V3& linear_velocity,
    BASIC::V3& angular_velocity) {
  if (!config_.odom_twist_filter_enabled) {
    return;
  }
  if (!linear_velocity.allFinite() || !angular_velocity.allFinite()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                         "Ignoring non-finite /lio/odom twist sample");
    if (odom_twist_filter_initialized_) {
      linear_velocity = filtered_odom_linear_velocity_;
      angular_velocity = filtered_odom_angular_velocity_;
    } else {
      linear_velocity.setZero();
      angular_velocity.setZero();
    }
    return;
  }
  if (config_.odom_twist_filter_zero_lateral_velocity) {
    linear_velocity.y() = 0.0;
  }
  const auto clamp_norm = [](BASIC::V3& value, const double limit) {
    const double norm = value.norm();
    if (std::isfinite(norm) && norm > limit) {
      value *= limit / norm;
    }
  };
  clamp_norm(linear_velocity, config_.odom_twist_filter_max_linear_speed);
  clamp_norm(angular_velocity, config_.odom_twist_filter_max_angular_speed);

  const double stamp_seconds = stamp.seconds();
  const double dt = stamp_seconds - odom_twist_filter_last_stamp_;
  if (!odom_twist_filter_initialized_ || !std::isfinite(dt) || dt <= 0.0 ||
      dt > 0.5) {
    filtered_odom_linear_velocity_ = linear_velocity;
    filtered_odom_angular_velocity_ = angular_velocity;
    odom_twist_filter_last_stamp_ = stamp_seconds;
    odom_twist_filter_initialized_ = true;
    return;
  }

  const auto limit_delta = [](double target, double current, double max_rate,
                              double dt_seconds) {
    return current + std::clamp(target - current, -max_rate * dt_seconds,
                                max_rate * dt_seconds);
  };
  for (int i = 0; i < 3; ++i) {
    linear_velocity[i] = limit_delta(
        linear_velocity[i], filtered_odom_linear_velocity_[i],
        config_.odom_twist_filter_max_linear_acceleration, dt);
    angular_velocity[i] = limit_delta(
        angular_velocity[i], filtered_odom_angular_velocity_[i],
        config_.odom_twist_filter_max_angular_acceleration, dt);
  }
  const auto low_pass = [](const BASIC::V3& target, const BASIC::V3& previous,
                           double time_constant, double dt_seconds)
      -> BASIC::V3 {
    if (time_constant <= 0.0) {
      return target;
    }
    const double alpha = std::clamp(
        dt_seconds / (time_constant + dt_seconds), 0.0, 1.0);
    return previous + alpha * (target - previous);
  };
  filtered_odom_linear_velocity_ = low_pass(
      linear_velocity, filtered_odom_linear_velocity_,
      config_.odom_twist_filter_linear_time_constant, dt);
  filtered_odom_angular_velocity_ = low_pass(
      angular_velocity, filtered_odom_angular_velocity_,
      config_.odom_twist_filter_angular_time_constant, dt);
  odom_twist_filter_last_stamp_ = stamp_seconds;
  linear_velocity = filtered_odom_linear_velocity_;
  angular_velocity = filtered_odom_angular_velocity_;
  if (config_.odom_twist_filter_zero_lateral_velocity) {
    linear_velocity.y() = 0.0;
  }
}

void PublicationBridge::publishOdom(const NavState& state,
                                     const BASIC::V3& angular_velocity) {
  if (!pub_odom_) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  odom.child_frame_id =
      config_.nav2_tf_enable ? config_.base_frame : config_.imu_frame;
  odom.header.stamp = toRosTime(state.timestamp);

  // State.R is T_odom_imu.  Convert it with T_imu_base, the inverse of the
  // configured static T_base_imu, before publishing base-frame products.
  const auto& T_imu_base = config_.T_base_imu.inverse().T_target_source;
  const BASIC::M3 base_R = T_imu_base.rotation.toRotationMatrix()
                               .cast<BASIC::scalar>();
  const BASIC::V3 base_t = T_imu_base.translation.cast<BASIC::scalar>();
  const BASIC::V3 robot_position = state.R.R_ * base_t + state.p;
  const BASIC::M3 robot_rotation = state.R.R_ * base_R;
  BASIC::V3 odom_robot_position = robot_position;
  BASIC::M3 odom_robot_rotation = robot_rotation;
  if (config_.nav2_tf_enable && config_.ground_robot_planar_constraint) {
    const BASIC::scalar yaw = std::atan2(
        odom_robot_rotation(1, 0), odom_robot_rotation(0, 0));
    odom_robot_rotation = Eigen::AngleAxis<BASIC::scalar>(
                              yaw, BASIC::V3::UnitZ())
                              .toRotationMatrix();
    odom_robot_position.z() =
        static_cast<BASIC::scalar>(config_.ground_robot_base_z);
  }

  if (config_.nav2_tf_enable) {
    odom.pose.pose = toPose(odom_robot_position, odom_robot_rotation);
  } else {
    odom.pose.pose = toPose(state.p, state.R.R_);
  }

  BASIC::V3 linear_velocity;
  BASIC::V3 body_angular_velocity;
  if (config_.nav2_tf_enable) {
    linear_velocity = odom_robot_rotation.transpose() * state.v;
    body_angular_velocity = base_R.transpose() * angular_velocity;
  } else {
    linear_velocity = state.R.R_.transpose() * state.v;
    body_angular_velocity = angular_velocity;
  }
  filterOdomTwist(odom.header.stamp, linear_velocity, body_angular_velocity);
  odom.twist.twist.linear.x = linear_velocity.x();
  odom.twist.twist.linear.y = linear_velocity.y();
  odom.twist.twist.linear.z = linear_velocity.z();
  odom.twist.twist.angular.x = body_angular_velocity.x();
  odom.twist.twist.angular.y = body_angular_velocity.y();
  odom.twist.twist.angular.z = body_angular_velocity.z();
  pub_odom_->publish(odom);

  if (config_.publish_robot_pose && pub_msg2uav_) {
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose = toPose(robot_position, robot_rotation);
    pub_msg2uav_->publish(msg2uav_);
  }

  if ((last_path_point_ - robot_position).norm() > 0.1) {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.header = odom.header;
    point.pose = odom.pose.pose;
    if (path_.poses.size() >= config_.path_max_poses) {
      path_.poses.erase(path_.poses.begin());
    }
    path_.poses.push_back(std::move(point));
    pub_path_->publish(path_);
    last_path_point_ = robot_position;
  }

  if (tf_bridge_) {
    if (config_.nav2_tf_enable) {
      tf_bridge_->publishOdomBase(odom.header.stamp, odom_robot_position,
                                  odom_robot_rotation);
    } else {
      tf_bridge_->publishWorldImu(odom.header.stamp, state.p, state.R.R_);
    }
  }
}

void PublicationBridge::publishLegacyImuPrediction(
    const builtin_interfaces::msg::Time& stamp,
    const DynamicState& imu_state,
    const DynamicState& robot_state) {
  if (!pub_imu_odom_ || !pub_robo_odom_) {
    return;
  }

  nav_msgs::msg::Odometry imu_odom;
  imu_odom.header.stamp = stamp;
  imu_odom.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  imu_odom.child_frame_id = config_.imu_frame;
  imu_odom.pose.pose.position.x = imu_state.p.x();
  imu_odom.pose.pose.position.y = imu_state.p.y();
  imu_odom.pose.pose.position.z = imu_state.p.z();
  const Eigen::Quaterniond imu_rotation(imu_state.R.cast<double>());
  const auto normalized_imu_rotation = imu_rotation.normalized();
  imu_odom.pose.pose.orientation.x = normalized_imu_rotation.x();
  imu_odom.pose.pose.orientation.y = normalized_imu_rotation.y();
  imu_odom.pose.pose.orientation.z = normalized_imu_rotation.z();
  imu_odom.pose.pose.orientation.w = normalized_imu_rotation.w();
  const BASIC::V3 imu_linear_velocity =
      imu_state.R.transpose() * imu_state.v;
  imu_odom.twist.twist.linear.x = imu_linear_velocity.x();
  imu_odom.twist.twist.linear.y = imu_linear_velocity.y();
  imu_odom.twist.twist.linear.z = imu_linear_velocity.z();
  imu_odom.twist.twist.angular.x = imu_state.w.x();
  imu_odom.twist.twist.angular.y = imu_state.w.y();
  imu_odom.twist.twist.angular.z = imu_state.w.z();
  pub_imu_odom_->publish(imu_odom);

  nav_msgs::msg::Odometry robot_odom;
  robot_odom.header.stamp = stamp;
  robot_odom.header.frame_id = imu_odom.header.frame_id;
  robot_odom.child_frame_id = config_.nav2_tf_enable
                                  ? config_.base_frame
                                  : config_.imu_frame;
  robot_odom.pose.pose.position.x = robot_state.p.x();
  robot_odom.pose.pose.position.y = robot_state.p.y();
  robot_odom.pose.pose.position.z = robot_state.p.z();
  const Eigen::Quaterniond robot_rotation(robot_state.R.cast<double>());
  const auto normalized_robot_rotation = robot_rotation.normalized();
  robot_odom.pose.pose.orientation.x = normalized_robot_rotation.x();
  robot_odom.pose.pose.orientation.y = normalized_robot_rotation.y();
  robot_odom.pose.pose.orientation.z = normalized_robot_rotation.z();
  robot_odom.pose.pose.orientation.w = normalized_robot_rotation.w();
  pub_robo_odom_->publish(robot_odom);

  if (config_.nav2_tf_enable && tf_bridge_) {
    tf_bridge_->publishOdomBase(stamp, robot_state.p, robot_state.R);
  }
}

void PublicationBridge::publishCloudWorld(const BASIC::CloudPtr& cloud,
                                           const double timestamp) {
  if (!pub_cloud_world_ || !cloud) {
    return;
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  message.header.stamp = toRosTime(timestamp);
  pub_cloud_world_->publish(message);
}

void PublicationBridge::updateOccupancyGridFromBodyCloud(
    const BASIC::CloudPtr& cloud, const NavState& state) {
  if (!occupancy_grid_publisher_ || !occupancy_grid_publisher_->enabled() ||
      !cloud) {
    return;
  }
  BASIC::SE3 body_to_grid(state.R.R_, state.p);
  if (prior_map_localization_active_) {
    std::optional<BASIC::SE3> map_to_odom;
    BASIC::SE3 legacy_map_to_odom;
    if (getMapToOdom(legacy_map_to_odom)) {
      map_to_odom = legacy_map_to_odom;
    }
    if (!map_to_odom) {
      return;
    }
    body_to_grid = *map_to_odom * body_to_grid;
  }
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = body_to_grid.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = body_to_grid.t_.cast<float>();
  const Eigen::Quaterniond rotation(body_to_grid.R_.cast<double>());
  occupancy_grid_publisher_->insertBodyCloud(
      cloud, transformation, rotation, rclcpp::Time(toRosTime(state.timestamp)));
}

void PublicationBridge::setGlobalMap(const BASIC::CloudPtr& global_map) {
  if (!global_map) {
    return;
  }
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id =
      config_.nav2_tf_enable ? config_.map_frame : "world";
  if (occupancy_grid_publisher_ && occupancy_grid_publisher_->enabled()) {
    occupancy_grid_publisher_->resetFromCloud(global_map, node_.now());
  }
  if (!pub_global_map_) {
    pub_global_map_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/global_map", rclcpp::QoS(1).transient_local());
  }
  global_map_msg_.header.stamp = node_.now();
  pub_global_map_->publish(global_map_msg_);
  global_map_dirty_ = false;
  global_map_sub_count_ = pub_global_map_->get_subscription_count();
  if (!global_map_timer_) {
    global_map_timer_ = node_.create_wall_timer(
        std::chrono::duration<double>(config_.global_map_pub_period),
        [this] {
          if (!pub_global_map_ || global_map_msg_.data.empty()) {
            return;
          }
          const std::size_t subscribers =
              pub_global_map_->get_subscription_count();
          const bool new_subscriber = subscribers > global_map_sub_count_;
          global_map_sub_count_ = subscribers;
          if (!global_map_dirty_ && !new_subscriber) {
            return;
          }
          global_map_msg_.header.stamp = node_.now();
          pub_global_map_->publish(global_map_msg_);
          global_map_dirty_ = false;
        },
        callback_group_);
  }
}

void PublicationBridge::publishBackendPath() {
  if (!pub_backend_optimized_path_) {
    return;
  }
  const auto snapshot = pending_backend_snapshot_.load(
      std::memory_order_acquire);
  if (!snapshot || snapshot->version <= published_backend_snapshot_version_) {
    return;
  }
  const auto trajectory =
      adagraph::mapping::SnapshotProductBuilder::BuildTrajectory(*snapshot);
  if (!trajectory || trajectory->poses.empty()) {
    return;
  }
  nav_msgs::msg::Path path;
  path.header.frame_id = config_.nav2_tf_enable ? config_.map_frame : "map";
  path.header.stamp = toRosTime(
      trajectory->poses.back().T_map_base.timestamp.seconds());
  path.poses.reserve(trajectory->poses.size());
  for (const auto& pose : trajectory->poses) {
    geometry_msgs::msg::PoseStamped message;
    message.header.frame_id = path.header.frame_id;
    message.header.stamp =
        toRosTime(pose.T_map_base.timestamp.seconds());
    const auto& transform = pose.T_map_base.T_target_source;
    message.pose.position.x = transform.translation.x();
    message.pose.position.y = transform.translation.y();
    message.pose.position.z = transform.translation.z();
    const Eigen::Quaterniond quaternion = transform.rotation.normalized();
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    message.pose.orientation.w = quaternion.w();
    path.poses.push_back(std::move(message));
  }
  pub_backend_optimized_path_->publish(path);
  published_backend_snapshot_version_ = trajectory->snapshot_version;
}

void PublicationBridge::publishCloud2Planner(const BASIC::CloudPtr& cloud,
                                              const double timestamp) {
  if (!pub_cloud2robot_) {
    pub_cloud2robot_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  }
  if (!cloud) {
    return;
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*cloud, message);
  message.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  message.header.stamp = toRosTime(timestamp);
  pub_cloud2robot_->publish(message);
}

void PublicationBridge::publishCloudBodyPose(const BASIC::CloudPtr& cloud,
                                              const NavState& state) {
  if (!pub_cloud_body_pose_) {
    pub_cloud_body_pose_ =
        node_.create_publisher<super_lio::msg::CloudPose>(
            "/lio/body/cloud_pose", 10);
  }
  if (!cloud) {
    return;
  }
  super_lio::msg::CloudPose message;
  pcl::toROSMsg(*cloud, message.cloud);
  message.cloud.header.stamp = toRosTime(state.timestamp);
  message.pose = toPose(state.p, state.R.R_);
  pub_cloud_body_pose_->publish(message);
}

void PublicationBridge::publishCloudWorldPose(const BASIC::CloudPtr& cloud,
                                               const NavState& state) {
  if (!pub_cloud_world_pose_) {
    pub_cloud_world_pose_ =
        node_.create_publisher<super_lio::msg::CloudPose>(
            "/lio/world/cloud_pose", 10);
  }
  if (!cloud) {
    return;
  }
  super_lio::msg::CloudPose message;
  pcl::toROSMsg(*cloud, message.cloud);
  message.cloud.header.stamp = toRosTime(state.timestamp);
  message.pose = toPose(state.p, state.R.R_);
  pub_cloud_world_pose_->publish(message);
}

void PublicationBridge::publishProcessingTime(const double timestamp,
                                               const double current_time,
                                               const double mean_time,
                                               const double std_time) {
  if (!pub_processing_time_) {
    pub_processing_time_ =
        node_.create_publisher<geometry_msgs::msg::PoseStamped>(
            "/lio/processing_time", 10);
  }
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = toRosTime(timestamp);
  message.pose.position.x = current_time;
  message.pose.position.y = mean_time;
  message.pose.position.z = std_time;
  pub_processing_time_->publish(message);
}

}  // namespace LI2Sup
