#include "ros/publication_bridge.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>

#include "hybrid_slam/adapter/pcl_types.h"
#include "hybrid_slam/mapping/snapshot_products.h"
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

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey&) const = default;
};

struct VoxelKeyHash {
  [[nodiscard]] std::size_t operator()(const VoxelKey& key) const noexcept {
    const auto mix = [](std::uint64_t value) {
      value += 0x9e3779b97f4a7c15ULL;
      value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
      value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
      return value ^ (value >> 31U);
    };
    return static_cast<std::size_t>(
        mix(static_cast<std::uint64_t>(key.x)) ^
        (mix(static_cast<std::uint64_t>(key.y)) << 1U) ^
        (mix(static_cast<std::uint64_t>(key.z)) << 2U));
  }
};

std::optional<VoxelKey> MakeVoxelKey(
    const hybrid_slam::mapping::MapPoint3f& point,
    const float voxel_size_m) {
  const Eigen::Vector3f xyz(point.x, point.y, point.z);
  if (!xyz.allFinite()) {
    return std::nullopt;
  }
  const Eigen::Array3d scaled = xyz.cast<double>().array() / voxel_size_m;
  constexpr double kInt64Min = static_cast<double>(
      std::numeric_limits<std::int64_t>::min());
  constexpr double kInt64Max = static_cast<double>(
      std::numeric_limits<std::int64_t>::max());
  if ((scaled < kInt64Min).any() || (scaled > kInt64Max).any()) {
    return std::nullopt;
  }
  return VoxelKey{static_cast<std::int64_t>(std::floor(scaled.x())),
                  static_cast<std::int64_t>(std::floor(scaled.y())),
                  static_cast<std::int64_t>(std::floor(scaled.z()))};
}

hybrid_slam::adapter::PclPointCloud::ConstPtr ToPclCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
  if (!cloud.valid() || cloud.empty()) {
    return {};
  }
  if (const auto native = cloud.ownerAs<hybrid_slam::adapter::PclPointCloud>()) {
    return native;
  }

  hybrid_slam::adapter::PclCloudPtr converted(new hybrid_slam::adapter::PclPointCloud());
  converted->reserve(cloud.size());
  std::array<float, 3> point{};
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    if (cloud.view().xyz(index, point)) {
      converted->emplace_back(point[0], point[1], point[2], 0.0F);
    }
  }
  converted->width = static_cast<std::uint32_t>(converted->size());
  converted->height = 1;
  converted->is_dense = false;
  return converted;
}

std::shared_ptr<sensor_msgs::msg::PointCloud2> BuildGlobalMapMessage(
    const hybrid_slam::mapping::SnapshotPointCloudMap& map,
    const float voxel_size_m, const std::string& frame_id,
    const rclcpp::Time& stamp) {
  hybrid_slam::adapter::PclPointCloud filtered;
  filtered.reserve(map.points.size());
  if (voxel_size_m > 0.0F) {
    // PCL VoxelGrid allocates an int-indexed dense voxel layout, which
    // overflows for sparse maps spanning long trajectories. Publishing needs
    // stable thinning, not dense-grid layout, so retain the first point per
    // metric voxel with 64-bit keys instead.
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
    occupied.reserve(map.points.size());
    for (const auto& point : map.points) {
      const auto key = MakeVoxelKey(point, voxel_size_m);
      if (key && occupied.insert(*key).second) {
        filtered.emplace_back(point.x, point.y, point.z, 0.0F);
      }
    }
  } else {
    for (const auto& point : map.points) {
      if (std::isfinite(point.x) && std::isfinite(point.y) &&
          std::isfinite(point.z)) {
        filtered.emplace_back(point.x, point.y, point.z, 0.0F);
      }
    }
  }
  filtered.width = static_cast<std::uint32_t>(filtered.size());
  filtered.height = 1;
  filtered.is_dense = false;

  auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(filtered, *message);
  message->header.frame_id = frame_id;
  message->header.stamp = stamp;
  return message;
}

std::size_t FinishedSubmapCount(
    const hybrid_slam::mapping::MapSnapshot& snapshot) {
  return static_cast<std::size_t>(std::count_if(
      snapshot.submaps.begin(), snapshot.submaps.end(),
      [](const auto& submap) {
        return submap.state == hybrid_slam::mapping::SubmapState::kFinished &&
               submap.geometry_ready && submap.valid();
      }));
}

std::size_t GlobalConstraintCount(
    const hybrid_slam::mapping::MapSnapshot& snapshot) {
  return static_cast<std::size_t>(std::count_if(
      snapshot.constraints.begin(), snapshot.constraints.end(),
      [](const auto& constraint) {
        return constraint.type == hybrid_slam::backend::ConstraintType::kLoopClosure ||
               constraint.type == hybrid_slam::backend::ConstraintType::kPrior ||
               constraint.type == hybrid_slam::backend::ConstraintType::kRelocalization;
      }));
}

std::optional<hybrid_slam::mapping::SnapshotPointCloudMap>
BuildFinishedSubmapPointCloudMap(
    const hybrid_slam::mapping::MapSnapshot& snapshot) {
  hybrid_slam::mapping::SnapshotPointCloudMap map;
  map.snapshot_version = snapshot.version;

  std::size_t point_count = 0;
  for (const auto& submap : snapshot.submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid()) {
      continue;
    }
    if (submap.cloud_submap.size() >
        std::numeric_limits<std::size_t>::max() - point_count) {
      return std::nullopt;
    }
    point_count += submap.cloud_submap.size();
  }
  if (point_count == 0) {
    return std::nullopt;
  }

  map.points.reserve(point_count);
  map.minimum = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::infinity());
  map.maximum = Eigen::Vector3f::Constant(
      -std::numeric_limits<float>::infinity());
  for (const auto& submap : snapshot.submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid()) {
      continue;
    }
    const auto& view = submap.cloud_submap.view();
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> source{};
      if (!view.xyz(index, source)) {
        return std::nullopt;
      }
      const Eigen::Vector3d transformed = submap.T_map_submap.transformPoint(
          Eigen::Vector3d(source[0], source[1], source[2]));
      if (!transformed.allFinite()) {
        return std::nullopt;
      }
      const Eigen::Vector3f point = transformed.cast<float>();
      map.points.push_back({point.x(), point.y(), point.z()});
      map.minimum = map.minimum.cwiseMin(point);
      map.maximum = map.maximum.cwiseMax(point);
    }
  }
  return map.points.empty()
             ? std::nullopt
             : std::optional<hybrid_slam::mapping::SnapshotPointCloudMap>(
                   std::move(map));
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
      hybrid_slam::geometry::FrameId::kBase,
      hybrid_slam::geometry::FrameId::kImu,
      {Eigen::Quaterniond(rotation).normalized(),
       Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2])}};

  const auto path_max_poses = ReadParameter<int>(
      node, "lio.output.path_max_poses", 5000);
  config.path_max_poses = static_cast<std::size_t>(std::max(1, path_max_poses));
  config.odometry_publish_rate_hz = ReadParameter<double>(
      node, "lio.output.odometry_rate_hz", 0.0);
  if (!std::isfinite(config.odometry_publish_rate_hz) ||
      config.odometry_publish_rate_hz < 0.0) {
    config.odometry_publish_rate_hz = 0.0;
  }
  config.publish_global_map = ReadParameter<bool>(
      node, "lio.output.global_map.enable", true);
  config.global_map_voxel_size_m = ReadParameter<double>(
      node, "lio.output.global_map_voxel_size_m", 0.20);
  if (!std::isfinite(config.global_map_voxel_size_m) ||
      config.global_map_voxel_size_m < 0.0) {
    config.global_map_voxel_size_m = 0.20;
  }
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
      config_(std::move(config)),
      odometry_rate_limiter_(config_.odometry_publish_rate_hz) {}

PublicationBridge::~PublicationBridge() {
  shutdown();
}

void PublicationBridge::shutdown() {
  if (global_map_worker_.joinable()) {
    global_map_worker_.request_stop();
    global_map_work_cv_.notify_all();
    global_map_worker_.join();
  }
}

builtin_interfaces::msg::Time PublicationBridge::toRosTime(
    const double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::max(0.0, seconds - static_cast<double>(time.sec)) * 1e9);
  return time;
}

geometry_msgs::msg::Pose PublicationBridge::toPose(
    const Eigen::Vector3d& position, const Eigen::Matrix3d& rotation) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = position.x();
  pose.position.y = position.y();
  pose.position.z = position.z();
  const Eigen::Quaterniond quaternion(rotation);
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
  pub_path_ = node_.create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);
  pub_backend_optimized_path_ = node_.create_publisher<nav_msgs::msg::Path>(
      "/lio/backend/optimized_path", 10);
  pub_cloud_world_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      "/lio/cloud_world", 10);
  if (config_.publish_global_map) {
    pub_global_map_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/global_map", rclcpp::SensorDataQoS().keep_last(1));
    global_map_worker_ = std::jthread(
        [this](std::stop_token stop_token) { globalMapWorker(stop_token); });
  }
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
          -> std::optional<hybrid_slam::geometry::RigidTransform3d> {
        hybrid_slam::geometry::RigidTransform3d map_to_odom;
        if (!getMapToOdom(map_to_odom)) {
          return std::nullopt;
        }
        return hybrid_slam::geometry::RigidTransform3d{
            map_to_odom.rotation.normalized(), map_to_odom.translation};
      });

  backend_path_timer_ = node_.create_wall_timer(
      std::chrono::duration<double>(config_.backend_path_pub_period),
      [this] { publishBackendPath(); }, callback_group_);
  if (config_.publish_global_map && config_.global_map_pub_period > 0.0) {
    global_map_timer_ = node_.create_wall_timer(
        std::chrono::duration<double>(config_.global_map_pub_period),
        [this] {
          std::lock_guard<std::mutex> lock(global_map_mutex_);
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

void PublicationBridge::resetSession() {
  path_.poses.clear();
  path_.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  path_.header.stamp = builtin_interfaces::msg::Time{};
  msg2uav_.header.frame_id = path_.header.frame_id;
  {
    std::lock_guard<std::mutex> lock(global_map_mutex_);
    ++global_map_generation_;
    global_map_msg_ = sensor_msgs::msg::PointCloud2{};
    global_map_dirty_ = false;
    global_map_sub_count_ = 0;
  }
  last_path_point_ = Eigen::Vector3d(0, 0, -100);
  pending_backend_snapshot_.store(nullptr, std::memory_order_release);
  published_backend_snapshot_version_ = 0;
  {
    std::lock_guard<std::mutex> lock(global_map_work_mutex_);
    global_map_build_requested_ = false;
    requested_finished_submaps_ = 0;
    requested_global_constraints_ = 0;
    requested_snapshot_without_submaps_ = false;
  }
  prior_map_localization_active_ = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
    map_to_odom_ = hybrid_slam::geometry::RigidTransform3d::Identity();
    has_map_to_odom_ = false;
  }
  odom_twist_filter_initialized_ = false;
  odom_twist_filter_last_stamp_ = -1.0;
  filtered_odom_linear_velocity_.setZero();
  filtered_odom_angular_velocity_.setZero();
  odometry_rate_limiter_.reset();
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

void PublicationBridge::setMapToOdom(
    const hybrid_slam::geometry::RigidTransform3d& map_to_odom) {
  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  map_to_odom_ = map_to_odom;
  has_map_to_odom_ = true;
}

bool PublicationBridge::getMapToOdom(
    hybrid_slam::geometry::RigidTransform3d& map_to_odom) const {
  std::lock_guard<std::mutex> lock(map_to_odom_mutex_);
  if (!has_map_to_odom_) {
    return false;
  }
  map_to_odom = map_to_odom_;
  return true;
}

TfBridge::PublicationStats PublicationBridge::transformPublicationStats() const noexcept {
  return tf_bridge_ ? tf_bridge_->publicationStats()
                    : TfBridge::PublicationStats{};
}

void PublicationBridge::setBackendMapSnapshot(
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) noexcept {
  // BackendScheduler validates the immutable snapshot before invoking this
  // observer. Keep this boundary as an O(1) handoff; structural validation is
  // intentionally not repeated on the backend single-writer thread.
  if (!snapshot) {
    return;
  }
  pending_backend_snapshot_.store(std::move(snapshot),
                                  std::memory_order_release);
  const auto latest = pending_backend_snapshot_.load(
      std::memory_order_acquire);
  if (occupancy_grid_publisher_) {
    occupancy_grid_publisher_->setMapSnapshot(latest);
  }
  if (config_.publish_global_map) {
    bool request_build = false;
    {
      std::lock_guard<std::mutex> lock(global_map_work_mutex_);
      const std::size_t finished_submaps = FinishedSubmapCount(*latest);
      const std::size_t global_constraints = GlobalConstraintCount(*latest);
      // Rendering active keyframes requires repeatedly materializing the full
      // history. Finished submaps already contain compact immutable geometry,
      // so publish only when that geometry grows or a global factor moves it.
      if (finished_submaps != requested_finished_submaps_ ||
          global_constraints != requested_global_constraints_ ||
          (latest->submaps.empty() &&
           !requested_snapshot_without_submaps_)) {
        requested_finished_submaps_ = finished_submaps;
        requested_global_constraints_ = global_constraints;
        requested_snapshot_without_submaps_ = latest->submaps.empty();
        global_map_build_requested_ = true;
        request_build = true;
      }
    }
    if (request_build) {
      global_map_work_cv_.notify_one();
    }
  }
}

void PublicationBridge::filterOdomTwist(
    const rclcpp::Time& stamp, Eigen::Vector3d& linear_velocity,
    Eigen::Vector3d& angular_velocity) {
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
  const auto clamp_norm = [](Eigen::Vector3d& value, const double limit) {
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
  const auto low_pass = [](const Eigen::Vector3d& target,
                           const Eigen::Vector3d& previous,
                           double time_constant, double dt_seconds)
      -> Eigen::Vector3d {
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
                                     const Eigen::Vector3d& angular_velocity) {
  if (!pub_odom_ || !odometry_rate_limiter_.shouldPublish(state.timestamp)) {
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
  const Eigen::Matrix3d base_R = T_imu_base.rotation.toRotationMatrix();
  const Eigen::Vector3d base_t = T_imu_base.translation;
  const Eigen::Vector3d robot_position =
      state.T_odom_imu.rotation * base_t + state.T_odom_imu.translation;
  const Eigen::Matrix3d robot_rotation =
      state.T_odom_imu.rotation.toRotationMatrix() * base_R;
  Eigen::Vector3d odom_robot_position = robot_position;
  Eigen::Matrix3d odom_robot_rotation = robot_rotation;
  if (config_.nav2_tf_enable && config_.ground_robot_planar_constraint) {
    const double yaw = std::atan2(
        odom_robot_rotation(1, 0), odom_robot_rotation(0, 0));
    odom_robot_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
                              .toRotationMatrix();
    odom_robot_position.z() = config_.ground_robot_base_z;
  }

  if (config_.nav2_tf_enable) {
    odom.pose.pose = toPose(odom_robot_position, odom_robot_rotation);
  } else {
    odom.pose.pose = toPose(state.T_odom_imu.translation,
                            state.T_odom_imu.rotation.toRotationMatrix());
  }

  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d body_angular_velocity;
  if (config_.nav2_tf_enable) {
    linear_velocity = odom_robot_rotation.transpose() * state.velocity_odom;
    body_angular_velocity = base_R.transpose() * angular_velocity;
  } else {
    linear_velocity = state.T_odom_imu.rotation.conjugate() *
                      state.velocity_odom;
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
      tf_bridge_->publishOdomBase(
          odom.header.stamp,
          {Eigen::Quaterniond(odom_robot_rotation).normalized(),
           odom_robot_position});
    } else {
      tf_bridge_->publishWorldImu(odom.header.stamp, state.T_odom_imu);
    }
  }
}

void PublicationBridge::publishCloudWorld(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const double timestamp) {
  const auto pcl_cloud = ToPclCloud(cloud);
  if (!pub_cloud_world_ || !pcl_cloud) {
    return;
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*pcl_cloud, message);
  message.header.frame_id =
      config_.nav2_tf_enable ? config_.odom_frame : "world";
  message.header.stamp = toRosTime(timestamp);
  pub_cloud_world_->publish(message);
}

void PublicationBridge::updateOccupancyGridFromBodyCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const NavState& state) {
  if (!occupancy_grid_publisher_ || !occupancy_grid_publisher_->enabled() ||
      !cloud.valid() || cloud.empty()) {
    return;
  }
  auto body_to_grid = state.T_odom_imu;
  if (prior_map_localization_active_) {
    std::optional<hybrid_slam::geometry::RigidTransform3d> map_to_odom;
    hybrid_slam::geometry::RigidTransform3d current_map_to_odom;
    if (getMapToOdom(current_map_to_odom)) {
      map_to_odom = current_map_to_odom;
    }
    if (!map_to_odom) {
      return;
    }
    body_to_grid = map_to_odom->compose(body_to_grid);
  }
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) =
      body_to_grid.rotation.toRotationMatrix().cast<float>();
  transformation.block<3, 1>(0, 3) = body_to_grid.translation.cast<float>();
  const Eigen::Quaterniond rotation = body_to_grid.rotation.normalized();
  occupancy_grid_publisher_->insertBodyCloud(
      cloud, transformation, rotation, rclcpp::Time(toRosTime(state.timestamp)));
}

void PublicationBridge::setGlobalMap(
    const hybrid_slam::sensor::ImmutablePointCloud& global_map) {
  const auto pcl_map = ToPclCloud(global_map);
  if (!pcl_map) {
    return;
  }
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(*pcl_map, message);
  message.header.frame_id =
      config_.nav2_tf_enable ? config_.map_frame : "world";
  if (occupancy_grid_publisher_ && occupancy_grid_publisher_->enabled()) {
    occupancy_grid_publisher_->resetFromCloud(global_map, node_.now());
  }
  publishGlobalMapMessage(std::move(message));
}

void PublicationBridge::publishGlobalMapMessage(
    sensor_msgs::msg::PointCloud2 message) {
  if (!config_.publish_global_map) {
    return;
  }
  std::lock_guard<std::mutex> lock(global_map_mutex_);
  if (!pub_global_map_) {
    pub_global_map_ = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/global_map", rclcpp::SensorDataQoS().keep_last(1));
  }
  global_map_msg_ = std::move(message);
  global_map_msg_.header.stamp = node_.now();
  pub_global_map_->publish(global_map_msg_);
  global_map_dirty_ = false;
  global_map_sub_count_ = pub_global_map_->get_subscription_count();
}

void PublicationBridge::globalMapWorker(std::stop_token stop_token) {
  std::uint64_t materialized_snapshot_version = 0;
  std::uint64_t materialized_generation = 0;
  while (!stop_token.stop_requested()) {
    {
      std::unique_lock<std::mutex> lock(global_map_work_mutex_);
      global_map_work_cv_.wait(lock, [this, &stop_token] {
        return stop_token.stop_requested() || global_map_build_requested_;
      });
      if (stop_token.stop_requested()) {
        return;
      }
      global_map_build_requested_ = false;
    }

    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(global_map_mutex_);
      generation = global_map_generation_;
    }
    if (generation != materialized_generation) {
      materialized_snapshot_version = 0;
      materialized_generation = generation;
    }
    const auto snapshot = pending_backend_snapshot_.load(
        std::memory_order_acquire);
    if (!snapshot || snapshot->version <= materialized_snapshot_version) {
      continue;
    }
    auto product = BuildFinishedSubmapPointCloudMap(*snapshot);
    // Compatibility for small synthetic maps and snapshots produced before
    // submap geometry existed. Runtime mapping snapshots have active/finished
    // submaps and take the bounded finished-submap path above.
    if (!product && snapshot->submaps.empty()) {
      product = hybrid_slam::mapping::SnapshotProductBuilder::BuildPointCloudMap(
          *snapshot);
    }
    if (!product || product->points.empty() || stop_token.stop_requested()) {
      continue;
    }
    if (stop_token.stop_requested()) {
      return;
    }
    auto message = BuildGlobalMapMessage(
        *product, static_cast<float>(config_.global_map_voxel_size_m),
        config_.nav2_tf_enable ? config_.map_frame : "world", node_.now());

    std::lock_guard<std::mutex> lock(global_map_mutex_);
    if (stop_token.stop_requested()) {
      return;
    }
    // Backend snapshots arrive faster than a full point-cloud materialization.
    // Requiring pointer identity here starves publication forever: every built
    // result becomes stale before it can be sent. A generation change denotes
    // a session reset and must still discard the product; within a generation,
    // publishing the most recently completed product gives RViz a coherent map
    // while the next worker pass catches up to the latest snapshot.
    if (global_map_generation_ != generation || !pub_global_map_) {
      continue;
    }
    global_map_msg_ = std::move(*message);
    pub_global_map_->publish(global_map_msg_);
    global_map_dirty_ = false;
    global_map_sub_count_ = pub_global_map_->get_subscription_count();
    RCLCPP_INFO(
        node_.get_logger(),
        "Published /lio/global_map snapshot=%lu finished_submaps=%zu "
        "source_points=%zu published_points=%u",
        static_cast<unsigned long>(snapshot->version),
        FinishedSubmapCount(*snapshot), product->points.size(),
        global_map_msg_.width);
    materialized_snapshot_version = snapshot->version;
    materialized_generation = generation;
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
      hybrid_slam::mapping::SnapshotProductBuilder::BuildTrajectory(*snapshot);
  if (!trajectory || trajectory->poses.empty()) {
    return;
  }
  nav_msgs::msg::Path path;
  // Without Nav2 TF, optimized poses and the global map are both published
  // directly in world. Publishing this path in an unconnected map frame
  // makes RViz silently drop it when its fixed frame is world.
  path.header.frame_id = config_.nav2_tf_enable ? config_.map_frame : "world";
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

}  // namespace LI2Sup
