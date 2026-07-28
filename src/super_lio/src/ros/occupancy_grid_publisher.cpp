#include "ros/occupancy_grid_publisher.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace LI2Sup {

namespace {

// Guards the double->int32 cell cast: a divergent LIO state or a corrupt
// point must not overflow the cell index.
bool toCellIndex(double meters, double resolution, std::int32_t& cell) {
  const double scaled = std::floor(meters / resolution);
  if (!(scaled >= -2.0e9 && scaled <= 2.0e9)) {
    return false;
  }
  cell = static_cast<std::int32_t>(scaled);
  return true;
}

}  // namespace

void OccupancyGridPublisher::setup(
    rclcpp::Node& node,
    const rclcpp::CallbackGroup::SharedPtr& callback_group) {
  clock_ = node.get_clock();
  logger_ = node.get_logger();

  node.declare_parameter<bool>("lio.occupancy_grid.enable", false);
  node.get_parameter("lio.occupancy_grid.enable", enabled_);

  node.declare_parameter<std::string>("lio.occupancy_grid.topic", "/map");
  node.get_parameter("lio.occupancy_grid.topic", topic_);

  node.declare_parameter<std::string>("lio.occupancy_grid.frame_id", "world");
  node.get_parameter("lio.occupancy_grid.frame_id", frame_id_);

  node.declare_parameter<double>("lio.occupancy_grid.resolution", 0.1);
  node.get_parameter("lio.occupancy_grid.resolution", resolution_);
  resolution_ = std::max(0.01, resolution_);

  node.declare_parameter<double>("lio.occupancy_grid.z_min", -0.3);
  node.get_parameter("lio.occupancy_grid.z_min", z_min_);

  node.declare_parameter<double>("lio.occupancy_grid.z_max", 2.0);
  node.get_parameter("lio.occupancy_grid.z_max", z_max_);
  if (z_min_ > z_max_) {
    std::swap(z_min_, z_max_);
  }

  node.declare_parameter<double>("lio.occupancy_grid.publish_period", 1.0);
  node.get_parameter("lio.occupancy_grid.publish_period", publish_period_);
  publish_period_ = std::max(0.1, publish_period_);

  node.declare_parameter<double>("lio.occupancy_grid.map_padding", 1.0);
  node.get_parameter("lio.occupancy_grid.map_padding", map_padding_);
  map_padding_ = std::max(0.0, map_padding_);

  node.declare_parameter<double>("lio.occupancy_grid.inflation_radius", 0.0);
  node.get_parameter("lio.occupancy_grid.inflation_radius", inflation_radius_);
  inflation_radius_ = std::max(0.0, inflation_radius_);

  node.declare_parameter<double>("lio.occupancy_grid.max_range", 30.0);
  node.get_parameter("lio.occupancy_grid.max_range", max_range_);
  max_range_ = std::max(resolution_, max_range_);

  node.declare_parameter<double>("lio.occupancy_grid.max_extent", 400.0);
  node.get_parameter("lio.occupancy_grid.max_extent", max_extent_);
  max_extent_ = std::max(10.0 * resolution_, max_extent_);
  max_extent_cells_ = static_cast<std::int64_t>(std::llround(max_extent_ / resolution_));

  node.declare_parameter<bool>("lio.occupancy_grid.raycast.enable", true);
  node.get_parameter("lio.occupancy_grid.raycast.enable", raycast_enable_);

  node.declare_parameter<double>("lio.occupancy_grid.raycast.max_ray_length", 15.0);
  node.get_parameter("lio.occupancy_grid.raycast.max_ray_length",
                     raycast_max_ray_length_);
  raycast_max_ray_length_ = std::max(resolution_, raycast_max_ray_length_);

  node.declare_parameter<bool>("lio.occupancy_grid.use_unknown", true);
  node.get_parameter("lio.occupancy_grid.use_unknown", use_unknown_);

  node.declare_parameter<bool>("lio.occupancy_grid.reset_on_global_map", true);
  node.get_parameter("lio.occupancy_grid.reset_on_global_map", reset_on_global_map_);

  node.declare_parameter<int>("lio.occupancy_grid.occupied_value", 100);
  node.get_parameter("lio.occupancy_grid.occupied_value", occupied_value_);
  occupied_value_ = std::clamp(occupied_value_, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.free_value", 0);
  node.get_parameter("lio.occupancy_grid.free_value", free_value_);
  free_value_ = std::clamp(free_value_, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.unknown_value", -1);
  node.get_parameter("lio.occupancy_grid.unknown_value", unknown_value_);
  unknown_value_ = std::clamp(unknown_value_, -1, 100);

  ground_filter_.setup(node, "lio.occupancy_grid.ground_filter");

  node.declare_parameter<bool>(
      "lio.occupancy_grid.ground_filter.publish_debug_clouds", false);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.publish_debug_clouds",
      publish_ground_debug_);

  node.declare_parameter<std::string>(
      "lio.occupancy_grid.ground_filter.ground_topic", ground_debug_topic_);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.ground_topic", ground_debug_topic_);

  node.declare_parameter<std::string>(
      "lio.occupancy_grid.ground_filter.non_ground_topic", non_ground_debug_topic_);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.non_ground_topic",
      non_ground_debug_topic_);

  if (!enabled_) {
    RCLCPP_INFO(node.get_logger(), "Occupancy grid publisher disabled.");
    return;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  publisher_ = node.create_publisher<nav_msgs::msg::OccupancyGrid>(topic_, qos);
  if (publish_ground_debug_) {
    auto debug_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    ground_debug_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            ground_debug_topic_, debug_qos);
    non_ground_debug_pub_ =
        node.create_publisher<sensor_msgs::msg::PointCloud2>(
            non_ground_debug_topic_, debug_qos);
  }

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(publish_period_));
  timer_ = node.create_wall_timer(
      period,
      [this]() { publish(); },
      callback_group);

  RCLCPP_INFO(node.get_logger(),
              "Occupancy grid enabled: topic=%s frame=%s resolution=%.3f z=[%.2f, %.2f] "
              "max_range=%.1f raycast=%s max_extent=%.1f",
              topic_.c_str(), frame_id_.c_str(), resolution_, z_min_, z_max_,
              max_range_, raycast_enable_ ? "on" : "off", max_extent_);
}

void OccupancyGridPublisher::setFrameMode(
    bool prior_map_localization,
    const std::string& odom_frame_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!prior_map_localization && !odom_frame_id.empty()) {
    // SLAM mode: cells live in the LIO odometry frame and no map->odom
    // correction exists, so the configured (map) frame id would displace the
    // grid whenever TF diverges from identity.
    frame_id_ = odom_frame_id;
  }
  RCLCPP_INFO(logger_, "Occupancy grid frame mode: %s, frame_id=%s",
              prior_map_localization ? "relocation" : "slam",
              frame_id_.c_str());
}

void OccupancyGridPublisher::insertCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !cloud || cloud->empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  // No sensor origin on this path: no range gating, no free-space carving,
  // and the z band stays an absolute world-frame band.
  insertCloudLocked(cloud, true);
  latest_stamp_ = stamp;
  dirty_ = true;
}

void OccupancyGridPublisher::insertBodyCloud(
    const BASIC::CloudPtr& body_cloud,
    const Eigen::Matrix4f& body_to_world,
    const Eigen::Quaterniond& body_to_world_rotation,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !body_cloud || body_cloud->empty()) {
    return;
  }

  const auto segmented =
      ground_filter_.segment(body_cloud, body_to_world_rotation);
  publishDebugClouds(segmented, body_to_world, stamp);

  if (!segmented.non_ground || segmented.non_ground->empty()) {
    return;
  }

  BASIC::CloudPtr non_ground_world(new BASIC::PointCloudType());
  pcl::transformPointCloud(*segmented.non_ground, *non_ground_world, body_to_world);

  const Eigen::Vector3f sensor_origin = body_to_world.block<3, 1>(0, 3);
  std::lock_guard<std::mutex> lock(mutex_);
  insertScanLocked(non_ground_world, sensor_origin);
  latest_stamp_ = stamp;
  dirty_ = true;
}

void OccupancyGridPublisher::resetFromCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !cloud) {
    return;
  }

  // Prior-map path: apply the same ground rejection as the online path. The
  // scan segmenter needs a sensor-centric cloud, so the map-frame variant is
  // used here; its ground-relative band replaces the absolute z band.
  BASIC::CloudPtr insert_cloud = cloud;
  BASIC::CloudPtr ground_cloud;
  bool ground_relative_filtered = false;
  if (ground_filter_.enabled()) {
    // z_max_ is relative to the sensor origin; the segmenter wants a ceiling
    // above local ground, so add the sensor's height above ground.
    const auto segmented = ground_filter_.segmentWorldMap(
        cloud, z_max_ + ground_filter_.sensorMountHeight());
    ground_cloud = segmented.ground;
    insert_cloud = segmented.non_ground;
    ground_relative_filtered = true;
  }

  nav_msgs::msg::OccupancyGrid msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reset_on_global_map_) {
      observed_cells_.clear();
      has_bounds_ = false;
    }
    if (ground_relative_filtered &&
        ground_cloud && !ground_cloud->empty()) {
      // Static prior maps have no per-scan sensor origins for ray casting.
      // Ground returns still provide direct free-space observations. Insert
      // them first so non-ground hits below override free evidence when both
      // project into the same 2D cell.
      insertFreeCloudLocked(ground_cloud);
    }
    if (insert_cloud && !insert_cloud->empty()) {
      insertCloudLocked(insert_cloud, !ground_relative_filtered);
    }
    latest_stamp_ = stamp;
    // Publish directly so the transient_local sample never keeps advertising
    // the pre-reset grid, including when the reset empties the map.
    msg = (!observed_cells_.empty() && has_bounds_)
              ? buildMessageLocked(stamp)
              : buildEmptyMessageLocked(stamp);
    dirty_ = false;
  }

  if (publisher_) {
    publisher_->publish(msg);
  }
}

void OccupancyGridPublisher::publish() {
  if (!enabled_ || !publisher_) {
    return;
  }

  nav_msgs::msg::OccupancyGrid msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dirty_ || observed_cells_.empty() || !has_bounds_) {
      return;
    }
    msg = buildMessageLocked(latest_stamp_);
    dirty_ = false;
  }

  publisher_->publish(msg);
}

void OccupancyGridPublisher::publishDebugClouds(
    const GroundSegmentationFilter::Result& result,
    const Eigen::Matrix4f& body_to_world,
    const rclcpp::Time& stamp) {
  if (!publish_ground_debug_ || !ground_debug_pub_ || !non_ground_debug_pub_) {
    return;
  }

  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id = frame_id_;
  }

  auto publish_cloud =
      [&](const BASIC::CloudPtr& body_cloud,
          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
        if (!body_cloud) {
          return;
        }

        BASIC::CloudPtr world_cloud(new BASIC::PointCloudType());
        pcl::transformPointCloud(*body_cloud, *world_cloud, body_to_world);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*world_cloud, msg);
        msg.header.frame_id = frame_id;
        msg.header.stamp = stamp;
        pub->publish(msg);
      };

  publish_cloud(result.ground, ground_debug_pub_);
  publish_cloud(result.non_ground, non_ground_debug_pub_);
}

void OccupancyGridPublisher::insertCloudLocked(
    const BASIC::CloudPtr& cloud,
    bool apply_height_filter) {
  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& point : cloud->points) {
    if (apply_height_filter) {
      if (!pointPassesHeightFilter(point)) {
        continue;
      }
    } else if (!std::isfinite(point.x) ||
               !std::isfinite(point.y) ||
               !std::isfinite(point.z)) {
      continue;
    }

    std::int32_t cell_x = 0;
    std::int32_t cell_y = 0;
    if (!toCellIndex(point.x, resolution_, cell_x) ||
        !toCellIndex(point.y, resolution_, cell_y)) {
      continue;
    }

    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
          continue;
        }

        applyScoreLocked(cellKey(cell_x + dx, cell_y + dy), kHitScore);
      }
    }
  }
}

void OccupancyGridPublisher::insertFreeCloudLocked(
    const BASIC::CloudPtr& cloud) {
  std::unordered_set<std::int64_t> free_cells;
  free_cells.reserve(cloud->size());

  for (const auto& point : cloud->points) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    std::int32_t cell_x = 0;
    std::int32_t cell_y = 0;
    if (!toCellIndex(point.x, resolution_, cell_x) ||
        !toCellIndex(point.y, resolution_, cell_y)) {
      continue;
    }
    free_cells.insert(cellKey(cell_x, cell_y));
  }

  for (const auto key : free_cells) {
    applyScoreLocked(key, -kMissScore);
  }
}

void OccupancyGridPublisher::insertScanLocked(
    const BASIC::CloudPtr& world_cloud,
    const Eigen::Vector3f& sensor_origin) {
  const int inflation_cells =
      static_cast<int>(std::ceil(inflation_radius_ / resolution_));
  const double origin_x = sensor_origin.x();
  const double origin_y = sensor_origin.y();
  const double origin_z = sensor_origin.z();

  std::int32_t origin_cell_x = 0;
  std::int32_t origin_cell_y = 0;
  const bool origin_valid =
      std::isfinite(origin_x) && std::isfinite(origin_y) && std::isfinite(origin_z) &&
      toCellIndex(origin_x, resolution_, origin_cell_x) &&
      toCellIndex(origin_y, resolution_, origin_cell_y);

  const double max_range2 = max_range_ * max_range_;
  const double max_ray2 = raycast_max_ray_length_ * raycast_max_ray_length_;

  std::unordered_set<std::int64_t> scan_hits;
  std::unordered_set<std::int64_t> ray_targets;
  std::unordered_set<std::int64_t> scan_free;

  for (const auto& point : world_cloud->points) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    // z band relative to the current body origin so slopes and LIO z drift
    // do not push real obstacles out of the band.
    const double rel_z = static_cast<double>(point.z) - origin_z;
    if (rel_z < z_min_ || rel_z > z_max_) {
      continue;
    }

    const double dx = static_cast<double>(point.x) - origin_x;
    const double dy = static_cast<double>(point.y) - origin_y;
    const double planar2 = dx * dx + dy * dy;
    if (planar2 + rel_z * rel_z > max_range2) {
      continue;
    }

    std::int32_t cell_x = 0;
    std::int32_t cell_y = 0;
    if (!toCellIndex(point.x, resolution_, cell_x) ||
        !toCellIndex(point.y, resolution_, cell_y)) {
      continue;
    }

    for (int dxc = -inflation_cells; dxc <= inflation_cells; ++dxc) {
      for (int dyc = -inflation_cells; dyc <= inflation_cells; ++dyc) {
        if (dxc * dxc + dyc * dyc > inflation_cells * inflation_cells) {
          continue;
        }

        scan_hits.insert(cellKey(cell_x + dxc, cell_y + dyc));
      }
    }

    if (raycast_enable_ && origin_valid && planar2 <= max_ray2) {
      // One trace per unique endpoint cell: rays to the same cell are
      // identical, and tracing runs on the sensor callback path.
      ray_targets.insert(cellKey(cell_x, cell_y));
    }
  }

  for (const auto key : ray_targets) {
    traceRayCells(origin_cell_x, origin_cell_y, keyX(key), keyY(key), scan_free);
  }

  // Hits win over same-scan pass-throughs so grazing rays do not erode
  // surfaces observed in the same scan; each cell gets at most one score
  // update per scan so the decay dynamics are scan-rate based.
  for (const auto key : scan_hits) {
    applyScoreLocked(key, kHitScore);
  }
  for (const auto key : scan_free) {
    if (scan_hits.count(key) == 0) {
      applyScoreLocked(key, -kMissScore);
    }
  }
}

void OccupancyGridPublisher::applyScoreLocked(std::int64_t key, int delta) {
  const auto x = keyX(key);
  const auto y = keyY(key);
  if (!boundsAllowLocked(x, y)) {
    if (clock_) {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "Occupancy grid max_extent (%.1f m) reached; dropping cell updates "
          "outside current bounds.",
          max_extent_);
    }
    return;
  }

  auto it = observed_cells_.try_emplace(key, static_cast<std::int8_t>(0)).first;
  const int score = std::clamp(static_cast<int>(it->second) + delta,
                               kCellScoreMin, kCellScoreMax);
  it->second = static_cast<std::int8_t>(score);

  if (!has_bounds_) {
    min_x_ = max_x_ = x;
    min_y_ = max_y_ = y;
    has_bounds_ = true;
  } else {
    min_x_ = std::min(min_x_, x);
    max_x_ = std::max(max_x_, x);
    min_y_ = std::min(min_y_, y);
    max_y_ = std::max(max_y_, y);
  }
}

bool OccupancyGridPublisher::boundsAllowLocked(
    std::int32_t x,
    std::int32_t y) const {
  if (!has_bounds_) {
    return max_extent_cells_ >= 1;
  }

  const auto span_x =
      static_cast<std::int64_t>(std::max(max_x_, x)) - std::min(min_x_, x) + 1;
  const auto span_y =
      static_cast<std::int64_t>(std::max(max_y_, y)) - std::min(min_y_, y) + 1;
  return span_x <= max_extent_cells_ && span_y <= max_extent_cells_;
}

nav_msgs::msg::OccupancyGrid OccupancyGridPublisher::buildMessageLocked(
    const rclcpp::Time& stamp) const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id_;
  // Node clock fallback respects use_sim_time.
  msg.header.stamp =
      stamp.nanoseconds() > 0 ? stamp : (clock_ ? clock_->now() : stamp);
  msg.info.resolution = static_cast<float>(resolution_);

  const int padding_cells = static_cast<int>(std::ceil(map_padding_ / resolution_));
  const std::int32_t origin_x = min_x_ - padding_cells;
  const std::int32_t origin_y = min_y_ - padding_cells;
  const std::int32_t end_x = max_x_ + padding_cells;
  const std::int32_t end_y = max_y_ + padding_cells;

  const auto width64 = static_cast<std::int64_t>(end_x) - origin_x + 1;
  const auto height64 = static_cast<std::int64_t>(end_y) - origin_y + 1;
  if (width64 <= 0 || height64 <= 0 ||
      width64 > std::numeric_limits<std::uint32_t>::max() ||
      height64 > std::numeric_limits<std::uint32_t>::max()) {
    return msg;
  }

  msg.info.width = static_cast<std::uint32_t>(width64);
  msg.info.height = static_cast<std::uint32_t>(height64);
  msg.info.origin.position.x = static_cast<double>(origin_x) * resolution_;
  msg.info.origin.position.y = static_cast<double>(origin_y) * resolution_;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  const int8_t default_value =
      static_cast<int8_t>(use_unknown_ ? unknown_value_ : free_value_);
  msg.data.assign(static_cast<std::size_t>(msg.info.width) * msg.info.height,
                  default_value);

  for (const auto& [key, score] : observed_cells_) {
    const auto x = keyX(key) - origin_x;
    const auto y = keyY(key) - origin_y;
    if (x < 0 || y < 0 ||
        x >= static_cast<std::int32_t>(msg.info.width) ||
        y >= static_cast<std::int32_t>(msg.info.height)) {
      continue;
    }

    const auto index = static_cast<std::size_t>(y) * msg.info.width +
                       static_cast<std::size_t>(x);
    msg.data[index] =
        static_cast<int8_t>(score > 0 ? occupied_value_ : free_value_);
  }

  return msg;
}

nav_msgs::msg::OccupancyGrid OccupancyGridPublisher::buildEmptyMessageLocked(
    const rclcpp::Time& stamp) const {
  // Minimal all-unknown grid: replaces the latched sample after a reset that
  // left no observed cells.
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp =
      stamp.nanoseconds() > 0 ? stamp : (clock_ ? clock_->now() : stamp);
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = 1;
  msg.info.height = 1;
  msg.info.origin.orientation.w = 1.0;
  msg.data.assign(1, static_cast<int8_t>(unknown_value_));
  return msg;
}

std::int64_t OccupancyGridPublisher::cellKey(
    std::int32_t x,
    std::int32_t y) {
  const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
  return static_cast<std::int64_t>((ux << 32) | uy);
}

std::int32_t OccupancyGridPublisher::keyX(std::int64_t key) {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey >> 32));
}

std::int32_t OccupancyGridPublisher::keyY(std::int64_t key) {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey & 0xffffffff));
}

void OccupancyGridPublisher::traceRayCells(
    std::int32_t x0, std::int32_t y0,
    std::int32_t x1, std::int32_t y1,
    std::unordered_set<std::int64_t>& cells) {
  // 2D Bresenham from the sensor cell to the hit cell, endpoint excluded.
  const std::int32_t dx = std::abs(x1 - x0);
  const std::int32_t dy = std::abs(y1 - y0);
  const std::int32_t sx = x0 < x1 ? 1 : -1;
  const std::int32_t sy = y0 < y1 ? 1 : -1;
  std::int32_t err = dx - dy;
  std::int32_t x = x0;
  std::int32_t y = y0;

  while (x != x1 || y != y1) {
    cells.insert(cellKey(x, y));
    const std::int32_t e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

bool OccupancyGridPublisher::pointPassesHeightFilter(
    const BASIC::PointType& point) const {
  return std::isfinite(point.x) &&
         std::isfinite(point.y) &&
         std::isfinite(point.z) &&
         point.z >= z_min_ &&
         point.z <= z_max_;
}

}  // namespace LI2Sup
