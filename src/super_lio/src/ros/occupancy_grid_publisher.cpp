#include "ros/occupancy_grid_publisher.h"

#include <algorithm>
#include <cstddef>
#include <string_view>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace LI2Sup {

namespace {

adagraph::mapping::OccupancyGridUpdatePolicy ParseUpdatePolicy(
    std::string_view value, const rclcpp::Logger& logger) {
  if (value == "incremental") {
    return adagraph::mapping::OccupancyGridUpdatePolicy::kIncremental;
  }
  if (value == "rebuild") {
    return adagraph::mapping::OccupancyGridUpdatePolicy::kRebuild;
  }
  if (value != "lazy") {
    RCLCPP_WARN(logger,
                "Unknown lio.occupancy_grid.update_policy='%s'; using lazy",
                std::string(value).c_str());
  }
  return adagraph::mapping::OccupancyGridUpdatePolicy::kLazy;
}

OccupancyGridPublisher::SourcePolicy ParseSourcePolicy(
    std::string_view value, const rclcpp::Logger& logger) {
  if (value == "online") {
    return OccupancyGridPublisher::SourcePolicy::kOnline;
  }
  if (value != "snapshot") {
    RCLCPP_WARN(logger,
                "Unknown lio.occupancy_grid.source='%s'; using snapshot",
                std::string(value).c_str());
  }
  return OccupancyGridPublisher::SourcePolicy::kSnapshot;
}

}  // namespace

OccupancyGridPublisher::~OccupancyGridPublisher() {
  stopSnapshotWorker();
}

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
  configured_frame_id_ = frame_id_;

  std::string source = "snapshot";
  node.declare_parameter<std::string>("lio.occupancy_grid.source", source);
  node.get_parameter("lio.occupancy_grid.source", source);
  source_policy_ = ParseSourcePolicy(source, node.get_logger());

  node.declare_parameter<double>("lio.occupancy_grid.resolution", 0.1);
  node.get_parameter("lio.occupancy_grid.resolution",
                    occupancy_config_.resolution);
  occupancy_config_.resolution =
      std::max(0.01, occupancy_config_.resolution);

  node.declare_parameter<double>("lio.occupancy_grid.z_min", -0.3);
  node.get_parameter("lio.occupancy_grid.z_min", occupancy_config_.z_min);

  node.declare_parameter<double>("lio.occupancy_grid.z_max", 2.0);
  node.get_parameter("lio.occupancy_grid.z_max", occupancy_config_.z_max);
  if (occupancy_config_.z_min > occupancy_config_.z_max) {
    std::swap(occupancy_config_.z_min, occupancy_config_.z_max);
  }

  node.declare_parameter<double>("lio.occupancy_grid.publish_period", 1.0);
  node.get_parameter("lio.occupancy_grid.publish_period", publish_period_);
  publish_period_ = std::max(0.1, publish_period_);

  std::string update_policy = "lazy";
  node.declare_parameter<std::string>("lio.occupancy_grid.update_policy",
                                       update_policy);
  node.get_parameter("lio.occupancy_grid.update_policy", update_policy);
  update_policy_ = ParseUpdatePolicy(update_policy, node.get_logger());
  occupancy_config_.update_policy = update_policy_;

  node.declare_parameter<double>("lio.occupancy_grid.map_padding", 1.0);
  node.get_parameter("lio.occupancy_grid.map_padding",
                    occupancy_config_.map_padding);
  occupancy_config_.map_padding =
      std::max(0.0, occupancy_config_.map_padding);

  node.declare_parameter<double>("lio.occupancy_grid.inflation_radius", 0.0);
  node.get_parameter("lio.occupancy_grid.inflation_radius",
                    occupancy_config_.inflation_radius);
  occupancy_config_.inflation_radius =
      std::max(0.0, occupancy_config_.inflation_radius);

  node.declare_parameter<double>("lio.occupancy_grid.max_range", 30.0);
  node.get_parameter("lio.occupancy_grid.max_range",
                    occupancy_config_.max_range);
  occupancy_config_.max_range = std::max(
      occupancy_config_.resolution, occupancy_config_.max_range);

  node.declare_parameter<double>("lio.occupancy_grid.max_extent", 400.0);
  node.get_parameter("lio.occupancy_grid.max_extent",
                    occupancy_config_.max_extent);
  occupancy_config_.max_extent = std::max(
      10.0 * occupancy_config_.resolution, occupancy_config_.max_extent);

  node.declare_parameter<bool>("lio.occupancy_grid.raycast.enable", true);
  node.get_parameter("lio.occupancy_grid.raycast.enable",
                    occupancy_config_.raycast_enable);

  node.declare_parameter<double>(
      "lio.occupancy_grid.raycast.max_ray_length", 15.0);
  node.get_parameter("lio.occupancy_grid.raycast.max_ray_length",
                    occupancy_config_.raycast_max_ray_length);
  occupancy_config_.raycast_max_ray_length = std::max(
      occupancy_config_.resolution, occupancy_config_.raycast_max_ray_length);

  node.declare_parameter<bool>("lio.occupancy_grid.use_unknown", true);
  node.get_parameter("lio.occupancy_grid.use_unknown",
                    occupancy_config_.use_unknown);

  node.declare_parameter<bool>("lio.occupancy_grid.reset_on_global_map", true);
  node.get_parameter("lio.occupancy_grid.reset_on_global_map",
                    reset_on_global_map_);

  node.declare_parameter<int>("lio.occupancy_grid.occupied_value", 100);
  node.get_parameter("lio.occupancy_grid.occupied_value",
                    occupancy_config_.occupied_value);
  occupancy_config_.occupied_value =
      std::clamp(occupancy_config_.occupied_value, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.free_value", 0);
  node.get_parameter("lio.occupancy_grid.free_value",
                    occupancy_config_.free_value);
  occupancy_config_.free_value =
      std::clamp(occupancy_config_.free_value, 0, 100);

  node.declare_parameter<int>("lio.occupancy_grid.unknown_value", -1);
  node.get_parameter("lio.occupancy_grid.unknown_value",
                    occupancy_config_.unknown_value);
  occupancy_config_.unknown_value =
      std::clamp(occupancy_config_.unknown_value, -1, 100);

  int rebuild_scratch_memory_mb = 64;
  node.declare_parameter<int>(
      "lio.occupancy_grid.rebuild.max_scratch_memory_mb",
      rebuild_scratch_memory_mb);
  node.get_parameter(
      "lio.occupancy_grid.rebuild.max_scratch_memory_mb",
      rebuild_scratch_memory_mb);
  rebuild_scratch_memory_mb = std::max(1, rebuild_scratch_memory_mb);
  occupancy_config_.rebuild_scratch_memory_bytes =
      static_cast<std::size_t>(rebuild_scratch_memory_mb) * 1024U * 1024U;

  int rebuild_max_points = 5'000'000;
  node.declare_parameter<int>("lio.occupancy_grid.rebuild.max_points",
                              rebuild_max_points);
  node.get_parameter("lio.occupancy_grid.rebuild.max_points",
                    rebuild_max_points);
  occupancy_config_.rebuild_max_points = static_cast<std::size_t>(
      std::max(1, rebuild_max_points));

  int rebuild_check_interval = 4096;
  node.declare_parameter<int>(
      "lio.occupancy_grid.rebuild.cancellation_check_interval",
      rebuild_check_interval);
  node.get_parameter(
      "lio.occupancy_grid.rebuild.cancellation_check_interval",
      rebuild_check_interval);
  occupancy_config_.rebuild_cancellation_check_interval =
      static_cast<std::size_t>(std::max(1, rebuild_check_interval));

  int max_materialized_cells = 16'000'000;
  node.declare_parameter<int>("lio.occupancy_grid.max_materialized_cells",
                              max_materialized_cells);
  node.get_parameter("lio.occupancy_grid.max_materialized_cells",
                    max_materialized_cells);
  occupancy_config_.max_materialized_cells = static_cast<std::size_t>(
      std::max(1, max_materialized_cells));

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
      "lio.occupancy_grid.ground_filter.non_ground_topic",
      non_ground_debug_topic_);
  node.get_parameter(
      "lio.occupancy_grid.ground_filter.non_ground_topic",
      non_ground_debug_topic_);

  if (!enabled_) {
    RCLCPP_INFO(node.get_logger(), "Occupancy grid publisher disabled.");
    return;
  }

  builder_ = std::make_unique<adagraph::mapping::OccupancyGridBuilder>(
      occupancy_config_);
  if (!builder_->valid()) {
    RCLCPP_ERROR(node.get_logger(), "Invalid occupancy grid configuration.");
    enabled_ = false;
    builder_.reset();
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
      period, [this]() { publish(); }, callback_group);
  if (source_policy_ == SourcePolicy::kSnapshot) {
    startSnapshotWorker();
  }

  RCLCPP_INFO(
      node.get_logger(),
      "Occupancy grid enabled: topic=%s frame=%s resolution=%.3f z=[%.2f, %.2f] "
      "max_range=%.1f raycast=%s max_extent=%.1f policy=%s source=%s",
      topic_.c_str(), frame_id_.c_str(), occupancy_config_.resolution,
      occupancy_config_.z_min, occupancy_config_.z_max,
      occupancy_config_.max_range,
      occupancy_config_.raycast_enable ? "on" : "off",
      occupancy_config_.max_extent, update_policy.c_str(),
      source_policy_ == SourcePolicy::kSnapshot ? "snapshot" : "online");
  RCLCPP_INFO(
      node.get_logger(),
      "Occupancy rebuild budget: scratch=%zu bytes, points=%zu, "
      "cancel_check=%zu, max_cells=%zu",
      occupancy_config_.rebuild_scratch_memory_bytes,
      occupancy_config_.rebuild_max_points,
      occupancy_config_.rebuild_cancellation_check_interval,
      occupancy_config_.max_materialized_cells);
}

void OccupancyGridPublisher::setFrameMode(
    bool prior_map_localization,
    const std::string& odom_frame_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  prior_map_localization_ = prior_map_localization;
  if (!prior_map_localization && !odom_frame_id.empty()) {
    // SLAM mode: cells live in the LIO odometry frame and no map->odom
    // correction exists, so the configured (map) frame id would displace the
    // grid whenever TF diverges from identity.
    frame_id_ = source_policy_ == SourcePolicy::kSnapshot
                    ? configured_frame_id_
                    : odom_frame_id;
  } else if (prior_map_localization) {
    frame_id_ = configured_frame_id_;
  }
  RCLCPP_INFO(logger_, "Occupancy grid frame mode: %s, frame_id=%s",
              prior_map_localization ? "relocation" : "slam",
              frame_id_.c_str());
}

void OccupancyGridPublisher::setMapSnapshot(
    std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept {
  if (!snapshot || source_policy_ != SourcePolicy::kSnapshot) {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (prior_map_localization_) {
      return;
    }
  }
  // Snapshot versions are commit versions. Coalesce duplicate/stale observer
  // calls before changing the generation so a "superseded" cancellation
  // always means a strictly newer MapSnapshot arrived.
  auto pending = pending_map_snapshot_.load(std::memory_order_acquire);
  while (!pending || snapshot->version > pending->version) {
    if (pending_map_snapshot_.compare_exchange_weak(
            pending, snapshot, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      snapshot_handoffs_.fetch_add(1, std::memory_order_relaxed);
      snapshot_request_generation_.fetch_add(1, std::memory_order_acq_rel);
      snapshot_worker_cv_.notify_one();
      return;
    }
  }
}

void OccupancyGridPublisher::resetSession() {
  // Joining here makes the session boundary explicit: an old rebuild cannot
  // publish a dense product after the new frontend/downstream state starts.
  stopSnapshotWorker();
  {
    std::lock_guard<std::mutex> lock(snapshot_worker_mutex_);
    pending_map_snapshot_.store(nullptr, std::memory_order_release);
    rebuilt_map_snapshot_version_ = 0;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (builder_) {
      builder_->reset();
    }
    dirty_ = false;
    latest_stamp_ = rclcpp::Time(0);
    next_snapshot_version_ = 1;
    latest_snapshot_.store(nullptr, std::memory_order_release);
    published_snapshot_version_.store(0, std::memory_order_release);
  }
  snapshot_request_generation_.store(0, std::memory_order_release);
  snapshot_worker_stop_requested_.store(false, std::memory_order_release);
  if (source_policy_ == SourcePolicy::kSnapshot && enabled_) {
    startSnapshotWorker();
  }
}

void OccupancyGridPublisher::insertCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !cloud || cloud->empty()) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (source_policy_ == SourcePolicy::kSnapshot &&
      !prior_map_localization_) {
    return;
  }
  // No sensor origin on this path: no range gating, no free-space carving,
  // and the z band stays an absolute world-frame band.
  if (update_policy_ ==
      adagraph::mapping::OccupancyGridUpdatePolicy::kRebuild) {
    builder_->reset();
  }
  (void)builder_->insertCloud(toView(cloud), true);
  latest_stamp_ = stamp;
  dirty_ = true;
  if (update_policy_ ==
      adagraph::mapping::OccupancyGridUpdatePolicy::kIncremental) {
    materializeSnapshotLocked();
  }
}

void OccupancyGridPublisher::insertBodyCloud(
    const BASIC::CloudPtr& body_cloud,
    const Eigen::Matrix4f& body_to_world,
    const Eigen::Quaterniond& body_to_world_rotation,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !body_cloud || body_cloud->empty()) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (source_policy_ == SourcePolicy::kSnapshot &&
        !prior_map_localization_) {
      return;
    }
  }

  const auto segmented =
      ground_filter_.segment(body_cloud, body_to_world_rotation);
  publishDebugClouds(segmented, body_to_world, stamp);
  if (!segmented.non_ground || segmented.non_ground->empty()) {
    return;
  }

  BASIC::CloudPtr non_ground_world(new BASIC::PointCloudType());
  pcl::transformPointCloud(
      *segmented.non_ground, *non_ground_world, body_to_world);

  const Eigen::Vector3f sensor_origin = body_to_world.block<3, 1>(0, 3);
  std::lock_guard<std::mutex> lock(mutex_);
  if (update_policy_ ==
      adagraph::mapping::OccupancyGridUpdatePolicy::kRebuild) {
    builder_->reset();
  }
  (void)builder_->insertScan(toView(non_ground_world),
                             sensor_origin.cast<double>());
  latest_stamp_ = stamp;
  dirty_ = true;
  if (update_policy_ ==
      adagraph::mapping::OccupancyGridUpdatePolicy::kIncremental) {
    materializeSnapshotLocked();
  }
}

void OccupancyGridPublisher::resetFromCloud(
    const BASIC::CloudPtr& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !cloud) {
    return;
  }

  // Prior-map path: apply the same ground rejection as the online path. The
  // scan segmenter needs a sensor-centric cloud, so the map-frame variant is
  // used here; its ground-relative band replaces the absolute z band.
  BASIC::CloudPtr insert_cloud = cloud;
  BASIC::CloudPtr ground_cloud;
  bool ground_relative_filtered = false;
  if (ground_filter_.enabled()) {
    const auto segmented = ground_filter_.segmentWorldMap(
        cloud, occupancy_config_.z_max + ground_filter_.sensorMountHeight());
    ground_cloud = segmented.ground;
    insert_cloud = segmented.non_ground;
    ground_relative_filtered = true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reset_on_global_map_ ||
        update_policy_ ==
            adagraph::mapping::OccupancyGridUpdatePolicy::kRebuild) {
      builder_->reset();
    }
    if (ground_relative_filtered && ground_cloud && !ground_cloud->empty()) {
      // Static prior maps have no per-scan sensor origins for ray casting.
      // Ground returns still provide direct free-space observations. Insert
      // them first so non-ground hits below override free evidence when both
      // project into the same 2-D cell.
      (void)builder_->insertFreeCloud(toView(ground_cloud));
    }
    if (insert_cloud && !insert_cloud->empty()) {
      (void)builder_->insertCloud(toView(insert_cloud),
                                  !ground_relative_filtered);
    }
    latest_stamp_ = stamp;
    dirty_ = true;
    materializeSnapshotLocked();
  }
  publishLatestSnapshot();
}

void OccupancyGridPublisher::publish() {
  if (!enabled_ || !publisher_ || !builder_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (dirty_) {
      materializeSnapshotLocked();
    }
  }
  publishLatestSnapshot();
}

void OccupancyGridPublisher::materializeSnapshotLocked() {
  if (!builder_ || !dirty_) {
    return;
  }
  auto snapshot = std::make_shared<adagraph::mapping::OccupancyGridSnapshot>(
      builder_->snapshot(next_snapshot_version_++, latest_stamp_.nanoseconds()));
  dirty_ = false;
  latest_snapshot_.store(std::move(snapshot), std::memory_order_release);
}

void OccupancyGridPublisher::publishLatestSnapshot() {
  const auto snapshot = latest_snapshot_.load(std::memory_order_acquire);
  if (!snapshot || !snapshot->valid() || !publisher_) {
    return;
  }

  std::uint64_t expected = published_snapshot_version_.load(
      std::memory_order_acquire);
  for (;;) {
    if (snapshot->version <= expected) {
      return;
    }
    if (published_snapshot_version_.compare_exchange_weak(
            expected, snapshot->version, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      break;
    }
  }

  std::string frame_id;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_id = frame_id_;
  }
  publisher_->publish(toRosMessage(*snapshot, frame_id, clock_));
}

void OccupancyGridPublisher::startSnapshotWorker() {
  std::lock_guard<std::mutex> lock(snapshot_worker_mutex_);
  if (snapshot_worker_started_) {
    return;
  }
  snapshot_worker_stop_ = false;
  snapshot_worker_stop_requested_.store(false, std::memory_order_release);
  snapshot_worker_started_ = true;
  snapshot_worker_ = std::thread(
      &OccupancyGridPublisher::snapshotWorkerLoop, this);
}

void OccupancyGridPublisher::stopSnapshotWorker() {
  snapshot_worker_stop_requested_.store(true, std::memory_order_release);
  snapshot_request_generation_.fetch_add(1, std::memory_order_acq_rel);
  {
    std::lock_guard<std::mutex> lock(snapshot_worker_mutex_);
    if (!snapshot_worker_started_) {
      return;
    }
    snapshot_worker_stop_ = true;
  }
  snapshot_worker_cv_.notify_all();
  if (snapshot_worker_.joinable()) {
    snapshot_worker_.join();
  }
  std::lock_guard<std::mutex> lock(snapshot_worker_mutex_);
  snapshot_worker_started_ = false;
  RCLCPP_INFO(
      logger_,
      "Occupancy snapshot handoffs/rebuilds requested/completed/"
      "cancelled_superseded/cancelled_shutdown/discarded_obsolete/"
      "budget_exceeded/failed=%lu/%lu/%lu/%lu/%lu/%lu/%lu/%lu, "
      "peak_scratch=%zu bytes",
      static_cast<unsigned long>(snapshot_handoffs_.load()),
      static_cast<unsigned long>(snapshot_rebuild_requested_.load()),
      static_cast<unsigned long>(snapshot_rebuild_completed_.load()),
      static_cast<unsigned long>(
          snapshot_rebuild_cancelled_superseded_.load()),
      static_cast<unsigned long>(
          snapshot_rebuild_cancelled_shutdown_.load()),
      static_cast<unsigned long>(snapshot_rebuild_discarded_obsolete_.load()),
      static_cast<unsigned long>(snapshot_rebuild_budget_exceeded_.load()),
      static_cast<unsigned long>(snapshot_rebuild_failed_.load()),
      snapshot_rebuild_peak_scratch_bytes_.load());
}

void OccupancyGridPublisher::snapshotWorkerLoop() {
  for (;;) {
    std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot;
    {
      std::unique_lock<std::mutex> lock(snapshot_worker_mutex_);
      snapshot_worker_cv_.wait(lock, [this] {
        const auto pending =
            pending_map_snapshot_.load(std::memory_order_acquire);
        return snapshot_worker_stop_ ||
               (pending && pending->version > rebuilt_map_snapshot_version_);
      });
      if (snapshot_worker_stop_) {
        return;
      }
      snapshot = pending_map_snapshot_.load(std::memory_order_acquire);
      if (!snapshot || snapshot->version <= rebuilt_map_snapshot_version_) {
        continue;
      }
    }

    ++snapshot_rebuild_requested_;
    const auto request_generation =
        snapshot_request_generation_.load(std::memory_order_acquire);
    const auto newer_snapshot_pending =
        [this, request_generation, version = snapshot->version] {
      const auto pending =
          pending_map_snapshot_.load(std::memory_order_acquire);
      return snapshot_worker_stop_requested_.load(std::memory_order_acquire) ||
             snapshot_request_generation_.load(std::memory_order_acquire) !=
                 request_generation ||
             (pending && pending->version > version);
    };

    adagraph::mapping::OccupancyGridBuilder scratch_builder(
        occupancy_config_);
    adagraph::mapping::OccupancyGridRebuildOptions options;
    options.max_scratch_memory_bytes =
        occupancy_config_.rebuild_scratch_memory_bytes;
    options.max_points = occupancy_config_.rebuild_max_points;
    options.cancellation_check_interval =
        occupancy_config_.rebuild_cancellation_check_interval;
    options.should_cancel = newer_snapshot_pending;
    const auto rebuild = scratch_builder.rebuildFromSnapshot(*snapshot, options);

    auto peak = snapshot_rebuild_peak_scratch_bytes_.load(
        std::memory_order_acquire);
    while (peak < rebuild.peak_estimated_scratch_memory_bytes &&
           !snapshot_rebuild_peak_scratch_bytes_.compare_exchange_weak(
               peak, rebuild.peak_estimated_scratch_memory_bytes,
               std::memory_order_acq_rel, std::memory_order_acquire)) {
    }

    if (rebuild.status ==
        adagraph::mapping::OccupancyGridRebuildStatus::kCancelled) {
      if (snapshot_worker_stop_requested_.load(std::memory_order_acquire)) {
        ++snapshot_rebuild_cancelled_shutdown_;
        return;
      }
      ++snapshot_rebuild_cancelled_superseded_;
      // A newer pending version will wake the next iteration. If cancellation
      // was only a duplicate handoff, retry the same version rather than
      // losing the only requested product.
      continue;
    }

    const auto mark_rebuilt = [this, version = snapshot->version] {
      std::lock_guard<std::mutex> lock(snapshot_worker_mutex_);
      rebuilt_map_snapshot_version_ =
          std::max(rebuilt_map_snapshot_version_, version);
    };

    if (!rebuild.succeeded()) {
      if (rebuild.status ==
          adagraph::mapping::OccupancyGridRebuildStatus::kScratchBudgetExceeded ||
          rebuild.status ==
              adagraph::mapping::OccupancyGridRebuildStatus::kPointBudgetExceeded) {
        ++snapshot_rebuild_budget_exceeded_;
        RCLCPP_WARN(
            logger_,
            "Occupancy rebuild v%lu exceeded its budget: points=%zu, "
            "peak_scratch=%zu/%zu bytes",
            static_cast<unsigned long>(snapshot->version),
            rebuild.processed_points,
            rebuild.peak_estimated_scratch_memory_bytes,
            occupancy_config_.rebuild_scratch_memory_bytes);
      } else {
        ++snapshot_rebuild_failed_;
        RCLCPP_WARN(logger_,
                    "Failed to rebuild occupancy grid from MapSnapshot v%lu",
                    static_cast<unsigned long>(snapshot->version));
      }
      mark_rebuilt();
      continue;
    }

    if (snapshot_worker_stop_requested_.load(std::memory_order_acquire)) {
      ++snapshot_rebuild_cancelled_shutdown_;
      return;
    }

    const std::int64_t timestamp_nanoseconds = snapshot->keyframes.empty()
        ? 0
        : snapshot->keyframes.back().T_map_base.timestamp.nanoseconds();
    auto materialized = std::make_shared<
        adagraph::mapping::OccupancyGridSnapshot>(
            scratch_builder.snapshot(0, timestamp_nanoseconds));
    if (!materialized->valid()) {
      ++snapshot_rebuild_failed_;
      RCLCPP_WARN(logger_,
                  "Occupancy rebuild v%lu produced an invalid dense product",
                  static_cast<unsigned long>(snapshot->version));
      mark_rebuilt();
      continue;
    }

    const auto pending =
        pending_map_snapshot_.load(std::memory_order_acquire);
    if (pending && pending->version > snapshot->version) {
      // Do not publish an obsolete dense map when a newer immutable snapshot
      // is already available. The next loop iteration consumes the newest
      // pending handoff.
      ++snapshot_rebuild_discarded_obsolete_;
      mark_rebuilt();
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      materialized->version = next_snapshot_version_++;
      latest_stamp_ = rclcpp::Time(timestamp_nanoseconds);
      dirty_ = false;
      latest_snapshot_.store(std::move(materialized),
                             std::memory_order_release);
    }
    ++snapshot_rebuild_completed_;
    mark_rebuilt();
    RCLCPP_DEBUG(
        logger_,
        "Occupancy rebuild v%lu completed: points=%zu peak_scratch=%zu bytes",
        static_cast<unsigned long>(snapshot->version),
        rebuild.processed_points,
        rebuild.peak_estimated_scratch_memory_bytes);
  }
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

  const auto publish_cloud =
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

adagraph::sensor::PointCloudView OccupancyGridPublisher::toView(
    const BASIC::CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  const auto& first = cloud->front();
  const auto* origin = reinterpret_cast<const std::byte*>(cloud->data());
  return adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      cloud->data(), cloud->size(), sizeof(BASIC::PointType),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      adagraph::geometry::FrameId::kUnknown);
}

nav_msgs::msg::OccupancyGrid OccupancyGridPublisher::toRosMessage(
    const adagraph::mapping::OccupancyGridSnapshot& snapshot,
    const std::string& frame_id,
    const rclcpp::Clock::SharedPtr& clock) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = snapshot.timestamp_nanoseconds > 0
                         ? rclcpp::Time(snapshot.timestamp_nanoseconds)
                         : (clock ? clock->now() : rclcpp::Time(0));
  msg.info.resolution = static_cast<float>(snapshot.resolution);
  msg.info.width = snapshot.width;
  msg.info.height = snapshot.height;
  msg.info.origin.position.x =
      static_cast<double>(snapshot.origin_x) * snapshot.resolution;
  msg.info.origin.position.y =
      static_cast<double>(snapshot.origin_y) * snapshot.resolution;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  msg.data = snapshot.data;
  return msg;
}

}  // namespace LI2Sup
