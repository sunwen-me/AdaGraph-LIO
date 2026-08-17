#include "ros/occupancy_grid_publisher.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstddef>
#include <exception>
#include <string_view>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include "hybrid_slam/adapter/pcl_types.h"

namespace LI2Sup {

namespace {

hybrid_slam::mapping::OccupancyGridUpdatePolicy ParseUpdatePolicy(
    std::string_view value, const rclcpp::Logger& logger) {
  if (value == "incremental") {
    return hybrid_slam::mapping::OccupancyGridUpdatePolicy::kIncremental;
  }
  if (value == "rebuild") {
    return hybrid_slam::mapping::OccupancyGridUpdatePolicy::kRebuild;
  }
  if (value != "lazy") {
    RCLCPP_WARN(logger,
                "Unknown lio.occupancy_grid.update_policy='%s'; using lazy",
                std::string(value).c_str());
  }
  return hybrid_slam::mapping::OccupancyGridUpdatePolicy::kLazy;
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

hybrid_slam::sensor::PointCloudView ToView(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  const auto& first = cloud->front();
  const auto* origin = reinterpret_cast<const std::byte*>(cloud->data());
  return hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      cloud->data(), cloud->size(), sizeof(hybrid_slam::adapter::PclPoint),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      hybrid_slam::geometry::FrameId::kUnknown);
}

}  // namespace

void OccupancyGridPublisher::TimingHistogram::record(
    const std::chrono::nanoseconds duration) noexcept {
  const auto raw_nanoseconds = duration.count();
  const auto nanoseconds = raw_nanoseconds > 0
      ? static_cast<std::uint64_t>(raw_nanoseconds)
      : 0U;
  const auto microseconds = (nanoseconds + 999U) / 1000U;
  std::size_t bucket = 0;
  if (microseconds > 0U) {
    if (microseconds <= kLinearBuckets) {
      bucket = static_cast<std::size_t>(microseconds - 1U);
    } else {
      const auto overflow_ms =
          (microseconds - kLinearBuckets + 999U) / 1000U;
      const auto logarithmic_bucket = static_cast<std::size_t>(
          std::bit_width(overflow_ms) - 1U);
      bucket = std::min(kLinearBuckets + logarithmic_bucket,
                        kBucketCount - 1U);
    }
  }
  buckets[bucket].fetch_add(1, std::memory_order_relaxed);
  count.fetch_add(1, std::memory_order_relaxed);
  total_nanoseconds.fetch_add(nanoseconds, std::memory_order_relaxed);
  auto maximum = max_nanoseconds.load(std::memory_order_relaxed);
  while (maximum < nanoseconds &&
         !max_nanoseconds.compare_exchange_weak(
             maximum, nanoseconds, std::memory_order_relaxed,
             std::memory_order_relaxed)) {
  }
}

OccupancyLatencySummary OccupancyGridPublisher::TimingHistogram::snapshot()
    const noexcept {
  OccupancyLatencySummary summary;
  summary.sample_count = count.load(std::memory_order_acquire);
  if (summary.sample_count == 0U) {
    return summary;
  }
  const auto total = total_nanoseconds.load(std::memory_order_acquire);
  summary.mean_us = static_cast<double>(total) /
                    static_cast<double>(summary.sample_count) / 1000.0;
  summary.max_us = (max_nanoseconds.load(std::memory_order_acquire) + 999U) /
                   1000U;

  const auto percentile = [this, samples = summary.sample_count,
                           maximum_us = summary.max_us](
                              const std::uint64_t numerator) -> std::uint64_t {
    const auto target = (samples * numerator + 99U) / 100U;
    std::uint64_t seen = 0;
    for (std::size_t index = 0; index < kBucketCount; ++index) {
      seen += buckets[index].load(std::memory_order_acquire);
      if (seen >= target) {
        if (index < kLinearBuckets) {
          return static_cast<std::uint64_t>(index + 1U);
        }
        const auto overflow_index = index - kLinearBuckets;
        return static_cast<std::uint64_t>(kLinearBuckets) +
               (std::uint64_t{1} << overflow_index) * std::uint64_t{1000};
      }
    }
    return maximum_us;
  };
  summary.p50_us = percentile(50U);
  summary.p95_us = percentile(95U);
  summary.p99_us = percentile(99U);
  return summary;
}

void OccupancyGridPublisher::TimingHistogram::reset() noexcept {
  for (auto& bucket : buckets) {
    bucket.store(0, std::memory_order_relaxed);
  }
  count.store(0, std::memory_order_relaxed);
  total_nanoseconds.store(0, std::memory_order_relaxed);
  max_nanoseconds.store(0, std::memory_order_relaxed);
}

OnlineOccupancyTimingStats OccupancyGridPublisher::onlineTimingStats() const
    noexcept {
  return {
      online_update_lock_wait_.snapshot(),
      online_update_lock_hold_.snapshot(),
      online_update_end_to_end_.snapshot(),
      online_publish_lock_wait_.snapshot(),
      online_publish_lock_hold_.snapshot(),
      online_publish_end_to_end_.snapshot(),
  };
}

void OccupancyGridPublisher::resetOnlineTimingStats() noexcept {
  online_update_lock_wait_.reset();
  online_update_lock_hold_.reset();
  online_update_end_to_end_.reset();
  online_publish_lock_wait_.reset();
  online_publish_lock_hold_.reset();
  online_publish_end_to_end_.reset();
}

OccupancyGridPublisher::~OccupancyGridPublisher() {
  if (grid_artifact_enabled_) {
    const auto snapshot = latest_snapshot_.load(std::memory_order_acquire);
    std::string frame_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame_id = frame_id_;
    }
    enqueueGridArtifact(snapshot, frame_id, true);
  }
  stopGridArtifactWorker();
  stopSnapshotWorker();
}

void OccupancyGridPublisher::setup(
    rclcpp::Node& node,
    const rclcpp::CallbackGroup::SharedPtr& callback_group) {
  clock_ = node.get_clock();
  logger_ = node.get_logger();
  resetOnlineTimingStats();

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

  node.declare_parameter<bool>("lio.occupancy_grid.artifact.enable", false);
  node.get_parameter("lio.occupancy_grid.artifact.enable",
                     grid_artifact_enabled_);
  std::string artifact_yaml_path;
  node.declare_parameter<std::string>(
      "lio.occupancy_grid.artifact.yaml_path", artifact_yaml_path);
  node.get_parameter("lio.occupancy_grid.artifact.yaml_path",
                     artifact_yaml_path);
  node.declare_parameter<double>(
      "lio.occupancy_grid.artifact.write_period_s", 5.0);
  node.get_parameter("lio.occupancy_grid.artifact.write_period_s",
                     grid_artifact_period_s_);
  grid_artifact_period_s_ = std::max(0.1, grid_artifact_period_s_);

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

  if (grid_artifact_enabled_) {
    grid_artifact_yaml_path_ = std::filesystem::path(artifact_yaml_path);
    if (source_policy_ != SourcePolicy::kOnline) {
      RCLCPP_ERROR(
          node.get_logger(),
          "Occupancy grid artifact requires lio.occupancy_grid.source=online; "
          "Snapshot products do not retain scan-origin free-space evidence.");
      grid_artifact_enabled_ = false;
    } else if (grid_artifact_yaml_path_.empty() ||
               !grid_artifact_yaml_path_.is_absolute() ||
               (grid_artifact_yaml_path_.extension() != ".yaml" &&
                grid_artifact_yaml_path_.extension() != ".yml")) {
      RCLCPP_ERROR(
          node.get_logger(),
          "Occupancy grid artifact yaml_path must be an absolute .yaml/.yml "
          "file path; durable artifact saving is disabled.");
      grid_artifact_enabled_ = false;
      grid_artifact_yaml_path_.clear();
    }
  }

  builder_ = std::make_unique<hybrid_slam::mapping::OccupancyGridBuilder>(
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
  if (grid_artifact_enabled_) {
    startGridArtifactWorker();
    RCLCPP_INFO(node.get_logger(),
                "Online occupancy Nav2 artifact enabled: yaml=%s period=%.1fs",
                grid_artifact_yaml_path_.c_str(), grid_artifact_period_s_);
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
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) noexcept {
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
  // Flush the final online product before version numbering restarts at one.
  if (grid_artifact_enabled_) {
    const auto snapshot = latest_snapshot_.load(std::memory_order_acquire);
    std::string frame_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frame_id = frame_id_;
    }
    enqueueGridArtifact(snapshot, frame_id, true);
  }
  stopGridArtifactWorker();
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
  resetOnlineTimingStats();
  snapshot_request_generation_.store(0, std::memory_order_release);
  snapshot_worker_stop_requested_.store(false, std::memory_order_release);
  grid_artifact_written_version_.store(0, std::memory_order_release);
  {
    std::lock_guard<std::mutex> lock(grid_artifact_mutex_);
    next_grid_artifact_submission_ = {};
  }
  if (source_policy_ == SourcePolicy::kSnapshot && enabled_) {
    startSnapshotWorker();
  }
  if (grid_artifact_enabled_ && enabled_) {
    startGridArtifactWorker();
  }
}

void OccupancyGridPublisher::insertCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !cloud.valid() || cloud.empty()) {
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
      hybrid_slam::mapping::OccupancyGridUpdatePolicy::kRebuild) {
    builder_->reset();
  }
  (void)builder_->insertCloud(cloud.view(), true);
  latest_stamp_ = stamp;
  dirty_ = true;
  if (update_policy_ ==
      hybrid_slam::mapping::OccupancyGridUpdatePolicy::kIncremental) {
    materializeSnapshotLocked();
  }
}

void OccupancyGridPublisher::insertBodyCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& body_cloud,
    const Eigen::Matrix4f& body_to_world,
    const Eigen::Quaterniond& body_to_world_rotation,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !body_cloud.valid() || body_cloud.empty()) {
    return;
  }
  const bool collect_online_timing = source_policy_ == SourcePolicy::kOnline;
  const auto update_started = std::chrono::steady_clock::now();

  const auto pcl_body_cloud = ToPclCloud(body_cloud);
  if (!pcl_body_cloud) {
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
      ground_filter_.segment(
          std::const_pointer_cast<hybrid_slam::adapter::PclPointCloud>(pcl_body_cloud),
          body_to_world_rotation);
  publishDebugClouds(segmented, body_to_world, stamp);
  if (!segmented.non_ground || segmented.non_ground->empty()) {
    return;
  }

  hybrid_slam::adapter::PclCloudPtr non_ground_world(new hybrid_slam::adapter::PclPointCloud());
  pcl::transformPointCloud(
      *segmented.non_ground, *non_ground_world, body_to_world);

  const Eigen::Vector3f sensor_origin = body_to_world.block<3, 1>(0, 3);
  const auto lock_wait_started = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  const auto lock_acquired = std::chrono::steady_clock::now();
  if (update_policy_ ==
      hybrid_slam::mapping::OccupancyGridUpdatePolicy::kRebuild) {
    builder_->reset();
  }
  (void)builder_->insertScan(ToView(non_ground_world),
                             sensor_origin.cast<double>());
  latest_stamp_ = stamp;
  dirty_ = true;
  if (update_policy_ ==
      hybrid_slam::mapping::OccupancyGridUpdatePolicy::kIncremental) {
    materializeSnapshotLocked();
  }
  const auto lock_released = std::chrono::steady_clock::now();
  lock.unlock();
  if (collect_online_timing) {
    online_update_lock_wait_.record(lock_acquired - lock_wait_started);
    online_update_lock_hold_.record(lock_released - lock_acquired);
    online_update_end_to_end_.record(lock_released - update_started);
  }
}

void OccupancyGridPublisher::resetFromCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const rclcpp::Time& stamp) {
  if (!enabled_ || !builder_ || !cloud.valid()) {
    return;
  }

  const auto pcl_cloud = ToPclCloud(cloud);
  if (!pcl_cloud) {
    return;
  }

  // Prior-map path: apply the same ground rejection as the online path. The
  // scan segmenter needs a sensor-centric cloud, so the map-frame variant is
  // used here; its ground-relative band replaces the absolute z band.
  hybrid_slam::adapter::PclCloudPtr insert_cloud =
      std::const_pointer_cast<hybrid_slam::adapter::PclPointCloud>(pcl_cloud);
  hybrid_slam::adapter::PclCloudPtr ground_cloud;
  bool ground_relative_filtered = false;
  if (ground_filter_.enabled()) {
    const auto segmented = ground_filter_.segmentWorldMap(
        insert_cloud,
        occupancy_config_.z_max + ground_filter_.sensorMountHeight());
    ground_cloud = segmented.ground;
    insert_cloud = segmented.non_ground;
    ground_relative_filtered = true;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (reset_on_global_map_ ||
        update_policy_ ==
            hybrid_slam::mapping::OccupancyGridUpdatePolicy::kRebuild) {
      builder_->reset();
    }
    if (ground_relative_filtered && ground_cloud && !ground_cloud->empty()) {
      // Static prior maps have no per-scan sensor origins for ray casting.
      // Ground returns still provide direct free-space observations. Insert
      // them first so non-ground hits below override free evidence when both
      // project into the same 2-D cell.
      (void)builder_->insertFreeCloud(ToView(ground_cloud));
    }
    if (insert_cloud && !insert_cloud->empty()) {
      (void)builder_->insertCloud(ToView(insert_cloud),
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

  const bool collect_online_timing = source_policy_ == SourcePolicy::kOnline;
  const auto publish_started = std::chrono::steady_clock::now();
  const auto lock_wait_started = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(mutex_);
  const auto lock_acquired = std::chrono::steady_clock::now();
  const bool materialized = dirty_;
  if (materialized) {
    materializeSnapshotLocked();
  }
  const auto lock_released = std::chrono::steady_clock::now();
  lock.unlock();
  publishLatestSnapshot();
  if (collect_online_timing && materialized) {
    online_publish_lock_wait_.record(lock_acquired - lock_wait_started);
    online_publish_lock_hold_.record(lock_released - lock_acquired);
    online_publish_end_to_end_.record(
        std::chrono::steady_clock::now() - publish_started);
  }
}

void OccupancyGridPublisher::materializeSnapshotLocked() {
  if (!builder_ || !dirty_) {
    return;
  }
  auto snapshot = std::make_shared<hybrid_slam::mapping::OccupancyGridSnapshot>(
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
  enqueueGridArtifact(snapshot, frame_id, false);
}

void OccupancyGridPublisher::enqueueGridArtifact(
    std::shared_ptr<const hybrid_slam::mapping::OccupancyGridSnapshot> grid,
    const std::string& frame_id, const bool force) {
  if (!grid_artifact_enabled_ || !grid || !grid->valid() ||
      frame_id.empty()) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(grid_artifact_mutex_);
  if (!grid_artifact_worker_started_ || grid_artifact_worker_stop_) {
    return;
  }
  if (!force && grid->version <=
                    grid_artifact_written_version_.load(
                        std::memory_order_acquire)) {
    return;
  }
  if (!force && now < next_grid_artifact_submission_) {
    return;
  }
  if (pending_grid_artifact_ && pending_grid_artifact_->grid &&
      pending_grid_artifact_->grid->version >= grid->version) {
    return;
  }
  if (pending_grid_artifact_) {
    ++grid_artifact_superseded_;
  }

  auto pending = std::make_shared<PendingGridArtifact>();
  pending->request_id = next_grid_artifact_request_id_++;
  pending->grid = std::move(grid);
  pending->frame_id = frame_id;
  pending->occupied_value = occupancy_config_.occupied_value;
  pending->free_value = occupancy_config_.free_value;
  pending->unknown_value = occupancy_config_.unknown_value;
  pending_grid_artifact_ = std::move(pending);
  next_grid_artifact_submission_ =
      now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(grid_artifact_period_s_));
  ++grid_artifact_submitted_;
  grid_artifact_cv_.notify_one();
}

void OccupancyGridPublisher::startGridArtifactWorker() {
  std::lock_guard<std::mutex> lock(grid_artifact_mutex_);
  if (grid_artifact_worker_started_) {
    return;
  }
  grid_artifact_worker_stop_ = false;
  grid_artifact_worker_started_ = true;
  grid_artifact_worker_ = std::thread(
      &OccupancyGridPublisher::gridArtifactWorkerLoop, this);
}

void OccupancyGridPublisher::stopGridArtifactWorker() {
  {
    std::lock_guard<std::mutex> lock(grid_artifact_mutex_);
    if (!grid_artifact_worker_started_) {
      return;
    }
    grid_artifact_worker_stop_ = true;
  }
  grid_artifact_cv_.notify_all();
  if (grid_artifact_worker_.joinable()) {
    grid_artifact_worker_.join();
  }
  {
    std::lock_guard<std::mutex> lock(grid_artifact_mutex_);
    grid_artifact_worker_started_ = false;
    pending_grid_artifact_.reset();
  }
  RCLCPP_INFO(
      logger_,
      "Occupancy grid artifacts submitted/completed/failed/superseded="
      "%lu/%lu/%lu/%lu, last_written_version=%lu",
      static_cast<unsigned long>(grid_artifact_submitted_.load()),
      static_cast<unsigned long>(grid_artifact_completed_.load()),
      static_cast<unsigned long>(grid_artifact_failed_.load()),
      static_cast<unsigned long>(grid_artifact_superseded_.load()),
      static_cast<unsigned long>(grid_artifact_written_version_.load()));
}

void OccupancyGridPublisher::gridArtifactWorkerLoop() {
  for (;;) {
    std::shared_ptr<const PendingGridArtifact> pending;
    {
      std::unique_lock<std::mutex> lock(grid_artifact_mutex_);
      grid_artifact_cv_.wait(lock, [this] {
        return grid_artifact_worker_stop_ || pending_grid_artifact_;
      });
      if (!pending_grid_artifact_) {
        if (grid_artifact_worker_stop_) {
          return;
        }
        continue;
      }
      pending = std::move(pending_grid_artifact_);
    }

    hybrid_slam::io::Nav2GridArtifactResult result;
    try {
      hybrid_slam::io::Nav2GridArtifactRequest request;
      request.request_id = pending->request_id;
      request.grid = pending->grid;
      request.yaml_path = grid_artifact_yaml_path_;
      request.frame_id = pending->frame_id;
      request.source = "online";
      request.occupied_value = pending->occupied_value;
      request.free_value = pending->free_value;
      request.unknown_value = pending->unknown_value;
      result = hybrid_slam::io::WriteNav2GridArtifact(std::move(request));
    } catch (const std::exception& error) {
      result.request_id = pending->request_id;
      result.grid_version = pending->grid ? pending->grid->version : 0;
      result.message = std::string("exception while saving Nav2 artifact: ") +
                       error.what();
    }

    if (!result.success) {
      ++grid_artifact_failed_;
      RCLCPP_ERROR(logger_,
                   "Failed to save online occupancy Nav2 artifact v%lu: %s",
                   static_cast<unsigned long>(result.grid_version),
                   result.message.c_str());
      continue;
    }

    auto written = grid_artifact_written_version_.load(
        std::memory_order_acquire);
    while (written < result.grid_version &&
           !grid_artifact_written_version_.compare_exchange_weak(
               written, result.grid_version, std::memory_order_acq_rel,
               std::memory_order_acquire)) {
    }
    ++grid_artifact_completed_;
    RCLCPP_INFO(logger_,
                "Saved online occupancy Nav2 artifact v%lu: yaml=%s",
                static_cast<unsigned long>(result.grid_version),
                result.yaml_path.c_str());
  }
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
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot;
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

    hybrid_slam::mapping::OccupancyGridBuilder scratch_builder(
        occupancy_config_);
    hybrid_slam::mapping::OccupancyGridRebuildOptions options;
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
        hybrid_slam::mapping::OccupancyGridRebuildStatus::kCancelled) {
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
          hybrid_slam::mapping::OccupancyGridRebuildStatus::kScratchBudgetExceeded ||
          rebuild.status ==
              hybrid_slam::mapping::OccupancyGridRebuildStatus::kPointBudgetExceeded) {
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
        hybrid_slam::mapping::OccupancyGridSnapshot>(
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
      [&](const hybrid_slam::adapter::PclCloudPtr& body_cloud,
          const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& pub) {
        if (!body_cloud) {
          return;
        }

        hybrid_slam::adapter::PclCloudPtr world_cloud(new hybrid_slam::adapter::PclPointCloud());
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

nav_msgs::msg::OccupancyGrid OccupancyGridPublisher::toRosMessage(
    const hybrid_slam::mapping::OccupancyGridSnapshot& snapshot,
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
