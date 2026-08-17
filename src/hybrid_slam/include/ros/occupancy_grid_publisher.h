#ifndef LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_
#define LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_

#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hybrid_slam/mapping/occupancy_grid.h"
#include "hybrid_slam/io/map_io.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "ros/ground_segmentation_filter.h"

namespace LI2Sup {

// Latencies are reported in microseconds. Percentiles are upper bounds of
// 1-us buckets through 1 ms, then logarithmic overflow buckets.
struct OccupancyLatencySummary {
  std::uint64_t sample_count = 0;
  double mean_us = 0.0;
  std::uint64_t p50_us = 0;
  std::uint64_t p95_us = 0;
  std::uint64_t p99_us = 0;
  std::uint64_t max_us = 0;
};

// Online-path observability only. Update lock timings describe the builder
// mutation critical section; publish timings describe snapshot materializing
// under the same mutex. End-to-end timings include work outside that lock.
struct OnlineOccupancyTimingStats {
  OccupancyLatencySummary update_lock_wait;
  OccupancyLatencySummary update_lock_hold;
  OccupancyLatencySummary update_end_to_end;
  OccupancyLatencySummary publish_lock_wait;
  OccupancyLatencySummary publish_lock_hold;
  OccupancyLatencySummary publish_end_to_end;
};

class OccupancyGridPublisher {
public:
  OccupancyGridPublisher() = default;
  ~OccupancyGridPublisher();

  void setup(rclcpp::Node& node,
             const rclcpp::CallbackGroup::SharedPtr& callback_group);

  bool enabled() const { return enabled_; }
  [[nodiscard]] OnlineOccupancyTimingStats onlineTimingStats() const noexcept;

  // SLAM cells live in the LIO odometry frame. Prior-map localization cells
  // live in map: the caller must transform online scans through map->odom
  // before insertion. Call once after setup(), before the first insert.
  void setFrameMode(bool prior_map_localization,
                    const std::string& odom_frame_id);

  // O(1) latest-only handoff from the backend snapshot observer. The
  // immutable snapshot is rebuilt by this adapter's worker, never by the
  // graph writer or frontend callback.
  void setMapSnapshot(
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) noexcept;

  // Clear all derived occupancy state and cancel an in-flight snapshot
  // rebuild. The publisher/subscriptions stay installed for the next
  // lifecycle session.
  void resetSession();

  void insertCloud(const hybrid_slam::sensor::ImmutablePointCloud& cloud,
                   const rclcpp::Time& stamp);
  void insertBodyCloud(const hybrid_slam::sensor::ImmutablePointCloud& body_cloud,
                       const Eigen::Matrix4f& body_to_world,
                       const Eigen::Quaterniond& body_to_world_rotation,
                       const rclcpp::Time& stamp);
  void resetFromCloud(
      const hybrid_slam::sensor::ImmutablePointCloud& cloud,
      const rclcpp::Time& stamp);

  enum class SourcePolicy : std::uint8_t {
    kOnline = 0,
    kSnapshot,
  };

private:
  struct TimingHistogram {
    static constexpr std::size_t kLinearBuckets = 1000;
    static constexpr std::size_t kOverflowBuckets = 32;
    static constexpr std::size_t kBucketCount =
        kLinearBuckets + kOverflowBuckets;

    TimingHistogram() = default;
    void record(std::chrono::nanoseconds duration) noexcept;
    [[nodiscard]] OccupancyLatencySummary snapshot() const noexcept;
    void reset() noexcept;

    std::array<std::atomic<std::uint64_t>, kBucketCount> buckets{};
    std::atomic<std::uint64_t> count{0};
    std::atomic<std::uint64_t> total_nanoseconds{0};
    std::atomic<std::uint64_t> max_nanoseconds{0};
  };

  void resetOnlineTimingStats() noexcept;
  void publish();
  void publishDebugClouds(const GroundSegmentationFilter::Result& result,
                          const Eigen::Matrix4f& body_to_world,
                          const rclcpp::Time& stamp);
  static nav_msgs::msg::OccupancyGrid toRosMessage(
      const hybrid_slam::mapping::OccupancyGridSnapshot& snapshot,
      const std::string& frame_id,
      const rclcpp::Clock::SharedPtr& clock);
  void materializeSnapshotLocked();
  void publishLatestSnapshot();
  void startSnapshotWorker();
  void stopSnapshotWorker();
  void snapshotWorkerLoop();
  struct PendingGridArtifact {
    std::uint64_t request_id = 0;
    std::shared_ptr<const hybrid_slam::mapping::OccupancyGridSnapshot> grid;
    std::string frame_id;
    int occupied_value = 100;
    int free_value = 0;
    int unknown_value = -1;
  };
  void enqueueGridArtifact(
      std::shared_ptr<const hybrid_slam::mapping::OccupancyGridSnapshot> grid,
      const std::string& frame_id, bool force);
  void startGridArtifactWorker();
  void stopGridArtifactWorker();
  void gridArtifactWorkerLoop();

private:
  bool enabled_ = false;
  bool reset_on_global_map_ = true;
  double publish_period_ = 1.0;
  SourcePolicy source_policy_ = SourcePolicy::kSnapshot;
  bool prior_map_localization_ = false;
  hybrid_slam::mapping::OccupancyGridUpdatePolicy update_policy_ =
      hybrid_slam::mapping::OccupancyGridUpdatePolicy::kLazy;
  hybrid_slam::mapping::OccupancyGridConfig occupancy_config_;
  std::unique_ptr<hybrid_slam::mapping::OccupancyGridBuilder> builder_;
  std::string frame_id_ = "world";
  std::string configured_frame_id_ = "world";
  std::string topic_ = "/map";
  bool publish_ground_debug_ = false;
  std::string ground_debug_topic_ = "/lio/ground_points";
  std::string non_ground_debug_topic_ = "/lio/non_ground_points";

  mutable std::mutex mutex_;
  bool dirty_ = false;
  rclcpp::Time latest_stamp_;
  std::uint64_t next_snapshot_version_ = 1;
  std::atomic<std::shared_ptr<const hybrid_slam::mapping::OccupancyGridSnapshot>>
      latest_snapshot_{nullptr};
  std::atomic<std::uint64_t> published_snapshot_version_{0};

  TimingHistogram online_update_lock_wait_;
  TimingHistogram online_update_lock_hold_;
  TimingHistogram online_update_end_to_end_;
  TimingHistogram online_publish_lock_wait_;
  TimingHistogram online_publish_lock_hold_;
  TimingHistogram online_publish_end_to_end_;

  std::atomic<std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>>
      pending_map_snapshot_{nullptr};
  std::mutex snapshot_worker_mutex_;
  std::condition_variable snapshot_worker_cv_;
  std::thread snapshot_worker_;
  bool snapshot_worker_stop_ = false;
  bool snapshot_worker_started_ = false;
  std::uint64_t rebuilt_map_snapshot_version_ = 0;
  std::atomic<bool> snapshot_worker_stop_requested_{false};
  // Monotonic handoff generation.  A worker captures the generation for its
  // active snapshot and cooperatively cancels when a newer handoff arrives.
  // This avoids the lost-cancel race of a shared boolean reset by the worker.
  std::atomic<std::uint64_t> snapshot_request_generation_{0};
  std::atomic<std::uint64_t> snapshot_handoffs_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_requested_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_completed_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_cancelled_superseded_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_cancelled_shutdown_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_discarded_obsolete_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_budget_exceeded_{0};
  std::atomic<std::uint64_t> snapshot_rebuild_failed_{0};
  std::atomic<std::size_t> snapshot_rebuild_peak_scratch_bytes_{0};

  // Only online occupancy retains per-scan origins and ray-carved free-space
  // history, so only that product is eligible for a durable Nav2 map artifact.
  bool grid_artifact_enabled_ = false;
  std::filesystem::path grid_artifact_yaml_path_;
  double grid_artifact_period_s_ = 5.0;
  std::mutex grid_artifact_mutex_;
  std::condition_variable grid_artifact_cv_;
  std::shared_ptr<const PendingGridArtifact> pending_grid_artifact_;
  std::thread grid_artifact_worker_;
  bool grid_artifact_worker_stop_ = false;
  bool grid_artifact_worker_started_ = false;
  std::chrono::steady_clock::time_point next_grid_artifact_submission_{};
  std::uint64_t next_grid_artifact_request_id_ = 1;
  std::atomic<std::uint64_t> grid_artifact_written_version_{0};
  std::atomic<std::uint64_t> grid_artifact_submitted_{0};
  std::atomic<std::uint64_t> grid_artifact_completed_{0};
  std::atomic<std::uint64_t> grid_artifact_failed_{0};
  std::atomic<std::uint64_t> grid_artifact_superseded_{0};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_debug_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_ = rclcpp::get_logger("occupancy_grid_publisher");

  GroundSegmentationFilter ground_filter_;
};

}  // namespace LI2Sup

#endif
