#ifndef LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_
#define LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "adagraph/mapping/occupancy_grid.h"
#include "basic/alias.h"
#include "ros/ground_segmentation_filter.h"

namespace LI2Sup {

class OccupancyGridPublisher {
public:
  OccupancyGridPublisher() = default;
  ~OccupancyGridPublisher();

  void setup(rclcpp::Node& node,
             const rclcpp::CallbackGroup::SharedPtr& callback_group);

  bool enabled() const { return enabled_; }

  // SLAM cells live in the LIO odometry frame. Prior-map localization cells
  // live in map: the caller must transform online scans through map->odom
  // before insertion. Call once after setup(), before the first insert.
  void setFrameMode(bool prior_map_localization,
                    const std::string& odom_frame_id);

  // O(1) latest-only handoff from the backend snapshot observer. The
  // immutable snapshot is rebuilt by this adapter's worker, never by the
  // graph writer or frontend callback.
  void setMapSnapshot(
      std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept;

  // Clear all derived occupancy state and cancel an in-flight snapshot
  // rebuild. The publisher/subscriptions stay installed for the next
  // lifecycle session.
  void resetSession();

  void insertCloud(const BASIC::CloudPtr& cloud, const rclcpp::Time& stamp);
  void insertBodyCloud(const BASIC::CloudPtr& body_cloud,
                       const Eigen::Matrix4f& body_to_world,
                       const Eigen::Quaterniond& body_to_world_rotation,
                       const rclcpp::Time& stamp);
  void resetFromCloud(const BASIC::CloudPtr& cloud, const rclcpp::Time& stamp);

  enum class SourcePolicy : std::uint8_t {
    kOnline = 0,
    kSnapshot,
  };

private:
  void publish();
  void publishDebugClouds(const GroundSegmentationFilter::Result& result,
                          const Eigen::Matrix4f& body_to_world,
                          const rclcpp::Time& stamp);
  static adagraph::sensor::PointCloudView toView(
      const BASIC::CloudPtr& cloud);
  static nav_msgs::msg::OccupancyGrid toRosMessage(
      const adagraph::mapping::OccupancyGridSnapshot& snapshot,
      const std::string& frame_id,
      const rclcpp::Clock::SharedPtr& clock);
  void materializeSnapshotLocked();
  void publishLatestSnapshot();
  void startSnapshotWorker();
  void stopSnapshotWorker();
  void snapshotWorkerLoop();

private:
  bool enabled_ = false;
  bool reset_on_global_map_ = true;
  double publish_period_ = 1.0;
  SourcePolicy source_policy_ = SourcePolicy::kSnapshot;
  bool prior_map_localization_ = false;
  adagraph::mapping::OccupancyGridUpdatePolicy update_policy_ =
      adagraph::mapping::OccupancyGridUpdatePolicy::kLazy;
  adagraph::mapping::OccupancyGridConfig occupancy_config_;
  std::unique_ptr<adagraph::mapping::OccupancyGridBuilder> builder_;
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
  std::atomic<std::shared_ptr<const adagraph::mapping::OccupancyGridSnapshot>>
      latest_snapshot_{nullptr};
  std::atomic<std::uint64_t> published_snapshot_version_{0};

  std::atomic<std::shared_ptr<const adagraph::mapping::MapSnapshot>>
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
