#ifndef LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_
#define LI2SUP_OCCUPANCY_GRID_PUBLISHER_H_

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "basic/alias.h"
#include "ros/ground_segmentation_filter.h"

namespace LI2Sup {

class OccupancyGridPublisher {
public:
  OccupancyGridPublisher() = default;

  void setup(rclcpp::Node& node,
             const rclcpp::CallbackGroup::SharedPtr& callback_group);

  bool enabled() const { return enabled_; }

  // Grid cells are produced in the LIO odometry frame. In SLAM mode (no prior
  // map, map->odom not maintained) the grid must be stamped with the odom
  // frame id; only in relocation mode (prior map anchored, map->odom
  // published) does the configured map frame id apply. Call once after
  // setup(), before the first insert.
  void setFrameMode(bool prior_map_localization,
                    const std::string& odom_frame_id);

  void insertCloud(const BASIC::CloudPtr& cloud, const rclcpp::Time& stamp);
  void insertBodyCloud(const BASIC::CloudPtr& body_cloud,
                       const Eigen::Matrix4f& body_to_world,
                       const Eigen::Quaterniond& body_to_world_rotation,
                       const rclcpp::Time& stamp);
  void resetFromCloud(const BASIC::CloudPtr& cloud, const rclcpp::Time& stamp);

private:
  void publish();
  void publishDebugClouds(const GroundSegmentationFilter::Result& result,
                          const Eigen::Matrix4f& body_to_world,
                          const rclcpp::Time& stamp);
  void insertCloudLocked(const BASIC::CloudPtr& cloud, bool apply_height_filter);
  void insertScanLocked(const BASIC::CloudPtr& world_cloud,
                        const Eigen::Vector3f& sensor_origin);
  void applyScoreLocked(std::int64_t key, int delta);
  bool boundsAllowLocked(std::int32_t x, std::int32_t y) const;
  nav_msgs::msg::OccupancyGrid buildMessageLocked(const rclcpp::Time& stamp) const;
  nav_msgs::msg::OccupancyGrid buildEmptyMessageLocked(const rclcpp::Time& stamp) const;

  static std::int64_t cellKey(std::int32_t x, std::int32_t y);
  static std::int32_t keyX(std::int64_t key);
  static std::int32_t keyY(std::int64_t key);
  static void traceRayCells(std::int32_t x0, std::int32_t y0,
                            std::int32_t x1, std::int32_t y1,
                            std::unordered_set<std::int64_t>& cells);

  bool pointPassesHeightFilter(const BASIC::PointType& point) const;

private:
  // Cell evidence score: absent = unknown, score > 0 = occupied,
  // score <= 0 = free. A single hit is enough to flag an obstacle; misses
  // decay one step per scan so transient obstacles clear within a few scans
  // while persistent surfaces stay saturated.
  static constexpr int kCellScoreMax = 6;
  static constexpr int kCellScoreMin = -3;
  static constexpr int kHitScore = 3;
  static constexpr int kMissScore = 1;

  bool enabled_ = false;
  bool use_unknown_ = true;
  bool reset_on_global_map_ = true;
  double resolution_ = 0.1;
  double z_min_ = -0.3;
  double z_max_ = 2.0;
  double publish_period_ = 1.0;
  double map_padding_ = 1.0;
  double inflation_radius_ = 0.0;
  double max_range_ = 30.0;
  double max_extent_ = 400.0;
  bool raycast_enable_ = true;
  double raycast_max_ray_length_ = 15.0;
  int occupied_value_ = 100;
  int free_value_ = 0;
  int unknown_value_ = -1;
  std::string frame_id_ = "world";
  std::string topic_ = "/map";
  bool publish_ground_debug_ = false;
  std::string ground_debug_topic_ = "/lio/ground_points";
  std::string non_ground_debug_topic_ = "/lio/non_ground_points";

  mutable std::mutex mutex_;
  std::unordered_map<std::int64_t, std::int8_t> observed_cells_;
  bool has_bounds_ = false;
  bool dirty_ = false;
  std::int64_t max_extent_cells_ = 0;
  std::int32_t min_x_ = 0;
  std::int32_t max_x_ = 0;
  std::int32_t min_y_ = 0;
  std::int32_t max_y_ = 0;
  rclcpp::Time latest_stamp_;

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
