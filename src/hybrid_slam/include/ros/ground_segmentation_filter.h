#ifndef LI2SUP_GROUND_SEGMENTATION_FILTER_H_
#define LI2SUP_GROUND_SEGMENTATION_FILTER_H_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "hybrid_slam/adapter/pcl_types.h"

namespace LI2Sup {

class GroundSegmentationFilter {
public:
  struct Options {
    bool enabled = false;
    double cell_size_x = 1.0;
    double cell_size_y = 1.0;
    double cell_size_z = 10.0;
    double slope_threshold_degrees = 20.0;
    double ground_inlier_threshold = 0.125;
    double centroid_search_radius = 5.0;
    double dist_to_ground = 0.0;
    bool auto_dist_to_ground = false;
    // Height of the sensor/body origin above ground; fallback ground seed
    // when the auto dist_to_ground estimate has too few points.
    double sensor_mount_height = 0.5;
    double auto_seed_radius = 2.0;
    double auto_ground_quantile = 0.1;
    int auto_min_points = 20;
    double max_ground_height_deviation = 0.1;
    int processing_phase = 1;
    int min_input_points = 30;
  };

  struct Result {
    hybrid_slam::adapter::PclCloudPtr ground;
    hybrid_slam::adapter::PclCloudPtr non_ground;
    double dist_to_ground = 0.0;
    bool used_auto_dist_to_ground = false;
  };

  void setup(rclcpp::Node& node, const std::string& prefix);

  bool enabled() const { return options_.enabled; }

  // Height of the sensor/body origin above ground; converts sensor-relative
  // z limits into heights above ground.
  double sensorMountHeight() const { return options_.sensor_mount_height; }

  Result segment(
      const hybrid_slam::adapter::PclCloudPtr& body_cloud,
      const Eigen::Quaterniond& body_to_world) const;

  // Ground removal for map-frame clouds (prior maps): per-XY-cell
  // low-quantile ground height plus a clearance test. segment() requires a
  // sensor-centric cloud and cannot be applied to a global map. Points more
  // than max_height_above_ground above the local ground estimate (overhead
  // structure) are returned in the ground bucket, i.e. as non-obstacles.
  Result segmentWorldMap(
      const hybrid_slam::adapter::PclCloudPtr& world_cloud,
      double max_height_above_ground) const;

private:
  double estimateDistToGround(const hybrid_slam::adapter::PclCloudPtr& body_cloud) const;

  Options options_;
  rclcpp::Logger logger_ = rclcpp::get_logger("ground_segmentation_filter");
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace LI2Sup

#endif
