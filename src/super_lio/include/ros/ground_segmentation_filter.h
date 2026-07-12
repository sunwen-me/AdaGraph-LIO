#ifndef LI2SUP_GROUND_SEGMENTATION_FILTER_H_
#define LI2SUP_GROUND_SEGMENTATION_FILTER_H_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include <string>

#include "basic/alias.h"

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
    double auto_seed_radius = 2.0;
    double auto_ground_quantile = 0.1;
    int auto_min_points = 20;
    double max_ground_height_deviation = 0.1;
    int processing_phase = 1;
    int min_input_points = 30;
  };

  struct Result {
    BASIC::CloudPtr ground;
    BASIC::CloudPtr non_ground;
    double dist_to_ground = 0.0;
    bool used_auto_dist_to_ground = false;
  };

  void setup(rclcpp::Node& node, const std::string& prefix);

  bool enabled() const { return options_.enabled; }

  Result segment(
      const BASIC::CloudPtr& body_cloud,
      const Eigen::Quaterniond& body_to_world) const;

private:
  double estimateDistToGround(const BASIC::CloudPtr& body_cloud) const;

  Options options_;
};

}  // namespace LI2Sup

#endif
