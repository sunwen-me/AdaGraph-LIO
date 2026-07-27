#include "ros/ground_segmentation_filter.h"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "ground_detection.hpp"

namespace LI2Sup {

void GroundSegmentationFilter::setup(rclcpp::Node& node, const std::string& prefix) {
  logger_ = node.get_logger();
  clock_ = node.get_clock();

  node.declare_parameter<bool>(prefix + ".enable", false);
  node.get_parameter(prefix + ".enable", options_.enabled);

  node.declare_parameter<double>(prefix + ".cell_size_x", options_.cell_size_x);
  node.get_parameter(prefix + ".cell_size_x", options_.cell_size_x);
  options_.cell_size_x = std::max(0.05, options_.cell_size_x);

  node.declare_parameter<double>(prefix + ".cell_size_y", options_.cell_size_y);
  node.get_parameter(prefix + ".cell_size_y", options_.cell_size_y);
  options_.cell_size_y = std::max(0.05, options_.cell_size_y);

  node.declare_parameter<double>(prefix + ".cell_size_z", options_.cell_size_z);
  node.get_parameter(prefix + ".cell_size_z", options_.cell_size_z);
  options_.cell_size_z = std::max(0.05, options_.cell_size_z);

  node.declare_parameter<double>(
      prefix + ".slope_threshold_degrees", options_.slope_threshold_degrees);
  node.get_parameter(
      prefix + ".slope_threshold_degrees", options_.slope_threshold_degrees);
  options_.slope_threshold_degrees =
      std::clamp(options_.slope_threshold_degrees, 1.0, 89.0);

  node.declare_parameter<double>(
      prefix + ".ground_inlier_threshold", options_.ground_inlier_threshold);
  node.get_parameter(
      prefix + ".ground_inlier_threshold", options_.ground_inlier_threshold);
  options_.ground_inlier_threshold =
      std::max(0.01, options_.ground_inlier_threshold);

  node.declare_parameter<double>(
      prefix + ".centroid_search_radius", options_.centroid_search_radius);
  node.get_parameter(
      prefix + ".centroid_search_radius", options_.centroid_search_radius);
  options_.centroid_search_radius =
      std::max(0.1, options_.centroid_search_radius);

  node.declare_parameter<double>(prefix + ".dist_to_ground", options_.dist_to_ground);
  node.get_parameter(prefix + ".dist_to_ground", options_.dist_to_ground);

  node.declare_parameter<bool>(
      prefix + ".auto_dist_to_ground", options_.auto_dist_to_ground);
  node.get_parameter(
      prefix + ".auto_dist_to_ground", options_.auto_dist_to_ground);

  node.declare_parameter<double>(
      prefix + ".sensor_mount_height", options_.sensor_mount_height);
  node.get_parameter(
      prefix + ".sensor_mount_height", options_.sensor_mount_height);
  options_.sensor_mount_height = std::max(0.0, options_.sensor_mount_height);

  node.declare_parameter<double>(
      prefix + ".auto_seed_radius", options_.auto_seed_radius);
  node.get_parameter(
      prefix + ".auto_seed_radius", options_.auto_seed_radius);
  options_.auto_seed_radius = std::max(0.1, options_.auto_seed_radius);

  node.declare_parameter<double>(
      prefix + ".auto_ground_quantile", options_.auto_ground_quantile);
  node.get_parameter(
      prefix + ".auto_ground_quantile", options_.auto_ground_quantile);
  options_.auto_ground_quantile =
      std::clamp(options_.auto_ground_quantile, 0.01, 0.5);

  node.declare_parameter<int>(
      prefix + ".auto_min_points", options_.auto_min_points);
  node.get_parameter(
      prefix + ".auto_min_points", options_.auto_min_points);
  options_.auto_min_points = std::max(3, options_.auto_min_points);

  node.declare_parameter<double>(
      prefix + ".max_ground_height_deviation", options_.max_ground_height_deviation);
  node.get_parameter(
      prefix + ".max_ground_height_deviation", options_.max_ground_height_deviation);
  options_.max_ground_height_deviation =
      std::max(0.01, options_.max_ground_height_deviation);

  node.declare_parameter<int>(prefix + ".processing_phase", options_.processing_phase);
  node.get_parameter(prefix + ".processing_phase", options_.processing_phase);
  options_.processing_phase = std::clamp(options_.processing_phase, 1, 2);

  node.declare_parameter<int>(prefix + ".min_input_points", options_.min_input_points);
  node.get_parameter(prefix + ".min_input_points", options_.min_input_points);
  options_.min_input_points = std::max(1, options_.min_input_points);

  if (options_.enabled) {
    RCLCPP_INFO(
        node.get_logger(),
        "Ground segmentation enabled: cell=(%.2f, %.2f, %.2f) slope=%.1f deg dist_to_ground=%.2f auto=%s phase=%d",
        options_.cell_size_x,
        options_.cell_size_y,
        options_.cell_size_z,
        options_.slope_threshold_degrees,
        options_.dist_to_ground,
        options_.auto_dist_to_ground ? "true" : "false",
        options_.processing_phase);
  }
}

GroundSegmentationFilter::Result GroundSegmentationFilter::segment(
    const BASIC::CloudPtr& body_cloud,
    const Eigen::Quaterniond& body_to_world) const {
  Result result;
  result.ground.reset(new BASIC::PointCloudType());
  result.non_ground.reset(new BASIC::PointCloudType());
  result.dist_to_ground = options_.dist_to_ground;

  if (!body_cloud || body_cloud->empty()) {
    return result;
  }

  if (!options_.enabled ||
      body_cloud->size() < static_cast<std::size_t>(options_.min_input_points)) {
    *result.non_ground = *body_cloud;
    return result;
  }

  if (options_.auto_dist_to_ground) {
    result.dist_to_ground = estimateDistToGround(body_cloud);
    result.used_auto_dist_to_ground = true;
  }

  ground_segmentation::GridConfig config;
  config.cellSizeX = options_.cell_size_x;
  config.cellSizeY = options_.cell_size_y;
  config.cellSizeZ = options_.cell_size_z;
  config.slopeThresholdDegrees = options_.slope_threshold_degrees;
  config.groundInlierThreshold = options_.ground_inlier_threshold;
  config.centroidSearchRadius = options_.centroid_search_radius;
  config.distToGround = result.dist_to_ground;
  config.maxGroundHeightDeviation = options_.max_ground_height_deviation;
  config.processing_phase = static_cast<uint16_t>(options_.processing_phase);

  try {
    ground_segmentation::PointCloudGrid<BASIC::PointType> grid(config);
    grid.setInputCloud(body_cloud, body_to_world.normalized());
    auto segmented = grid.segmentPoints();

    if (segmented.first) {
      *result.ground = *segmented.first;
    }
    if (segmented.second) {
      *result.non_ground = *segmented.second;
    }
    return result;
  } catch (const std::exception& e) {
    if (clock_) {
      RCLCPP_ERROR_THROTTLE(
          logger_, *clock_, 5000,
          "Ground segmentation failed (%s); passing full cloud as non-ground.",
          e.what());
    }
    result.ground->clear();
    *result.non_ground = *body_cloud;
    return result;
  }
}

GroundSegmentationFilter::Result GroundSegmentationFilter::segmentWorldMap(
    const BASIC::CloudPtr& world_cloud,
    double max_height_above_ground) const {
  Result result;
  result.ground.reset(new BASIC::PointCloudType());
  result.non_ground.reset(new BASIC::PointCloudType());
  result.dist_to_ground = options_.dist_to_ground;

  if (!world_cloud || world_cloud->empty()) {
    return result;
  }

  if (!options_.enabled ||
      world_cloud->size() < static_cast<std::size_t>(options_.min_input_points)) {
    *result.non_ground = *world_cloud;
    return result;
  }

  const auto cell_of = [this](const BASIC::PointType& p) {
    const auto cx =
        static_cast<std::int32_t>(std::floor(p.x / options_.cell_size_x));
    const auto cy =
        static_cast<std::int32_t>(std::floor(p.y / options_.cell_size_y));
    const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cx));
    const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(cy));
    return static_cast<std::int64_t>((ux << 32) | uy);
  };

  std::unordered_map<std::int64_t, std::vector<float>> cell_heights;
  for (const auto& point : world_cloud->points) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    cell_heights[cell_of(point)].push_back(point.z);
  }

  // Low quantile instead of the minimum: robust against sub-floor noise
  // points in the aggregated map.
  std::unordered_map<std::int64_t, float> cell_ground;
  cell_ground.reserve(cell_heights.size());
  for (auto& [key, heights] : cell_heights) {
    const auto index = static_cast<std::size_t>(
        options_.auto_ground_quantile * static_cast<double>(heights.size() - 1));
    std::nth_element(heights.begin(), heights.begin() + index, heights.end());
    cell_ground.emplace(key, heights[index]);
  }

  for (const auto& point : world_cloud->points) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const auto it = cell_ground.find(cell_of(point));
    if (it == cell_ground.end()) {
      continue;
    }

    const double height = static_cast<double>(point.z) - it->second;
    if (height > options_.ground_inlier_threshold &&
        height <= max_height_above_ground) {
      result.non_ground->push_back(point);
    } else {
      result.ground->push_back(point);
    }
  }

  return result;
}

double GroundSegmentationFilter::estimateDistToGround(
    const BASIC::CloudPtr& body_cloud) const {
  std::vector<float> z_values;
  z_values.reserve(body_cloud->size());

  const double radius2 = options_.auto_seed_radius * options_.auto_seed_radius;
  for (const auto& point : body_cloud->points) {
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    const double r2 =
        static_cast<double>(point.x) * point.x +
        static_cast<double>(point.y) * point.y;
    if (r2 <= radius2) {
      z_values.push_back(point.z);
    }
  }

  if (z_values.size() < static_cast<std::size_t>(options_.auto_min_points)) {
    // Seed region empty (self-occlusion / near-field blind zone): seed the
    // ground one mount height below the sensor instead of at the sensor.
    if (clock_) {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 10000,
          "dist_to_ground auto estimate has only %zu points within %.1f m; "
          "falling back to -sensor_mount_height (%.2f m).",
          z_values.size(), options_.auto_seed_radius,
          options_.sensor_mount_height);
    }
    return -options_.sensor_mount_height;
  }

  const auto index = static_cast<std::size_t>(
      std::clamp(options_.auto_ground_quantile, 0.0, 1.0) *
      static_cast<double>(z_values.size() - 1));
  std::nth_element(z_values.begin(), z_values.begin() + index, z_values.end());
  return z_values[index];
}

}  // namespace LI2Sup
