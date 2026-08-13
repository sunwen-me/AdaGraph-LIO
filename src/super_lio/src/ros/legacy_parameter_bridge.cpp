#include "ros/legacy_parameter_bridge.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include "basic/logs.h"

using namespace BASIC;

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

template <typename T>
std::size_t ReadPositiveSize(rclcpp::Node& node, const char* name,
                             T default_value) {
  return static_cast<std::size_t>(std::max<T>(
      static_cast<T>(1), ReadParameter<T>(node, name, default_value)));
}

}  // namespace

LegacyRuntimeConfig ReadLegacyRuntimeConfigFromRos(rclcpp::Node& node) {
  LegacyRuntimeConfig config;

  config.save_map = ReadParameter<bool>(node, "lio.map.save_map", false);
  config.filter_map = ReadParameter<bool>(node, "lio.map.if_filter", false);
  config.save_map_dir =
      ReadParameter<std::string>(node, "lio.map.save_map_dir", "");
  config.map_name =
      ReadParameter<std::string>(node, "lio.map.map_name", "default");
  config.map_voxel_leaf_size =
      ReadParameter<double>(node, "lio.map.ds_size", 0.5);
  config.pcd_save_interval =
      ReadParameter<int>(node, "lio.map.save_interval", 1);

#ifdef ROOT
  config.prior_map.root_dir = std::string(ROOT);
#else
  config.prior_map.root_dir = std::filesystem::current_path().string() + "/";
#endif
  if (!std::filesystem::path(config.save_map_dir).is_absolute()) {
    config.save_map_dir = config.prior_map.root_dir + config.save_map_dir;
  }
  config.prior_map.fallback_map_file =
      (std::filesystem::path(config.save_map_dir) / config.map_name).string();

  config.grid_update_step = std::max(
      1, ReadParameter<int>(node, "lio.output.grid_update_step", 1));
  config.visualization_step = std::max(
      1, ReadParameter<int>(node, "lio.output.pub_step", 0));
  config.visualize_map =
      ReadParameter<bool>(node, "lio.output.map", true);
  config.visualize_dense =
      ReadParameter<bool>(node, "lio.output.dense", false);

  config.backend.keyframe_translation_m = ReadParameter<double>(
      node, "lio.backend.keyframe_translation_m",
      config.backend.keyframe_translation_m);
  config.backend.keyframe_rotation_rad = ReadParameter<double>(
      node, "lio.backend.keyframe_rotation_rad",
      config.backend.keyframe_rotation_rad);
  config.backend.keyframe_max_frames = ReadPositiveSize(
      node, "lio.backend.keyframe_max_frames",
      static_cast<int>(config.backend.keyframe_max_frames));
  config.backend.submap_max_keyframes = ReadPositiveSize(
      node, "lio.backend.submap_max_keyframes",
      static_cast<int>(config.backend.submap_max_keyframes));
  config.backend.submission_queue_capacity = ReadPositiveSize(
      node, "lio.backend.submission_queue_capacity",
      static_cast<int>(config.backend.submission_queue_capacity));
  config.backend.use_localization_factor = ReadParameter<bool>(
      node, "lio.backend.use_localization_factor", false);
  config.backend.localization_translation_sigma_m = ReadParameter<double>(
      node, "lio.backend.loc_factor_sigma_t", 0.1);
  config.backend.localization_rotation_sigma_rad = ReadParameter<double>(
      node, "lio.backend.loc_factor_sigma_r", 0.02);
  config.backend.localization_fitness_reference = ReadParameter<double>(
      node, "lio.backend.loc_factor_fitness_ref", 0.1);
  config.backend.localization_max_time_difference_s = ReadParameter<double>(
      node, "lio.backend.loc_factor_max_time_diff", 0.15);
  config.backend_drive_map_to_odom = ReadParameter<bool>(
      node, "lio.backend.drive_map_to_odom", false);

  auto& localization = config.prior_map.relocalization;
  localization.enabled = ReadParameter<bool>(
      node, "lio.prior_map_localization.enable", false);
  localization.use_global_descriptor = ReadParameter<bool>(
      node, "lio.kiss_matcher.enable", false);
  localization.map_file = ReadParameter<std::string>(
      node, "lio.prior_map_localization.map_file", "");
  localization.local_frame_count = ReadPositiveSize(
      node, "lio.prior_map_localization.local_frames", 8);
  localization.minimum_global_search_frames = ReadPositiveSize(
      node, "lio.prior_map_localization.min_kiss_frames", 8);
  localization.minimum_frame_interval = ReadPositiveSize(
      node, "lio.prior_map_localization.min_frame_interval", 5);
  localization.source_voxel_size_m = ReadParameter<double>(
      node, "lio.prior_map_localization.local_voxel_size", 0.5);
  localization.target_voxel_size_m = ReadParameter<double>(
      node, "lio.prior_map_localization.map_voxel_size", 0.5);
  localization.crop_radius_m = ReadParameter<double>(
      node, "lio.prior_map_localization.crop_radius", 50.0);
  localization.crop_grid_cell_size_m = ReadParameter<double>(
      node, "lio.prior_map_localization.crop_grid_cell_size", 10.0);
  localization.max_source_points = ReadPositiveSize(
      node, "lio.prior_map_localization.max_source_points", 20'000);
  localization.max_target_points = ReadPositiveSize(
      node, "lio.prior_map_localization.max_target_points", 100'000);
  localization.max_registration_iterations = ReadPositiveSize(
      node, "lio.prior_map_localization.max_iterations", 64);
  localization.registration_threads = ReadPositiveSize(
      node, "lio.prior_map_localization.num_threads", 1);
  localization.max_fitness = ReadParameter<double>(
      node, "lio.prior_map_localization.max_fitness", 1.0);
  localization.initial_max_fitness = ReadParameter<double>(
      node, "lio.prior_map_localization.initial_max_fitness", 2.0);
  localization.lost_fitness = ReadParameter<double>(
      node, "lio.prior_map_localization.lost_fitness", 2.0);
  localization.max_translation_m = ReadParameter<double>(
      node, "lio.prior_map_localization.max_translation", 5.0);
  localization.max_rotation_deg = ReadParameter<double>(
      node, "lio.prior_map_localization.max_rotation_deg", 30.0);
  localization.translation_smoothing_alpha = ReadParameter<double>(
      node, "lio.prior_map_localization.smooth_translation_alpha", 1.0);
  localization.rotation_smoothing_alpha = ReadParameter<double>(
      node, "lio.prior_map_localization.smooth_rotation_alpha", 1.0);
  localization.global_retry_cooldown_s = ReadParameter<double>(
      node, "lio.prior_map_localization.kiss_retry_cooldown_sec", 5.0);
  localization.global_retry_max_backoff = ReadPositiveSize(
      node, "lio.prior_map_localization.kiss_retry_max_backoff", 6);
  localization.max_delta_rejections = ReadPositiveSize(
      node, "lio.prior_map_localization.max_delta_rejections", 10);

  auto& kiss = config.prior_map.kiss_matcher;
  kiss.enabled = ReadParameter<bool>(
      node, "lio.kiss_matcher.enable", false);
  kiss.voxel_size = static_cast<float>(ReadParameter<double>(
      node, "lio.kiss_matcher.voxel_size", 0.5));
  kiss.use_quatro = ReadParameter<bool>(
      node, "lio.kiss_matcher.use_quatro", false);
  kiss.use_voxel_sampling = ReadParameter<bool>(
      node, "lio.kiss_matcher.use_voxel_sampling", true);
  kiss.use_ratio_test = ReadParameter<bool>(
      node, "lio.kiss_matcher.use_ratio_test", true);
  kiss.adaptive_ratio_test = ReadParameter<bool>(
      node, "lio.kiss_matcher.adaptive_ratio_test", false);
  kiss.deterministic_voxel_sampling = ReadParameter<bool>(
      node, "lio.kiss_matcher.deterministic_voxel_sampling", false);
  kiss.min_final_inliers = ReadParameter<int>(
      node, "lio.kiss_matcher.min_final_inliers", 5);
  kiss.max_source_points = ReadParameter<int>(
      node, "lio.kiss_matcher.max_source_points", 30'000);
  kiss.max_target_points = ReadParameter<int>(
      node, "lio.kiss_matcher.max_target_points", 120'000);
  config.prior_map.use_backend_localization_factor =
      config.backend.use_localization_factor;

  return config;
}

}  // namespace LI2Sup
