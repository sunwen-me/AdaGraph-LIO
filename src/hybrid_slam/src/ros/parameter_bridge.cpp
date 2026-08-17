#include "ros/parameter_bridge.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
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

PointLIORuntimeConfig ReadPointLIORuntimeConfigFromRos(rclcpp::Node& node) {
  PointLIORuntimeConfig config;

  config.save_map = ReadParameter<bool>(node, "lio.map.save_map", false);
  config.save_map_dir =
      ReadParameter<std::string>(node, "lio.map.save_map_dir", "");
  config.map_name =
      ReadParameter<std::string>(node, "lio.map.map_name", "default");
  config.map_voxel_leaf_size =
      ReadParameter<double>(node, "lio.map.ds_size", 0.5);
  config.map_endpoint_diagnostic_keyframes =
      static_cast<std::size_t>(std::max(
          0, ReadParameter<int>(
                 node, "lio.map.endpoint_diagnostic_keyframes", 0)));

  config.prior_map.root_dir = std::filesystem::current_path().string() + "/";
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

  auto& health = config.frontend_health;
  health.enabled = ReadParameter<bool>(
      node, "lio.frontend_health.enable", health.enabled);
  health.suspect_bad_frames = ReadPositiveSize(
      node, "lio.frontend_health.suspect_bad_frames",
      static_cast<int>(health.suspect_bad_frames));
  health.lost_bad_frames = ReadPositiveSize(
      node, "lio.frontend_health.lost_bad_frames",
      static_cast<int>(health.lost_bad_frames));
  health.recover_good_frames = ReadPositiveSize(
      node, "lio.frontend_health.recover_good_frames",
      static_cast<int>(health.recover_good_frames));
  health.verification_grace_frames = static_cast<std::size_t>(std::max(
      0, ReadParameter<int>(node, "lio.frontend_health.verification_grace_frames",
                            static_cast<int>(health.verification_grace_frames))));
  health.recovery_cooldown_frames = ReadPositiveSize(
      node, "lio.frontend_health.recovery_cooldown_frames",
      static_cast<int>(health.recovery_cooldown_frames));
  health.min_inlier_ratio = ReadParameter<double>(
      node, "lio.frontend_health.min_inlier_ratio", health.min_inlier_ratio);
  health.max_condition_number = ReadParameter<double>(
      node, "lio.frontend_health.max_condition_number",
      health.max_condition_number);
  health.max_timestamp_gap_s = ReadParameter<double>(
      node, "lio.frontend_health.max_timestamp_gap_s",
      health.max_timestamp_gap_s);
  health.lidar_dropout_lost_timeout_s = ReadParameter<double>(
      node, "lio.frontend_health.lidar_dropout_lost_timeout_s",
      health.lidar_dropout_lost_timeout_s);
  health.min_effective_rank = ReadPositiveSize(
      node, "lio.frontend_health.min_effective_rank",
      static_cast<int>(health.min_effective_rank));
  health.min_effective_points = ReadPositiveSize(
      node, "lio.frontend_health.min_effective_points",
      static_cast<int>(health.min_effective_points));

  config.backend.keyframe_translation_m = ReadParameter<double>(
      node, "lio.backend.keyframe_translation_m",
      config.backend.keyframe_translation_m);
  config.backend.keyframe_rotation_rad = ReadParameter<double>(
      node, "lio.backend.keyframe_rotation_rad",
      config.backend.keyframe_rotation_rad);
  config.backend.keyframe_max_frames = ReadPositiveSize(
      node, "lio.backend.keyframe_max_frames",
      static_cast<int>(config.backend.keyframe_max_frames));
  config.backend.keyframe_max_points = ReadPositiveSize(
      node, "lio.backend.keyframe_max_points",
      static_cast<int>(config.backend.keyframe_max_points));
  config.backend.submap_max_keyframes = ReadPositiveSize(
      node, "lio.backend.submap_max_keyframes",
      static_cast<int>(config.backend.submap_max_keyframes));
  config.backend.submission_queue_capacity = ReadPositiveSize(
      node, "lio.backend.submission_queue_capacity",
      static_cast<int>(config.backend.submission_queue_capacity));
  config.backend.loop_closure_huber_delta = ReadParameter<double>(
      node, "lio.backend.loop_closure_huber_delta",
      config.backend.loop_closure_huber_delta);
  config.backend.evaluation_odometry_translation_scale = ReadParameter<double>(
      node, "lio.evaluation.odometry_translation_scale",
      config.backend.evaluation_odometry_translation_scale);
  config.backend.evaluation_odometry_yaw_rad_per_meter = ReadParameter<double>(
      node, "lio.evaluation.odometry_yaw_rad_per_meter",
      config.backend.evaluation_odometry_yaw_rad_per_meter);
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
  localization.max_local_history_points = ReadPositiveSize(
      node, "lio.prior_map_localization.max_local_history_points",
      static_cast<int>(localization.max_source_points));
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
  localization.minimum_inlier_ratio = ReadParameter<double>(
      node, "lio.prior_map_localization.minimum_inlier_ratio", 0.20);
  localization.minimum_overlap = ReadParameter<double>(
      node, "lio.prior_map_localization.minimum_overlap", 0.10);
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
  kiss.min_initial_matches = ReadParameter<int>(
      node, "lio.kiss_matcher.min_initial_matches", 12);
  kiss.min_rotation_inliers = ReadParameter<int>(
      node, "lio.kiss_matcher.min_rotation_inliers", 8);
  kiss.min_final_inliers = ReadParameter<int>(
      node, "lio.kiss_matcher.min_final_inliers", 5);
  kiss.max_source_points = ReadParameter<int>(
      node, "lio.kiss_matcher.max_source_points", 30'000);
  kiss.max_target_points = ReadParameter<int>(
      node, "lio.kiss_matcher.max_target_points", 120'000);

  auto& submap_recovery = config.submap_recovery;
  config.submap_recovery_enabled = ReadParameter<bool>(
      node, "lio.submap_recovery.enable", false);
  submap_recovery.registration = localization;
  // A submap target comes from MapSnapshot, never from a map file.
  // Recovery is an initial/lost-state registration, so use the wider
  // startup gate rather than the tighter tracking gate.
  submap_recovery.registration.max_fitness =
      localization.initial_max_fitness;
  submap_recovery.registration.enabled = false;
  submap_recovery.kiss = config.prior_map.kiss_matcher;
  submap_recovery.kiss.voxel_size = static_cast<float>(ReadParameter<double>(
      node, "lio.submap_recovery.kiss_voxel_size",
      static_cast<double>(submap_recovery.kiss.voxel_size)));
  submap_recovery.kiss.min_initial_matches = ReadParameter<int>(
      node, "lio.submap_recovery.kiss_min_initial_matches",
      submap_recovery.kiss.min_initial_matches);
  submap_recovery.kiss.min_rotation_inliers = ReadParameter<int>(
      node, "lio.submap_recovery.kiss_min_rotation_inliers",
      submap_recovery.kiss.min_rotation_inliers);
  submap_recovery.kiss.min_final_inliers = ReadParameter<int>(
      node, "lio.submap_recovery.kiss_min_final_inliers",
      submap_recovery.kiss.min_final_inliers);
  submap_recovery.kiss.adaptive_ratio_test = ReadParameter<bool>(
      node, "lio.submap_recovery.kiss_adaptive_ratio_test", false);
  submap_recovery.kiss.multi_scale = ReadParameter<bool>(
      node, "lio.submap_recovery.kiss_multi_scale",
      submap_recovery.kiss.multi_scale);
  submap_recovery.kiss.coarse_voxel_scale = static_cast<float>(
      ReadParameter<double>(node, "lio.submap_recovery.kiss_coarse_voxel_scale",
                            submap_recovery.kiss.coarse_voxel_scale));
  submap_recovery.kiss.max_candidates = ReadParameter<int>(
      node, "lio.submap_recovery.kiss_max_candidates",
      submap_recovery.kiss.max_candidates);
  submap_recovery.query_window_frames = ReadPositiveSize(
      node, "lio.submap_recovery.query_window_frames", 4);
  submap_recovery.neighbor_submaps = static_cast<std::size_t>(std::max(
      0, ReadParameter<int>(node, "lio.submap_recovery.neighbor_submaps",
                            static_cast<int>(submap_recovery.neighbor_submaps))));
  submap_recovery.map_odom_prior_max_age_seconds = ReadParameter<double>(
      node, "lio.submap_recovery.map_odom_prior_max_age_seconds", 10.0);
  submap_recovery.minimum_inlier_ratio = ReadParameter<double>(
      node, "lio.submap_recovery.minimum_inlier_ratio", 0.20);
  submap_recovery.minimum_overlap = ReadParameter<double>(
      node, "lio.submap_recovery.minimum_overlap", 0.10);

  config.loop_closure_enabled = ReadParameter<bool>(
      node, "lio.loop_closure.enable", false);
  auto& loop_closure = config.loop_closure;
  loop_closure.registration = localization;
  loop_closure.registration.enabled = false;
  loop_closure.plugin = ReadParameter<std::string>(
      node, "lio.loop_closure.plugin", "scan_context");
  // Loop closure aligns completed submaps rather than a live single scan.
  // Keep its registration budget and residual gate independent from prior-map
  // localization, whose defaults are intentionally much tighter.
  loop_closure.registration.max_source_points = ReadPositiveSize(
      node, "lio.loop_closure.max_source_points", 8000);
  loop_closure.registration.max_target_points = ReadPositiveSize(
      node, "lio.loop_closure.max_target_points", 20000);
  loop_closure.registration.max_registration_iterations = ReadPositiveSize(
      node, "lio.loop_closure.max_registration_iterations", 32);
  loop_closure.registration.coarse_registration_voxel_size_m =
      ReadParameter<double>(
          node, "lio.loop_closure.coarse_registration_voxel_size_m", 0.0);
  loop_closure.registration.max_fitness = ReadParameter<double>(
      node, "lio.loop_closure.max_fitness", 4.0);
  loop_closure.maximum_candidates = ReadPositiveSize(
      node, "lio.loop_closure.maximum_candidates", 5);
  loop_closure.scan_context.ring_count = ReadPositiveSize(
      node, "lio.loop_closure.scan_context.ring_count", 20);
  loop_closure.scan_context.sector_count = ReadPositiveSize(
      node, "lio.loop_closure.scan_context.sector_count", 60);
  loop_closure.scan_context.ring_key_candidates = ReadPositiveSize(
      node, "lio.loop_closure.scan_context.ring_key_candidates", 10);
  loop_closure.scan_context.max_radius_m = ReadParameter<double>(
      node, "lio.loop_closure.scan_context.max_radius_m", 80.0);
  loop_closure.scan_context.maximum_registration_initial_translation_m =
      ReadParameter<double>(
          node,
          "lio.loop_closure.scan_context.maximum_registration_initial_translation_m",
          100.0);
  loop_closure.bevplace2.socket_path = ReadParameter<std::string>(
      node, "lio.loop_closure.bevplace2.socket_path",
      loop_closure.bevplace2.socket_path);
  loop_closure.bevplace2.request_timeout_ms = ReadParameter<int>(
      node, "lio.loop_closure.bevplace2.request_timeout_ms",
      loop_closure.bevplace2.request_timeout_ms);
  loop_closure.bevplace2.minimum_pose_inliers =
      static_cast<std::size_t>(std::max(
          3, ReadParameter<int>(
                 node, "lio.loop_closure.bevplace2.minimum_pose_inliers",
                 static_cast<int>(loop_closure.bevplace2.minimum_pose_inliers))));
  loop_closure.bevplace2.maximum_pose_residual_m = ReadParameter<double>(
      node, "lio.loop_closure.bevplace2.maximum_pose_residual_m",
      loop_closure.bevplace2.maximum_pose_residual_m);
  loop_closure.bevplace2.maximum_registration_initial_translation_m =
      ReadParameter<double>(
          node,
          "lio.loop_closure.bevplace2.maximum_registration_initial_translation_m",
          loop_closure.bevplace2.maximum_registration_initial_translation_m);
  loop_closure.bevplace2.minimum_sequence_consensus = ReadPositiveSize(
      node, "lio.loop_closure.bevplace2.minimum_sequence_consensus",
      static_cast<int>(
          loop_closure.bevplace2.minimum_sequence_consensus));
  loop_closure.bevplace2.maximum_sequence_gap_submaps = ReadPositiveSize(
      node, "lio.loop_closure.bevplace2.maximum_sequence_gap_submaps",
      static_cast<int>(
          loop_closure.bevplace2.maximum_sequence_gap_submaps));
  loop_closure.bevplace2.minimum_pose_free_sequence_consensus =
      ReadPositiveSize(
          node,
          "lio.loop_closure.bevplace2.minimum_pose_free_sequence_consensus",
          static_cast<int>(
              loop_closure.bevplace2.minimum_pose_free_sequence_consensus));
  loop_closure.bevplace2.representative_keyframes_per_submap =
      ReadPositiveSize(
          node,
          "lio.loop_closure.bevplace2.representative_keyframes_per_submap",
          static_cast<int>(
              loop_closure.bevplace2.representative_keyframes_per_submap));
  loop_closure.bevplace2.use_submap_geometry_descriptor = ReadParameter<bool>(
      node, "lio.loop_closure.bevplace2.use_submap_geometry_descriptor",
      loop_closure.bevplace2.use_submap_geometry_descriptor);
  loop_closure.neighbor_submaps = static_cast<std::size_t>(std::max(
      0, ReadParameter<int>(node, "lio.loop_closure.neighbor_submaps",
                            static_cast<int>(
                                loop_closure.neighbor_submaps))));
  loop_closure.registration_crop_radius_m = ReadParameter<double>(
      node, "lio.loop_closure.registration_crop_radius_m",
      loop_closure.registration_crop_radius_m);
  loop_closure.minimum_temporal_separation_s = ReadParameter<double>(
      node, "lio.loop_closure.minimum_temporal_separation_s", 20.0);
  loop_closure.maximum_initial_translation_m = ReadParameter<double>(
      node, "lio.loop_closure.maximum_initial_translation_m",
      loop_closure.maximum_initial_translation_m);
  loop_closure.minimum_inlier_ratio = ReadParameter<double>(
      node, "lio.loop_closure.minimum_inlier_ratio", 0.20);
  loop_closure.minimum_overlap = ReadParameter<double>(
      node, "lio.loop_closure.minimum_overlap", 0.10);
  loop_closure.minimum_source_points = ReadPositiveSize(
      node, "lio.loop_closure.minimum_source_points", 1000);
  loop_closure.minimum_target_points = ReadPositiveSize(
      node, "lio.loop_closure.minimum_target_points", 1000);
  loop_closure.minimum_bidirectional_overlap = ReadParameter<double>(
      node, "lio.loop_closure.minimum_bidirectional_overlap",
      loop_closure.minimum_bidirectional_overlap);
  loop_closure.minimum_mutual_inlier_ratio = ReadParameter<double>(
      node, "lio.loop_closure.minimum_mutual_inlier_ratio",
      loop_closure.minimum_mutual_inlier_ratio);
  loop_closure.maximum_bidirectional_inlier_rmse_m = ReadParameter<double>(
      node, "lio.loop_closure.maximum_bidirectional_inlier_rmse_m",
      loop_closure.maximum_bidirectional_inlier_rmse_m);
  loop_closure.minimum_support_extent_m = ReadParameter<double>(
      node, "lio.loop_closure.minimum_support_extent_m",
      loop_closure.minimum_support_extent_m);
  loop_closure.minimum_support_z_extent_m = ReadParameter<double>(
      node, "lio.loop_closure.minimum_support_z_extent_m",
      loop_closure.minimum_support_z_extent_m);
  loop_closure.minimum_support_eigenvalue_ratio = ReadParameter<double>(
      node, "lio.loop_closure.minimum_support_eigenvalue_ratio",
      loop_closure.minimum_support_eigenvalue_ratio);
  loop_closure.minimum_support_voxels = ReadPositiveSize(
      node, "lio.loop_closure.minimum_support_voxels",
      static_cast<int>(loop_closure.minimum_support_voxels));
  loop_closure.support_voxel_size_m = ReadParameter<double>(
      node, "lio.loop_closure.support_voxel_size_m",
      loop_closure.support_voxel_size_m);
  loop_closure.maximum_coarse_translation_disagreement_m =
      ReadParameter<double>(
          node,
          "lio.loop_closure.maximum_coarse_translation_disagreement_m",
          loop_closure.maximum_coarse_translation_disagreement_m);
  loop_closure.maximum_coarse_yaw_disagreement_rad = ReadParameter<double>(
      node, "lio.loop_closure.maximum_coarse_yaw_disagreement_rad",
      loop_closure.maximum_coarse_yaw_disagreement_rad);
  loop_closure.maximum_pose_free_graph_correction_m = ReadParameter<double>(
      node, "lio.loop_closure.maximum_pose_free_graph_correction_m",
      loop_closure.maximum_pose_free_graph_correction_m);
  loop_closure.maximum_pose_free_graph_correction_rad = ReadParameter<double>(
      node, "lio.loop_closure.maximum_pose_free_graph_correction_rad",
      loop_closure.maximum_pose_free_graph_correction_rad);
  loop_closure.maximum_relative_z_m = ReadParameter<double>(
      node, "lio.loop_closure.maximum_relative_z_m",
      loop_closure.maximum_relative_z_m);
  loop_closure.maximum_relative_roll_pitch_rad = ReadParameter<double>(
      node, "lio.loop_closure.maximum_relative_roll_pitch_rad",
      loop_closure.maximum_relative_roll_pitch_rad);
  loop_closure.maximum_solution_disagreement_m = ReadParameter<double>(
      node, "lio.loop_closure.maximum_solution_disagreement_m",
      loop_closure.maximum_solution_disagreement_m);
  loop_closure.maximum_solution_disagreement_rad = ReadParameter<double>(
      node, "lio.loop_closure.maximum_solution_disagreement_rad",
      loop_closure.maximum_solution_disagreement_rad);
  loop_closure.uniqueness_probe_translation_m = ReadParameter<double>(
      node, "lio.loop_closure.uniqueness_probe_translation_m",
      loop_closure.uniqueness_probe_translation_m);
  loop_closure.minimum_uniqueness_probe_consensus = ReadPositiveSize(
      node, "lio.loop_closure.minimum_uniqueness_probe_consensus",
      static_cast<int>(loop_closure.minimum_uniqueness_probe_consensus));
  loop_closure.translation_sigma_m = ReadParameter<double>(
      node, "lio.loop_closure.translation_sigma_m",
      loop_closure.translation_sigma_m);
  loop_closure.rotation_sigma_rad = ReadParameter<double>(
      node, "lio.loop_closure.rotation_sigma_rad",
      loop_closure.rotation_sigma_rad);
  loop_closure.debug_directory = ReadParameter<std::string>(
      node, "lio.loop_closure.debug_directory", "");
  config.prior_map.use_backend_localization_factor =
      config.backend.use_localization_factor;

  return config;
}

}  // namespace LI2Sup
