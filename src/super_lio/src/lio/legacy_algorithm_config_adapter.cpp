#include "lio/adagraph_backend.h"
#include "lio/params.h"
#include "lio/prior_map_localization.h"

#include <filesystem>
#include <utility>

namespace LI2Sup {

void AdaGraphBackend::init() {
  if (!has_explicit_config_) {
    config_.use_localization_factor = g_backend_use_localization_factor;
    config_.localization_translation_sigma_m =
        g_backend_loc_factor_sigma_t;
    config_.localization_rotation_sigma_rad = g_backend_loc_factor_sigma_r;
    config_.localization_fitness_reference =
        g_backend_loc_factor_fitness_ref;
    config_.localization_max_time_difference_s =
        g_backend_loc_factor_max_time_diff;
  }
  initWithConfig();
}

bool PriorMapLocalization::init(
    PriorMapCallback prior_map_callback,
    adagraph::geometry::StaticTransform3d T_base_imu,
    ResultCallback result_callback) {
  adagraph::localization::RelocalizationConfig config;
  config.enabled = g_prior_map_loc_enable;
  config.use_global_descriptor = g_kiss_matcher_enable;
  config.map_file = g_prior_map_loc_map_file;
  config.local_frame_count = g_prior_map_loc_local_frames > 0
      ? static_cast<std::size_t>(g_prior_map_loc_local_frames) : 0;
  config.minimum_global_search_frames = g_prior_map_loc_min_kiss_frames > 0
      ? static_cast<std::size_t>(g_prior_map_loc_min_kiss_frames) : 0;
  config.minimum_frame_interval = g_prior_map_loc_min_frame_interval > 0
      ? static_cast<std::size_t>(g_prior_map_loc_min_frame_interval) : 0;
  config.source_voxel_size_m = g_prior_map_loc_local_voxel_size;
  config.target_voxel_size_m = g_prior_map_loc_map_voxel_size;
  config.crop_radius_m = g_prior_map_loc_crop_radius;
  config.crop_grid_cell_size_m = g_prior_map_loc_crop_grid_cell_size;
  config.max_source_points = g_prior_map_loc_max_source_points > 0
      ? static_cast<std::size_t>(g_prior_map_loc_max_source_points) : 0;
  config.max_target_points = g_prior_map_loc_max_target_points > 0
      ? static_cast<std::size_t>(g_prior_map_loc_max_target_points) : 0;
  config.max_registration_iterations = g_prior_map_loc_max_iterations > 0
      ? static_cast<std::size_t>(g_prior_map_loc_max_iterations) : 0;
  config.registration_threads = g_prior_map_loc_num_threads > 0
      ? static_cast<std::size_t>(g_prior_map_loc_num_threads) : 0;
  config.max_fitness = g_prior_map_loc_max_fitness;
  config.initial_max_fitness = g_prior_map_loc_initial_max_fitness;
  config.lost_fitness = g_prior_map_loc_lost_fitness;
  config.max_translation_m = g_prior_map_loc_max_translation;
  config.max_rotation_deg = g_prior_map_loc_max_rotation_deg;
  config.translation_smoothing_alpha =
      g_prior_map_loc_smooth_translation_alpha;
  config.rotation_smoothing_alpha = g_prior_map_loc_smooth_rotation_alpha;
  config.global_retry_cooldown_s = g_prior_map_loc_kiss_retry_cooldown_sec;
  config.global_retry_max_backoff = g_prior_map_loc_kiss_retry_max_backoff > 0
      ? static_cast<std::size_t>(g_prior_map_loc_kiss_retry_max_backoff) : 0;
  config.max_delta_rejections = g_prior_map_loc_max_delta_rejections > 0
      ? static_cast<std::size_t>(g_prior_map_loc_max_delta_rejections) : 0;

  KISSMatcherRelocalizer::Config kiss_config;
  kiss_config.enabled = g_kiss_matcher_enable;
  kiss_config.voxel_size = g_kiss_matcher_voxel_size;
  kiss_config.use_quatro = g_kiss_matcher_use_quatro;
  kiss_config.use_voxel_sampling = g_kiss_matcher_use_voxel_sampling;
  kiss_config.use_ratio_test = g_kiss_matcher_use_ratio_test;
  kiss_config.adaptive_ratio_test = g_kiss_matcher_adaptive_ratio_test;
  kiss_config.deterministic_voxel_sampling =
      g_kiss_matcher_deterministic_voxel_sampling;
  kiss_config.min_final_inliers = g_kiss_matcher_min_final_inliers;
  kiss_config.max_source_points = g_kiss_matcher_max_source_points;
  kiss_config.max_target_points = g_kiss_matcher_max_target_points;
  return init(std::move(prior_map_callback), std::move(T_base_imu),
              std::move(result_callback), std::move(config),
              std::move(kiss_config), g_backend_use_localization_factor,
              g_root_dir,
              (std::filesystem::path(g_save_map_dir) / g_map_name).string());
}

}  // namespace LI2Sup
