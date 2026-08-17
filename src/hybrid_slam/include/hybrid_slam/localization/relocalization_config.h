#ifndef HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_CONFIG_H_
#define HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace hybrid_slam::localization {

struct RelocalizationConfig {
  bool enabled = false;
  bool use_global_descriptor = true;
  std::string map_file;
  std::size_t local_frame_count = 10;
  std::size_t minimum_global_search_frames = 3;
  std::size_t minimum_frame_interval = 1;
  double source_voxel_size_m = 0.5;
  double target_voxel_size_m = 0.5;
  double crop_radius_m = 50.0;
  double crop_grid_cell_size_m = 10.0;
  std::size_t max_source_points = 20'000;
  // Bounded retained input for the local registration/KISS history. The
  // final registration source remains separately limited by max_source_points.
  std::size_t max_local_history_points = 20'000;
  std::size_t max_target_points = 100'000;
  std::size_t max_registration_iterations = 64;
  // Zero keeps the single-resolution registration path.  A positive value
  // enables a coarse FastVGICP pass before fine registration; it is useful
  // for descriptor-selected loop candidates whose graph pose has drifted.
  double coarse_registration_voxel_size_m = 0.0;
  std::size_t registration_threads = 1;
  double max_fitness = 1.0;
  double initial_max_fitness = 2.0;
  double lost_fitness = 2.0;
  double minimum_inlier_ratio = 0.20;
  double minimum_overlap = 0.10;
  double max_translation_m = 5.0;
  double max_rotation_deg = 30.0;
  double translation_smoothing_alpha = 1.0;
  double rotation_smoothing_alpha = 1.0;
  double global_retry_cooldown_s = 5.0;
  std::size_t global_retry_max_backoff = 6;
  std::size_t max_delta_rejections = 10;

  [[nodiscard]] std::optional<std::string> validate() const {
    if (!std::isfinite(source_voxel_size_m) ||
        source_voxel_size_m <= 0.0 ||
        !std::isfinite(target_voxel_size_m) ||
        target_voxel_size_m <= 0.0) {
      return "relocalization voxel sizes must be finite and positive";
    }
    if (enabled && map_file.empty()) {
      return "relocalization.map_file is required when enabled";
    }
    if (local_frame_count == 0 || minimum_global_search_frames == 0 ||
        minimum_frame_interval == 0) {
      return "relocalization frame counts and intervals must be positive";
    }
    if (!std::isfinite(crop_radius_m) || crop_radius_m <= 0.0 ||
        !std::isfinite(crop_grid_cell_size_m) ||
        crop_grid_cell_size_m <= 0.0) {
      return "relocalization crop sizes must be finite and positive";
    }
    if (max_source_points == 0 || max_local_history_points == 0 ||
        max_target_points == 0) {
      return "relocalization point limits must be greater than zero";
    }
    if (max_registration_iterations == 0 || registration_threads == 0) {
      return "relocalization registration iterations/threads must be positive";
    }
    if (!std::isfinite(coarse_registration_voxel_size_m) ||
        coarse_registration_voxel_size_m < 0.0) {
      return "coarse registration voxel size must be finite and non-negative";
    }
    if (!std::isfinite(max_fitness) || max_fitness <= 0.0 ||
        !std::isfinite(initial_max_fitness) || initial_max_fitness <= 0.0 ||
        !std::isfinite(lost_fitness) || lost_fitness <= 0.0) {
      return "relocalization fitness gates must be finite and positive";
    }
    if (!std::isfinite(minimum_inlier_ratio) || minimum_inlier_ratio <= 0.0 ||
        minimum_inlier_ratio > 1.0 || !std::isfinite(minimum_overlap) ||
        minimum_overlap <= 0.0 || minimum_overlap > 1.0) {
      return "relocalization geometry ratios must be within (0, 1]";
    }
    if (!std::isfinite(max_translation_m) || max_translation_m <= 0.0 ||
        !std::isfinite(max_rotation_deg) || max_rotation_deg <= 0.0) {
      return "relocalization jump gates must be finite and positive";
    }
    if (!std::isfinite(translation_smoothing_alpha) ||
        translation_smoothing_alpha < 0.0 ||
        translation_smoothing_alpha > 1.0 ||
        !std::isfinite(rotation_smoothing_alpha) ||
        rotation_smoothing_alpha < 0.0 || rotation_smoothing_alpha > 1.0) {
      return "relocalization smoothing alpha must be within [0, 1]";
    }
    if (!std::isfinite(global_retry_cooldown_s) ||
        global_retry_cooldown_s < 0.0 || global_retry_max_backoff == 0 ||
        max_delta_rejections == 0) {
      return "relocalization retry policy must be finite and positive";
    }
    return std::nullopt;
  }
};

}  // namespace hybrid_slam::localization

#endif  // HYBRID_SLAM_LOCALIZATION_RELOCALIZATION_CONFIG_H_
