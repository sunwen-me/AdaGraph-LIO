#ifndef ADAGRAPH_BACKEND_BACKEND_CONFIG_H_
#define ADAGRAPH_BACKEND_BACKEND_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace adagraph::backend {

struct BackendConfig {
  double keyframe_translation_m = 0.5;
  double keyframe_rotation_rad = 0.1;
  std::size_t keyframe_max_frames = 30;
  std::size_t submap_max_keyframes = 100;
  std::size_t submission_queue_capacity = 16;
  double odometry_translation_sigma_m = 0.035;
  double odometry_rotation_sigma_rad = 0.012;
  bool use_localization_factor = false;
  double localization_translation_sigma_m = 0.1;
  double localization_rotation_sigma_rad = 0.02;
  double localization_fitness_reference = 0.1;
  double localization_max_time_difference_s = 0.15;

  [[nodiscard]] std::optional<std::string> validate() const {
    if (!std::isfinite(keyframe_translation_m) ||
        keyframe_translation_m <= 0.0) {
      return "backend.keyframe_translation_m must be finite and positive";
    }
    if (!std::isfinite(keyframe_rotation_rad) ||
        keyframe_rotation_rad <= 0.0) {
      return "backend.keyframe_rotation_rad must be finite and positive";
    }
    if (keyframe_max_frames == 0 || submap_max_keyframes == 0 ||
        submission_queue_capacity == 0) {
      return "backend keyframe/submap/queue capacities must be greater than zero";
    }
    if (!std::isfinite(odometry_translation_sigma_m) ||
        odometry_translation_sigma_m <= 0.0 ||
        !std::isfinite(odometry_rotation_sigma_rad) ||
        odometry_rotation_sigma_rad <= 0.0) {
      return "backend odometry sigmas must be finite and positive";
    }
    if (!std::isfinite(localization_translation_sigma_m) ||
        localization_translation_sigma_m <= 0.0 ||
        !std::isfinite(localization_rotation_sigma_rad) ||
        localization_rotation_sigma_rad <= 0.0 ||
        !std::isfinite(localization_fitness_reference) ||
        localization_fitness_reference <= 0.0 ||
        !std::isfinite(localization_max_time_difference_s) ||
        localization_max_time_difference_s < 0.0) {
      return "backend localization factor tuning must be finite and valid";
    }
    return std::nullopt;
  }
};

}  // namespace adagraph::backend

#endif  // ADAGRAPH_BACKEND_BACKEND_CONFIG_H_
