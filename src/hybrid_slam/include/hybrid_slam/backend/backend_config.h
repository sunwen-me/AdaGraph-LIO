#ifndef HYBRID_SLAM_BACKEND_BACKEND_CONFIG_H_
#define HYBRID_SLAM_BACKEND_BACKEND_CONFIG_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>

namespace hybrid_slam::backend {

struct BackendConfig {
  double keyframe_translation_m = 0.5;
  double keyframe_rotation_rad = 0.1;
  std::size_t keyframe_max_frames = 30;
  std::size_t keyframe_max_points = 50'000;
  // Voxel edge length applied to a keyframe cloud before it enters the graph.
  // Retained keyframe clouds are the dominant session memory cost: nothing
  // evicts them, and every published snapshot keeps them alive for the whole
  // run. Every consumer already works at a coarser scale than the raw matching
  // points - the saved PCD is voxelized at lio.map.ds_size, prior-map VGICP at
  // lio.prior_map_localization.local_voxel_size, and loop verification
  // re-voxelizes its own inputs - so filtering once here bounds the footprint
  // without changing what those stages actually see. Zero disables filtering
  // and restores the previous unfiltered behaviour.
  double keyframe_cloud_voxel_size_m = 0.1;
  std::size_t submap_max_keyframes = 100;
  std::size_t submission_queue_capacity = 16;
  double odometry_translation_sigma_m = 0.035;
  double odometry_rotation_sigma_rad = 0.012;
  // Strictly geometry-verified loop factors may initially be several standard
  // deviations from a drifted graph.  Give them a wider quadratic Huber basin
  // than ordinary relative factors without removing robust protection.
  double loop_closure_huber_delta = 3.0;
  // Evaluation-only perturbation of relative odometry factors. These values
  // never alter frontend poses or sensor input; defaults preserve production
  // behavior exactly.
  double evaluation_odometry_translation_scale = 1.0;
  double evaluation_odometry_yaw_rad_per_meter = 0.0;
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
    if (keyframe_max_frames == 0 || keyframe_max_points == 0 ||
        submap_max_keyframes == 0 ||
        submission_queue_capacity == 0) {
      return "backend keyframe/submap/queue capacities must be greater than zero";
    }
    if (!std::isfinite(keyframe_cloud_voxel_size_m) ||
        keyframe_cloud_voxel_size_m < 0.0) {
      return "backend.keyframe_cloud_voxel_size_m must be finite and "
             "non-negative";
    }
    if (!std::isfinite(odometry_translation_sigma_m) ||
        odometry_translation_sigma_m <= 0.0 ||
        !std::isfinite(odometry_rotation_sigma_rad) ||
        odometry_rotation_sigma_rad <= 0.0 ||
        !std::isfinite(loop_closure_huber_delta) ||
        loop_closure_huber_delta <= 0.0) {
      return "backend relative-factor noise settings must be finite and positive";
    }
    if (!std::isfinite(evaluation_odometry_translation_scale) ||
        evaluation_odometry_translation_scale <= 0.0 ||
        !std::isfinite(evaluation_odometry_yaw_rad_per_meter)) {
      return "backend evaluation odometry drift must be finite and valid";
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

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_BACKEND_CONFIG_H_
