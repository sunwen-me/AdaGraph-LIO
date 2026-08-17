#ifndef LI2SUP_SYSTEM_POINT_LIO_RUNTIME_CONFIG_H_
#define LI2SUP_SYSTEM_POINT_LIO_RUNTIME_CONFIG_H_

#include <cmath>
#include <optional>
#include <string>

#include "hybrid_slam/backend/backend_config.h"
#include "hybrid_slam/frontend/frontend_health.h"
#include "hybrid_slam/localization/relocalization_config.h"
#include "lio/kiss_matcher_relocalizer.h"
#include "lio/pcl_loop_closure.h"
#include "lio/pcl_submap_relocalizer.h"

namespace LI2Sup {

// Explicit values consumed by the Point-LIO downstream composition. Standard
// applications pass this snapshot so runtime processing remains instance-owned.
struct PriorMapConfig {
  hybrid_slam::localization::RelocalizationConfig relocalization;
  KISSMatcherRelocalizer::Config kiss_matcher;
  bool use_backend_localization_factor = false;
  std::string root_dir;
  std::string fallback_map_file;

  [[nodiscard]] std::optional<std::string> validate() const {
    auto relocalization_copy = relocalization;
    if (relocalization_copy.enabled && relocalization_copy.map_file.empty()) {
      relocalization_copy.map_file = fallback_map_file;
    }
    if (relocalization_copy.enabled && relocalization_copy.map_file.empty()) {
      return "runtime prior-map file is required when localization is enabled";
    }
    if (const auto error = relocalization_copy.validate()) {
      return error;
    }
    return std::nullopt;
  }
};

struct PointLIORuntimeConfig {
  bool save_map = false;
  std::string save_map_dir;
  std::string map_name = "default";
  double map_voxel_leaf_size = 0.5;
  std::size_t map_endpoint_diagnostic_keyframes = 0;

  int grid_update_step = 1;
  int visualization_step = 1;
  bool visualize_map = true;

  hybrid_slam::backend::BackendConfig backend;
  hybrid_slam::frontend::FrontendHealthConfig frontend_health;
  bool submap_recovery_enabled = false;
  PclSubmapRelocalizationConfig submap_recovery;
  bool loop_closure_enabled = false;
  PclLoopClosureConfig loop_closure;
  bool backend_drive_map_to_odom = false;
  PriorMapConfig prior_map;

  [[nodiscard]] std::optional<std::string> validate() const {
    if (save_map_dir.empty() && save_map) {
      return "runtime save_map_dir is required when map saving is enabled";
    }
    if (map_name.empty()) {
      return "runtime map_name must not be empty";
    }
    if (grid_update_step < 1 || visualization_step < 1) {
      return "runtime update/publish steps must be positive";
    }
    if (!std::isfinite(map_voxel_leaf_size) || map_voxel_leaf_size < 0.0) {
      return "runtime map_voxel_leaf_size must be finite and non-negative";
    }
    if (const auto error = backend.validate()) {
      return error;
    }
    if (const auto error = frontend_health.validate()) {
      return error;
    }
    if (submap_recovery_enabled) {
      if (const auto error = submap_recovery.validate()) {
        return error;
      }
    }
    if (loop_closure_enabled) {
      if (const auto error = loop_closure.validate()) {
        return error;
      }
    }
    if (const auto error = prior_map.validate()) {
      return error;
    }
    return std::nullopt;
  }
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_POINT_LIO_RUNTIME_CONFIG_H_
