#ifndef LI2SUP_SYSTEM_LEGACY_RUNTIME_CONFIG_H_
#define LI2SUP_SYSTEM_LEGACY_RUNTIME_CONFIG_H_

#include <cmath>
#include <optional>
#include <string>

#include "adagraph/backend/backend_config.h"
#include "adagraph/localization/relocalization_config.h"
#include "lio/kiss_matcher_relocalizer.h"

namespace LI2Sup {

// Explicit values consumed by the transitional downstream composition.  The
// compatibility ROS parameter loader can still populate the legacy globals,
// but standard applications pass this snapshot so runtime processing does not
// read process-wide configuration.
struct LegacyPriorMapConfig {
  adagraph::localization::RelocalizationConfig relocalization;
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

struct LegacyRuntimeConfig {
  bool save_map = false;
  bool filter_map = false;
  std::string save_map_dir;
  std::string map_name = "default";
  double map_voxel_leaf_size = 0.5;
  int pcd_save_interval = 1;

  int grid_update_step = 1;
  int visualization_step = 1;
  bool visualize_map = true;
  bool visualize_dense = false;

  adagraph::backend::BackendConfig backend;
  bool backend_drive_map_to_odom = false;
  LegacyPriorMapConfig prior_map;

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
    if (const auto error = prior_map.validate()) {
      return error;
    }
    return std::nullopt;
  }
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_LEGACY_RUNTIME_CONFIG_H_
