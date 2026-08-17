#include "hybrid_slam/mapping/occupancy_grid.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace hybrid_slam::mapping {

std::optional<std::string> OccupancyGridConfig::validate() const {
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    return "resolution must be finite and positive";
  }
  if (!std::isfinite(z_min) || !std::isfinite(z_max) || z_min > z_max) {
    return "z_min/z_max must be finite and ordered";
  }
  if (!std::isfinite(map_padding) || map_padding < 0.0 ||
      !std::isfinite(inflation_radius) || inflation_radius < 0.0) {
    return "map padding and inflation radius must be finite and non-negative";
  }
  if (!std::isfinite(max_range) || max_range < resolution ||
      !std::isfinite(max_extent) || max_extent < resolution ||
      !std::isfinite(raycast_max_ray_length) ||
      raycast_max_ray_length < resolution) {
    return "range, extent and ray length must be finite and at least one cell";
  }
  if (occupied_value < 0 || occupied_value > 100 || free_value < 0 ||
      free_value > 100 || unknown_value < -1 || unknown_value > 100) {
    return "occupancy values are outside their valid ranges";
  }
  if (rebuild_scratch_memory_bytes < 1024U * 1024U ||
      rebuild_max_points == 0 ||
      rebuild_cancellation_check_interval == 0 ||
      max_materialized_cells == 0) {
    return "occupancy rebuild/materialization budgets must be positive and "
           "the scratch budget must be at least 1 MiB";
  }
  return std::nullopt;
}

OccupancyGridBuilder::OccupancyGridBuilder(OccupancyGridConfig config)
    : config_(std::move(config)) {
  if (const auto error = config_.validate()) {
    return;
  }
  max_extent_cells_ = static_cast<std::int64_t>(std::floor(
      config_.max_extent / config_.resolution));
  valid_ = max_extent_cells_ >= 1;
}

void OccupancyGridBuilder::reset() noexcept {
  observed_cells_.clear();
  has_bounds_ = false;
  min_x_ = max_x_ = min_y_ = max_y_ = 0;
  dropped_cell_updates_ = 0;
}

bool OccupancyGridBuilder::toCellIndex(
    double meters, std::int32_t& cell) const noexcept {
  const double scaled = std::floor(meters / config_.resolution);
  if (!(scaled >= -2.0e9 && scaled <= 2.0e9)) {
    return false;
  }
  cell = static_cast<std::int32_t>(scaled);
  return true;
}

bool OccupancyGridBuilder::offsetCell(
    std::int32_t cell, int offset, std::int32_t& result) const noexcept {
  const auto candidate = static_cast<std::int64_t>(cell) + offset;
  if (candidate < std::numeric_limits<std::int32_t>::min() ||
      candidate > std::numeric_limits<std::int32_t>::max()) {
    return false;
  }
  result = static_cast<std::int32_t>(candidate);
  return true;
}

bool OccupancyGridBuilder::pointPassesHeightFilter(
    const std::array<float, 3>& point) const noexcept {
  return std::isfinite(point[0]) && std::isfinite(point[1]) &&
         std::isfinite(point[2]) && point[2] >= config_.z_min &&
         point[2] <= config_.z_max;
}

bool OccupancyGridBuilder::boundsAllow(
    std::int32_t x, std::int32_t y) const noexcept {
  if (!has_bounds_) {
    return max_extent_cells_ >= 1;
  }

  const auto span_x = static_cast<std::int64_t>(std::max(max_x_, x)) -
                      std::min(min_x_, x) + 1;
  const auto span_y = static_cast<std::int64_t>(std::max(max_y_, y)) -
                      std::min(min_y_, y) + 1;
  return span_x <= max_extent_cells_ && span_y <= max_extent_cells_;
}

void OccupancyGridBuilder::applyScore(std::int64_t key, int delta) noexcept {
  const auto x = keyX(key);
  const auto y = keyY(key);
  if (!boundsAllow(x, y)) {
    ++dropped_cell_updates_;
    return;
  }

  auto it = observed_cells_.try_emplace(key, static_cast<std::int8_t>(0)).first;
  const int score = std::clamp(static_cast<int>(it->second) + delta,
                               kCellScoreMin, kCellScoreMax);
  it->second = static_cast<std::int8_t>(score);

  if (!has_bounds_) {
    min_x_ = max_x_ = x;
    min_y_ = max_y_ = y;
    has_bounds_ = true;
  } else {
    min_x_ = std::min(min_x_, x);
    max_x_ = std::max(max_x_, x);
    min_y_ = std::min(min_y_, y);
    max_y_ = std::max(max_y_, y);
  }
}

void OccupancyGridBuilder::insertCloudPoint(
    const std::array<float, 3>& point, bool apply_height_filter) noexcept {
  if (apply_height_filter) {
    if (!pointPassesHeightFilter(point)) {
      return;
    }
  } else if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
             !std::isfinite(point[2])) {
    return;
  }

  std::int32_t cell_x = 0;
  std::int32_t cell_y = 0;
  if (!toCellIndex(point[0], cell_x) || !toCellIndex(point[1], cell_y)) {
    return;
  }

  const int inflation_cells = static_cast<int>(std::ceil(
      config_.inflation_radius / config_.resolution));
  for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
        continue;
      }
      std::int32_t inflated_x = 0;
      std::int32_t inflated_y = 0;
      if (offsetCell(cell_x, dx, inflated_x) &&
          offsetCell(cell_y, dy, inflated_y)) {
        applyScore(cellKey(inflated_x, inflated_y), kHitScore);
      }
    }
  }
}

bool OccupancyGridBuilder::insertCloud(
    const hybrid_slam::sensor::PointCloudView& cloud,
    bool apply_height_filter) noexcept {
  if (!valid_ || !cloud.valid()) {
    return false;
  }
  std::array<float, 3> point{};
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    if (cloud.xyz(index, point)) {
      insertCloudPoint(point, apply_height_filter);
    }
  }
  return true;
}

bool OccupancyGridBuilder::insertFreeCloud(
    const hybrid_slam::sensor::PointCloudView& cloud) noexcept {
  if (!valid_ || !cloud.valid()) {
    return false;
  }
  std::unordered_set<std::int64_t> free_cells;
  free_cells.reserve(cloud.size());
  std::array<float, 3> point{};
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    if (!cloud.xyz(index, point) || !std::isfinite(point[0]) ||
        !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }
    std::int32_t cell_x = 0;
    std::int32_t cell_y = 0;
    if (toCellIndex(point[0], cell_x) && toCellIndex(point[1], cell_y)) {
      free_cells.insert(cellKey(cell_x, cell_y));
    }
  }
  for (const auto key : free_cells) {
    applyScore(key, -kMissScore);
  }
  return true;
}

bool OccupancyGridBuilder::insertScan(
    const hybrid_slam::sensor::PointCloudView& world_cloud,
    const Eigen::Vector3d& sensor_origin) noexcept {
  if (!valid_ || !world_cloud.valid() || !sensor_origin.allFinite()) {
    return false;
  }

  const int inflation_cells = static_cast<int>(std::ceil(
      config_.inflation_radius / config_.resolution));
  const double max_range2 = config_.max_range * config_.max_range;
  const double max_ray2 = config_.raycast_max_ray_length *
                          config_.raycast_max_ray_length;

  std::int32_t origin_cell_x = 0;
  std::int32_t origin_cell_y = 0;
  const bool origin_valid =
      toCellIndex(sensor_origin.x(), origin_cell_x) &&
      toCellIndex(sensor_origin.y(), origin_cell_y);

  std::unordered_set<std::int64_t> scan_hits;
  std::unordered_set<std::int64_t> ray_targets;
  std::unordered_set<std::int64_t> scan_free;
  std::array<float, 3> point{};
  for (std::size_t index = 0; index < world_cloud.size(); ++index) {
    if (!world_cloud.xyz(index, point) || !std::isfinite(point[0]) ||
        !std::isfinite(point[1]) || !std::isfinite(point[2])) {
      continue;
    }

    const double rel_z = static_cast<double>(point[2]) - sensor_origin.z();
    if (rel_z < config_.z_min || rel_z > config_.z_max) {
      continue;
    }
    const double dx = static_cast<double>(point[0]) - sensor_origin.x();
    const double dy = static_cast<double>(point[1]) - sensor_origin.y();
    const double planar2 = dx * dx + dy * dy;
    if (planar2 + rel_z * rel_z > max_range2) {
      continue;
    }

    std::int32_t cell_x = 0;
    std::int32_t cell_y = 0;
    if (!toCellIndex(point[0], cell_x) || !toCellIndex(point[1], cell_y)) {
      continue;
    }
    for (int dx_cell = -inflation_cells; dx_cell <= inflation_cells;
         ++dx_cell) {
      for (int dy_cell = -inflation_cells; dy_cell <= inflation_cells;
           ++dy_cell) {
        if (dx_cell * dx_cell + dy_cell * dy_cell >
            inflation_cells * inflation_cells) {
          continue;
        }
        std::int32_t inflated_x = 0;
        std::int32_t inflated_y = 0;
        if (offsetCell(cell_x, dx_cell, inflated_x) &&
            offsetCell(cell_y, dy_cell, inflated_y)) {
          scan_hits.insert(cellKey(inflated_x, inflated_y));
        }
      }
    }
    if (config_.raycast_enable && origin_valid && planar2 <= max_ray2) {
      ray_targets.insert(cellKey(cell_x, cell_y));
    }
  }

  if (config_.raycast_enable && origin_valid) {
    for (const auto key : ray_targets) {
      traceRayCells(origin_cell_x, origin_cell_y, keyX(key), keyY(key),
                    scan_free);
    }
  }
  for (const auto key : scan_hits) {
    applyScore(key, kHitScore);
  }
  for (const auto key : scan_free) {
    if (!scan_hits.contains(key)) {
      applyScore(key, -kMissScore);
    }
  }
  return true;
}

bool OccupancyGridBuilder::rebuildFromSnapshot(
    const MapSnapshot& snapshot) {
  return rebuildFromSnapshot(snapshot, {}).succeeded();
}

OccupancyGridRebuildStats OccupancyGridBuilder::rebuildFromSnapshot(
    const MapSnapshot& snapshot,
    const OccupancyGridRebuildOptions& options) {
  OccupancyGridRebuildStats stats;
  if (!valid_ || !snapshot.valid()) {
    stats.status = OccupancyGridRebuildStatus::kInvalidInput;
    return stats;
  }

  const std::size_t max_scratch_bytes =
      options.max_scratch_memory_bytes == 0
          ? config_.rebuild_scratch_memory_bytes
          : options.max_scratch_memory_bytes;
  const std::size_t max_points =
      options.max_points == 0 ? config_.rebuild_max_points : options.max_points;
  const std::size_t check_interval = std::max<std::size_t>(
      1, options.cancellation_check_interval == 0
             ? config_.rebuild_cancellation_check_interval
             : options.cancellation_check_interval);

  // Build into an isolated builder. A cancelled or budget-limited rebuild
  // must not leave the currently published evidence partially replaced.
  OccupancyGridBuilder scratch(config_);
  if (!scratch.valid()) {
    stats.status = OccupancyGridRebuildStatus::kInvalidInput;
    return stats;
  }

  const auto cancelled = [&options] {
    return options.should_cancel && options.should_cancel();
  };
  std::array<float, 3> source{};
  std::array<float, 3> world{};
  for (const auto& keyframe : snapshot.keyframes) {
    const auto& view = keyframe.cloud_base.view();
    const auto& T_map_base = keyframe.T_map_base.T_target_source;
    for (std::size_t index = 0; index < view.size(); ++index) {
      if (stats.processed_points >= max_points) {
        stats.status = OccupancyGridRebuildStatus::kPointBudgetExceeded;
        return stats;
      }
      if ((stats.processed_points % check_interval) == 0 && cancelled()) {
        stats.status = OccupancyGridRebuildStatus::kCancelled;
        return stats;
      }
      if (!view.xyz(index, source)) {
        stats.status = OccupancyGridRebuildStatus::kInvalidInput;
        return stats;
      }
      const Eigen::Vector3d transformed = T_map_base.transformPoint(
          Eigen::Vector3d(source[0], source[1], source[2]));
      if (!transformed.allFinite()) {
        stats.status = OccupancyGridRebuildStatus::kInvalidInput;
        return stats;
      }
      world = {static_cast<float>(transformed.x()),
               static_cast<float>(transformed.y()),
               static_cast<float>(transformed.z())};
      if (!std::isfinite(world[0]) || !std::isfinite(world[1]) ||
          !std::isfinite(world[2])) {
        stats.status = OccupancyGridRebuildStatus::kInvalidInput;
        return stats;
      }
      scratch.insertCloudPoint(world, true);
      ++stats.processed_points;
      stats.peak_estimated_scratch_memory_bytes = std::max(
          stats.peak_estimated_scratch_memory_bytes,
          scratch.estimatedScratchMemoryBytes());
      if (stats.peak_estimated_scratch_memory_bytes > max_scratch_bytes) {
        stats.status = OccupancyGridRebuildStatus::kScratchBudgetExceeded;
        return stats;
      }
    }
  }
  if (cancelled()) {
    stats.status = OccupancyGridRebuildStatus::kCancelled;
    return stats;
  }

  *this = std::move(scratch);
  stats.status = OccupancyGridRebuildStatus::kSuccess;
  return stats;
}

OccupancyGridSnapshot OccupancyGridBuilder::snapshot(
    std::uint64_t version, std::int64_t timestamp_nanoseconds) const {
  OccupancyGridSnapshot snapshot;
  snapshot.version = version;
  snapshot.timestamp_nanoseconds = timestamp_nanoseconds;
  snapshot.resolution = config_.resolution;
  if (!valid_ || !has_bounds_) {
    snapshot.width = 1;
    snapshot.height = 1;
    snapshot.data = {static_cast<std::int8_t>(config_.unknown_value)};
    return snapshot;
  }

  const auto padding = static_cast<std::int64_t>(std::ceil(
      config_.map_padding / config_.resolution));
  const auto origin_x = static_cast<std::int64_t>(min_x_) - padding;
  const auto origin_y = static_cast<std::int64_t>(min_y_) - padding;
  const auto end_x = static_cast<std::int64_t>(max_x_) + padding;
  const auto end_y = static_cast<std::int64_t>(max_y_) + padding;
  const auto width = end_x - origin_x + 1;
  const auto height = end_y - origin_y + 1;
  if (origin_x < std::numeric_limits<std::int32_t>::min() ||
      origin_x > std::numeric_limits<std::int32_t>::max() ||
      origin_y < std::numeric_limits<std::int32_t>::min() ||
      origin_y > std::numeric_limits<std::int32_t>::max() || width <= 0 ||
      height <= 0 ||
      static_cast<std::uint64_t>(width) >
          std::numeric_limits<std::uint32_t>::max() ||
      static_cast<std::uint64_t>(height) >
          std::numeric_limits<std::uint32_t>::max()) {
    return snapshot;
  }
  const auto cell_count = static_cast<std::uint64_t>(width) *
                          static_cast<std::uint64_t>(height);
  if (cell_count > std::numeric_limits<std::size_t>::max() ||
      cell_count > config_.max_materialized_cells) {
    return snapshot;
  }

  snapshot.origin_x = static_cast<std::int32_t>(origin_x);
  snapshot.origin_y = static_cast<std::int32_t>(origin_y);
  snapshot.width = static_cast<std::uint32_t>(width);
  snapshot.height = static_cast<std::uint32_t>(height);
  const auto default_value = static_cast<std::int8_t>(
      config_.use_unknown ? config_.unknown_value : config_.free_value);
  snapshot.data.assign(static_cast<std::size_t>(width) *
                           static_cast<std::size_t>(height),
                       default_value);

  for (const auto& [key, score] : observed_cells_) {
    const auto x = static_cast<std::int64_t>(keyX(key)) - origin_x;
    const auto y = static_cast<std::int64_t>(keyY(key)) - origin_y;
    if (x < 0 || y < 0 || x >= width || y >= height) {
      continue;
    }
    const auto index = static_cast<std::size_t>(y) *
                           static_cast<std::size_t>(width) +
                       static_cast<std::size_t>(x);
    snapshot.data[index] = static_cast<std::int8_t>(
        score > 0 ? config_.occupied_value : config_.free_value);
  }
  return snapshot;
}

std::size_t OccupancyGridBuilder::estimatedScratchMemoryBytes() const noexcept {
  // This is intentionally conservative and diagnostic rather than an ABI
  // claim about libstdc++'s unordered_map node layout. It includes a bucket
  // pointer estimate so the configured limit bounds both node and index
  // growth before a dense snapshot is materialized.
  constexpr std::size_t kEstimatedNodeBytes = 64U;
  const auto node_count = observed_cells_.size();
  const auto bucket_count = observed_cells_.bucket_count();
  if (node_count > std::numeric_limits<std::size_t>::max() /
                       kEstimatedNodeBytes ||
      bucket_count >
          (std::numeric_limits<std::size_t>::max() -
           node_count * kEstimatedNodeBytes) /
              sizeof(void*)) {
    return std::numeric_limits<std::size_t>::max();
  }
  return node_count * kEstimatedNodeBytes + bucket_count * sizeof(void*);
}

std::int64_t OccupancyGridBuilder::cellKey(
    std::int32_t x, std::int32_t y) noexcept {
  const auto ux = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto uy = static_cast<std::uint64_t>(static_cast<std::uint32_t>(y));
  return static_cast<std::int64_t>((ux << 32) | uy);
}

std::int32_t OccupancyGridBuilder::keyX(std::int64_t key) noexcept {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey >> 32));
}

std::int32_t OccupancyGridBuilder::keyY(std::int64_t key) noexcept {
  const auto ukey = static_cast<std::uint64_t>(key);
  return static_cast<std::int32_t>(static_cast<std::uint32_t>(ukey & 0xffffffff));
}

void OccupancyGridBuilder::traceRayCells(
    std::int32_t x0, std::int32_t y0,
    std::int32_t x1, std::int32_t y1,
    std::unordered_set<std::int64_t>& cells) noexcept {
  const std::int32_t dx = std::abs(x1 - x0);
  const std::int32_t dy = std::abs(y1 - y0);
  const std::int32_t sx = x0 < x1 ? 1 : -1;
  const std::int32_t sy = y0 < y1 ? 1 : -1;
  std::int32_t err = dx - dy;
  std::int32_t x = x0;
  std::int32_t y = y0;
  while (x != x1 || y != y1) {
    cells.insert(cellKey(x, y));
    const std::int32_t e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

}  // namespace hybrid_slam::mapping
