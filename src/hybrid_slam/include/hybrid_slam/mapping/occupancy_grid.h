#ifndef HYBRID_SLAM_MAPPING_OCCUPANCY_GRID_H_
#define HYBRID_SLAM_MAPPING_OCCUPANCY_GRID_H_

#include <array>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/point_cloud_view.h"

namespace hybrid_slam::mapping {

enum class OccupancyGridUpdatePolicy : std::uint8_t {
  kIncremental = 0,
  // Rebuild from the latest input and discard prior evidence.
  kRebuild,
  // Update evidence incrementally, but defer dense snapshot materialization.
  kLazy,
};

// ROS-independent evidence-grid configuration. Parameter loading belongs to
// the adapter; this value is validated before the builder is constructed.
struct OccupancyGridConfig {
  bool use_unknown = true;
  double resolution = 0.1;
  double z_min = -0.3;
  double z_max = 2.0;
  double map_padding = 1.0;
  double inflation_radius = 0.0;
  double max_range = 30.0;
  double max_extent = 400.0;
  bool raycast_enable = true;
  double raycast_max_ray_length = 15.0;
  int occupied_value = 100;
  int free_value = 0;
  int unknown_value = -1;
  // Rebuild work is performed in a scratch builder. These limits are
  // explicit because a latest-only snapshot worker may otherwise retain the
  // immutable MapSnapshot, the scratch hash grid and a dense product at the
  // same time.
  std::size_t rebuild_scratch_memory_bytes = 64U * 1024U * 1024U;
  std::size_t rebuild_max_points = 5U * 1000U * 1000U;
  std::size_t rebuild_cancellation_check_interval = 4096U;
  std::size_t max_materialized_cells = 16U * 1000U * 1000U;
  OccupancyGridUpdatePolicy update_policy =
      OccupancyGridUpdatePolicy::kLazy;

  [[nodiscard]] std::optional<std::string> validate() const;
};

enum class OccupancyGridRebuildStatus : std::uint8_t {
  kSuccess = 0,
  kInvalidInput,
  kCancelled,
  kScratchBudgetExceeded,
  kPointBudgetExceeded,
};

struct OccupancyGridRebuildOptions {
  std::size_t max_scratch_memory_bytes = 0;
  std::size_t max_points = 0;
  std::size_t cancellation_check_interval = 0;
  std::function<bool()> should_cancel;
};

struct OccupancyGridRebuildStats {
  OccupancyGridRebuildStatus status =
      OccupancyGridRebuildStatus::kInvalidInput;
  std::size_t processed_points = 0;
  std::size_t peak_estimated_scratch_memory_bytes = 0;

  [[nodiscard]] bool succeeded() const noexcept {
    return status == OccupancyGridRebuildStatus::kSuccess;
  }
};

// Immutable materialized grid product. Cell coordinates are expressed in the
// builder's resolution; origin_x/origin_y are the lower-left cell indices.
// The ROS adapter adds frame_id and converts the timestamp to rclcpp::Time.
struct OccupancyGridSnapshot {
  std::uint64_t version = 0;
  std::int64_t timestamp_nanoseconds = 0;
  double resolution = 0.0;
  std::int32_t origin_x = 0;
  std::int32_t origin_y = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<std::int8_t> data;

  [[nodiscard]] bool valid() const noexcept {
    if (!(resolution > 0.0) || width == 0 || height == 0) {
      return false;
    }
    const auto width_size = static_cast<std::size_t>(width);
    const auto height_size = static_cast<std::size_t>(height);
    return height_size <=
               std::numeric_limits<std::size_t>::max() / width_size &&
           data.size() == width_size * height_size;
  }
};

// Single-writer, ROS-independent 2-D occupancy evidence builder. It consumes
// borrowed XYZ views and never owns or copies the source point cloud. A
// snapshot copies only the materialized grid representation for publication.
class OccupancyGridBuilder {
 public:
  explicit OccupancyGridBuilder(OccupancyGridConfig config = {});

  [[nodiscard]] bool valid() const noexcept { return valid_; }
  [[nodiscard]] const OccupancyGridConfig& config() const noexcept {
    return config_;
  }
  [[nodiscard]] bool empty() const noexcept { return observed_cells_.empty(); }
  [[nodiscard]] std::uint64_t droppedCellUpdates() const noexcept {
    return dropped_cell_updates_;
  }

  void reset() noexcept;

  // Static/world-cloud insertion. When apply_height_filter is true, the
  // configured absolute z band is applied. The map-reset path can disable it
  // after a map-frame ground filter has already classified the cloud.
  [[nodiscard]] bool insertCloud(
      const hybrid_slam::sensor::PointCloudView& cloud,
      bool apply_height_filter = true) noexcept;

  // Inserts direct free evidence, used for ground returns in a static map.
  [[nodiscard]] bool insertFreeCloud(
      const hybrid_slam::sensor::PointCloudView& cloud) noexcept;

  // Online scan insertion with relative height/range filtering and optional
  // 2-D ray carving from sensor_origin to unique endpoint cells.
  [[nodiscard]] bool insertScan(
      const hybrid_slam::sensor::PointCloudView& world_cloud,
      const Eigen::Vector3d& sensor_origin) noexcept;

  // Rebuild static evidence from one immutable optimized map snapshot. The
  // snapshot is borrowed for the duration of this call; no snapshot or point
  // cloud ownership crosses the builder boundary.
  [[nodiscard]] bool rebuildFromSnapshot(const MapSnapshot& snapshot);
  [[nodiscard]] OccupancyGridRebuildStats rebuildFromSnapshot(
      const MapSnapshot& snapshot,
      const OccupancyGridRebuildOptions& options);

  [[nodiscard]] OccupancyGridSnapshot snapshot(
      std::uint64_t version, std::int64_t timestamp_nanoseconds) const;

 private:
  static constexpr int kCellScoreMax = 6;
  static constexpr int kCellScoreMin = -3;
  static constexpr int kHitScore = 3;
  static constexpr int kMissScore = 1;

  static std::int64_t cellKey(std::int32_t x, std::int32_t y) noexcept;
  static std::int32_t keyX(std::int64_t key) noexcept;
  static std::int32_t keyY(std::int64_t key) noexcept;
  static void traceRayCells(
      std::int32_t x0, std::int32_t y0,
      std::int32_t x1, std::int32_t y1,
      std::unordered_set<std::int64_t>& cells) noexcept;

  [[nodiscard]] bool toCellIndex(
      double meters, std::int32_t& cell) const noexcept;
  [[nodiscard]] bool offsetCell(
      std::int32_t cell, int offset, std::int32_t& result) const noexcept;
  [[nodiscard]] bool pointPassesHeightFilter(
      const std::array<float, 3>& point) const noexcept;
  [[nodiscard]] bool boundsAllow(
      std::int32_t x, std::int32_t y) const noexcept;
  void applyScore(std::int64_t key, int delta) noexcept;
  void insertCloudPoint(
      const std::array<float, 3>& point, bool apply_height_filter) noexcept;
  [[nodiscard]] std::size_t estimatedScratchMemoryBytes() const noexcept;

  OccupancyGridConfig config_;
  bool valid_ = false;
  std::int64_t max_extent_cells_ = 0;
  std::unordered_map<std::int64_t, std::int8_t> observed_cells_;
  bool has_bounds_ = false;
  std::int32_t min_x_ = 0;
  std::int32_t max_x_ = 0;
  std::int32_t min_y_ = 0;
  std::int32_t max_y_ = 0;
  std::uint64_t dropped_cell_updates_ = 0;
};

}  // namespace hybrid_slam::mapping

#endif  // HYBRID_SLAM_MAPPING_OCCUPANCY_GRID_H_
