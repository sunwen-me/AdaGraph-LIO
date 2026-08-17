#ifndef HYBRID_SLAM_IO_MAP_IO_H_
#define HYBRID_SLAM_IO_MAP_IO_H_

#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/mapping/occupancy_grid.h"
#include "hybrid_slam/mapping/snapshot_products.h"

namespace hybrid_slam::io {

// Immutable description of an already materialized PCD fragment set.  The
// scheduler consumes the files in one FIFO request, so a final merge cannot
// observe a fragment before the preceding fragment-write requests complete.
// Paths are intentionally explicit rather than discovered by directory
// iteration; this makes the output deterministic and prevents unrelated files
// from being merged into a map.
struct PcdFragmentSet {
  std::vector<std::filesystem::path> input_paths;
  bool voxelize_each_fragment = false;
  double fragment_voxel_leaf_size = 0.0;
  bool require_all_inputs = true;

  [[nodiscard]] bool valid() const noexcept {
    if (input_paths.empty() ||
        !std::isfinite(fragment_voxel_leaf_size) ||
        fragment_voxel_leaf_size < 0.0) {
      return false;
    }
    for (const auto& path : input_paths) {
      if (path.empty() || path.filename().empty()) {
        return false;
      }
    }
    return true;
  }
};

// A save request is deliberately value-like and owns only immutable inputs.
// The worker materializes a MapSnapshot after dequeue, so submitting a large
// snapshot does not perform an O(points) copy on the caller's thread.
struct MapSaveRequest {
  std::uint64_t request_id = 0;
  std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot;
  std::shared_ptr<const hybrid_slam::mapping::SnapshotPointCloudMap>
      point_cloud;
  std::shared_ptr<const PcdFragmentSet> fragment_set;
  std::filesystem::path output_path;
  double voxel_leaf_size = 0.0;
  std::optional<std::pair<std::size_t, std::size_t>> keyframe_range;

  [[nodiscard]] bool valid() const noexcept {
    const bool has_snapshot = static_cast<bool>(snapshot);
    const bool has_point_cloud = static_cast<bool>(point_cloud);
    const bool has_fragment_set = static_cast<bool>(fragment_set);
    const int input_kinds = static_cast<int>(has_snapshot) +
                            static_cast<int>(has_point_cloud) +
                            static_cast<int>(has_fragment_set);
    return input_kinds == 1 && !output_path.empty() &&
           output_path.filename() != std::filesystem::path() &&
           std::isfinite(voxel_leaf_size) && voxel_leaf_size >= 0.0 &&
           (!keyframe_range ||
            (has_snapshot && keyframe_range->first < keyframe_range->second &&
             keyframe_range->second <= snapshot->keyframes.size())) &&
           (!has_fragment_set || fragment_set->valid());
  }
};

struct MapSaveResult {
  std::uint64_t request_id = 0;
  std::uint64_t snapshot_version = 0;
  bool success = false;
  std::size_t raw_point_count = 0;
  std::size_t saved_point_count = 0;
  std::size_t input_file_count = 0;
  std::string message;
};

enum class MapSaveSubmissionStatus {
  kAccepted = 0,
  kInvalid,
  kClosed,
};

struct MapSaveSchedulerStats {
  std::uint64_t submitted = 0;
  std::uint64_t processed = 0;
  std::uint64_t failed = 0;
  std::uint64_t capacity_waits = 0;
  std::size_t maximum_queue_depth = 0;
};

// A durable Nav2-compatible product generated while mapping from the online
// occupancy evidence path.  The standard YAML and PGM can be passed directly
// to map_server; a neighboring metadata YAML preserves the dynamic frame and
// provenance that Nav2's map YAML format does not represent.
struct Nav2GridArtifactRequest {
  std::uint64_t request_id = 0;
  std::shared_ptr<const hybrid_slam::mapping::OccupancyGridSnapshot> grid;
  std::filesystem::path yaml_path;
  std::string frame_id;
  std::string source = "online";
  int occupied_value = 100;
  int free_value = 0;
  int unknown_value = -1;

  [[nodiscard]] bool valid() const noexcept;
};

struct Nav2GridArtifactResult {
  std::uint64_t request_id = 0;
  std::uint64_t grid_version = 0;
  bool success = false;
  std::filesystem::path yaml_path;
  std::filesystem::path image_path;
  std::filesystem::path metadata_path;
  std::string message;
};

// Writes PGM first, metadata second and the map-server YAML last, installing
// each through a same-directory atomic rename.  This ordering ensures that a
// reader which observes a new YAML can also resolve its referenced image.
[[nodiscard]] Nav2GridArtifactResult WriteNav2GridArtifact(
    Nav2GridArtifactRequest request);

// Reliable bounded map-IO worker. Accepted save commands are never silently
// discarded. A caller may wait for bounded capacity, but no filesystem or
// point-cloud work runs while the queue mutex is held. finish() closes,
// drains and joins the worker and is idempotent.
class MapSaveScheduler {
 public:
  using CompletionCallback = std::function<void(const MapSaveResult&)>;

  explicit MapSaveScheduler(std::size_t queue_capacity = 2,
                            CompletionCallback completion = {});
  ~MapSaveScheduler();

  MapSaveScheduler(const MapSaveScheduler&) = delete;
  MapSaveScheduler& operator=(const MapSaveScheduler&) = delete;

  [[nodiscard]] MapSaveSubmissionStatus submit(MapSaveRequest request);
  void finish();

  [[nodiscard]] MapSaveSchedulerStats stats() const;

 private:
  struct State {
    enum Value {
      kRunning = 0,
      kClosing,
      kStopped,
    };
  };

  void workerLoop();
  [[nodiscard]] static MapSaveResult process(MapSaveRequest request);

  const std::size_t queue_capacity_;
  CompletionCallback completion_;

  mutable std::mutex mutex_;
  std::condition_variable not_empty_cv_;
  std::condition_variable not_full_cv_;
  std::deque<MapSaveRequest> queue_;
  State::Value state_ = State::kRunning;
  std::thread worker_;
  MapSaveSchedulerStats stats_;
};

}  // namespace hybrid_slam::io

#endif  // HYBRID_SLAM_IO_MAP_IO_H_
