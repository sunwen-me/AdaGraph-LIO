#include "adagraph/io/map_io.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <system_error>
#include <utility>
#include <vector>

namespace adagraph::io {
namespace {

struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator<(const VoxelKey& other) const noexcept {
    if (x != other.x) {
      return x < other.x;
    }
    if (y != other.y) {
      return y < other.y;
    }
    return z < other.z;
  }
};

struct VoxelAccumulator {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  std::size_t count = 0;
};

std::optional<std::vector<adagraph::mapping::MapPoint3f>> Voxelize(
    const std::vector<adagraph::mapping::MapPoint3f>& source,
    double leaf_size) {
  if (!(leaf_size > 0.0)) {
    for (const auto& point : source) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        return std::nullopt;
      }
    }
    return source;
  }

  std::map<VoxelKey, VoxelAccumulator> voxels;
  for (const auto& point : source) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return std::nullopt;
    }
    const double scaled_x = std::floor(static_cast<double>(point.x) /
                                       leaf_size);
    const double scaled_y = std::floor(static_cast<double>(point.y) /
                                       leaf_size);
    const double scaled_z = std::floor(static_cast<double>(point.z) /
                                       leaf_size);
    constexpr double kMaxExactInt64 =
        static_cast<double>(std::numeric_limits<std::int64_t>::max());
    constexpr double kMinExactInt64 =
        static_cast<double>(std::numeric_limits<std::int64_t>::min());
    if (!std::isfinite(scaled_x) || !std::isfinite(scaled_y) ||
        !std::isfinite(scaled_z) || scaled_x < kMinExactInt64 ||
        scaled_x > kMaxExactInt64 || scaled_y < kMinExactInt64 ||
        scaled_y > kMaxExactInt64 || scaled_z < kMinExactInt64 ||
        scaled_z > kMaxExactInt64) {
      return std::nullopt;
    }

    auto& voxel = voxels[{static_cast<std::int64_t>(scaled_x),
                          static_cast<std::int64_t>(scaled_y),
                          static_cast<std::int64_t>(scaled_z)}];
    voxel.x += point.x;
    voxel.y += point.y;
    voxel.z += point.z;
    ++voxel.count;
  }

  std::vector<adagraph::mapping::MapPoint3f> result;
  result.reserve(voxels.size());
  for (const auto& [key, voxel] : voxels) {
    (void)key;
    if (voxel.count == 0) {
      continue;
    }
    const double count = static_cast<double>(voxel.count);
    const adagraph::mapping::MapPoint3f point{
        static_cast<float>(voxel.x / count),
        static_cast<float>(voxel.y / count),
        static_cast<float>(voxel.z / count)};
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return std::nullopt;
    }
    result.push_back(point);
  }
  return result;
}

struct PcdHeader {
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::size_t point_count = 0;
  std::streampos data_offset = std::streampos(-1);
};

bool ReadPcdHeader(std::ifstream& input, PcdHeader& header,
                   std::string& error_message) {
  std::string line;
  std::size_t width = 0;
  std::size_t height = 1;
  bool has_points = false;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    std::istringstream stream(line);
    std::string tag;
    stream >> tag;
    if (tag.empty() || tag[0] == '#') {
      continue;
    }
    if (tag == "FIELDS") {
      std::string field;
      while (stream >> field) {
        header.fields.push_back(std::move(field));
      }
    } else if (tag == "SIZE") {
      int size = 0;
      while (stream >> size) {
        header.sizes.push_back(size);
      }
    } else if (tag == "TYPE") {
      char type = 0;
      while (stream >> type) {
        header.types.push_back(type);
      }
    } else if (tag == "COUNT") {
      int count = 0;
      while (stream >> count) {
        header.counts.push_back(count);
      }
    } else if (tag == "WIDTH") {
      stream >> width;
    } else if (tag == "HEIGHT") {
      stream >> height;
    } else if (tag == "POINTS") {
      stream >> header.point_count;
      has_points = true;
    } else if (tag == "DATA") {
      std::string encoding;
      stream >> encoding;
      if (encoding != "binary") {
        error_message = "only uncompressed binary PCD is supported";
        return false;
      }
      header.data_offset = input.tellg();
      break;
    }
  }

  if (header.data_offset == std::streampos(-1) || header.fields.empty() ||
      header.sizes.size() != header.fields.size() ||
      header.types.size() != header.fields.size()) {
    error_message = "incomplete binary PCD header";
    return false;
  }
  if (header.counts.empty()) {
    header.counts.assign(header.fields.size(), 1);
  }
  if (header.counts.size() != header.fields.size()) {
    error_message = "PCD field/count metadata has different lengths";
    return false;
  }
  if (!has_points) {
    if (height != 0 && width <= std::numeric_limits<std::size_t>::max() /
                              height) {
      header.point_count = width * height;
    } else {
      error_message = "PCD width/height overflows point count";
      return false;
    }
  }
  if (header.point_count == 0) {
    error_message = "PCD contains no points";
    return false;
  }
  return true;
}

bool ReadPcdScalar(const std::byte* record, std::size_t record_size,
                   std::size_t offset, int size, char type, float& value) {
  if (offset > record_size || static_cast<std::size_t>(size) >
                                 record_size - offset) {
    return false;
  }
  if (type == 'F' && size == static_cast<int>(sizeof(float))) {
    std::memcpy(&value, record + offset, sizeof(float));
    return std::isfinite(value);
  }
  if (type == 'F' && size == static_cast<int>(sizeof(double))) {
    double double_value = 0.0;
    std::memcpy(&double_value, record + offset, sizeof(double));
    value = static_cast<float>(double_value);
    return std::isfinite(double_value) && std::isfinite(value);
  }
  return false;
}

bool ReadBinaryPcd(const std::filesystem::path& path,
                   std::vector<adagraph::mapping::MapPoint3f>& points,
                   std::string& error_message) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    error_message = "failed to open " + path.string();
    return false;
  }

  PcdHeader header;
  if (!ReadPcdHeader(input, header, error_message)) {
    return false;
  }

  std::vector<std::size_t> offsets(header.fields.size(), 0);
  std::size_t record_size = 0;
  int x_field = -1;
  int y_field = -1;
  int z_field = -1;
  for (std::size_t index = 0; index < header.fields.size(); ++index) {
    if (header.sizes[index] <= 0 || header.counts[index] <= 0 ||
        static_cast<std::size_t>(header.sizes[index]) >
            std::numeric_limits<std::size_t>::max() /
                static_cast<std::size_t>(header.counts[index])) {
      error_message = "invalid PCD field size/count";
      return false;
    }
    const std::size_t field_bytes =
        static_cast<std::size_t>(header.sizes[index]) *
        static_cast<std::size_t>(header.counts[index]);
    if (record_size > std::numeric_limits<std::size_t>::max() - field_bytes) {
      error_message = "PCD record size overflows";
      return false;
    }
    offsets[index] = record_size;
    record_size += field_bytes;
    if (header.fields[index] == "x") {
      x_field = static_cast<int>(index);
    } else if (header.fields[index] == "y") {
      y_field = static_cast<int>(index);
    } else if (header.fields[index] == "z") {
      z_field = static_cast<int>(index);
    }
  }
  if (x_field < 0 || y_field < 0 || z_field < 0 || record_size == 0) {
    error_message = "PCD does not contain x/y/z fields";
    return false;
  }

  if (header.point_count > std::numeric_limits<std::size_t>::max() /
                              record_size) {
    error_message = "PCD payload size overflows";
    return false;
  }
  input.seekg(header.data_offset);
  if (!input) {
    error_message = "failed to seek to PCD payload";
    return false;
  }

  points.clear();
  points.reserve(header.point_count);
  std::vector<std::byte> record(record_size);
  for (std::size_t index = 0; index < header.point_count; ++index) {
    input.read(reinterpret_cast<char*>(record.data()),
               static_cast<std::streamsize>(record.size()));
    if (!input) {
      error_message = "truncated PCD payload in " + path.string();
      return false;
    }
    adagraph::mapping::MapPoint3f point;
    if (!ReadPcdScalar(record.data(), record.size(), offsets[x_field],
                       header.sizes[x_field], header.types[x_field], point.x) ||
        !ReadPcdScalar(record.data(), record.size(), offsets[y_field],
                       header.sizes[y_field], header.types[y_field], point.y) ||
        !ReadPcdScalar(record.data(), record.size(), offsets[z_field],
                       header.sizes[z_field], header.types[z_field], point.z)) {
      error_message = "unsupported or non-finite x/y/z field in " +
                      path.string();
      return false;
    }
    points.push_back(point);
  }
  return true;
}

MapSaveResult WritePoints(
    std::uint64_t request_id, std::uint64_t snapshot_version,
    const std::vector<adagraph::mapping::MapPoint3f>& points,
    const std::filesystem::path& output_path, std::string message) {
  MapSaveResult result;
  result.request_id = request_id;
  result.snapshot_version = snapshot_version;
  result.saved_point_count = points.size();
  if (points.empty()) {
    result.message = "point-cloud product is empty";
    return result;
  }
  if (output_path.empty() || output_path.filename().empty()) {
    result.message = "output path must name a file";
    return result;
  }

  std::error_code error;
  const auto parent = output_path.parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, error);
    if (error) {
      result.message = "failed to create output directory: " +
                       error.message();
      return result;
    }
  }

  const auto temporary_path = output_path.string() + ".tmp." +
                              std::to_string(request_id);
  std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
  if (!output) {
    result.message = "failed to open temporary output file";
    return result;
  }

  output << "# .PCD v0.7 - Point Cloud Data file format\n"
          << "VERSION 0.7\n"
          << "FIELDS x y z intensity\n"
          << "SIZE 4 4 4 4\n"
          << "TYPE F F F F\n"
          << "COUNT 1 1 1 1\n"
          << "WIDTH " << points.size() << "\n"
          << "HEIGHT 1\n"
          << "VIEWPOINT 0 0 0 1 0 0 0\n"
          << "POINTS " << points.size() << "\n"
          << "DATA binary\n";
  if (!output) {
    output.close();
    std::filesystem::remove(temporary_path, error);
    result.message = "failed to write PCD header";
    return result;
  }

  for (const auto& point : points) {
    const std::array<float, 4> record{point.x, point.y, point.z, 0.0F};
    output.write(reinterpret_cast<const char*>(record.data()),
                 static_cast<std::streamsize>(sizeof(record)));
    if (!output) {
      output.close();
      std::filesystem::remove(temporary_path, error);
      result.message = "failed while writing PCD payload";
      return result;
    }
  }
  output.close();
  if (!output) {
    std::filesystem::remove(temporary_path, error);
    result.message = "failed to close temporary output file";
    return result;
  }

  std::filesystem::rename(temporary_path, output_path, error);
  if (error) {
    const std::string rename_error = error.message();
    std::filesystem::remove(temporary_path, error);
    result.message = "failed to atomically install output file: " +
                     rename_error;
    return result;
  }

  result.success = true;
  result.message = std::move(message);
  return result;
}

MapSaveResult SaveProduct(
    std::uint64_t request_id,
    std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot,
    std::shared_ptr<const adagraph::mapping::SnapshotPointCloudMap> product,
    const std::filesystem::path& output_path, double leaf_size) {
  MapSaveResult result;
  result.request_id = request_id;

  std::optional<adagraph::mapping::SnapshotPointCloudMap> materialized;
  if (snapshot) {
    result.snapshot_version = snapshot->version;
    materialized =
        adagraph::mapping::SnapshotProductBuilder::BuildPointCloudMap(*snapshot);
    if (!materialized) {
      result.message = "failed to materialize MapSnapshot";
      return result;
    }
  } else if (product) {
    result.snapshot_version = product->snapshot_version;
    materialized = *product;
  } else {
    result.message = "save request has no immutable input";
    return result;
  }

  result.raw_point_count = materialized->points.size();
  const auto points = Voxelize(materialized->points, leaf_size);
  if (!points) {
    result.message = "failed to voxelize non-finite or out-of-range points";
    return result;
  }
  auto written = WritePoints(request_id, result.snapshot_version, *points,
                             output_path, "saved binary PCD");
  written.raw_point_count = result.raw_point_count;
  return written;
}

MapSaveResult SaveFragmentSet(
    std::uint64_t request_id,
    std::shared_ptr<const PcdFragmentSet> fragment_set,
    const std::filesystem::path& output_path, double final_leaf_size) {
  MapSaveResult result;
  result.request_id = request_id;
  if (!fragment_set || !fragment_set->valid()) {
    result.message = "invalid PCD fragment set";
    return result;
  }

  std::vector<std::filesystem::path> paths = fragment_set->input_paths;
  std::sort(paths.begin(), paths.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.generic_string() < rhs.generic_string();
            });
  result.input_file_count = paths.size();

  std::vector<adagraph::mapping::MapPoint3f> merged;
  for (const auto& path : paths) {
    std::vector<adagraph::mapping::MapPoint3f> fragment;
    std::string error_message;
    if (!ReadBinaryPcd(path, fragment, error_message)) {
      if (fragment_set->require_all_inputs) {
        result.message = error_message;
        return result;
      }
      continue;
    }
    std::vector<adagraph::mapping::MapPoint3f> points = std::move(fragment);
    if (fragment_set->voxelize_each_fragment) {
      const auto reduced = Voxelize(points, fragment_set->fragment_voxel_leaf_size);
      if (!reduced) {
        result.message = "failed to voxelize fragment " + path.string();
        return result;
      }
      points = *reduced;
    }
    if (points.size() > std::numeric_limits<std::size_t>::max() -
                            merged.size()) {
      result.message = "merged PCD point count overflows";
      return result;
    }
    merged.insert(merged.end(), points.begin(), points.end());
  }

  result.raw_point_count = merged.size();
  const auto reduced = Voxelize(merged, final_leaf_size);
  if (!reduced) {
    result.message = "failed to voxelize merged PCD fragments";
    return result;
  }
  auto written = WritePoints(request_id, 0, *reduced, output_path,
                             "merged binary PCD fragments");
  written.raw_point_count = result.raw_point_count;
  written.input_file_count = result.input_file_count;
  return written;
}

}  // namespace

MapSaveScheduler::MapSaveScheduler(std::size_t queue_capacity,
                                   CompletionCallback completion)
    : queue_capacity_(std::max<std::size_t>(1, queue_capacity)),
      completion_(std::move(completion)),
      worker_(&MapSaveScheduler::workerLoop, this) {}

MapSaveScheduler::~MapSaveScheduler() {
  finish();
}

MapSaveSubmissionStatus MapSaveScheduler::submit(MapSaveRequest request) {
  if (!request.valid()) {
    return MapSaveSubmissionStatus::kInvalid;
  }

  std::unique_lock<std::mutex> lock(mutex_);
  if (state_ != State::kRunning) {
    return MapSaveSubmissionStatus::kClosed;
  }
  if (queue_.size() >= queue_capacity_) {
    ++stats_.capacity_waits;
  }
  not_full_cv_.wait(lock, [this] {
    return state_ != State::kRunning || queue_.size() < queue_capacity_;
  });
  if (state_ != State::kRunning) {
    return MapSaveSubmissionStatus::kClosed;
  }

  queue_.push_back(std::move(request));
  ++stats_.submitted;
  stats_.maximum_queue_depth =
      std::max(stats_.maximum_queue_depth, queue_.size());
  lock.unlock();
  not_empty_cv_.notify_one();
  return MapSaveSubmissionStatus::kAccepted;
}

void MapSaveScheduler::finish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == State::kStopped) {
      return;
    }
    state_ = State::kClosing;
  }
  not_empty_cv_.notify_all();
  not_full_cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = State::kStopped;
}

MapSaveSchedulerStats MapSaveScheduler::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void MapSaveScheduler::workerLoop() {
  for (;;) {
    MapSaveRequest request;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      not_empty_cv_.wait(lock, [this] {
        return state_ != State::kRunning || !queue_.empty();
      });
      if (queue_.empty() && state_ != State::kRunning) {
        return;
      }
      request = std::move(queue_.front());
      queue_.pop_front();
    }
    not_full_cv_.notify_one();

    MapSaveResult result = process(std::move(request));
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.processed;
      if (!result.success) {
        ++stats_.failed;
      }
    }
    if (completion_) {
      try {
        completion_(result);
      } catch (...) {
        // A logging/telemetry callback must not terminate the IO worker.
      }
    }
  }
}

MapSaveResult MapSaveScheduler::process(MapSaveRequest request) {
  if (request.fragment_set) {
    return SaveFragmentSet(request.request_id, std::move(request.fragment_set),
                           request.output_path, request.voxel_leaf_size);
  }
  return SaveProduct(request.request_id, std::move(request.snapshot),
                     std::move(request.point_cloud), request.output_path,
                     request.voxel_leaf_size);
}

}  // namespace adagraph::io
