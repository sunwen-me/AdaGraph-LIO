#ifndef HYBRID_SLAM_SYSTEM_DATASET_RUNNER_H_
#define HYBRID_SLAM_SYSTEM_DATASET_RUNNER_H_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <variant>

#include "hybrid_slam/localization/localization_map.h"
#include "hybrid_slam/sensor/imu_data.h"
#include "hybrid_slam/sensor/point_cloud_frame.h"
#include "hybrid_slam/system/slam_system.h"

namespace hybrid_slam::system {

// A dataset/player emits the same value types as a ROS SensorBridge.  The
// packet is move-only when it contains a PointCloudFrame, so a runner cannot
// accidentally copy a large scan while dispatching it to SlamSystem.
struct DatasetPacket {
  using Payload =
      std::variant<hybrid_slam::sensor::ImuData,
                   hybrid_slam::sensor::PointCloudFrame>;

  Payload payload;

  [[nodiscard]] bool valid() const noexcept;
  [[nodiscard]] std::optional<hybrid_slam::common::Timestamp> timestamp()
      const noexcept;
};

// The source owns dataset-specific file/bag decoding.  It must return packets
// in nondecreasing sensor time order; the runner records violations as input
// errors rather than silently reordering or dropping them.
class DatasetInputSource {
 public:
  virtual ~DatasetInputSource() = default;

  [[nodiscard]] virtual std::optional<DatasetPacket> next() = 0;
  virtual void close() noexcept {}
  // A source returns no packet both at EOF and after a decoding failure. This
  // side channel lets DatasetRunner report malformed input instead of
  // silently treating it as a short, successful dataset.
  [[nodiscard]] virtual std::optional<std::string> error() const noexcept {
    return std::nullopt;
  }
};

// Small ROS-independent interchange source for deterministic offline tests
// and dataset conversion tools. Each non-comment line is one packet:
//
//   imu <time_sec> <ax> <ay> <az> <gx> <gy> <gz>
//   cloud <start_sec> <end_sec> <point_count>
//         <x> <y> <z> <intensity> <offset_sec> ...
//
// Point records are repeated point_count times on the same line. The source
// deliberately accepts no ROS serialization or vendor point type.
class TextDatasetInputSource final : public DatasetInputSource {
 public:
  struct Config {
    std::size_t maximum_points_per_cloud = 2'000'000;
  };

  explicit TextDatasetInputSource(std::filesystem::path path);
  TextDatasetInputSource(std::filesystem::path path, Config config);

  [[nodiscard]] std::optional<DatasetPacket> next() override;
  void close() noexcept override;
  [[nodiscard]] std::optional<std::string> error() const noexcept override {
    return error_;
  }
  [[nodiscard]] std::size_t lineNumber() const noexcept { return line_number_; }

 private:
  [[nodiscard]] bool setError(std::string message) noexcept;

  std::ifstream input_;
  Config config_;
  std::filesystem::path path_;
  std::optional<std::string> error_;
  std::size_t line_number_ = 0;
  bool closed_ = false;
};

enum class DatasetRunMode : std::uint8_t {
  kMapping = 0,
  kLocalization,
};

struct DatasetRunOptions {
  DatasetRunMode mode = DatasetRunMode::kMapping;
  MappingOptions mapping;
  LocalizationOptions localization;
  std::size_t maximum_packets = 0;  // zero means no limit
  bool finish_trajectory = true;
};

struct DatasetRunResult {
  SystemStatus status;
  bool reached_end_of_input = false;
  bool finished_trajectory = false;
  std::uint64_t packets_seen = 0;
  std::uint64_t imu_packets = 0;
  std::uint64_t point_cloud_packets = 0;
  std::uint64_t rejected_packets = 0;
  std::optional<hybrid_slam::common::Timestamp> first_timestamp;
  std::optional<hybrid_slam::common::Timestamp> last_timestamp;
};

// ROS-independent deterministic player loop.  It deliberately does not
// know how a frontend, backend or file format works; those remain injected in
// SlamSystem and DatasetInputSource implementations.
class DatasetRunner final {
 public:
  explicit DatasetRunner(SlamSystem& system) noexcept : system_(system) {}

  [[nodiscard]] DatasetRunResult Run(DatasetInputSource& source,
                                     const DatasetRunOptions& options = {});

 private:
  SlamSystem& system_;
};

}  // namespace hybrid_slam::system

#endif  // HYBRID_SLAM_SYSTEM_DATASET_RUNNER_H_
