#include "hybrid_slam/system/dataset_runner.h"

#include <exception>
#include <cmath>
#include <limits>
#include <sstream>
#include <type_traits>
#include <utility>

namespace hybrid_slam::system {

bool DatasetPacket::valid() const noexcept {
  return std::visit(
      [](const auto& payload) {
        using Payload = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<Payload, hybrid_slam::sensor::ImuData>) {
          return payload.valid();
        } else {
          return payload.valid() && !payload.points().empty();
        }
      },
      payload);
}

std::optional<hybrid_slam::common::Timestamp> DatasetPacket::timestamp()
    const noexcept {
  return std::visit(
      [](const auto& payload) -> std::optional<hybrid_slam::common::Timestamp> {
        using Payload = std::decay_t<decltype(payload)>;
        if constexpr (std::is_same_v<Payload, hybrid_slam::sensor::ImuData>) {
          return payload.timestamp;
        } else {
          return payload.endTime();
        }
      },
      payload);
}

TextDatasetInputSource::TextDatasetInputSource(std::filesystem::path path)
    : TextDatasetInputSource(std::move(path), Config{}) {}

TextDatasetInputSource::TextDatasetInputSource(
    std::filesystem::path path, Config config)
    : config_(config), path_(std::move(path)) {
  if (path_.empty()) {
    error_ = "dataset path is empty";
    return;
  }
  if (config_.maximum_points_per_cloud == 0) {
    error_ = "maximum_points_per_cloud must be positive";
    return;
  }
  input_.open(path_);
  if (!input_) {
    error_ = "failed to open dataset file: " + path_.string();
  }
}

bool TextDatasetInputSource::setError(std::string message) noexcept {
  if (!error_) {
    error_ = std::move(message);
  }
  return false;
}

std::optional<DatasetPacket> TextDatasetInputSource::next() {
  if (closed_ || error_) {
    return std::nullopt;
  }

  std::string line;
  while (std::getline(input_, line)) {
    ++line_number_;
    std::istringstream stream(line);
    std::string kind;
    if (!(stream >> kind) || kind.empty() || kind.front() == '#') {
      continue;
    }

    const auto lineError = [this](const std::string& message) {
      return setError("line " + std::to_string(line_number_) + ": " +
                      message);
    };
    const auto finite = [](double value) {
      return std::isfinite(value);
    };
    const auto readDouble = [&stream, &finite](double& value) {
      return static_cast<bool>(stream >> value) && finite(value);
    };

    if (kind == "imu") {
      double time = 0.0;
      double ax = 0.0;
      double ay = 0.0;
      double az = 0.0;
      double gx = 0.0;
      double gy = 0.0;
      double gz = 0.0;
      if (!readDouble(time) || !readDouble(ax) || !readDouble(ay) ||
          !readDouble(az) || !readDouble(gx) || !readDouble(gy) ||
          !readDouble(gz)) {
        lineError("imu requires seven finite numeric values");
        return std::nullopt;
      }
      if (stream >> std::ws && stream.peek() != std::char_traits<char>::eof()) {
        lineError("imu line has trailing tokens");
        return std::nullopt;
      }
      const auto timestamp = hybrid_slam::common::Timestamp::FromSeconds(time);
      if (!timestamp) {
        lineError("imu timestamp is not representable");
        return std::nullopt;
      }
      hybrid_slam::sensor::ImuData imu;
      imu.timestamp = *timestamp;
      imu.frame = hybrid_slam::geometry::FrameId::kImu;
      imu.linear_acceleration_mps2 = Eigen::Vector3d(ax, ay, az);
      imu.angular_velocity_radps = Eigen::Vector3d(gx, gy, gz);
      return DatasetPacket{std::move(imu)};
    }

    if (kind == "cloud") {
      double start = 0.0;
      double end = 0.0;
      long long point_count = 0;
      if (!readDouble(start) || !readDouble(end) ||
          !(stream >> point_count) || point_count < 0 ||
          static_cast<unsigned long long>(point_count) >
              config_.maximum_points_per_cloud) {
        lineError("cloud requires finite start/end and a bounded point count");
        return std::nullopt;
      }
      const auto start_time = hybrid_slam::common::Timestamp::FromSeconds(start);
      const auto end_time = hybrid_slam::common::Timestamp::FromSeconds(end);
      if (!start_time || !end_time || *end_time < *start_time) {
        lineError("cloud timestamps are invalid or not ordered");
        return std::nullopt;
      }

      hybrid_slam::sensor::PointCloudBuffer points;
      points.reserve(static_cast<std::size_t>(point_count));
      for (long long index = 0; index < point_count; ++index) {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double intensity = 0.0;
        double offset = 0.0;
        if (!readDouble(x) || !readDouble(y) || !readDouble(z) ||
            !readDouble(intensity) || !readDouble(offset)) {
          lineError("cloud point record is incomplete or non-finite");
          return std::nullopt;
        }
        const auto offset_time =
            hybrid_slam::common::Timestamp::FromSeconds(offset);
        if (!offset_time ||
            !std::isfinite(static_cast<float>(x)) ||
            !std::isfinite(static_cast<float>(y)) ||
            !std::isfinite(static_cast<float>(z)) ||
            !std::isfinite(static_cast<float>(intensity))) {
          lineError("cloud point value is outside the float domain");
          return std::nullopt;
        }
        points.push_back({static_cast<float>(x), static_cast<float>(y),
                          static_cast<float>(z), static_cast<float>(intensity),
                          offset_time->nanoseconds()});
      }
      if (stream >> std::ws && stream.peek() != std::char_traits<char>::eof()) {
        lineError("cloud line has trailing tokens");
        return std::nullopt;
      }
      return DatasetPacket{hybrid_slam::sensor::PointCloudFrame(
          *start_time, *end_time, hybrid_slam::geometry::FrameId::kLidar,
          std::move(points))};
    }

    lineError("unknown packet type '" + kind + "'");
    return std::nullopt;
  }

  if (input_.bad()) {
    (void)setError("failed while reading dataset file: " + path_.string());
  }
  return std::nullopt;
}

void TextDatasetInputSource::close() noexcept {
  closed_ = true;
  if (input_.is_open()) {
    input_.close();
  }
}

DatasetRunResult DatasetRunner::Run(DatasetInputSource& source,
                                    const DatasetRunOptions& options) {
  DatasetRunResult result;
  result.status = SystemStatus::Ok();

  const auto start_status = [&]() {
    if (options.mode == DatasetRunMode::kMapping) {
      return system_.StartMapping(options.mapping);
    }
    return system_.StartLocalization(options.localization);
  }();
  if (!start_status.ok()) {
    result.status = start_status;
    source.close();
    return result;
  }

  std::optional<hybrid_slam::common::Timestamp> previous_timestamp;
  try {
    for (;;) {
      if (options.maximum_packets != 0 &&
          result.packets_seen >= options.maximum_packets) {
        break;
      }

      std::optional<DatasetPacket> packet = source.next();
      if (!packet) {
        if (const auto source_error = source.error()) {
          result.status = SystemStatus::Error(SystemStatusCode::kInvalidArgument,
                                              *source_error);
        } else {
          result.reached_end_of_input = true;
        }
        break;
      }

      ++result.packets_seen;
      if (!packet->valid()) {
        ++result.rejected_packets;
        result.status = SystemStatus::Error(
            SystemStatusCode::kInvalidArgument,
            "dataset source emitted an invalid packet");
        break;
      }

      const auto packet_timestamp = packet->timestamp();
      if (!packet_timestamp) {
        ++result.rejected_packets;
        result.status = SystemStatus::Error(
            SystemStatusCode::kInvalidArgument,
            "dataset source emitted a packet without a timestamp");
        break;
      }
      if (previous_timestamp && *packet_timestamp < *previous_timestamp) {
        ++result.rejected_packets;
        result.status = SystemStatus::Error(
            SystemStatusCode::kInvalidArgument,
            "dataset source timestamps are not monotonic");
        break;
      }
      previous_timestamp = packet_timestamp;
      if (!result.first_timestamp) {
        result.first_timestamp = packet_timestamp;
      }
      result.last_timestamp = packet_timestamp;

      const SystemStatus packet_status = std::visit(
          [this](auto&& payload) -> SystemStatus {
            using Payload = std::decay_t<decltype(payload)>;
            if constexpr (std::is_same_v<Payload,
                                         hybrid_slam::sensor::ImuData>) {
              return system_.AddImu(std::move(payload));
            } else {
              return system_.AddPointCloud(std::move(payload));
            }
          },
          std::move(packet->payload));

      if (std::holds_alternative<hybrid_slam::sensor::ImuData>(
              packet->payload)) {
        ++result.imu_packets;
      } else {
        ++result.point_cloud_packets;
      }

      if (!packet_status.ok()) {
        ++result.rejected_packets;
        result.status = packet_status;
        break;
      }

      // A typed source may be feeding an asynchronous frontend queue.  Drive
      // that queue through SlamSystem rather than calling a concrete frontend
      // from the dataset runner.  Synchronous/core compositions can omit the
      // optional handler and remain valid DatasetRunner users.
      if (system_.hasProcessHandler()) {
        const auto process_status = system_.ProcessPending();
        if (!process_status.ok()) {
          ++result.rejected_packets;
          result.status = process_status;
          break;
        }
      }
    }
  } catch (const std::exception& error) {
    result.status = SystemStatus::Error(SystemStatusCode::kRejected,
                                        error.what());
  } catch (...) {
    result.status = SystemStatus::Error(SystemStatusCode::kRejected,
                                        "dataset source threw an unknown exception");
  }

  source.close();
  if (result.status.ok() && system_.hasProcessHandler()) {
    // A source may end immediately after enqueueing the final packet.  Give
    // an asynchronous frontend one last system-owned pump before the
    // lifecycle handler closes its ingress and joins its workers.  This keeps
    // DatasetRunner independent of concrete frontend drain APIs while making
    // EOF/maximum-packet completion deterministic.
    result.status = system_.ProcessPending();
  }

  if (options.finish_trajectory && system_.state() != LifecycleState::kIdle) {
    // Always close a session that this runner successfully started, including
    // malformed-input and pump-failure paths.  Preserve the first failure as
    // the run result while still giving the injected lifecycle owner a chance
    // to drain and release its resources.
    const auto finish_status = system_.FinishTrajectory();
    result.finished_trajectory = finish_status.ok();
    if (result.status.ok()) {
      result.status = finish_status;
    }
  }
  return result;
}

}  // namespace hybrid_slam::system
