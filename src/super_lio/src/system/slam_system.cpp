#include "adagraph/system/slam_system.h"

#include <exception>
#include <utility>

namespace adagraph::system {

namespace {

SystemStatus activeStateError(const LifecycleState state) {
  if (state == LifecycleState::kStopped) {
    return SystemStatus::Error(SystemStatusCode::kClosed,
                               "SlamSystem is stopped");
  }
  return SystemStatus::Error(SystemStatusCode::kInvalidState,
                             "SlamSystem is already active");
}

}  // namespace

void SlamSystem::setImuSink(ImuSink sink) {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_sink_ = std::move(sink);
}

void SlamSystem::setPointCloudSink(PointCloudSink sink) {
  std::lock_guard<std::mutex> lock(mutex_);
  point_cloud_sink_ = std::move(sink);
}

void SlamSystem::setProcessHandler(ProcessHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  process_handler_ = std::move(handler);
}

void SlamSystem::setSaveMapHandler(SaveMapHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  save_map_handler_ = std::move(handler);
}

void SlamSystem::setMappingStartHandler(MappingStartHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  mapping_start_handler_ = std::move(handler);
}

void SlamSystem::setLocalizationStartHandler(
    LocalizationStartHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  localization_start_handler_ = std::move(handler);
}

void SlamSystem::setFinishTrajectoryHandler(LifecycleHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  finish_trajectory_handler_ = std::move(handler);
}

void SlamSystem::setStopHandler(LifecycleHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  stop_handler_ = std::move(handler);
}

void SlamSystem::setResetHandler(LifecycleHandler handler) {
  std::lock_guard<std::mutex> lock(mutex_);
  reset_handler_ = std::move(handler);
}

void SlamSystem::setSnapshotProvider(SnapshotProvider provider) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_provider_ = std::move(provider);
}

void SlamSystem::setAlignmentProvider(AlignmentProvider provider) {
  std::lock_guard<std::mutex> lock(mutex_);
  alignment_provider_ = std::move(provider);
}

SystemStatus SlamSystem::StartMapping(const MappingOptions& options) {
  std::unique_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!options.valid()) {
    return SystemStatus::Error(SystemStatusCode::kInvalidArgument,
                               "invalid mapping session or trajectory id");
  }
  MappingStartHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kIdle) {
      return activeStateError(state_);
    }
    handler = mapping_start_handler_;
  }
  if (handler) {
    try {
      const auto status = handler(options);
      if (!status.ok()) {
        return status;
      }
    } catch (const std::exception& error) {
      return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
    } catch (...) {
      return SystemStatus::Error(SystemStatusCode::kRejected,
                                 "mapping start handler threw an unknown exception");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kIdle) {
    return activeStateError(state_);
  }
  state_ = LifecycleState::kMapping;
  session_id_ = options.session_id;
  trajectory_id_ = options.trajectory_id;
  localization_map_.reset();
  return SystemStatus::Ok();
}

SystemStatus SlamSystem::StartLocalization(
    const LocalizationOptions& options) {
  std::unique_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!options.valid()) {
    return SystemStatus::Error(
        SystemStatusCode::kInvalidArgument,
        "localization requires a valid typed map unless deferred loading is "
        "explicitly enabled");
  }
  LocalizationStartHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kIdle) {
      return activeStateError(state_);
    }
    handler = localization_start_handler_;
  }
  if (handler) {
    try {
      const auto status = handler(options);
      if (!status.ok()) {
        return status;
      }
    } catch (const std::exception& error) {
      return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
    } catch (...) {
      return SystemStatus::Error(
          SystemStatusCode::kRejected,
          "localization start handler threw an unknown exception");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != LifecycleState::kIdle) {
    return activeStateError(state_);
  }
  state_ = LifecycleState::kLocalization;
  session_id_ = options.session_id;
  trajectory_id_ = options.trajectory_id;
  localization_map_ = options.map;
  return SystemStatus::Ok();
}

SystemStatus SlamSystem::AddImu(adagraph::sensor::ImuData imu) {
  std::shared_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!imu.valid()) {
    return SystemStatus::Error(SystemStatusCode::kInvalidArgument,
                               "invalid IMU sample");
  }
  ImuSink sink;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kMapping &&
        state_ != LifecycleState::kLocalization) {
      return SystemStatus::Error(SystemStatusCode::kInvalidState,
                                  "IMU submitted while system is inactive");
    }
    sink = imu_sink_;
  }
  if (!sink) {
    return SystemStatus::Error(SystemStatusCode::kNotConfigured,
                               "IMU sink is not configured");
  }
  try {
    return sink(std::move(imu));
  } catch (const std::exception& error) {
    return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
  } catch (...) {
    return SystemStatus::Error(SystemStatusCode::kRejected,
                               "IMU sink threw an unknown exception");
  }
}

SystemStatus SlamSystem::AddPointCloud(
    adagraph::sensor::PointCloudFrame&& cloud) {
  std::shared_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!cloud.valid() || cloud.points().empty()) {
    return SystemStatus::Error(SystemStatusCode::kInvalidArgument,
                               "invalid or empty point-cloud frame");
  }
  PointCloudSink sink;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kMapping &&
        state_ != LifecycleState::kLocalization) {
      return SystemStatus::Error(
          SystemStatusCode::kInvalidState,
          "point cloud submitted while system is inactive");
    }
    sink = point_cloud_sink_;
  }
  if (!sink) {
    return SystemStatus::Error(SystemStatusCode::kNotConfigured,
                               "point-cloud sink is not configured");
  }
  try {
    return sink(std::move(cloud));
  } catch (const std::exception& error) {
    return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
  } catch (...) {
    return SystemStatus::Error(SystemStatusCode::kRejected,
                               "point-cloud sink threw an unknown exception");
  }
}

SystemStatus SlamSystem::ProcessPending() {
  std::shared_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  ProcessHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ != LifecycleState::kMapping &&
        state_ != LifecycleState::kLocalization) {
      if (state_ == LifecycleState::kStopped) {
        return SystemStatus::Error(SystemStatusCode::kClosed,
                                   "SlamSystem is stopped");
      }
      return SystemStatus::Error(
          SystemStatusCode::kInvalidState,
          "frontend processing requested while system is inactive");
    }
    handler = process_handler_;
  }
  if (!handler) {
    return SystemStatus::Error(
        SystemStatusCode::kNotConfigured,
        "frontend processing handler is not configured");
  }
  try {
    return handler();
  } catch (const std::exception& error) {
    return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
  } catch (...) {
    return SystemStatus::Error(
        SystemStatusCode::kRejected,
        "frontend processing handler threw an unknown exception");
  }
}

bool SlamSystem::hasProcessHandler() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<bool>(process_handler_);
}

std::future<SystemStatus> SlamSystem::readyFuture(SystemStatus status) {
  std::promise<SystemStatus> promise;
  auto future = promise.get_future();
  promise.set_value(std::move(status));
  return future;
}

std::future<SystemStatus> SlamSystem::SaveMapAsync(
    adagraph::io::MapSaveRequest request) {
  std::shared_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  if (!request.valid()) {
    return readyFuture(SystemStatus::Error(
        SystemStatusCode::kInvalidArgument, "invalid map-save request"));
  }
  SaveMapHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == LifecycleState::kStopped) {
      return readyFuture(SystemStatus::Error(SystemStatusCode::kClosed,
                                             "SlamSystem is stopped"));
    }
    handler = save_map_handler_;
  }
  if (!handler) {
    return readyFuture(SystemStatus::Error(
        SystemStatusCode::kNotConfigured, "map-save handler is not configured"));
  }
  try {
    return handler(std::move(request));
  } catch (const std::exception& error) {
    return readyFuture(SystemStatus::Error(SystemStatusCode::kRejected,
                                           error.what()));
  } catch (...) {
    return readyFuture(SystemStatus::Error(
        SystemStatusCode::kRejected, "map-save handler threw an unknown exception"));
  }
}

SystemStatus SlamSystem::FinishTrajectory() {
  std::unique_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  LifecycleHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == LifecycleState::kStopped) {
      return SystemStatus::Error(SystemStatusCode::kClosed,
                                 "SlamSystem is stopped");
    }
    handler = finish_trajectory_handler_;
  }
  if (handler) {
    try {
      const auto status = handler();
      if (!status.ok()) {
        return status;
      }
    } catch (const std::exception& error) {
      return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
    } catch (...) {
      return SystemStatus::Error(SystemStatusCode::kRejected,
                                 "finish handler threw an unknown exception");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ == LifecycleState::kStopped) {
    return SystemStatus::Error(SystemStatusCode::kClosed,
                               "SlamSystem is stopped");
  }
  state_ = LifecycleState::kIdle;
  localization_map_.reset();
  return SystemStatus::Ok();
}

SystemStatus SlamSystem::Stop() {
  std::unique_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  LifecycleHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == LifecycleState::kStopped) {
      return SystemStatus::Ok();
    }
    handler = stop_handler_;
  }
  if (handler) {
    try {
      const auto status = handler();
      if (!status.ok()) {
        return status;
      }
    } catch (const std::exception& error) {
      return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
    } catch (...) {
      return SystemStatus::Error(SystemStatusCode::kRejected,
                                 "stop handler threw an unknown exception");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LifecycleState::kStopped;
  localization_map_.reset();
  return SystemStatus::Ok();
}

SystemStatus SlamSystem::Reset() {
  std::unique_lock<std::shared_mutex> lifecycle_lock(lifecycle_mutex_);
  LifecycleHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = reset_handler_;
  }
  if (handler) {
    try {
      const auto status = handler();
      if (!status.ok()) {
        return status;
      }
    } catch (const std::exception& error) {
      return SystemStatus::Error(SystemStatusCode::kRejected, error.what());
    } catch (...) {
      return SystemStatus::Error(SystemStatusCode::kRejected,
                                 "reset handler threw an unknown exception");
    }
  }
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = LifecycleState::kIdle;
  session_id_ = adagraph::common::SessionId{};
  trajectory_id_ = adagraph::common::TrajectoryId{};
  localization_map_.reset();
  return SystemStatus::Ok();
}

LifecycleState SlamSystem::state() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool SlamSystem::acceptingSensorData() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ == LifecycleState::kMapping ||
         state_ == LifecycleState::kLocalization;
}

SystemSnapshot SlamSystem::Snapshot() const {
  SnapshotProvider snapshot_provider;
  AlignmentProvider alignment_provider;
  SystemSnapshot result;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    result.state = state_;
    result.session_id = session_id_;
    result.trajectory_id = trajectory_id_;
    snapshot_provider = snapshot_provider_;
    alignment_provider = alignment_provider_;
  }
  if (snapshot_provider) {
    result.map = snapshot_provider();
  }
  if (alignment_provider) {
    result.alignment = alignment_provider();
  }
  return result;
}

}  // namespace adagraph::system
