#ifndef HYBRID_SLAM_SYSTEM_SLAM_SYSTEM_H_
#define HYBRID_SLAM_SYSTEM_SLAM_SYSTEM_H_

#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>

#include "hybrid_slam/common/domain_ids.h"
#include "hybrid_slam/io/map_io.h"
#include "hybrid_slam/localization/localization_map.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/imu_data.h"
#include "hybrid_slam/sensor/point_cloud_frame.h"
#include "hybrid_slam/system/global_alignment_state.h"

namespace hybrid_slam::system {

enum class LifecycleState : std::uint8_t {
  kIdle = 0,
  kMapping,
  kLocalization,
  kStopped,
};

enum class SystemStatusCode : std::uint8_t {
  kOk = 0,
  kInvalidState,
  kInvalidArgument,
  kNotConfigured,
  kClosed,
  kRejected,
};

struct SystemStatus {
  SystemStatusCode code = SystemStatusCode::kOk;
  std::string message;

  [[nodiscard]] bool ok() const noexcept {
    return code == SystemStatusCode::kOk;
  }

  static SystemStatus Ok() { return {}; }
  static SystemStatus Error(SystemStatusCode code, std::string message) {
    return {code, std::move(message)};
  }
};

struct MappingOptions {
  hybrid_slam::common::SessionId session_id{1};
  hybrid_slam::common::TrajectoryId trajectory_id{1};

  [[nodiscard]] bool valid() const noexcept {
    return session_id.value() != 0 && trajectory_id.value() != 0;
  }
};

struct LocalizationOptions {
  std::shared_ptr<const hybrid_slam::localization::LocalizationMap> map;
  hybrid_slam::common::SessionId session_id{1};
  hybrid_slam::common::TrajectoryId trajectory_id{1};
  // ROS lifecycle services may begin localization before a compatibility
  // loader has produced a typed LocalizationMap. Core callers must provide a
  // map unless they explicitly opt into this transitional state.
  bool allow_deferred_map = false;

  [[nodiscard]] bool valid() const noexcept {
    return session_id.value() != 0 && trajectory_id.value() != 0 &&
           (allow_deferred_map || (map && map->valid()));
  }
};

struct SystemSnapshot {
  LifecycleState state = LifecycleState::kIdle;
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map;
  std::shared_ptr<const GlobalAlignmentSnapshot> alignment;
};

// ROS-independent lifecycle and input boundary. The concrete frontend,
// backend and map/IO composition is injected through sinks, so this class can
// be driven by ROS callbacks or a dataset player without importing ROS types.
class SlamSystem final {
 public:
  using ImuSink = std::function<SystemStatus(hybrid_slam::sensor::ImuData)>;
  using PointCloudSink =
      std::function<SystemStatus(hybrid_slam::sensor::PointCloudFrame&&)>;
  // The processing pump is the only system-level entry point that advances
  // a selected frontend after typed sensor values have been queued.  ROS
  // timers and offline dataset players call this seam instead of reaching
  // into a concrete frontend implementation.
  using ProcessHandler = std::function<SystemStatus()>;
  using SaveMapHandler =
      std::function<std::future<SystemStatus>(hybrid_slam::io::MapSaveRequest)>;
  using MappingStartHandler =
      std::function<SystemStatus(const MappingOptions&)>;
  using LocalizationStartHandler =
      std::function<SystemStatus(const LocalizationOptions&)>;
  using LifecycleHandler = std::function<SystemStatus()>;
  using SnapshotProvider =
      std::function<std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>()>;
  using AlignmentProvider =
      std::function<std::shared_ptr<const GlobalAlignmentSnapshot>()>;

  SlamSystem() = default;

  SlamSystem(const SlamSystem&) = delete;
  SlamSystem& operator=(const SlamSystem&) = delete;

  void setImuSink(ImuSink sink);
  void setPointCloudSink(PointCloudSink sink);
  void setProcessHandler(ProcessHandler handler);
  void setSaveMapHandler(SaveMapHandler handler);
  void setMappingStartHandler(MappingStartHandler handler);
  void setLocalizationStartHandler(LocalizationStartHandler handler);
  void setFinishTrajectoryHandler(LifecycleHandler handler);
  void setStopHandler(LifecycleHandler handler);
  void setResetHandler(LifecycleHandler handler);
  void setSnapshotProvider(SnapshotProvider provider);
  void setAlignmentProvider(AlignmentProvider provider);

  [[nodiscard]] SystemStatus StartMapping(
      const MappingOptions& options = {});
  [[nodiscard]] SystemStatus StartLocalization(
      const LocalizationOptions& options);

  [[nodiscard]] SystemStatus AddImu(hybrid_slam::sensor::ImuData imu);
  [[nodiscard]] SystemStatus AddPointCloud(
      hybrid_slam::sensor::PointCloudFrame&& cloud);

  // Advance the selected frontend using already submitted typed input.  The
  // shared lifecycle lock makes Finish/Stop/Reset wait for an in-flight pump,
  // just as they wait for an in-flight sensor submission.  The handler is
  // optional for synchronous/core compositions whose sinks process input
  // immediately; DatasetRunner checks hasProcessHandler() before calling it.
  [[nodiscard]] SystemStatus ProcessPending();
  [[nodiscard]] bool hasProcessHandler() const noexcept;

  [[nodiscard]] std::future<SystemStatus> SaveMapAsync(
      hybrid_slam::io::MapSaveRequest request);

  [[nodiscard]] SystemStatus FinishTrajectory();
  [[nodiscard]] SystemStatus Stop();
  [[nodiscard]] SystemStatus Reset();

  [[nodiscard]] LifecycleState state() const noexcept;
  [[nodiscard]] bool acceptingSensorData() const noexcept;
  [[nodiscard]] SystemSnapshot Snapshot() const;

 private:
  [[nodiscard]] static std::future<SystemStatus> readyFuture(
      SystemStatus status);

  // Lifecycle transitions take the exclusive side of this lock. Sensor
  // submission takes the shared side, so an in-flight callback completes
  // before Finish/Stop/Reset commits the next lifecycle state, while normal
  // independent ingress does not serialize on the lifecycle transition path.
  mutable std::shared_mutex lifecycle_mutex_;
  mutable std::mutex mutex_;
  LifecycleState state_ = LifecycleState::kIdle;
  hybrid_slam::common::SessionId session_id_;
  hybrid_slam::common::TrajectoryId trajectory_id_;
  std::shared_ptr<const hybrid_slam::localization::LocalizationMap>
      localization_map_;
  ImuSink imu_sink_;
  PointCloudSink point_cloud_sink_;
  ProcessHandler process_handler_;
  SaveMapHandler save_map_handler_;
  MappingStartHandler mapping_start_handler_;
  LocalizationStartHandler localization_start_handler_;
  LifecycleHandler finish_trajectory_handler_;
  LifecycleHandler stop_handler_;
  LifecycleHandler reset_handler_;
  SnapshotProvider snapshot_provider_;
  AlignmentProvider alignment_provider_;
};

}  // namespace hybrid_slam::system

#endif  // HYBRID_SLAM_SYSTEM_SLAM_SYSTEM_H_
