#ifndef LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_PIPELINE_H_
#define LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_PIPELINE_H_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <thread>
#include <vector>

#include "basic/alias.h"
#include "common/ds.h"
#include "adagraph/backend/backend_scheduler.h"
#include "adagraph/backend/isam2_optimizer.h"
#include "adagraph/backend/odometry_covariance_estimator.h"
#include "adagraph/backend/keyframe_policy.h"
#include "adagraph/frontend/frontend_config.h"
#include "adagraph/io/map_io.h"
#include "adagraph/system/global_alignment_state.h"
#include "adagraph/system/slam_system.h"
#include "lio/prior_map_localization.h"
#include "lio/system/legacy_runtime_config.h"
#include "lio/system/legacy_downstream_frame.h"

namespace LI2Sup {

enum class LegacyFrontendKind {
  kSuperLio = 0,
  kPointLio,
};

// Output and ROS-session effects are injected at the application boundary.
// The downstream algorithms themselves remain usable without including
// ROSWrapper or any ROS message type.  Empty callbacks intentionally mean
// "publication disabled", which is useful for offline and unit-test
// compositions.
struct LegacyDownstreamCallbacks {
  using MapSnapshotPtr =
      std::shared_ptr<const adagraph::mapping::MapSnapshot>;

  std::function<void(MapSnapshotPtr)> publish_backend_snapshot;
  std::function<void(const BASIC::CloudPtr&)> publish_global_map;
  std::function<void(bool)> set_prior_map_localization_active;
  std::function<void()> reset_session_state;
  std::function<void(const BASIC::SE3&)> set_map_to_odom;
  std::function<bool(BASIC::SE3&)> get_map_to_odom;
  std::function<bool()> consume_manual_relocalization_request;
  std::function<bool()> occupancy_grid_enabled;
  std::function<void(const BASIC::CloudPtr&, const NavState&)>
      update_occupancy_grid;
  std::function<void(const NavState&, const BASIC::V3&)> publish_odom;
  std::function<void(const BASIC::CloudPtr&, double)> publish_cloud_world;
};

// Transitional system boundary. The typed BackendScheduler/MapManager path is
// authoritative for standard graph and Snapshot state. The old AdaGraph
// facade is constructed only for the explicit backend-refinement compatibility
// mode, where its optimized pose is still consumed by the legacy localization
// smoothing policy. This keeps rollback behavior available without making a
// second graph a permanent standard-path owner.
class LegacyDownstreamPipeline {
 public:
  explicit LegacyDownstreamPipeline(
      LegacyDownstreamCallbacks callbacks = {});
  ~LegacyDownstreamPipeline();

  LegacyDownstreamPipeline(const LegacyDownstreamPipeline&) = delete;
  LegacyDownstreamPipeline& operator=(const LegacyDownstreamPipeline&) =
      delete;

  // Runtime values are always supplied by the application/configuration
  // adapter.  Keeping this boundary explicit prevents a downstream instance
  // from silently capturing process-wide legacy g_* state.
  void init(LegacyFrontendKind frontend_kind,
            adagraph::frontend::FrontendConfig frontend_config,
            LegacyRuntimeConfig runtime_config);
  // Lifecycle/application identity for typed graph/map records. It must be
  // set before the first frame of a session; the default is (1, 1) for
  // direct/offline compositions that do not expose lifecycle options.
  void setTrajectoryIdentity(adagraph::common::SessionId session_id,
                             adagraph::common::TrajectoryId trajectory_id);
  // Tear down session-owned backend/localization/map state and reconstruct it
  // with the same wrapper, frontend kind and validated frontend config.
  void reset();
  void processPointFrame(LegacyPointFrame&& frame,
                         const IMUData& latest_imu);
  [[nodiscard]] LegacySuperBuffers enqueueSuperFrame(
      LegacySuperFrame&& frame);
  void finish();
  void savePointMap();
  void saveSuperMap();

  [[nodiscard]] std::shared_ptr<const adagraph::mapping::MapSnapshot>
  mapSnapshot() const noexcept;
  [[nodiscard]] std::shared_ptr<const adagraph::system::GlobalAlignmentSnapshot>
  alignmentSnapshot() const noexcept;
  [[nodiscard]] std::shared_ptr<const adagraph::localization::LocalizationMap>
  localizationMap() const noexcept;
  [[nodiscard]] std::future<adagraph::system::SystemStatus>
  saveMapAsync(adagraph::io::MapSaveRequest request);
  [[nodiscard]] bool acceptingInput() const noexcept {
    return initialized_.load(std::memory_order_acquire) &&
           !finished_.load(std::memory_order_acquire);
  }

 private:
  [[nodiscard]] bool processPointBackend(
      const LegacyPointFrame& frame, const BASIC::SE3& odom_body,
      const BASIC::SE3& last_keyframe_pose,
      const BASIC::CloudPtr& matching_cloud, const IMUData& latest_imu);

  static BASIC::CloudPtr toCloud(
      const std::vector<Eigen::Vector3f>& points);
  static BASIC::CloudPtr toBodyCloud(
      const std::vector<Eigen::Vector3f>& odom_points,
      const BASIC::SE3& odom_body);
  static adagraph::geometry::RigidTransform3d toDomainPose(
      const BASIC::SE3& pose);
  static BASIC::SE3 toLegacyPose(
      const adagraph::geometry::RigidTransform3d& pose);
  [[nodiscard]] std::optional<adagraph::geometry::StampedTransform3d>
  toOdomBase(const BASIC::SE3& T_odom_imu,
             double timestamp_seconds) const;
  [[nodiscard]] BASIC::CloudPtr toBaseCloud(
      const BASIC::CloudPtr& cloud_imu) const;
  [[nodiscard]] bool submitGraphKeyFrame(
      const BASIC::SE3& T_odom_imu, double timestamp_seconds,
      const BASIC::CloudPtr& cloud_imu, const FeatureFrame& features,
      const std::vector<std::pair<BASIC::M6, BASIC::V6>>& hessian_terms,
      std::size_t input_point_count,
      std::size_t effective_constraint_count);
  void submitMapObservations(
      const std::vector<MapObservation>& map_observations);
  void enqueueRelocalizationUpdate(
      const adagraph::localization::RelocalizationUpdate& update);
  void applyPendingRelocalizationUpdate();
  [[nodiscard]] bool currentMapToOdom(BASIC::SE3& T_map_odom) const;
  [[nodiscard]] bool commitGlobalAlignment(
      const adagraph::geometry::StampedTransform3d& T_map_base,
      const adagraph::geometry::StampedTransform3d& T_odom_base,
      adagraph::system::GlobalAlignmentSource source,
      double score);
  void applyBackendAlignmentFromSnapshot();
  void processSuperFrame(const LegacySuperFrame& frame);
  void accumulateSuperMap(const LegacySuperFrame& frame);
  bool prepareSuperPcdDirectory();
  bool enqueueSuperFragmentSave();
  bool enqueueSuperCachedMapMerge();
  void saveSnapshotMap();
  bool enqueueSnapshotMap(
      const std::shared_ptr<const adagraph::mapping::MapSnapshot>& snapshot,
      const std::filesystem::path& output_path);
  bool enqueuePointCloudSave(const BASIC::CloudPtr& cloud,
                             const std::filesystem::path& output_path,
                             std::uint64_t snapshot_version = 0,
                             double voxel_leaf_size = 0.0);
  void finishMapIo();
  void ensureMapIo();
  void ensureMapIoLocked();
  [[nodiscard]] std::uint64_t allocateMapSaveRequestIdLocked(
      std::uint64_t requested_id);
  [[nodiscard]] adagraph::io::MapSaveSubmissionStatus submitMapIoRequest(
      adagraph::io::MapSaveRequest request);
  [[nodiscard]] bool mapIoAvailable() const noexcept;
  void completeMapSave(const adagraph::io::MapSaveResult& result) noexcept;
  void startSuperWorker();
  void stopSuperWorker(bool drain = true);
  void superWorkerLoop();
  [[nodiscard]] LegacySuperBuffers acquireSuperBuffers();
  void recycleSuperBuffers(LegacySuperFrame&& frame);

  LegacyDownstreamCallbacks callbacks_;
  LegacyRuntimeConfig runtime_config_;
  adagraph::backend::BackendConfig backend_config_;
  std::unique_ptr<adagraph::backend::KeyFramePolicy> keyframe_policy_;
  std::unique_ptr<adagraph::backend::BackendScheduler> backend_scheduler_;
  std::unique_ptr<adagraph::backend::OdometryCovarianceEstimator>
      covariance_estimator_;
  std::unique_ptr<adagraph::io::MapSaveScheduler> map_io_scheduler_;
  std::unique_ptr<PriorMapLocalization> prior_map_localization_;
  std::unique_ptr<adagraph::system::GlobalAlignmentState>
      global_alignment_state_;
  adagraph::frontend::FrontendConfig frontend_config_;
  // Point-LIO final output is derived from the immutable MapSnapshot. Super-
  // LIO retains only a bounded fragment buffer for its explicit legacy
  // fragment-save compatibility mode.
  BASIC::CloudPtr super_accumulated_map_;
  BASIC::CloudPtr super_world_cloud_;
  LegacyFrontendKind frontend_kind_ = LegacyFrontendKind::kSuperLio;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> finished_{false};
  std::uint64_t next_graph_keyframe_id_ = 0;
  std::uint64_t accepted_keyframe_count_ = 0;
  adagraph::common::SessionId session_id_{1};
  adagraph::common::TrajectoryId trajectory_id_{1};
  std::uint64_t next_global_alignment_version_ = 1;
  std::uint64_t last_backend_alignment_snapshot_version_ = 0;
  int point_grid_count_ = 0;
  int point_visualization_count_ = 0;
  int super_grid_count_ = -1;
  int super_visualization_count_ = -1;
  int super_scan_wait_count_ = 0;
  int super_pcd_index_ = -1;
  bool super_pcd_dir_prepared_ = false;
  std::vector<std::filesystem::path> super_pcd_paths_;
  mutable std::mutex map_io_state_mutex_;
  // Serializes potentially blocking submissions with scheduler shutdown. It
  // prevents finishMapIo() from waiting on the state lock while submit() is
  // waiting for queue capacity.
  mutable std::mutex map_io_submit_mutex_;
  std::uint64_t next_map_save_request_id_ = 1;
  bool map_io_finished_ = false;
  mutable std::mutex map_save_mutex_;
  std::unordered_map<std::uint64_t, std::promise<adagraph::system::SystemStatus>>
      pending_map_saves_;

  std::mutex super_queue_mutex_;
  std::condition_variable super_queue_not_empty_cv_;
  std::condition_variable super_queue_not_full_cv_;
  std::deque<LegacySuperFrame> super_queue_;
  std::thread super_worker_;
  std::size_t super_queue_capacity_ = 16;
  bool super_worker_stop_ = false;
  bool super_worker_started_ = false;

  std::mutex super_buffer_pool_mutex_;
  std::vector<LegacySuperBuffers> super_buffer_pool_;

  std::atomic<std::uint64_t> super_frames_enqueued_{0};
  std::atomic<std::uint64_t> super_frames_processed_{0};
  std::atomic<std::uint64_t> super_queue_waits_{0};
  std::atomic<std::uint64_t> super_max_queue_depth_{0};
  std::atomic<std::uint64_t> super_downstream_time_ns_{0};

  adagraph::system::GlobalAlignmentUpdateMailbox global_alignment_mailbox_;
  std::uint64_t alignment_updates_committed_ = 0;
  std::uint64_t alignment_updates_rejected_ = 0;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_LEGACY_DOWNSTREAM_PIPELINE_H_
