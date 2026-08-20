#ifndef LI2SUP_SYSTEM_POINT_LIO_PIPELINE_H_
#define LI2SUP_SYSTEM_POINT_LIO_PIPELINE_H_

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <unordered_map>
#include <vector>

#include "basic/Manifold.h"
#include "hybrid_slam/adapter/pcl_types.h"
#include "common/navigation_state.h"
#include "hybrid_slam/backend/backend_scheduler.h"
#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/backend/isam2_optimizer.h"
#include "hybrid_slam/backend/odometry_covariance_estimator.h"
#include "hybrid_slam/backend/keyframe_policy.h"
#include "hybrid_slam/frontend/frontend_config.h"
#include "hybrid_slam/frontend/frontend_health.h"
#include "hybrid_slam/io/map_io.h"
#include "hybrid_slam/system/global_alignment_state.h"
#include "hybrid_slam/system/slam_system.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "hybrid_slam/sensor/point_cloud_view.h"
#include "lio/prior_map_localization.h"
#include "lio/system/point_lio_runtime_config.h"
#include "lio/system/point_lio_output_frame.h"

namespace LI2Sup {

struct RecoveredTrajectoryStart {
  hybrid_slam::common::SessionId session_id;
  hybrid_slam::common::TrajectoryId trajectory_id;
  hybrid_slam::geometry::StampedTransform3d T_map_base;
  hybrid_slam::geometry::StampedTransform3d T_odom_base;
  hybrid_slam::backend::Covariance6d covariance =
      hybrid_slam::backend::Covariance6d::Identity();
  double score = 0.0;

  [[nodiscard]] bool valid() const noexcept {
    return session_id.value() != 0 && trajectory_id.value() != 0 &&
           T_map_base.valid() && T_odom_base.valid() &&
           T_map_base.timestamp == T_odom_base.timestamp &&
           T_map_base.target_frame == hybrid_slam::geometry::FrameId::kMap &&
           T_map_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           T_odom_base.target_frame == hybrid_slam::geometry::FrameId::kOdom &&
           T_odom_base.source_frame == hybrid_slam::geometry::FrameId::kBase &&
           hybrid_slam::backend::IsValidCovariance6d(covariance) &&
           std::isfinite(score);
  }
};

// Output and ROS-session effects are injected at the application boundary.
// The downstream algorithms themselves remain usable without including
// any ROS message type. Empty callbacks intentionally mean
// "publication disabled", which is useful for offline and unit-test
// compositions.
struct PointLIOPipelineCallbacks {
  using MapSnapshotPtr =
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>;
  using PointCloud = hybrid_slam::sensor::ImmutablePointCloud;

  struct TransformPublicationStats {
    std::uint64_t odom_base_published = 0;
    std::uint64_t map_odom_published = 0;
    std::uint64_t nonmonotonic_odom_base_stamps = 0;
    std::uint64_t nonmonotonic_map_odom_stamps = 0;
  };

  // Output. Invoked from the downstream worker and allowed to be slow: these
  // serialize clouds, raycast the occupancy grid and hand buffers to ROS. None
  // of them may run while the recovery transition lock is held, because the
  // frontend watchdog and the recovery scheduler would then queue behind
  // publication work.
  struct Publication {
    std::function<void(MapSnapshotPtr)> publish_backend_snapshot;
    std::function<void(const PointCloud&)> publish_global_map;
    std::function<void(const NavState&, const Eigen::Vector3d&)> publish_odom;
    std::function<void(const PointCloud&, double)> publish_cloud_world;
    std::function<bool()> occupancy_grid_enabled;
    std::function<void(const PointCloud&, const NavState&)>
        update_occupancy_grid;
    // Publication evidence read back at shutdown only. The pipeline stays
    // usable without any TF implementation.
    std::function<TransformPublicationStats()> get_transform_publication_stats;
  } publication;

  // Frontend admission control. Each one must be cheap and must never wait for
  // localization, backend or registration work; the estimator thread is on the
  // other side of them.
  struct FrontendControl {
    // Changes only future vendor map admission.
    std::function<void(bool)> set_frontend_local_map_write_enabled;
    // Enables the IMU-only recovery query path while health is LOST.
    std::function<void(bool)> set_frontend_recovery_query_mode;
    // Called after a failed recovery verification. The adapter pauses typed
    // ingress, discards queued packets, clears only the provisional online
    // layer and resumes ingress; the read-only prior layer is retained.
    std::function<void()> clear_frontend_online_local_map;
  } frontend;

  // Global alignment exchange with the ROS/TF adapter.
  struct Alignment {
    std::function<void(const hybrid_slam::geometry::RigidTransform3d&)>
        set_map_to_odom;
    std::function<bool(hybrid_slam::geometry::RigidTransform3d&)>
        get_map_to_odom;
    std::function<void(bool)> set_prior_map_localization_active;
  } alignment;

  // Recovery hand-off. Both must only enqueue background work and return
  // promptly; they are called from the downstream worker while health is LOST.
  struct Recovery {
    std::function<void(
        const hybrid_slam::localization::RelocalizationQuery&,
        MapSnapshotPtr)> request_submap_recovery;
    std::function<bool()> consume_manual_relocalization_request;
  } recovery;

  struct Session {
    std::function<void()> reset_session_state;
  } session;
};

// Point-LIO system boundary. The typed BackendScheduler/MapManager path is
// authoritative for graph and snapshot state.
class PointLIOPipeline {
 public:
  explicit PointLIOPipeline(PointLIOPipelineCallbacks callbacks = {});
  ~PointLIOPipeline();

  PointLIOPipeline(const PointLIOPipeline&) = delete;
  PointLIOPipeline& operator=(const PointLIOPipeline&) =
      delete;

  // Runtime values are always supplied by the application/configuration
  // adapter. Keeping this boundary explicit prevents a downstream instance
  // from capturing process-wide state.
  void init(hybrid_slam::frontend::FrontendConfig frontend_config,
            PointLIORuntimeConfig runtime_config);
  // Lifecycle/application identity for typed graph/map records. It must be
  // set before the first frame of a session; the default is (1, 1) for
  // direct/offline compositions that do not expose lifecycle options.
  void setTrajectoryIdentity(hybrid_slam::common::SessionId session_id,
                             hybrid_slam::common::TrajectoryId trajectory_id);
  // Preserve the graph/map and begin a new frontend odom segment after a
  // verified recovery. The initial absolute constraint is queued only after
  // the first new keyframe, so it cannot be dropped before that node exists.
  [[nodiscard]] bool beginRecoveredTrajectory(
      const RecoveredTrajectoryStart& start);
  // Two-phase recovery API used by PointLIORecoveryCoordinator. Prepare does
  // validation and fallible allocation without changing the active graph
  // identity or global alignment. Commit is called only after the frontend
  // has reset and loaded its validated prior seed.
  [[nodiscard]] bool prepareRecoveredTrajectory(
      const RecoveredTrajectoryStart& start);
  [[nodiscard]] bool commitPreparedRecoveredTrajectory();
  void abortPreparedRecoveredTrajectory() noexcept;
  // Recovery scheduler lifecycle notifications. They are instance-owned and
  // safe to call from the scheduler callback thread.
  void beginRelocalizationAttempt() noexcept;
  void markRecoveryAttemptFailed() noexcept;
  // Enqueue a verified asynchronous loop constraint. The registration worker
  // remains outside the backend; this method only crosses the typed queue.
  [[nodiscard]] bool submitLoopClosureConstraint(
      hybrid_slam::backend::Constraint constraint);
  // Tear down session-owned backend/localization/map state and reconstruct it
  // with the same wrapper and validated frontend config.
  void reset();
  void processPointFrame(PointLIOOutputFrame&& frame,
                         const IMUData& latest_imu);
  // Called by the application watchdog when no frontend frame has arrived
  // within the configured health timestamp gap. This keeps LiDAR dropout
  // observable even though Point-LIO cannot emit a frame without a scan.
  void observeFrontendTimeout();
  // Finalizes a non-empty submap without closing the backend, so loop
  // verification can still submit a constraint against the final snapshot.
  [[nodiscard]] bool finishActiveSubmaps();
  void finish();
  void saveMap();

  [[nodiscard]] std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>
  mapSnapshot() const noexcept;
  [[nodiscard]] std::shared_ptr<const hybrid_slam::system::GlobalAlignmentSnapshot>
  alignmentSnapshot() const noexcept;
  [[nodiscard]] std::shared_ptr<const hybrid_slam::localization::LocalizationMap>
  localizationMap() const noexcept;
  [[nodiscard]] std::future<hybrid_slam::system::SystemStatus>
  saveMapAsync(hybrid_slam::io::MapSaveRequest request);
  [[nodiscard]] bool acceptingInput() const noexcept {
    return initialized_.load(std::memory_order_acquire) &&
           !finished_.load(std::memory_order_acquire);
  }

 private:
  [[nodiscard]] bool processPointBackend(
      const PointLIOOutputFrame& frame, const BASIC::SE3& odom_body,
      const BASIC::SE3& last_keyframe_pose,
      const hybrid_slam::adapter::PclCloudPtr& matching_cloud, const IMUData& latest_imu);
  [[nodiscard]] hybrid_slam::frontend::FrontendHealthDecision
  updateFrontendHealth(const PointLIOOutputFrame& frame,
                       const IMUData& latest_imu);
  void applyFrontendHealthDecision(
      hybrid_slam::frontend::FrontendHealthState previous_state,
      const hybrid_slam::frontend::FrontendHealthDecision& decision);
  void maybeRequestSubmapRecovery(
      hybrid_slam::frontend::FrontendHealthState state,
      const std::optional<hybrid_slam::localization::RelocalizationQuery>&
          query);
  [[nodiscard]] std::optional<
      hybrid_slam::localization::RelocalizationQuery>
  appendRecoveryQueryFrame(
      const std::vector<Eigen::Vector3f>& points_odom,
      const hybrid_slam::geometry::StampedTransform3d& T_odom_base);
  void resetRecoveryQueryAccumulator() noexcept;

  static hybrid_slam::adapter::PclCloudPtr toCloud(
      const std::vector<Eigen::Vector3f>& points);
  static hybrid_slam::sensor::PointCloudView toPointCloudView(
      const hybrid_slam::adapter::PclCloudPtr& cloud,
      hybrid_slam::geometry::FrameId frame);
  // Point-LIO emits registered points in the IMU frame's odom pose. This
  // helper removes only T_odom_imu and therefore returns an IMU-frame cloud;
  // callers that need base coordinates must apply T_base_imu afterwards.
  static hybrid_slam::adapter::PclCloudPtr toImuCloud(
      const std::vector<Eigen::Vector3f>& odom_points,
      const BASIC::SE3& odom_body,
      std::size_t maximum_points = std::numeric_limits<std::size_t>::max());
  static hybrid_slam::geometry::RigidTransform3d toDomainPose(
      const BASIC::SE3& pose);
  static BASIC::SE3 toBasicPose(
      const hybrid_slam::geometry::RigidTransform3d& pose);
  [[nodiscard]] std::optional<hybrid_slam::geometry::StampedTransform3d>
  toOdomBase(const BASIC::SE3& T_odom_imu,
             double timestamp_seconds) const;
  [[nodiscard]] hybrid_slam::adapter::PclCloudPtr toBaseCloud(
      const hybrid_slam::adapter::PclCloudPtr& cloud_imu) const;
  [[nodiscard]] hybrid_slam::adapter::PclCloudPtr toBaseCloudFromOdom(
      const hybrid_slam::adapter::PclCloudPtr& cloud_odom,
      const hybrid_slam::geometry::StampedTransform3d& T_odom_base) const;
  [[nodiscard]] bool submitGraphKeyFrame(
      const BASIC::SE3& T_odom_imu, double timestamp_seconds,
      const hybrid_slam::adapter::PclCloudPtr& cloud_imu, const FeatureFrame& features,
      const std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                                  Eigen::Matrix<double, 6, 1>>>& hessian_terms,
      std::size_t input_point_count,
      std::size_t effective_constraint_count);
  void submitMapObservations(
      const std::vector<MapObservation>& map_observations);
  void submitPendingRecoveryConstraint(
      const hybrid_slam::geometry::StampedTransform3d& T_odom_base);
  void enqueueRelocalizationUpdate(
      const hybrid_slam::localization::RelocalizationUpdate& update);
  void applyPendingRelocalizationUpdate();
  [[nodiscard]] bool currentMapToOdom(BASIC::SE3& T_map_odom) const;
  [[nodiscard]] bool commitGlobalAlignment(
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const hybrid_slam::geometry::StampedTransform3d& T_odom_base,
      hybrid_slam::system::GlobalAlignmentSource source,
      double score);
  void applyBackendAlignmentFromSnapshot();
  void saveSnapshotMap();
  bool enqueueSnapshotMap(
      const std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>& snapshot,
      const std::filesystem::path& output_path);
  void finishMapIo();
  void ensureMapIo();
  void ensureMapIoLocked();
  [[nodiscard]] std::uint64_t allocateMapSaveRequestIdLocked(
      std::uint64_t requested_id);
  [[nodiscard]] hybrid_slam::io::MapSaveSubmissionStatus submitMapIoRequest(
      hybrid_slam::io::MapSaveRequest request);
  [[nodiscard]] bool mapIoAvailable() const noexcept;
  void completeMapSave(const hybrid_slam::io::MapSaveResult& result) noexcept;

  struct PreparedRecoveredTrajectory {
    RecoveredTrajectoryStart start;
    std::unique_ptr<hybrid_slam::backend::KeyFramePolicy> keyframe_policy;
  };

  PointLIOPipelineCallbacks callbacks_;
  PointLIORuntimeConfig runtime_config_;
  hybrid_slam::backend::BackendConfig backend_config_;
  std::unique_ptr<hybrid_slam::backend::KeyFramePolicy> keyframe_policy_;
  std::unique_ptr<hybrid_slam::backend::BackendScheduler> backend_scheduler_;
  std::unique_ptr<hybrid_slam::backend::OdometryCovarianceEstimator>
      covariance_estimator_;
  std::unique_ptr<hybrid_slam::frontend::FrontendHealthMonitor>
      frontend_health_monitor_;
  std::unique_ptr<hybrid_slam::io::MapSaveScheduler> map_io_scheduler_;
  std::unique_ptr<PriorMapLocalization> prior_map_localization_;
  std::unique_ptr<hybrid_slam::system::GlobalAlignmentState>
      global_alignment_state_;
  hybrid_slam::frontend::FrontendConfig frontend_config_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> finished_{false};
  std::uint64_t next_graph_keyframe_id_ = 0;
  std::uint64_t accepted_keyframe_count_ = 0;
  hybrid_slam::common::SessionId session_id_{1};
  hybrid_slam::common::TrajectoryId trajectory_id_{1};
  std::uint64_t next_global_alignment_version_ = 1;
  std::uint64_t last_backend_alignment_snapshot_version_ = 0;
  std::optional<double> last_frontend_timestamp_seconds_;
  std::optional<std::chrono::steady_clock::time_point>
      last_frontend_wall_time_;
  std::optional<std::chrono::steady_clock::time_point>
      last_frontend_timeout_observation_;
  std::optional<hybrid_slam::localization::RelocalizationQuery>
      last_relocalization_query_;
  struct RecoveryQueryAccumulator {
    std::vector<Eigen::Vector3f> points_odom;
    std::size_t frame_count = 0;
    bool ready = false;
    void clear() noexcept {
      points_odom.clear();
      frame_count = 0;
      ready = false;
    }
  } recovery_query_accumulator_;
  struct PendingRecoveryConstraint {
    hybrid_slam::geometry::RigidTransform3d T_map_odom;
    hybrid_slam::backend::Covariance6d covariance;
    double score = 0.0;
  };
  std::optional<PendingRecoveryConstraint> pending_recovery_constraint_;
  struct RecoveryEvidence {
    hybrid_slam::common::SessionId session_id;
    hybrid_slam::common::TrajectoryId old_trajectory_id;
    hybrid_slam::common::TrajectoryId new_trajectory_id;
    hybrid_slam::common::Timestamp timestamp;
    bool first_durable_keyframe_pending = false;
  };
  std::optional<RecoveryEvidence> latest_recovery_evidence_;
  std::optional<PreparedRecoveredTrajectory> prepared_recovery_;
  // Failed background candidates must not cause one expensive relocalization
  // submission per frontend frame. Access is serialized by
  // recovery_transition_mutex_.
  std::size_t recovery_retry_cooldown_frames_ = 0;
  mutable std::mutex recovery_transition_mutex_;
  // Unbounded frame counters used only for output decimation. Signed types
  // would overflow into undefined behaviour on a long-running session.
  std::uint64_t grid_count_ = 0;
  std::uint64_t visualization_count_ = 0;
  mutable std::mutex map_io_state_mutex_;
  // Serializes potentially blocking submissions with scheduler shutdown. It
  // prevents finishMapIo() from waiting on the state lock while submit() is
  // waiting for queue capacity.
  mutable std::mutex map_io_submit_mutex_;
  std::uint64_t next_map_save_request_id_ = 1;
  bool map_io_finished_ = false;
  mutable std::mutex map_save_mutex_;
  std::unordered_map<std::uint64_t, std::promise<hybrid_slam::system::SystemStatus>>
      pending_map_saves_;

  hybrid_slam::system::GlobalAlignmentUpdateMailbox global_alignment_mailbox_;
  std::uint64_t alignment_updates_committed_ = 0;
  std::uint64_t alignment_updates_rejected_ = 0;
  // Relocalization results the mailbox refused. They are otherwise invisible:
  // the mailbox is latest-only, so a rejection is not the same as a
  // supersession and must not be silently equivalent to success.
  std::atomic<std::uint64_t> alignment_updates_not_enqueued_{0};
  // Watchdog ticks that declined to wait for the downstream worker. A steadily
  // rising value means frame processing routinely outlives the health timer.
  std::atomic<std::uint64_t> frontend_timeout_observations_skipped_{0};
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_POINT_LIO_PIPELINE_H_
