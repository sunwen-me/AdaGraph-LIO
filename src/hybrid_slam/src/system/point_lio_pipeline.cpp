#include "lio/system/point_lio_pipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "hybrid_slam/logging/logging.h"
#include "hybrid_slam/mapping/snapshot_products.h"
#include "lio/hybrid_slam_feature_extractor.h"
#include "lio/runtime_memory.h"

namespace LI2Sup {

using hybrid_slam::adapter::PclCloudPtr;
using BASIC::M6;
using hybrid_slam::adapter::PclPointCloud;
using hybrid_slam::adapter::PclPoint;
using BASIC::SE3;
using BASIC::V6;

const char* FrontendHealthStateName(
    const hybrid_slam::frontend::FrontendHealthState state) noexcept {
  using State = hybrid_slam::frontend::FrontendHealthState;
  switch (state) {
    case State::kTracking:
      return "TRACKING";
    case State::kSuspect:
      return "SUSPECT";
    case State::kLost:
      return "LOST";
    case State::kRelocalizing:
      return "RELOCALIZING";
    case State::kRestarting:
      return "RESTARTING";
    case State::kVerifying:
      return "VERIFYING";
    case State::kRecovered:
      return "RECOVERED";
  }
  return "UNKNOWN";
}

std::string DescribeHealthFailure(
    const hybrid_slam::frontend::FrontendHealthObservation& observation,
    const hybrid_slam::frontend::FrontendHealthConfig& config) {
  std::string reason;
  const auto append = [&reason](const char* value) {
    if (!reason.empty()) {
      reason += ",";
    }
    reason += value;
  };
  if (!observation.has_information) {
    append("no_information");
  }
  if (!observation.imu_valid) {
    append("imu_invalid");
  }
  if (!observation.timestamp_continuous) {
    append("timestamp_discontinuous");
  }
  if (!std::isfinite(observation.inlier_ratio) ||
      observation.inlier_ratio < config.min_inlier_ratio) {
    append("inlier_ratio");
  }
  if (!std::isfinite(observation.condition_number) ||
      observation.condition_number > config.max_condition_number) {
    append("condition_number");
  }
  if (observation.effective_rank < config.min_effective_rank) {
    append("effective_rank");
  }
  if (observation.effective_point_count < config.min_effective_points) {
    append("effective_points");
  }
  return reason.empty() ? "unknown" : reason;
}

hybrid_slam::sensor::ImmutablePointCloud SharePclCloud(
    const PclCloudPtr& cloud, const hybrid_slam::geometry::FrameId frame) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  std::shared_ptr<const PclPointCloud> frozen = cloud;
  const auto* origin = reinterpret_cast<const std::byte*>(frozen->data());
  const auto& first = frozen->front();
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      frozen->data(), frozen->size(), sizeof(PclPoint),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      frame);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(
      std::move(frozen), view,
      cloud->points.capacity() * sizeof(PclPoint));
}

// Bounds what a keyframe contributes to retained session memory. Nothing
// evicts keyframe clouds, so an unfiltered run grows without limit. The input
// is returned unchanged when filtering is disabled or cannot help, so a
// keyframe never loses its cloud to a degenerate filter result: PCL empties
// the output when the leaf size would overflow its voxel index, and an empty
// cloud would fail the keyframe invariant downstream.
PclCloudPtr VoxelFilterKeyFrameCloud(const PclCloudPtr& cloud,
                                     const double leaf_size_m) {
  if (!cloud || cloud->empty() || !(leaf_size_m > 0.0)) {
    return cloud;
  }
  const auto leaf = static_cast<float>(leaf_size_m);
  auto filtered = std::make_shared<PclPointCloud>();
  pcl::VoxelGrid<PclPoint> voxel_filter;
  voxel_filter.setLeafSize(leaf, leaf, leaf);
  voxel_filter.setInputCloud(cloud);
  voxel_filter.filter(*filtered);
  if (filtered->empty() || filtered->size() >= cloud->size()) {
    return cloud;
  }
  return filtered;
}

PointLIOPipeline::PointLIOPipeline(
    PointLIOPipelineCallbacks callbacks)
    : callbacks_(std::move(callbacks)) {}

void PointLIOPipeline::setTrajectoryIdentity(
    const hybrid_slam::common::SessionId session_id,
    const hybrid_slam::common::TrajectoryId trajectory_id) {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  if (session_id.value() == 0 || trajectory_id.value() == 0) {
    throw std::invalid_argument(
        "downstream session and trajectory ids must be non-zero");
  }
  if (accepted_keyframe_count_ != 0 &&
      (session_id_ != session_id || trajectory_id_ != trajectory_id)) {
    throw std::logic_error(
        "downstream trajectory identity cannot change while initialized");
  }
  session_id_ = session_id;
  trajectory_id_ = trajectory_id;
}

bool PointLIOPipeline::beginRecoveredTrajectory(
    const RecoveredTrajectoryStart& start) {
  if (!prepareRecoveredTrajectory(start)) {
    return false;
  }
  if (!commitPreparedRecoveredTrajectory()) {
    abortPreparedRecoveredTrajectory();
    return false;
  }
  return true;
}

bool PointLIOPipeline::prepareRecoveredTrajectory(
    const RecoveredTrajectoryStart& start) {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  if (!initialized_.load(std::memory_order_acquire) || !backend_scheduler_ ||
      !keyframe_policy_ || !start.valid() ||
      (start.session_id == session_id_ && start.trajectory_id == trajectory_id_) ||
      prepared_recovery_) {
    return false;
  }

  // Force the transform composition and keyframe-policy allocation into the
  // preparation phase. No active pipeline state is changed until the
  // frontend has successfully reset and loaded the prior seed.
  if (!start.T_map_base.compose(start.T_odom_base.inverse())) {
    return false;
  }

  try {
    prepared_recovery_.emplace(
        PreparedRecoveredTrajectory{
            start,
            std::make_unique<hybrid_slam::backend::KeyFramePolicy>(
                backend_config_),
        });
  } catch (const std::exception&) {
    prepared_recovery_.reset();
    return false;
  }
  return true;
}

bool PointLIOPipeline::commitPreparedRecoveredTrajectory() {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  if (!prepared_recovery_) {
    return false;
  }
  const auto& start = prepared_recovery_->start;
  const auto old_trajectory_id = trajectory_id_;
  if (!commitGlobalAlignment(
          start.T_map_base, start.T_odom_base,
          hybrid_slam::system::GlobalAlignmentSource::kRelocalization,
          start.score)) {
    return false;
  }

  session_id_ = start.session_id;
  trajectory_id_ = start.trajectory_id;
  latest_recovery_evidence_ = RecoveryEvidence{
      start.session_id,
      old_trajectory_id,
      start.trajectory_id,
      start.T_map_base.timestamp,
      true,
  };
  keyframe_policy_ = std::move(prepared_recovery_->keyframe_policy);
  pending_recovery_constraint_ = PendingRecoveryConstraint{
      start.T_map_base.T_target_source.compose(
          start.T_odom_base.T_target_source.inverse()),
      start.covariance,
      start.score,
  };
  recovery_retry_cooldown_frames_ = 0;
  global_alignment_mailbox_.reset();
  // The first post-recovery frame may legitimately be separated from the
  // relocalization query by the sensor dropout that triggered recovery. Let
  // that frame establish the new timestamp baseline; continuity checking
  // resumes from the second post-recovery frame.
  last_frontend_wall_time_ = std::chrono::steady_clock::now();
  last_frontend_timeout_observation_.reset();
  last_frontend_timestamp_seconds_.reset();
  if (frontend_health_monitor_) {
    frontend_health_monitor_->beginVerifying();
  }
  if (callbacks_.set_frontend_local_map_write_enabled) {
    try {
      callbacks_.set_frontend_local_map_write_enabled(false);
    } catch (...) {
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [PointLIOPipeline]: local-map write callback "
                    "threw during recovery commit"
                 << hybrid_slam::logging::kReset;
    }
  }
  if (const auto snapshot = backend_scheduler_->mapSnapshot()) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
  }
  LOG(INFO) << " ---> [PointLIORecovery]: committed old_trajectory="
            << old_trajectory_id.value() << ", new_trajectory="
            << start.trajectory_id.value() << ", session="
            << start.session_id.value() << ", timestamp="
            << start.T_map_base.timestamp.seconds()
            << ", first_durable_keyframe_pending=true";
  prepared_recovery_.reset();
  return true;
}

void PointLIOPipeline::abortPreparedRecoveredTrajectory() noexcept {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  prepared_recovery_.reset();
}

void PointLIOPipeline::beginRelocalizationAttempt() noexcept {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  if (frontend_health_monitor_) {
    frontend_health_monitor_->beginRelocalizing();
  }
  recovery_retry_cooldown_frames_ = 0;
}

void PointLIOPipeline::markRecoveryAttemptFailed() noexcept {
  std::lock_guard<std::mutex> lock(recovery_transition_mutex_);
  if (frontend_health_monitor_) {
    frontend_health_monitor_->markLost();
  }
  recovery_retry_cooldown_frames_ =
      runtime_config_.frontend_health.recovery_cooldown_frames;
}

bool PointLIOPipeline::submitLoopClosureConstraint(
    hybrid_slam::backend::Constraint constraint) {
  if (constraint.type != hybrid_slam::backend::ConstraintType::kLoopClosure ||
      !constraint.valid() || !backend_scheduler_) {
    return false;
  }
  return backend_scheduler_->submitConstraint(std::move(constraint)) ==
         hybrid_slam::backend::SubmissionStatus::kAccepted;
}

PointLIOPipeline::~PointLIOPipeline() {
  finish();
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
  finishMapIo();
}

void PointLIOPipeline::ensureMapIo() {
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  ensureMapIoLocked();
}

void PointLIOPipeline::ensureMapIoLocked() {
  if (map_io_scheduler_ || map_io_finished_) {
    return;
  }
  map_io_scheduler_ = std::make_unique<hybrid_slam::io::MapSaveScheduler>(
      2, [this](const hybrid_slam::io::MapSaveResult& result) {
        completeMapSave(result);
      });
}

std::uint64_t PointLIOPipeline::allocateMapSaveRequestIdLocked(
    const std::uint64_t requested_id) {
  if (requested_id == 0) {
    return next_map_save_request_id_++;
  }
  if (requested_id >= next_map_save_request_id_) {
    next_map_save_request_id_ = requested_id + 1;
  }
  return requested_id;
}

hybrid_slam::io::MapSaveSubmissionStatus
PointLIOPipeline::submitMapIoRequest(
    hybrid_slam::io::MapSaveRequest request) {
  std::lock_guard<std::mutex> submit_lock(map_io_submit_mutex_);
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  ensureMapIoLocked();
  if (!map_io_scheduler_ || map_io_finished_) {
    return hybrid_slam::io::MapSaveSubmissionStatus::kClosed;
  }
  request.request_id =
      allocateMapSaveRequestIdLocked(request.request_id);
  return map_io_scheduler_->submit(std::move(request));
}

bool PointLIOPipeline::mapIoAvailable() const noexcept {
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  return map_io_scheduler_ != nullptr && !map_io_finished_;
}

void PointLIOPipeline::completeMapSave(
    const hybrid_slam::io::MapSaveResult& result) noexcept {
  std::promise<hybrid_slam::system::SystemStatus> promise;
  bool has_waiter = false;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    auto found = pending_map_saves_.find(result.request_id);
    if (found != pending_map_saves_.end()) {
      promise = std::move(found->second);
      pending_map_saves_.erase(found);
      has_waiter = true;
    }
  }

  if (result.success) {
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [MapSaveScheduler]: request=" << result.request_id
              << ", snapshot=" << result.snapshot_version
              << ", raw/saved points=" << result.raw_point_count << "/"
              << result.saved_point_count << ", input files="
              << result.input_file_count << ", " << result.message << hybrid_slam::logging::kReset;
  } else {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [MapSaveScheduler]: request=" << result.request_id
               << " failed: " << result.message << hybrid_slam::logging::kReset;
  }

  if (!has_waiter) {
    return;
  }
  if (result.success) {
    promise.set_value(hybrid_slam::system::SystemStatus::Ok());
  } else {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kRejected, result.message));
  }
}

void PointLIOPipeline::init(
    hybrid_slam::frontend::FrontendConfig frontend_config,
    PointLIORuntimeConfig runtime_config) {
  if (initialized_) {
    return;
  }
  if (const auto error = frontend_config.validate()) {
    throw std::invalid_argument("invalid downstream frontend config: " +
                                *error);
  }
  frontend_config_ = std::move(frontend_config);
  if (const auto error = runtime_config.validate()) {
    throw std::invalid_argument("invalid downstream runtime config: " +
                                *error);
  }
  runtime_config_ = std::move(runtime_config);
  frontend_health_monitor_ =
      std::make_unique<hybrid_slam::frontend::FrontendHealthMonitor>(
          runtime_config_.frontend_health);
  LOG(INFO) << hybrid_slam::logging::kGreen
            << " ---> [FrontendHealth]: enabled="
            << runtime_config_.frontend_health.enabled
            << ", suspect/lost/recover/grace/cooldown="
            << runtime_config_.frontend_health.suspect_bad_frames << "/"
            << runtime_config_.frontend_health.lost_bad_frames << "/"
            << runtime_config_.frontend_health.recover_good_frames << "/"
            << runtime_config_.frontend_health.verification_grace_frames << "/"
            << runtime_config_.frontend_health.recovery_cooldown_frames
            << hybrid_slam::logging::kReset;
  if (callbacks_.set_frontend_local_map_write_enabled) {
    callbacks_.set_frontend_local_map_write_enabled(true);
  }
  backend_config_ = runtime_config_.backend;
  if (const auto error = backend_config_.validate()) {
    throw std::invalid_argument("invalid downstream backend config: " +
                                *error);
  }

  keyframe_policy_ =
      std::make_unique<hybrid_slam::backend::KeyFramePolicy>(
          backend_config_);
  backend_scheduler_ =
      std::make_unique<hybrid_slam::backend::BackendScheduler>(
          backend_config_,
          std::make_unique<hybrid_slam::backend::Isam2Optimizer>(
              backend_config_),
          [publish = callbacks_.publish_backend_snapshot](
              std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) {
            if (publish) {
              publish(std::move(snapshot));
            }
          });
  covariance_estimator_ =
      std::make_unique<hybrid_slam::backend::OdometryCovarianceEstimator>(
      backend_config_);
  if (runtime_config_.save_map) {
    ensureMapIo();
  }
  global_alignment_state_ =
      std::make_unique<hybrid_slam::system::GlobalAlignmentState>();
  prior_map_localization_ = std::make_unique<PriorMapLocalization>();
  const bool localization_active = prior_map_localization_->init(
      [publish = callbacks_.publish_global_map](const PclCloudPtr& prior_map) {
        if (publish) {
          publish(SharePclCloud(
              prior_map, hybrid_slam::geometry::FrameId::kMap));
        }
      },
      frontend_config_.T_base_imu,
      [this](const hybrid_slam::localization::RelocalizationUpdate& update) {
        enqueueRelocalizationUpdate(update);
      },
      runtime_config_.prior_map.relocalization,
      runtime_config_.prior_map.kiss_matcher,
      runtime_config_.prior_map.use_backend_localization_factor,
      runtime_config_.prior_map.root_dir,
      runtime_config_.prior_map.fallback_map_file);
  if (callbacks_.set_prior_map_localization_active) {
    callbacks_.set_prior_map_localization_active(localization_active);
  }

  initialized_ = true;
}

std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>
PointLIOPipeline::mapSnapshot() const noexcept {
  if (!backend_scheduler_) {
    return {};
  }
  return backend_scheduler_->mapSnapshot();
}

std::shared_ptr<const hybrid_slam::system::GlobalAlignmentSnapshot>
PointLIOPipeline::alignmentSnapshot() const noexcept {
  if (!global_alignment_state_) {
    return {};
  }
  return global_alignment_state_->snapshot();
}

std::shared_ptr<const hybrid_slam::localization::LocalizationMap>
PointLIOPipeline::localizationMap() const noexcept {
  if (!prior_map_localization_) {
    return {};
  }
  return prior_map_localization_->localizationMap();
}

std::future<hybrid_slam::system::SystemStatus>
PointLIOPipeline::saveMapAsync(hybrid_slam::io::MapSaveRequest request) {
  std::promise<hybrid_slam::system::SystemStatus> promise;
  auto future = promise.get_future();
  if (!request.valid()) {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kInvalidArgument,
        "invalid map-save request"));
    return future;
  }
  std::unique_lock<std::mutex> submit_lock(map_io_submit_mutex_);
  std::unique_lock<std::mutex> io_lock(map_io_state_mutex_);
  ensureMapIoLocked();
  if (!map_io_scheduler_ || map_io_finished_) {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler is closed"));
    return future;
  }
  request.request_id =
      allocateMapSaveRequestIdLocked(request.request_id);
  auto* const scheduler = map_io_scheduler_.get();
  io_lock.unlock();
  const auto request_id = request.request_id;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    if (pending_map_saves_.contains(request_id)) {
      promise.set_value(hybrid_slam::system::SystemStatus::Error(
          hybrid_slam::system::SystemStatusCode::kRejected,
          "duplicate map-save request id"));
      return future;
    }
    pending_map_saves_.emplace(request_id, std::move(promise));
  }
  const auto submission = scheduler->submit(std::move(request));
  if (submission == hybrid_slam::io::MapSaveSubmissionStatus::kAccepted) {
    return future;
  }

  std::promise<hybrid_slam::system::SystemStatus> failed_promise;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    auto found = pending_map_saves_.find(request_id);
    if (found != pending_map_saves_.end()) {
      failed_promise = std::move(found->second);
      pending_map_saves_.erase(found);
    }
  }
  if (submission == hybrid_slam::io::MapSaveSubmissionStatus::kInvalid) {
    failed_promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kInvalidArgument,
        "map IO rejected an invalid request"));
  } else {
    failed_promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler is closed"));
  }
  return future;
}

PclCloudPtr PointLIOPipeline::toCloud(
    const std::vector<Eigen::Vector3f>& points) {
  PclCloudPtr cloud(new PclPointCloud());
  cloud->reserve(points.size());
  for (const auto& point : points) {
    if (!point.allFinite()) {
      continue;
    }
    cloud->emplace_back(point.x(), point.y(), point.z(), 0.0F);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

hybrid_slam::sensor::PointCloudView PointLIOPipeline::toPointCloudView(
    const hybrid_slam::adapter::PclCloudPtr& cloud, const hybrid_slam::geometry::FrameId frame) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  const auto& first = cloud->front();
  const auto* origin = reinterpret_cast<const std::byte*>(cloud->data());
  return hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      cloud->data(), cloud->size(), sizeof(PclPoint),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      frame);
}

PclCloudPtr PointLIOPipeline::toImuCloud(
    const std::vector<Eigen::Vector3f>& odom_points,
    const SE3& odom_body, const std::size_t maximum_points) {
  PclCloudPtr cloud(new PclPointCloud());
  const std::size_t stride = maximum_points >= odom_points.size()
      ? 1
      : (odom_points.size() + maximum_points - 1) / maximum_points;
  cloud->reserve((odom_points.size() + stride - 1) / stride);
  const Eigen::Matrix3f body_R_odom =
      odom_body.R_.transpose().cast<float>();
  const Eigen::Vector3f odom_t_body = odom_body.t_.cast<float>();
  for (std::size_t index = 0; index < odom_points.size(); index += stride) {
    const auto& point = odom_points[index];
    if (!point.allFinite()) {
      continue;
    }
    const Eigen::Vector3f point_body =
        body_R_odom * (point - odom_t_body);
    cloud->emplace_back(
        point_body.x(), point_body.y(), point_body.z(), 0.0F);
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  return cloud;
}

hybrid_slam::geometry::RigidTransform3d
PointLIOPipeline::toDomainPose(const SE3& pose) {
  return {
      Eigen::Quaterniond(pose.R_.cast<double>()).normalized(),
      pose.t_.cast<double>(),
  };
}

SE3 PointLIOPipeline::toBasicPose(
    const hybrid_slam::geometry::RigidTransform3d& pose) {
  return SE3(
      BASIC::SO3(pose.rotation.toRotationMatrix().cast<BASIC::scalar>()),
      pose.translation.cast<BASIC::scalar>());
}

bool PointLIOPipeline::finishActiveSubmaps() {
  return backend_scheduler_ && backend_scheduler_->finishActiveSubmaps();
}

void PointLIOPipeline::finish() {
  if (finished_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
  // No downstream producer remains. Commit a final fixed-size result before
  // closing backend/map state; this never runs concurrently with frame work.
  applyPendingRelocalizationUpdate();
  if (backend_scheduler_) {
    backend_scheduler_->finish();
    // A loop can finish after the final frontend frame. Consume the final
    // immutable snapshot here so its map->odom correction is not left pending.
    applyBackendAlignmentFromSnapshot();
    const auto stats = backend_scheduler_->stats();
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [BackendScheduler]: enqueued/processed/failed="
              << stats.enqueued << "/" << stats.processed << "/"
              << stats.failed << ", nodes/constraints=" << stats.node_count
              << "/" << stats.constraint_count << ", max_queue="
              << stats.maximum_queue_depth << ", backpressure_waits="
              << stats.capacity_waits << ", optimizer updates/failures="
              << stats.optimizer_updates << "/"
              << stats.optimizer_failures << ", optimizer max pose delta="
              << stats.maximum_optimizer_translation_delta_m << " m/"
              << stats.maximum_optimizer_rotation_delta_rad << " rad"
              << ", loop factor residuals count/max_translation/max_rotation="
              << stats.loop_constraint_residual_count << "/"
              << stats.maximum_loop_constraint_translation_residual_m << " m/"
              << stats.maximum_loop_constraint_rotation_residual_rad << " rad"
              << ", global_constraints_dropped="
              << stats.global_constraints_dropped
              << ", relocalization_constraints enqueued/processed/dropped="
              << stats.relocalization_constraints_enqueued << "/"
              << stats.relocalization_constraints_processed << "/"
              << stats.relocalization_constraints_dropped
              << ", map_snapshot_version="
              << stats.map_snapshot_version
              << ", submaps/finished=" << stats.submap_count << "/"
              << stats.finished_submap_count
              << ", accepted_keyframes=" << accepted_keyframe_count_
              << ", snapshot_observer_failures="
              << stats.snapshot_observer_failures
              << ", evaluation_drift constraints/distance/yaw="
              << stats.evaluation_drifted_odometry_constraints << "/"
              << stats.evaluation_drifted_distance_m << " m/"
              << stats.evaluation_drifted_yaw_rad << " rad"
              << ", keyframe_clouds(unique/points/logical/backing)="
              << stats.map_memory.unique_keyframe_cloud_count << "/"
              << stats.map_memory.unique_keyframe_cloud_points << "/"
              << stats.map_memory.unique_keyframe_cloud_logical_bytes << "/"
              << stats.map_memory.unique_keyframe_cloud_backing_bytes
              << hybrid_slam::logging::kReset;
    if (latest_recovery_evidence_) {
      const auto snapshot = backend_scheduler_->mapSnapshot();
      const auto has_trajectory = [&snapshot](
                                      const hybrid_slam::common::TrajectoryId id) {
        return snapshot && std::any_of(
            snapshot->trajectories.begin(), snapshot->trajectories.end(),
            [id](const hybrid_slam::mapping::TrajectorySnapshot& trajectory) {
              return trajectory.id == id;
            });
      };
      const std::size_t relocalization_constraints =
          snapshot ? static_cast<std::size_t>(std::count_if(
                         snapshot->constraints.begin(), snapshot->constraints.end(),
                         [](const hybrid_slam::backend::Constraint& constraint) {
                           return constraint.type ==
                                  hybrid_slam::backend::ConstraintType::kRelocalization;
                         }))
                   : 0U;
      LOG(INFO) << " ---> [PointLIORecovery]: final_snapshot old_trajectory="
                << latest_recovery_evidence_->old_trajectory_id.value()
                << ", new_trajectory="
                << latest_recovery_evidence_->new_trajectory_id.value()
                << ", old_present="
                << has_trajectory(latest_recovery_evidence_->old_trajectory_id)
                << ", new_present="
                << has_trajectory(latest_recovery_evidence_->new_trajectory_id)
                << ", relocalization_constraints="
                << relocalization_constraints;
    }
  }
  if (global_alignment_state_) {
    const auto alignment = global_alignment_state_->snapshot();
    hybrid_slam::geometry::RigidTransform3d published_map_to_odom;
    if (alignment && callbacks_.get_map_to_odom &&
        callbacks_.get_map_to_odom(published_map_to_odom)) {
      const auto& typed = alignment->T_map_odom.T_target_source;
      const double translation_delta =
          (typed.translation - published_map_to_odom.translation).norm();
      const Eigen::Quaterniond rotation_delta =
          (typed.rotation * published_map_to_odom.rotation.conjugate())
              .normalized();
      LOG(INFO) << hybrid_slam::logging::kGreen
                << " ---> [GlobalAlignmentState]: version="
                << alignment->version << ", publication delta="
                << translation_delta << " m/"
                << std::abs(Eigen::AngleAxisd(rotation_delta).angle())
                << " rad" << hybrid_slam::logging::kReset;
    }
  }
  if (callbacks_.get_transform_publication_stats) {
    const auto tf_stats = callbacks_.get_transform_publication_stats();
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [PointLIOTf]: odom_base_published="
              << tf_stats.odom_base_published
              << ", map_odom_published=" << tf_stats.map_odom_published
              << ", nonmonotonic_odom_base_stamps="
              << tf_stats.nonmonotonic_odom_base_stamps
              << ", nonmonotonic_map_odom_stamps="
              << tf_stats.nonmonotonic_map_odom_stamps
              << hybrid_slam::logging::kReset;
  }
  const auto mailbox_stats = global_alignment_mailbox_.stats();
  LOG(INFO) << hybrid_slam::logging::kGreen
            << " ---> [GlobalAlignmentQueue]: enqueued/replaced/committed/"
               "rejected="
            << mailbox_stats.submitted << "/"
            << mailbox_stats.replaced << "/"
            << alignment_updates_committed_ << "/"
            << (alignment_updates_rejected_ + mailbox_stats.invalid)
            << hybrid_slam::logging::kReset;
  // Sample after all frontend, localization and backend workers have drained.
  // Map IO intentionally remains outside this baseline because its temporary
  // serialization allocations are not part of the online working set.
  const auto runtime_memory = CaptureRuntimeMemory();
  LOG(INFO) << hybrid_slam::logging::kGreen
            << " ---> [PipelineRuntimeMemory]: rss/allocated/retained="
            << runtime_memory.resident_kib << "/"
            << runtime_memory.glibc_allocated_bytes << "/"
            << runtime_memory.glibc_retained_bytes << hybrid_slam::logging::kReset;
}

void PointLIOPipeline::reset() {
  if (!initialized_.load(std::memory_order_acquire)) {
    return;
  }

  // Stop all old-session producers first. Accepted frames already executing
  // are allowed to complete; queued frames are stale for the new trajectory.
  finished_.store(true, std::memory_order_release);
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
  if (backend_scheduler_) {
    backend_scheduler_->finish();
  }
  finishMapIo();

  if (callbacks_.set_prior_map_localization_active) {
    callbacks_.set_prior_map_localization_active(false);
  }
  if (callbacks_.reset_session_state) {
    callbacks_.reset_session_state();
  }

  keyframe_policy_.reset();
  backend_scheduler_.reset();
  covariance_estimator_.reset();
  prior_map_localization_.reset();
  global_alignment_state_.reset();
  map_io_scheduler_.reset();
  next_graph_keyframe_id_ = 0;
  accepted_keyframe_count_ = 0;
  next_global_alignment_version_ = 1;
  last_backend_alignment_snapshot_version_ = 0;
  last_frontend_timestamp_seconds_.reset();
  last_frontend_wall_time_.reset();
  last_frontend_timeout_observation_.reset();
  last_relocalization_query_.reset();
  resetRecoveryQueryAccumulator();
  grid_count_ = 0;
  visualization_count_ = 0;
  alignment_updates_committed_ = 0;
  alignment_updates_rejected_ = 0;
  global_alignment_mailbox_.reset();
  frontend_health_monitor_.reset();
  pending_recovery_constraint_.reset();
  prepared_recovery_.reset();
  recovery_retry_cooldown_frames_ = 0;
  {
    std::lock_guard<std::mutex> lock(map_io_state_mutex_);
    map_io_finished_ = false;
    next_map_save_request_id_ = 1;
  }
  initialized_.store(false, std::memory_order_release);
  finished_.store(false, std::memory_order_release);
  init(frontend_config_, runtime_config_);
}

std::optional<hybrid_slam::geometry::StampedTransform3d>
PointLIOPipeline::toOdomBase(
    const SE3& T_odom_imu, double timestamp_seconds) const {
  const auto timestamp =
      hybrid_slam::common::Timestamp::FromSeconds(timestamp_seconds);
  if (!timestamp) {
    return std::nullopt;
  }
  const hybrid_slam::geometry::StampedTransform3d typed_T_odom_imu{
      *timestamp,
      hybrid_slam::geometry::FrameId::kOdom,
      hybrid_slam::geometry::FrameId::kImu,
      toDomainPose(T_odom_imu),
  };
  return typed_T_odom_imu.compose(frontend_config_.T_base_imu.inverse());
}

PclCloudPtr PointLIOPipeline::toBaseCloud(
    const PclCloudPtr& cloud_imu) const {
  if (!cloud_imu) {
    return {};
  }
  const auto& T_base_imu = frontend_config_.T_base_imu.T_target_source;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
      T_base_imu.rotation.toRotationMatrix().cast<float>();
  transform.block<3, 1>(0, 3) =
      T_base_imu.translation.cast<float>();
  PclCloudPtr cloud_base(new PclPointCloud());
  pcl::transformPointCloud(*cloud_imu, *cloud_base, transform);
  return cloud_base;
}

PclCloudPtr PointLIOPipeline::toBaseCloudFromOdom(
    const PclCloudPtr& cloud_odom,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base) const {
  if (!cloud_odom || cloud_odom->empty() || !T_odom_base.valid() ||
      T_odom_base.target_frame != hybrid_slam::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != hybrid_slam::geometry::FrameId::kBase) {
    return {};
  }
  const auto T_base_odom = T_odom_base.T_target_source.inverse();
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
      T_base_odom.rotation.toRotationMatrix().cast<float>();
  transform.block<3, 1>(0, 3) = T_base_odom.translation.cast<float>();
  PclCloudPtr cloud_base(new PclPointCloud());
  pcl::transformPointCloud(*cloud_odom, *cloud_base, transform);
  return cloud_base;
}

bool PointLIOPipeline::submitGraphKeyFrame(
    const SE3& T_odom_imu, double timestamp_seconds,
    const PclCloudPtr& cloud_imu, const FeatureFrame& features,
    const std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                                Eigen::Matrix<double, 6, 1>>>& hessian_terms,
    std::size_t input_point_count,
    std::size_t effective_constraint_count) {
  if (!backend_scheduler_) {
    return false;
  }
  const auto T_odom_base = toOdomBase(T_odom_imu, timestamp_seconds);
  PclCloudPtr cloud_base = toBaseCloud(cloud_imu);
  if (!T_odom_base || !cloud_base || cloud_base->empty()) {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [BackendScheduler]: rejected invalid Point-LIO "
                  "keyframe adaptation"
               << hybrid_slam::logging::kReset;
    return false;
  }

  // This is the last point at which the cloud is still mutable. Once it is
  // frozen into the keyframe it is retained for the rest of the session by
  // every published snapshot, so the storage bound is applied here.
  const std::size_t unfiltered_point_count = cloud_base->size();
  cloud_base = VoxelFilterKeyFrameCloud(
      cloud_base, backend_config_.keyframe_cloud_voxel_size_m);
  if (cloud_base->size() != unfiltered_point_count) {
    LOG_EVERY_N(INFO, 200)
        << " ---> [BackendScheduler]: keyframe cloud voxel filter leaf="
        << backend_config_.keyframe_cloud_voxel_size_m << "m, points "
        << unfiltered_point_count << " -> " << cloud_base->size();
  }

  const auto& points = cloud_base->points;
  const auto* origin = reinterpret_cast<const std::byte*>(points.data());
  const std::size_t x_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().x) - origin);
  const std::size_t y_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().y) - origin);
  const std::size_t z_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().z) - origin);
  std::shared_ptr<const PclPointCloud> frozen_cloud = cloud_base;
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      frozen_cloud->data(), frozen_cloud->size(), sizeof(PclPoint),
      x_offset, y_offset, z_offset,
      hybrid_slam::geometry::FrameId::kBase);
  auto immutable_cloud = hybrid_slam::sensor::ImmutablePointCloud::Share(
      frozen_cloud, view,
      cloud_base->points.capacity() * sizeof(PclPoint));
  hybrid_slam::mapping::KeyFrameQuality quality;
  quality.input_point_count = input_point_count;
  quality.matching_point_count = cloud_base->size();
  quality.effective_constraint_count = effective_constraint_count;
  quality.inlier_ratio = features.inlier_ratio;
  quality.condition_number = features.condition_number;

  hybrid_slam::mapping::KeyFrame keyframe(
      hybrid_slam::common::KeyFrameId(next_graph_keyframe_id_++),
      trajectory_id_, session_id_, *T_odom_base,
      std::move(immutable_cloud), quality);
  const auto uncertainty =
      HybridSLAMFeatureExtractor::buildUncertaintyInput(features,
                                                       hessian_terms);
  const auto covariance = covariance_estimator_->estimate(uncertainty);
  const auto status =
      backend_scheduler_->submitKeyFrame(std::move(keyframe), covariance);
  if (status != hybrid_slam::backend::SubmissionStatus::kAccepted) {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [BackendScheduler]: accepted Point-LIO keyframe was "
                  "not queued, status="
               << static_cast<int>(status) << hybrid_slam::logging::kReset;
    return false;
  }
  if (latest_recovery_evidence_ &&
      latest_recovery_evidence_->first_durable_keyframe_pending &&
      trajectory_id_ == latest_recovery_evidence_->new_trajectory_id) {
    LOG(INFO) << " ---> [PointLIORecovery]: first_durable_keyframe="
              << next_graph_keyframe_id_ - 1 << ", trajectory="
              << trajectory_id_.value() << ", session="
              << session_id_.value() << ", timestamp="
              << timestamp_seconds;
    latest_recovery_evidence_->first_durable_keyframe_pending = false;
  }
  ++accepted_keyframe_count_;
  return true;
}

hybrid_slam::frontend::FrontendHealthDecision
PointLIOPipeline::updateFrontendHealth(const PointLIOOutputFrame& frame,
                                       const IMUData& latest_imu) {
  if (!frontend_health_monitor_) {
    return {};
  }

  last_frontend_wall_time_ = std::chrono::steady_clock::now();
  last_frontend_timeout_observation_.reset();

  hybrid_slam::frontend::FrontendHealthObservation observation;
  observation.imu_valid = latest_imu.valid();
  const double timestamp = frame.state.timestamp;
  const auto& health_config = frontend_health_monitor_->config();
  const auto previous_timestamp = last_frontend_timestamp_seconds_;
  double timestamp_delta = std::numeric_limits<double>::quiet_NaN();
  observation.timestamp_continuous = std::isfinite(timestamp);
  if (observation.timestamp_continuous &&
      last_frontend_timestamp_seconds_) {
    timestamp_delta = timestamp - *last_frontend_timestamp_seconds_;
    observation.timestamp_continuous =
        timestamp_delta > 0.0 &&
        timestamp_delta <= health_config.max_timestamp_gap_s;
  }
  if (std::isfinite(timestamp)) {
    last_frontend_timestamp_seconds_ = timestamp;
  } else {
    last_frontend_timestamp_seconds_.reset();
  }
  observation.effective_point_count = frame.effective_point_updates;
  if (!frame.matching_points_odom.empty()) {
    observation.inlier_ratio =
        static_cast<double>(frame.effective_point_updates) /
        static_cast<double>(frame.matching_points_odom.size());
  }
  if (frame.effective_point_updates > 0 &&
      frame.lidar_hessian.allFinite() && frame.lidar_gradient.allFinite()) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(
        frame.lidar_hessian);
    if (solver.info() == Eigen::Success) {
      const auto eigenvalues = solver.eigenvalues();
      const double minimum = std::max(eigenvalues(0), 1e-12);
      observation.condition_number = eigenvalues(5) / minimum;
      observation.effective_rank = static_cast<std::size_t>(
          (eigenvalues.array() > 1e3).count());
      observation.has_information = eigenvalues.allFinite() &&
                                    eigenvalues(5) > 0.0;
    }
  }

  const auto previous_state = frontend_health_monitor_->state();
  const auto decision = frontend_health_monitor_->observe(observation);
  if (previous_state ==
      hybrid_slam::frontend::FrontendHealthState::kVerifying) {
    LOG(INFO) << " ---> [PointLIORecovery]: verification_frame timestamp="
              << std::setprecision(17) << timestamp
              << ", good=" << decision.observation_good
              << ", resulting_state="
              << FrontendHealthStateName(decision.state)
              << ", good_frames=" << decision.good_frames
              << ", bad_frames=" << decision.bad_frames
              << ", grace_remaining="
              << decision.verification_grace_frames_remaining
              << ", effective_points=" << observation.effective_point_count
              << ", inlier_ratio=" << observation.inlier_ratio
              << ", rank=" << observation.effective_rank
              << ", condition_number=" << observation.condition_number;
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost) {
    LOG(WARNING) << " ---> [FrontendHealth]: verification failed source=frame"
                 << ", reasons="
                 << DescribeHealthFailure(observation, health_config)
                 << ", timestamp=" << timestamp
                 << ", previous_timestamp="
                 << (previous_timestamp ? *previous_timestamp
                                        : std::numeric_limits<double>::quiet_NaN())
                 << ", timestamp_delta=" << timestamp_delta
                 << ", effective_points="
                 << observation.effective_point_count
                 << ", inlier_ratio=" << observation.inlier_ratio
                 << ", rank=" << observation.effective_rank
                 << ", condition_number=" << observation.condition_number
                 << hybrid_slam::logging::kReset;
  }
  applyFrontendHealthDecision(previous_state, decision);
  return decision;
}

void PointLIOPipeline::applyFrontendHealthDecision(
    const hybrid_slam::frontend::FrontendHealthState previous_state,
    const hybrid_slam::frontend::FrontendHealthDecision& decision) {
  if (callbacks_.set_frontend_local_map_write_enabled) {
    callbacks_.set_frontend_local_map_write_enabled(
        decision.local_map_write_enabled);
  }
  if (callbacks_.set_frontend_recovery_query_mode) {
    callbacks_.set_frontend_recovery_query_mode(
        decision.state == hybrid_slam::frontend::FrontendHealthState::kLost);
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost &&
      callbacks_.clear_frontend_online_local_map) {
    callbacks_.clear_frontend_online_local_map();
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost) {
    // A failed verification is a recovery attempt failure, not an ordinary
    // tracking frame. Keep the provisional layer rollbackable and wait for a
    // materially newer query before submitting another expensive candidate.
    recovery_retry_cooldown_frames_ =
        runtime_config_.frontend_health.recovery_cooldown_frames;
    if (latest_recovery_evidence_) {
      LOG(WARNING) << " ---> [PointLIORecovery]: verification_rollback "
                   << "trajectory="
                   << latest_recovery_evidence_->new_trajectory_id.value()
                   << ", first_durable_keyframe_pending="
                   << latest_recovery_evidence_->first_durable_keyframe_pending;
    }
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kRecovered &&
      latest_recovery_evidence_) {
    LOG(INFO) << " ---> [PointLIORecovery]: verification_committed trajectory="
              << latest_recovery_evidence_->new_trajectory_id.value()
              << ", session=" << latest_recovery_evidence_->session_id.value()
              << ", timestamp="
              << latest_recovery_evidence_->timestamp.seconds();
  }
  if (decision.transitioned) {
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [FrontendHealth]: "
              << FrontendHealthStateName(previous_state) << " -> "
              << FrontendHealthStateName(decision.state)
              << ", good/bad/cooldown=" << decision.good_frames << "/"
              << decision.bad_frames << "/" << decision.cooldown_frames
              << ", local_map_write=" << decision.local_map_write_enabled
              << ", global_keyframe_write="
              << decision.global_keyframe_write_enabled
              << hybrid_slam::logging::kReset;
  }
}

void PointLIOPipeline::maybeRequestSubmapRecovery(
    const hybrid_slam::frontend::FrontendHealthState state,
    const std::optional<hybrid_slam::localization::RelocalizationQuery>&
        query) {
  if (state != hybrid_slam::frontend::FrontendHealthState::kLost ||
      !callbacks_.request_submap_recovery || !query || !query->valid()) {
    return;
  }
  if (recovery_retry_cooldown_frames_ > 0) {
    --recovery_retry_cooldown_frames_;
    return;
  }
  const auto snapshot = mapSnapshot();
  if (!snapshot) {
    return;
  }
  if (frontend_health_monitor_) {
    frontend_health_monitor_->beginRelocalizing();
  }
  callbacks_.request_submap_recovery(*query, snapshot);
  resetRecoveryQueryAccumulator();
}

std::optional<hybrid_slam::localization::RelocalizationQuery>
PointLIOPipeline::appendRecoveryQueryFrame(
    const std::vector<Eigen::Vector3f>& points_odom,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base) {
  if (!T_odom_base.valid() ||
      T_odom_base.target_frame != hybrid_slam::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != hybrid_slam::geometry::FrameId::kBase ||
      points_odom.empty()) {
    return std::nullopt;
  }

  auto& accumulator = recovery_query_accumulator_;
  const auto make_query = [this, &T_odom_base](
                              const auto& accumulated_base) {
    hybrid_slam::localization::RelocalizationQuery query{
        T_odom_base,
        SharePclCloud(accumulated_base, hybrid_slam::geometry::FrameId::kBase),
        std::nullopt};
    if (callbacks_.get_map_to_odom) {
      hybrid_slam::geometry::RigidTransform3d T_map_odom;
      if (callbacks_.get_map_to_odom(T_map_odom) &&
          T_map_odom.isFinite()) {
        query.prior_T_map_base =
            hybrid_slam::geometry::StampedTransform3d{
                T_odom_base.timestamp,
                hybrid_slam::geometry::FrameId::kMap,
                hybrid_slam::geometry::FrameId::kBase,
                T_map_odom.compose(T_odom_base.T_target_source)};
      }
    }
    return query;
  };
  if (accumulator.ready) {
    const auto accumulated_odom = toCloud(accumulator.points_odom);
    const auto accumulated_base =
        toBaseCloudFromOdom(accumulated_odom, T_odom_base);
    if (!accumulated_base || accumulated_base->empty()) {
      return std::nullopt;
    }
    return make_query(accumulated_base);
  }
  for (const auto& point : points_odom) {
    if (point.allFinite()) {
      accumulator.points_odom.push_back(point);
    }
  }
  if (accumulator.points_odom.empty()) {
    return std::nullopt;
  }
  ++accumulator.frame_count;
  if (accumulator.frame_count <
      runtime_config_.submap_recovery.query_window_frames) {
    return std::nullopt;
  }

  const auto accumulated_odom = toCloud(accumulator.points_odom);
  const auto accumulated_base =
      toBaseCloudFromOdom(accumulated_odom, T_odom_base);
  if (!accumulated_base || accumulated_base->empty()) {
    return std::nullopt;
  }
  accumulator.ready = true;
  LOG(INFO) << " ---> [SubmapRecovery]: accumulated query frames="
            << accumulator.frame_count
            << ", points=" << accumulated_base->size();
  return make_query(accumulated_base);
}

void PointLIOPipeline::resetRecoveryQueryAccumulator() noexcept {
  recovery_query_accumulator_.clear();
}

void PointLIOPipeline::observeFrontendTimeout() {
  // This runs on the ROS callback group that also owns sensor ingress and the
  // frontend pump, so it must never wait for the downstream worker. Failing to
  // acquire the lock means a frame is being processed right now, which is
  // itself evidence that the frontend is alive; the next tick re-evaluates the
  // gap. Blocking here would stall IMU/LiDAR subscriptions behind whatever the
  // downstream worker is doing, including a full backend submission queue.
  std::unique_lock<std::mutex> lock(recovery_transition_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    ++frontend_timeout_observations_skipped_;
    return;
  }
  if (!initialized_.load(std::memory_order_acquire) ||
      !frontend_health_monitor_ ||
      !frontend_health_monitor_->config().enabled ||
      !last_frontend_wall_time_) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(
      now - *last_frontend_wall_time_).count();
  const double max_gap = frontend_health_monitor_->config().max_timestamp_gap_s;
  if (!std::isfinite(timeout) || timeout < max_gap) {
    return;
  }
  if (last_frontend_timeout_observation_ &&
      std::chrono::duration<double>(now -
                                    *last_frontend_timeout_observation_)
              .count() < max_gap) {
    return;
  }
  last_frontend_timeout_observation_ = now;

  const auto previous_state = frontend_health_monitor_->state();
  hybrid_slam::frontend::FrontendHealthObservation observation;
  observation.has_information = false;
  observation.imu_valid = false;
  observation.timestamp_continuous = false;
  auto decision = frontend_health_monitor_->observe(observation);
  const double dropout_timeout =
      frontend_health_monitor_->config().lidar_dropout_lost_timeout_s;
  const bool can_force_loss =
      previous_state == hybrid_slam::frontend::FrontendHealthState::kTracking ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kSuspect ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kRecovered ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kVerifying;
  if (can_force_loss && timeout >= dropout_timeout &&
      decision.state != hybrid_slam::frontend::FrontendHealthState::kLost) {
    frontend_health_monitor_->markLost();
    decision = frontend_health_monitor_->observe(observation);
    decision.transitioned = true;
    LOG(WARNING) << " ---> [FrontendHealth]: LiDAR dropout forced LOST"
                 << " elapsed_s=" << timeout
                 << ", timeout_s=" << dropout_timeout
                 << hybrid_slam::logging::kReset;
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost) {
    LOG(WARNING) << " ---> [FrontendHealth]: verification failed "
                 << "source=watchdog, elapsed_s=" << timeout
                 << ", max_gap_s=" << max_gap
                 << hybrid_slam::logging::kReset;
  }
  applyFrontendHealthDecision(previous_state, decision);
}

void PointLIOPipeline::submitPendingRecoveryConstraint(
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base) {
  if (!pending_recovery_constraint_ || !backend_scheduler_ ||
      T_odom_base.target_frame != hybrid_slam::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != hybrid_slam::geometry::FrameId::kBase) {
    return;
  }
  const hybrid_slam::geometry::StampedTransform3d T_map_odom{
      T_odom_base.timestamp,
      hybrid_slam::geometry::FrameId::kMap,
      hybrid_slam::geometry::FrameId::kOdom,
      pending_recovery_constraint_->T_map_odom,
  };
  const auto T_map_base = T_map_odom.compose(T_odom_base);
  if (!T_map_base) {
    return;
  }
  hybrid_slam::backend::TimedGlobalPoseConstraint request;
  request.type = hybrid_slam::backend::ConstraintType::kRelocalization;
  request.session_id = session_id_;
  request.trajectory_id = trajectory_id_;
  request.T_map_base = *T_map_base;
  request.covariance = pending_recovery_constraint_->covariance;
  request.score = pending_recovery_constraint_->score;
  request.maximum_time_difference_s = 0.0;
  if (backend_scheduler_->submitGlobalPoseConstraint(std::move(request)) ==
      hybrid_slam::backend::SubmissionStatus::kAccepted) {
    LOG(INFO) << " ---> [PointLIORecovery]: reconnect_constraint_enqueued "
              << "trajectory=" << trajectory_id_.value()
              << ", session=" << session_id_.value()
              << ", timestamp=" << T_map_base->timestamp.seconds();
    pending_recovery_constraint_.reset();
  }
}

void PointLIOPipeline::submitMapObservations(
    const std::vector<MapObservation>& map_observations) {
  if (!backend_scheduler_ || map_observations.empty()) {
    return;
  }
  const auto& config = backend_config_;
  for (const auto& observation : map_observations) {
    const auto timestamp =
        hybrid_slam::common::Timestamp::FromSeconds(observation.timestamp);
    if (!timestamp) {
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [BackendScheduler]: invalid global observation "
                    "timestamp"
                 << hybrid_slam::logging::kReset;
      continue;
    }
    const hybrid_slam::geometry::StampedTransform3d T_map_imu{
        *timestamp,
        hybrid_slam::geometry::FrameId::kMap,
        hybrid_slam::geometry::FrameId::kImu,
      observation.T_map_imu,
    };
    const auto T_map_base =
        T_map_imu.compose(frontend_config_.T_base_imu.inverse());
    if (!T_map_base) {
      continue;
    }

    const double fitness = std::isfinite(observation.fitness)
                               ? std::max(0.0, observation.fitness)
                               : 10.0 * config.localization_fitness_reference;
    const double scale =
        1.0 + fitness / config.localization_fitness_reference;
    const double rotation_sigma =
        config.localization_rotation_sigma_rad * scale;
    const double translation_sigma =
        config.localization_translation_sigma_m * scale;
    hybrid_slam::backend::Covariance6d covariance =
        hybrid_slam::backend::Covariance6d::Zero();
    covariance.diagonal().head<3>().setConstant(
        rotation_sigma * rotation_sigma);
    covariance.diagonal().tail<3>().setConstant(
        translation_sigma * translation_sigma);

    hybrid_slam::backend::TimedGlobalPoseConstraint request;
    request.type = hybrid_slam::backend::ConstraintType::kRelocalization;
    request.session_id = session_id_;
    request.trajectory_id = trajectory_id_;
    request.T_map_base = *T_map_base;
    request.covariance = covariance;
    request.score = 1.0 / scale;
    request.maximum_time_difference_s =
        config.localization_max_time_difference_s;
    const auto status =
        backend_scheduler_->submitGlobalPoseConstraint(std::move(request));
    if (status != hybrid_slam::backend::SubmissionStatus::kAccepted) {
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [BackendScheduler]: global constraint was not "
                    "queued, status="
                 << static_cast<int>(status) << hybrid_slam::logging::kReset;
    }
  }
}

void PointLIOPipeline::enqueueRelocalizationUpdate(
    const hybrid_slam::localization::RelocalizationUpdate& update) {
  // Called from the relocalization worker. A refused submission means the
  // update was malformed, not merely superseded, so it must be visible.
  if (!global_alignment_mailbox_.submit(update)) {
    const auto total = alignment_updates_not_enqueued_.fetch_add(
                           1, std::memory_order_relaxed) +
                       1;
    LOG(WARNING) << hybrid_slam::logging::kYellow
                 << " ---> [GlobalAlignment]: relocalization update refused by"
                    " mailbox, not_enqueued_total="
                 << total << hybrid_slam::logging::kReset;
  }
}

bool PointLIOPipeline::commitGlobalAlignment(
    const hybrid_slam::geometry::StampedTransform3d& T_map_base,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base,
    hybrid_slam::system::GlobalAlignmentSource source,
    double score) {
  if (!global_alignment_state_ ||
      !global_alignment_state_->commit(next_global_alignment_version_++,
                                       T_map_base, T_odom_base, source,
                                       score)) {
    ++alignment_updates_rejected_;
    return false;
  }
  ++alignment_updates_committed_;
  const auto alignment = global_alignment_state_->snapshot();
  if (alignment && callbacks_.set_map_to_odom) {
    try {
      callbacks_.set_map_to_odom(alignment->T_map_odom.T_target_source);
    } catch (...) {
      // Publication is an external side effect. Keep the typed alignment
      // committed; the next snapshot/TF publication can repair a consumer
      // that rejected this notification.
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [GlobalAlignmentState]: map->odom callback threw"
                 << hybrid_slam::logging::kReset;
    }
  }
  return true;
}

void PointLIOPipeline::applyPendingRelocalizationUpdate() {
  auto update = global_alignment_mailbox_.takeLatest();
  if (!update) {
    return;
  }
  const auto& result = update->result;
  if (!update->valid()) {
    ++alignment_updates_rejected_;
    return;
  }
  const bool committed = commitGlobalAlignment(
      *result.T_map_base, update->T_odom_base,
      hybrid_slam::system::GlobalAlignmentSource::kRelocalization,
      result.fitness);
  (void)committed;
}

bool PointLIOPipeline::currentMapToOdom(SE3& T_map_odom) const {
  if (!global_alignment_state_) {
    return false;
  }
  const auto alignment = global_alignment_state_->snapshot();
  if (!alignment || !alignment->valid()) {
    return false;
  }
  T_map_odom = toBasicPose(alignment->T_map_odom.T_target_source);
  return true;
}

void PointLIOPipeline::applyBackendAlignmentFromSnapshot() {
  if (!runtime_config_.backend_drive_map_to_odom ||
      !backend_scheduler_) {
    return;
  }

  const auto snapshot = backend_scheduler_->mapSnapshot();
  if (!snapshot || snapshot->version <= last_backend_alignment_snapshot_version_) {
    return;
  }

  // Ordinary odometry snapshots use odom-frame poses as their map fallback.
  // Only a constraint touching this trajectory establishes a graph correction
  // that may drive map->odom. This includes same-trajectory loop closures,
  // cross-trajectory reconnect loops, and absolute prior/relocalization
  // factors. Point-LIO's ESKF and local map remain strictly odom-frame state.
  //
  // One pass over the keyframes collects both the node set used by the
  // constraint scan and this trajectory's latest keyframe. Rescanning every
  // keyframe per constraint would be O(constraints * keyframes) on the
  // downstream worker for every new snapshot.
  std::unordered_set<std::uint64_t> current_trajectory_nodes;
  current_trajectory_nodes.reserve(snapshot->keyframes.size());
  const hybrid_slam::mapping::KeyFrameSnapshot* latest = nullptr;
  for (const auto& keyframe : snapshot->keyframes) {
    if (keyframe.session_id != session_id_ ||
        keyframe.trajectory_id != trajectory_id_) {
      continue;
    }
    current_trajectory_nodes.insert(keyframe.node_id.value());
    if (latest == nullptr ||
        keyframe.T_odom_base.timestamp > latest->T_odom_base.timestamp) {
      latest = &keyframe;
    }
  }
  const auto belongs_to_current_trajectory =
      [&current_trajectory_nodes](const auto node_id) {
        return current_trajectory_nodes.count(node_id.value()) != 0;
      };
  const bool has_global_constraint = [&]() {
    for (const auto& constraint : snapshot->constraints) {
      if (const auto* relative = std::get_if<
              hybrid_slam::backend::RelativePoseMeasurement>(
              &constraint.measurement)) {
        if (constraint.type ==
                hybrid_slam::backend::ConstraintType::kLoopClosure &&
            (belongs_to_current_trajectory(relative->from_node) ||
             belongs_to_current_trajectory(relative->to_node))) {
          return true;
        }
        continue;
      }
      const auto& absolute = std::get<
          hybrid_slam::backend::AbsolutePoseMeasurement>(
          constraint.measurement);
      if ((constraint.type == hybrid_slam::backend::ConstraintType::kPrior ||
           constraint.type ==
               hybrid_slam::backend::ConstraintType::kRelocalization) &&
          belongs_to_current_trajectory(absolute.node_id)) {
        return true;
      }
    }
    return false;
  }();
  if (!has_global_constraint) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
    return;
  }

  if (latest == nullptr || !latest->T_map_base.valid() ||
      !latest->T_odom_base.valid()) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
    return;
  }

  // GlobalAlignmentState::commit derives map->odom from this pose pair and
  // validates the frame directions and the shared timestamp itself, so the
  // keyframe poses are forwarded as they are. Recomposing T_map_base from an
  // intermediate map->odom here would only reproduce latest->T_map_base with
  // extra rounding.
  last_backend_alignment_snapshot_version_ = snapshot->version;
  if (!commitGlobalAlignment(
          latest->T_map_base, latest->T_odom_base,
          hybrid_slam::system::GlobalAlignmentSource::kBackendOptimization,
          0.0)) {
    LOG_EVERY_N(WARNING, 20)
        << hybrid_slam::logging::kYellow
        << " ---> [GlobalAlignment]: backend-driven map->odom rejected"
        << ", snapshot_version=" << snapshot->version
        << ", rejected_total=" << alignment_updates_rejected_
        << hybrid_slam::logging::kReset;
  }
}

void PointLIOPipeline::processPointFrame(
    PointLIOOutputFrame&& owned_frame,
    const IMUData& latest_imu) {
  if (!initialized_) {
    throw std::logic_error(
        "PointLIOPipeline must be initialized before processing");
  }
  {
    std::lock_guard<std::mutex> recovery_lock(recovery_transition_mutex_);
    applyPendingRelocalizationUpdate();
  }

  // The frame retains unique ownership for the whole call. No point vector is
  // copied when crossing from the frontend queue into this system boundary.
  const auto& frame = owned_frame;
  if (frame.dense_points_odom.empty() &&
      frame.matching_points_odom.empty()) {
    return;
  }

  const NavState& state = frame.state;
  const SE3 odom_body = toBasicPose(state.T_odom_imu);

  PclCloudPtr body_cloud;
  const auto get_body_cloud = [&]() -> const PclCloudPtr& {
    if (!body_cloud) {
      const auto imu_cloud = toImuCloud(frame.dense_points_odom, odom_body);
      body_cloud = toBaseCloud(imu_cloud);
    }
    return body_cloud;
  };

  // Recovery transitions replace the keyframe policy, the trajectory identity
  // and the global alignment, so health evaluation, recovery-query
  // accumulation and graph submission must all observe one consistent
  // transition state. Everything that only reads this frame stays outside.
  {
    std::lock_guard<std::mutex> recovery_lock(recovery_transition_mutex_);
    // This decision is made on the downstream worker. It affects subsequent
    // vendor point admissions, so the estimator never waits for this path.
    const auto health_decision = updateFrontendHealth(frame, latest_imu);

    std::optional<hybrid_slam::localization::RelocalizationQuery> query;
    if (health_decision.state ==
        hybrid_slam::frontend::FrontendHealthState::kLost) {
      if (frame.recovery_query && frame.recovery_query->valid()) {
        query = appendRecoveryQueryFrame(
            frame.recovery_query->points_odom,
            frame.recovery_query->T_odom_base);
      } else if (const auto T_odom_base =
                     toOdomBase(odom_body, frame.state.timestamp)) {
        // Direct/unit-test compositions do not own a frontend raw-query path.
        // Production PointLIOFrontend supplies recovery_query above.
        query = appendRecoveryQueryFrame(frame.dense_points_odom, *T_odom_base);
        if (query && query->valid()) {
          last_relocalization_query_ = query;
        }
      }
    } else {
      resetRecoveryQueryAccumulator();
    }
    // A recovery result must be bound to a scan produced after the loss was
    // observed. Reusing the last pre-dropout scan would seed a trajectory at
    // an old physical pose and leave the first post-dropout scan with an
    // arbitrary timestamp/geometry gap. The watchdog only transitions the
    // lifecycle to LOST; this fresh frame provides the recovery query.
    maybeRequestSubmapRecovery(health_decision.state, query);

    if (backend_scheduler_ && keyframe_policy_ &&
        health_decision.global_keyframe_write_enabled) {
      const auto domain_pose = toDomainPose(odom_body);
      if (keyframe_policy_->shouldAccept(domain_pose)) {
        const SE3 last_keyframe_pose =
            toBasicPose(keyframe_policy_->lastAcceptedPose());
        const PclCloudPtr matching_cloud = frame.matching_points_odom.empty()
            ? get_body_cloud()
            : toImuCloud(frame.matching_points_odom, odom_body,
                         backend_config_.keyframe_max_points);
        if (processPointBackend(frame, odom_body, last_keyframe_pose,
                                matching_cloud, latest_imu)) {
          keyframe_policy_->accept(domain_pose);
        } else {
          keyframe_policy_->reject();
        }
      } else {
        keyframe_policy_->reject();
      }
    }
  }

  // Everything below is output: prior-map registration handoff, occupancy grid
  // insertion and cloud publication. None of it reads recovery state, and all
  // of it can be slow (PCL transforms, grid raycasting, ROS serialization).
  // Holding the recovery lock across it would make the frontend watchdog and
  // the recovery scheduler callbacks queue behind publication work.
  if (prior_map_localization_ && prior_map_localization_->enabled()) {
    if (callbacks_.consume_manual_relocalization_request &&
        callbacks_.consume_manual_relocalization_request()) {
      prior_map_localization_->requestGlobalRelocalization();
    }
    prior_map_localization_->submit(
        state.timestamp, get_body_cloud(), odom_body);
  }

  // Both decimation steps are validated to be >= 1 by PointLIORuntimeConfig
  // and clamped again by the parameter adapters, so the modulo is safe.
  if (callbacks_.occupancy_grid_enabled && callbacks_.update_occupancy_grid &&
      callbacks_.occupancy_grid_enabled()) {
    const auto grid_step =
        static_cast<std::uint64_t>(runtime_config_.grid_update_step);
    if ((grid_count_++ % grid_step) == 0) {
      callbacks_.update_occupancy_grid(
          SharePclCloud(get_body_cloud(),
                        hybrid_slam::geometry::FrameId::kBase),
          state);
    }
  }

  const auto pub_step =
      static_cast<std::uint64_t>(runtime_config_.visualization_step);
  const bool publish_world =
      runtime_config_.visualize_map &&
      (visualization_count_++ % pub_step) == 0;
  if (publish_world && callbacks_.publish_cloud_world) {
    const PclCloudPtr world_cloud = toCloud(frame.dense_points_odom);
    callbacks_.publish_cloud_world(
        SharePclCloud(world_cloud, hybrid_slam::geometry::FrameId::kOdom),
        state.timestamp);
  }

  {
    std::lock_guard<std::mutex> recovery_lock(recovery_transition_mutex_);
    applyPendingRelocalizationUpdate();
  }
}

bool PointLIOPipeline::processPointBackend(
    const PointLIOOutputFrame& frame,
    const SE3& odom_body,
    const SE3& last_keyframe_pose,
    const PclCloudPtr& matching_cloud,
    const IMUData& latest_imu) {
  if (!backend_scheduler_) {
    return false;
  }

  std::vector<std::pair<Eigen::Matrix<double, 6, 6>,
                        Eigen::Matrix<double, 6, 1>>>
      hessian_terms;
  if (frame.effective_point_updates > 0 &&
      frame.lidar_hessian.allFinite() &&
      frame.lidar_gradient.allFinite()) {
    hessian_terms.emplace_back(frame.lidar_hessian, frame.lidar_gradient);
  }

  const FeatureFrame features = HybridSLAMFeatureExtractor::build({
      toDomainPose(odom_body),
      toDomainPose(last_keyframe_pose),
      hessian_terms,
      toPointCloudView(matching_cloud, hybrid_slam::geometry::FrameId::kBase),
      frame.effective_point_updates,
      latest_imu});

  std::vector<MapObservation> map_observations;
  if (backend_config_.use_localization_factor && prior_map_localization_ &&
      prior_map_localization_->enabled()) {
    prior_map_localization_->takeMapObservations(map_observations);
  }

  const bool submitted = submitGraphKeyFrame(
      odom_body, frame.state.timestamp, matching_cloud, features,
      hessian_terms, frame.dense_points_odom.size(),
      frame.effective_point_updates);
  if (submitted) {
    if (const auto T_odom_base =
            toOdomBase(odom_body, frame.state.timestamp)) {
      submitPendingRecoveryConstraint(*T_odom_base);
    }
  }
  submitMapObservations(map_observations);
  applyBackendAlignmentFromSnapshot();
  return submitted;
}

void PointLIOPipeline::saveMap() {
  if (!runtime_config_.save_map) {
    return;
  }

  // The final map and comparison product are both Snapshot-derived.
  saveSnapshotMap();

  namespace fs = std::filesystem;
  const fs::path output_dir(runtime_config_.save_map_dir);
  const fs::path map_name(runtime_config_.map_name);
  if (output_dir.empty() || !output_dir.is_absolute()) {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [PointLIOPipeline]: refusing to save map to '"
                 << runtime_config_.save_map_dir
                 << "'; an absolute directory is required."
               << hybrid_slam::logging::kReset;
    return;
  }
  if (map_name.empty() || map_name.is_absolute() ||
      map_name.parent_path() != fs::path()) {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [PointLIOPipeline]: refusing map filename '"
               << runtime_config_.map_name
               << "'; it must be a plain filename." << hybrid_slam::logging::kReset;
    return;
  }

  const fs::path output_file = output_dir / map_name;
  const auto snapshot = backend_scheduler_ ? backend_scheduler_->mapSnapshot()
                                           : nullptr;
  if (!enqueueSnapshotMap(snapshot, output_file)) {
    LOG(WARNING) << hybrid_slam::logging::kYellow
                 << " ---> [SnapshotMapSave]: no final Point-LIO snapshot "
                    "was available; map was not written."
                 << hybrid_slam::logging::kReset;
  }
  finishMapIo();
}

bool PointLIOPipeline::enqueueSnapshotMap(
    const std::shared_ptr<const hybrid_slam::mapping::MapSnapshot>& snapshot,
    const std::filesystem::path& output_path) {
  if (!snapshot || !snapshot->consistent() || snapshot->keyframes.empty() ||
      output_path.empty()) {
    return false;
  }
  hybrid_slam::io::MapSaveRequest request;
  request.snapshot = snapshot;
  request.output_path = output_path;
  request.voxel_leaf_size =
      std::max(0.01, runtime_config_.map_voxel_leaf_size);
  const auto status = submitMapIoRequest(std::move(request));
  if (status != hybrid_slam::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << hybrid_slam::logging::kRed << " ---> [SnapshotMapSave]: unable to enqueue final "
               << output_path << " (status=" << static_cast<int>(status)
               << ")" << hybrid_slam::logging::kReset;
    return false;
  }
  LOG(INFO) << hybrid_slam::logging::kGreen << " ---> [SnapshotMapSave]: queued final version="
            << snapshot->version << ", file=" << output_path << hybrid_slam::logging::kReset;
  return true;
}

void PointLIOPipeline::saveSnapshotMap() {
  if (!backend_scheduler_ || !mapIoAvailable()) {
    return;
  }
  const auto snapshot = backend_scheduler_->mapSnapshot();
  if (!snapshot || !snapshot->consistent() || snapshot->keyframes.empty()) {
    LOG(WARNING) << hybrid_slam::logging::kYellow
                 << " ---> [SnapshotMapSave]: no consistent map snapshot to save."
                 << hybrid_slam::logging::kReset;
    return;
  }
  namespace fs = std::filesystem;
  const fs::path output_dir(runtime_config_.save_map_dir);
  const fs::path configured_name(runtime_config_.map_name);
  if (output_dir.empty() || !output_dir.is_absolute() ||
      configured_name.empty() || configured_name.is_absolute() ||
      configured_name.parent_path() != fs::path()) {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [SnapshotMapSave]: invalid output directory/name: "
               << output_dir << "/" << configured_name << hybrid_slam::logging::kReset;
    return;
  }

  const fs::path snapshot_name =
      configured_name.stem().string() + "_snapshot" +
      configured_name.extension().string();
  const fs::path output_file = output_dir / snapshot_name;
  hybrid_slam::io::MapSaveRequest request;
  request.snapshot = snapshot;
  request.output_path = output_file;
  request.voxel_leaf_size =
      std::max(0.01, runtime_config_.map_voxel_leaf_size);
  const auto status = submitMapIoRequest(std::move(request));
  if (status != hybrid_slam::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << hybrid_slam::logging::kRed << " ---> [SnapshotMapSave]: unable to enqueue snapshot "
               << snapshot->version << " (status="
               << static_cast<int>(status) << ")" << hybrid_slam::logging::kReset;
  } else {
    LOG(INFO) << hybrid_slam::logging::kGreen << " ---> [SnapshotMapSave]: queued version="
              << snapshot->version << ", keyframes="
              << snapshot->keyframes.size() << ", file=" << output_file
              << hybrid_slam::logging::kReset;
  }

  const std::size_t diagnostic_count = std::min(
      runtime_config_.map_endpoint_diagnostic_keyframes,
      snapshot->keyframes.size() / 2);
  if (diagnostic_count == 0) {
    return;
  }
  const auto enqueue_endpoint = [&](const char* suffix,
                                    const std::size_t begin,
                                    const std::size_t end) {
    const fs::path name = configured_name.stem().string() + suffix +
                          configured_name.extension().string();
    hybrid_slam::io::MapSaveRequest endpoint;
    endpoint.snapshot = snapshot;
    endpoint.output_path = output_dir / name;
    endpoint.voxel_leaf_size =
        std::max(0.01, runtime_config_.map_voxel_leaf_size);
    endpoint.keyframe_range = std::pair{begin, end};
    const auto endpoint_status = submitMapIoRequest(std::move(endpoint));
    if (endpoint_status !=
        hybrid_slam::io::MapSaveSubmissionStatus::kAccepted) {
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [SnapshotMapSave]: unable to enqueue endpoint "
                    "diagnostic range=["
                 << begin << ',' << end << ") status="
                 << static_cast<int>(endpoint_status)
                 << hybrid_slam::logging::kReset;
    } else {
      LOG(INFO) << hybrid_slam::logging::kGreen
                << " ---> [SnapshotMapSave]: queued endpoint diagnostic "
                   "range=["
                << begin << ',' << end << ") file=" << output_dir / name
                << hybrid_slam::logging::kReset;
    }
  };
  enqueue_endpoint("_head", 0, diagnostic_count);
  enqueue_endpoint("_tail", snapshot->keyframes.size() - diagnostic_count,
                   snapshot->keyframes.size());
}

void PointLIOPipeline::finishMapIo() {
  std::unique_lock<std::mutex> submit_lock(map_io_submit_mutex_);
  std::unique_lock<std::mutex> io_lock(map_io_state_mutex_);
  if (!map_io_scheduler_ || map_io_finished_) {
    return;
  }
  map_io_scheduler_->finish();
  map_io_finished_ = true;
  const auto stats = map_io_scheduler_->stats();
  io_lock.unlock();
  std::unordered_map<std::uint64_t,
                     std::promise<hybrid_slam::system::SystemStatus>> pending;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    pending.swap(pending_map_saves_);
  }
  for (auto& [request_id, promise] : pending) {
    (void)request_id;
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler stopped before completing request"));
  }
  LOG(INFO) << hybrid_slam::logging::kGreen
            << " ---> [MapSaveScheduler]: submitted/processed/failed="
            << stats.submitted << "/" << stats.processed << "/"
            << stats.failed << ", max_queue=" << stats.maximum_queue_depth
            << ", capacity_waits=" << stats.capacity_waits << hybrid_slam::logging::kReset;
}

}  // namespace LI2Sup
