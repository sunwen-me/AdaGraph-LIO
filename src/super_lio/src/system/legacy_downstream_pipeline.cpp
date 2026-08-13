#include "lio/system/legacy_downstream_pipeline.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/transforms.h>

#include "basic/logs.h"
#include "adagraph/mapping/snapshot_products.h"
#include "lio/adagraph_feature_extractor.h"

namespace LI2Sup {

using BASIC::CloudPtr;
using BASIC::M6;
using BASIC::PointCloudType;
using BASIC::PointType;
using BASIC::SE3;
using BASIC::V6;

LegacyDownstreamPipeline::LegacyDownstreamPipeline(
    LegacyDownstreamCallbacks callbacks)
    : callbacks_(std::move(callbacks)) {}

void LegacyDownstreamPipeline::setTrajectoryIdentity(
    const adagraph::common::SessionId session_id,
    const adagraph::common::TrajectoryId trajectory_id) {
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

LegacyDownstreamPipeline::~LegacyDownstreamPipeline() {
  finish();
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
  finishMapIo();
}

void LegacyDownstreamPipeline::ensureMapIo() {
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  ensureMapIoLocked();
}

void LegacyDownstreamPipeline::ensureMapIoLocked() {
  if (map_io_scheduler_ || map_io_finished_) {
    return;
  }
  map_io_scheduler_ = std::make_unique<adagraph::io::MapSaveScheduler>(
      2, [this](const adagraph::io::MapSaveResult& result) {
        completeMapSave(result);
      });
}

std::uint64_t LegacyDownstreamPipeline::allocateMapSaveRequestIdLocked(
    const std::uint64_t requested_id) {
  if (requested_id == 0) {
    return next_map_save_request_id_++;
  }
  if (requested_id >= next_map_save_request_id_) {
    next_map_save_request_id_ = requested_id + 1;
  }
  return requested_id;
}

adagraph::io::MapSaveSubmissionStatus
LegacyDownstreamPipeline::submitMapIoRequest(
    adagraph::io::MapSaveRequest request) {
  std::lock_guard<std::mutex> submit_lock(map_io_submit_mutex_);
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  ensureMapIoLocked();
  if (!map_io_scheduler_ || map_io_finished_) {
    return adagraph::io::MapSaveSubmissionStatus::kClosed;
  }
  request.request_id =
      allocateMapSaveRequestIdLocked(request.request_id);
  return map_io_scheduler_->submit(std::move(request));
}

bool LegacyDownstreamPipeline::mapIoAvailable() const noexcept {
  std::lock_guard<std::mutex> lock(map_io_state_mutex_);
  return map_io_scheduler_ != nullptr && !map_io_finished_;
}

void LegacyDownstreamPipeline::completeMapSave(
    const adagraph::io::MapSaveResult& result) noexcept {
  std::promise<adagraph::system::SystemStatus> promise;
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
    LOG(INFO) << GREEN
              << " ---> [MapSaveScheduler]: request=" << result.request_id
              << ", snapshot=" << result.snapshot_version
              << ", raw/saved points=" << result.raw_point_count << "/"
              << result.saved_point_count << ", input files="
              << result.input_file_count << ", " << result.message << RESET;
  } else {
    LOG(ERROR) << RED
               << " ---> [MapSaveScheduler]: request=" << result.request_id
               << " failed: " << result.message << RESET;
  }

  if (!has_waiter) {
    return;
  }
  if (result.success) {
    promise.set_value(adagraph::system::SystemStatus::Ok());
  } else {
    promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kRejected, result.message));
  }
}

void LegacyDownstreamPipeline::init(
    const LegacyFrontendKind frontend_kind,
    adagraph::frontend::FrontendConfig frontend_config,
    LegacyRuntimeConfig runtime_config) {
  if (initialized_) {
    return;
  }
  frontend_kind_ = frontend_kind;
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
  backend_config_ = runtime_config_.backend;
  if (const auto error = backend_config_.validate()) {
    throw std::invalid_argument("invalid downstream backend config: " +
                                *error);
  }

  keyframe_policy_ =
      std::make_unique<adagraph::backend::KeyFramePolicy>(
          backend_config_);
  backend_scheduler_ =
      std::make_unique<adagraph::backend::BackendScheduler>(
          backend_config_,
          std::make_unique<adagraph::backend::Isam2Optimizer>(
              backend_config_),
          [publish = callbacks_.publish_backend_snapshot](
              std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) {
            if (publish) {
              publish(std::move(snapshot));
            }
          });
  covariance_estimator_ =
      std::make_unique<adagraph::backend::OdometryCovarianceEstimator>(
      backend_config_);
  if (runtime_config_.save_map) {
    ensureMapIo();
  }
  global_alignment_state_ =
      std::make_unique<adagraph::system::GlobalAlignmentState>();
  super_queue_capacity_ =
      backend_config_.submission_queue_capacity;

  prior_map_localization_ = std::make_unique<PriorMapLocalization>();
  const bool localization_active = prior_map_localization_->init(
      [publish = callbacks_.publish_global_map](const CloudPtr& prior_map) {
        if (publish) {
          publish(prior_map);
        }
      },
      frontend_config_.T_base_imu,
      [this](const adagraph::localization::RelocalizationUpdate& update) {
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

  if (runtime_config_.save_map &&
      frontend_kind_ == LegacyFrontendKind::kSuperLio) {
    super_accumulated_map_.reset(new PointCloudType());
    super_world_cloud_.reset(new PointCloudType());
  }
  initialized_ = true;
  if (frontend_kind_ == LegacyFrontendKind::kSuperLio) {
    startSuperWorker();
  }
}

std::shared_ptr<const adagraph::mapping::MapSnapshot>
LegacyDownstreamPipeline::mapSnapshot() const noexcept {
  if (!backend_scheduler_) {
    return {};
  }
  return backend_scheduler_->mapSnapshot();
}

std::shared_ptr<const adagraph::system::GlobalAlignmentSnapshot>
LegacyDownstreamPipeline::alignmentSnapshot() const noexcept {
  if (!global_alignment_state_) {
    return {};
  }
  return global_alignment_state_->snapshot();
}

std::shared_ptr<const adagraph::localization::LocalizationMap>
LegacyDownstreamPipeline::localizationMap() const noexcept {
  if (!prior_map_localization_) {
    return {};
  }
  return prior_map_localization_->localizationMap();
}

std::future<adagraph::system::SystemStatus>
LegacyDownstreamPipeline::saveMapAsync(adagraph::io::MapSaveRequest request) {
  std::promise<adagraph::system::SystemStatus> promise;
  auto future = promise.get_future();
  if (!request.valid()) {
    promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kInvalidArgument,
        "invalid map-save request"));
    return future;
  }
  std::unique_lock<std::mutex> submit_lock(map_io_submit_mutex_);
  std::unique_lock<std::mutex> io_lock(map_io_state_mutex_);
  ensureMapIoLocked();
  if (!map_io_scheduler_ || map_io_finished_) {
    promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kClosed,
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
      promise.set_value(adagraph::system::SystemStatus::Error(
          adagraph::system::SystemStatusCode::kRejected,
          "duplicate map-save request id"));
      return future;
    }
    pending_map_saves_.emplace(request_id, std::move(promise));
  }
  const auto submission = scheduler->submit(std::move(request));
  if (submission == adagraph::io::MapSaveSubmissionStatus::kAccepted) {
    return future;
  }

  std::promise<adagraph::system::SystemStatus> failed_promise;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    auto found = pending_map_saves_.find(request_id);
    if (found != pending_map_saves_.end()) {
      failed_promise = std::move(found->second);
      pending_map_saves_.erase(found);
    }
  }
  if (submission == adagraph::io::MapSaveSubmissionStatus::kInvalid) {
    failed_promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kInvalidArgument,
        "map IO rejected an invalid request"));
  } else {
    failed_promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kClosed,
        "map IO scheduler is closed"));
  }
  return future;
}

CloudPtr LegacyDownstreamPipeline::toCloud(
    const std::vector<Eigen::Vector3f>& points) {
  CloudPtr cloud(new PointCloudType());
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

CloudPtr LegacyDownstreamPipeline::toBodyCloud(
    const std::vector<Eigen::Vector3f>& odom_points,
    const SE3& odom_body) {
  CloudPtr cloud(new PointCloudType());
  cloud->reserve(odom_points.size());
  const Eigen::Matrix3f body_R_odom =
      odom_body.R_.transpose().cast<float>();
  const Eigen::Vector3f odom_t_body = odom_body.t_.cast<float>();
  for (const auto& point : odom_points) {
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

adagraph::geometry::RigidTransform3d
LegacyDownstreamPipeline::toDomainPose(const SE3& pose) {
  return {
      Eigen::Quaterniond(pose.R_.cast<double>()).normalized(),
      pose.t_.cast<double>(),
  };
}

SE3 LegacyDownstreamPipeline::toLegacyPose(
    const adagraph::geometry::RigidTransform3d& pose) {
  return SE3(
      BASIC::SO3(pose.rotation.toRotationMatrix().cast<BASIC::scalar>()),
      pose.translation.cast<BASIC::scalar>());
}

void LegacyDownstreamPipeline::startSuperWorker() {
  std::lock_guard<std::mutex> lock(super_queue_mutex_);
  if (super_worker_started_) {
    return;
  }
  super_worker_stop_ = false;
  super_worker_started_ = true;
  super_worker_ =
      std::thread(&LegacyDownstreamPipeline::superWorkerLoop, this);
}

void LegacyDownstreamPipeline::stopSuperWorker(const bool drain) {
  {
    std::lock_guard<std::mutex> lock(super_queue_mutex_);
    if (!super_worker_started_) {
      return;
    }
    if (!drain) {
      super_queue_.clear();
    }
    super_worker_stop_ = true;
  }
  super_queue_not_empty_cv_.notify_all();
  super_queue_not_full_cv_.notify_all();
  if (super_worker_.joinable()) {
    super_worker_.join();
  }
  super_worker_started_ = false;

  const std::uint64_t processed = super_frames_processed_.load();
  const double mean_ms = processed == 0
      ? 0.0
      : static_cast<double>(super_downstream_time_ns_.load()) /
            (1e6 * static_cast<double>(processed));
  LOG(INFO) << GREEN
            << " ---> [LegacyDownstreamPipeline]: Super-LIO downstream "
            << "enqueued/processed=" << super_frames_enqueued_.load() << "/"
            << processed << ", max_queue=" << super_max_queue_depth_.load()
            << ", backpressure_waits=" << super_queue_waits_.load()
            << ", mean=" << mean_ms << " ms" << RESET;
}

void LegacyDownstreamPipeline::finish() {
  if (finished_.exchange(true, std::memory_order_acq_rel)) {
    return;
  }
  if (frontend_kind_ == LegacyFrontendKind::kSuperLio) {
    // Drain every accepted frontend frame before closing localization input.
    stopSuperWorker();
  }
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
  // No downstream producer remains. Commit a final fixed-size result before
  // closing backend/map state; this never runs concurrently with frame work.
  applyPendingRelocalizationUpdate();
  if (backend_scheduler_) {
    backend_scheduler_->finish();
    const auto stats = backend_scheduler_->stats();
    LOG(INFO) << GREEN
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
              << ", global_constraints_dropped="
              << stats.global_constraints_dropped
              << ", map_snapshot_version="
              << stats.map_snapshot_version
              << ", submaps/finished=" << stats.submap_count << "/"
              << stats.finished_submap_count
              << ", accepted_keyframes=" << accepted_keyframe_count_
              << ", snapshot_observer_failures="
              << stats.snapshot_observer_failures
              << ", keyframe_clouds(unique/points/logical/backing)="
              << stats.map_memory.unique_keyframe_cloud_count << "/"
              << stats.map_memory.unique_keyframe_cloud_points << "/"
              << stats.map_memory.unique_keyframe_cloud_logical_bytes << "/"
              << stats.map_memory.unique_keyframe_cloud_backing_bytes
              << RESET;
  }
  if (global_alignment_state_) {
    const auto alignment = global_alignment_state_->snapshot();
    SE3 legacy_map_to_odom;
    if (alignment && callbacks_.get_map_to_odom &&
        callbacks_.get_map_to_odom(legacy_map_to_odom)) {
      const auto& typed = alignment->T_map_odom.T_target_source;
      const auto legacy = toDomainPose(legacy_map_to_odom);
      const double translation_delta =
          (typed.translation - legacy.translation).norm();
      const Eigen::Quaterniond rotation_delta =
          (typed.rotation * legacy.rotation.conjugate()).normalized();
      LOG(INFO) << GREEN
                << " ---> [GlobalAlignmentState]: version="
                << alignment->version << ", legacy parity="
                << translation_delta << " m/"
                << std::abs(Eigen::AngleAxisd(rotation_delta).angle())
                << " rad" << RESET;
    }
  }
  const auto mailbox_stats = global_alignment_mailbox_.stats();
  LOG(INFO) << GREEN
            << " ---> [GlobalAlignmentQueue]: enqueued/replaced/committed/"
               "rejected="
            << mailbox_stats.submitted << "/"
            << mailbox_stats.replaced << "/"
            << alignment_updates_committed_ << "/"
            << (alignment_updates_rejected_ + mailbox_stats.invalid)
            << RESET;
}

void LegacyDownstreamPipeline::reset() {
  if (!initialized_.load(std::memory_order_acquire)) {
    return;
  }

  // Stop all old-session producers first. Accepted frames already executing
  // are allowed to complete; queued frames are stale for the new trajectory.
  finished_.store(true, std::memory_order_release);
  if (frontend_kind_ == LegacyFrontendKind::kSuperLio) {
    stopSuperWorker(false);
  }
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
  super_accumulated_map_.reset();
  super_world_cloud_.reset();

  {
    std::lock_guard<std::mutex> lock(super_buffer_pool_mutex_);
    super_buffer_pool_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(super_queue_mutex_);
    super_queue_.clear();
    super_worker_stop_ = false;
  }
  super_frames_enqueued_.store(0, std::memory_order_release);
  super_frames_processed_.store(0, std::memory_order_release);
  super_queue_waits_.store(0, std::memory_order_release);
  super_max_queue_depth_.store(0, std::memory_order_release);
  super_downstream_time_ns_.store(0, std::memory_order_release);
  next_graph_keyframe_id_ = 0;
  accepted_keyframe_count_ = 0;
  next_global_alignment_version_ = 1;
  last_backend_alignment_snapshot_version_ = 0;
  point_grid_count_ = 0;
  point_visualization_count_ = 0;
  super_grid_count_ = -1;
  super_visualization_count_ = -1;
  super_scan_wait_count_ = 0;
  super_pcd_index_ = -1;
  super_pcd_dir_prepared_ = false;
  super_pcd_paths_.clear();
  alignment_updates_committed_ = 0;
  alignment_updates_rejected_ = 0;
  global_alignment_mailbox_.reset();
  {
    std::lock_guard<std::mutex> lock(map_io_state_mutex_);
    map_io_finished_ = false;
    next_map_save_request_id_ = 1;
  }
  initialized_.store(false, std::memory_order_release);
  finished_.store(false, std::memory_order_release);
  init(frontend_kind_, frontend_config_, runtime_config_);
}

std::optional<adagraph::geometry::StampedTransform3d>
LegacyDownstreamPipeline::toOdomBase(
    const SE3& T_odom_imu, double timestamp_seconds) const {
  const auto timestamp =
      adagraph::common::Timestamp::FromSeconds(timestamp_seconds);
  if (!timestamp) {
    return std::nullopt;
  }
  const adagraph::geometry::StampedTransform3d typed_T_odom_imu{
      *timestamp,
      adagraph::geometry::FrameId::kOdom,
      adagraph::geometry::FrameId::kImu,
      toDomainPose(T_odom_imu),
  };
  return typed_T_odom_imu.compose(frontend_config_.T_base_imu.inverse());
}

CloudPtr LegacyDownstreamPipeline::toBaseCloud(
    const CloudPtr& cloud_imu) const {
  if (!cloud_imu) {
    return {};
  }
  const auto& T_base_imu = frontend_config_.T_base_imu.T_target_source;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) =
      T_base_imu.rotation.toRotationMatrix().cast<float>();
  transform.block<3, 1>(0, 3) =
      T_base_imu.translation.cast<float>();
  CloudPtr cloud_base(new PointCloudType());
  pcl::transformPointCloud(*cloud_imu, *cloud_base, transform);
  return cloud_base;
}

bool LegacyDownstreamPipeline::submitGraphKeyFrame(
    const SE3& T_odom_imu, double timestamp_seconds,
    const CloudPtr& cloud_imu, const FeatureFrame& features,
    const std::vector<std::pair<M6, V6>>& hessian_terms,
    std::size_t input_point_count,
    std::size_t effective_constraint_count) {
  if (!backend_scheduler_) {
    return false;
  }
  const auto T_odom_base = toOdomBase(T_odom_imu, timestamp_seconds);
  CloudPtr cloud_base = toBaseCloud(cloud_imu);
  if (!T_odom_base || !cloud_base || cloud_base->empty()) {
    LOG(ERROR) << RED
               << " ---> [BackendScheduler]: rejected invalid legacy "
                  "KeyFrame adaptation"
               << RESET;
    return false;
  }

  const auto& points = cloud_base->points;
  const auto* origin = reinterpret_cast<const std::byte*>(points.data());
  const std::size_t x_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().x) - origin);
  const std::size_t y_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().y) - origin);
  const std::size_t z_offset = static_cast<std::size_t>(
      reinterpret_cast<const std::byte*>(&points.front().z) - origin);
  std::shared_ptr<const PointCloudType> frozen_cloud = cloud_base;
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      frozen_cloud->data(), frozen_cloud->size(), sizeof(PointType),
      x_offset, y_offset, z_offset,
      adagraph::geometry::FrameId::kBase);
  auto immutable_cloud = adagraph::sensor::ImmutablePointCloud::Share(
      frozen_cloud, view,
      cloud_base->points.capacity() * sizeof(PointType));
  adagraph::mapping::KeyFrameQuality quality;
  quality.input_point_count = input_point_count;
  quality.matching_point_count = cloud_base->size();
  quality.effective_constraint_count = effective_constraint_count;
  quality.inlier_ratio = features.inlier_ratio;
  quality.condition_number = features.condition_number;

  adagraph::mapping::KeyFrame keyframe(
      adagraph::common::KeyFrameId(next_graph_keyframe_id_++),
      trajectory_id_, session_id_, *T_odom_base,
      std::move(immutable_cloud), quality);
  const auto uncertainty =
      AdaGraphFeatureExtractor::buildUncertaintyInput(features,
                                                       hessian_terms);
  const auto covariance = covariance_estimator_->estimate(uncertainty);
  const auto status =
      backend_scheduler_->submitKeyFrame(std::move(keyframe), covariance);
  if (status != adagraph::backend::SubmissionStatus::kAccepted) {
    LOG(ERROR) << RED
               << " ---> [BackendScheduler]: accepted legacy KeyFrame was "
                  "not queued, status="
               << static_cast<int>(status) << RESET;
    return false;
  }
  ++accepted_keyframe_count_;
  return true;
}

void LegacyDownstreamPipeline::submitMapObservations(
    const std::vector<MapObservation>& map_observations) {
  if (!backend_scheduler_ || map_observations.empty()) {
    return;
  }
  const auto& config = backend_config_;
  for (const auto& observation : map_observations) {
    const auto timestamp =
        adagraph::common::Timestamp::FromSeconds(observation.timestamp);
    if (!timestamp) {
      LOG(ERROR) << RED
                 << " ---> [BackendScheduler]: invalid global observation "
                    "timestamp"
                 << RESET;
      continue;
    }
    const adagraph::geometry::StampedTransform3d T_map_imu{
        *timestamp,
        adagraph::geometry::FrameId::kMap,
        adagraph::geometry::FrameId::kImu,
        toDomainPose(observation.map_lidar),
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
    adagraph::backend::Covariance6d covariance =
        adagraph::backend::Covariance6d::Zero();
    covariance.diagonal().head<3>().setConstant(
        rotation_sigma * rotation_sigma);
    covariance.diagonal().tail<3>().setConstant(
        translation_sigma * translation_sigma);

    adagraph::backend::TimedGlobalPoseConstraint request;
    request.type = adagraph::backend::ConstraintType::kRelocalization;
    request.session_id = session_id_;
    request.trajectory_id = trajectory_id_;
    request.T_map_base = *T_map_base;
    request.covariance = covariance;
    request.score = 1.0 / scale;
    request.maximum_time_difference_s =
        config.localization_max_time_difference_s;
    const auto status =
        backend_scheduler_->submitGlobalPoseConstraint(std::move(request));
    if (status != adagraph::backend::SubmissionStatus::kAccepted) {
      LOG(ERROR) << RED
                 << " ---> [BackendScheduler]: global constraint was not "
                    "queued, status="
                 << static_cast<int>(status) << RESET;
    }
  }
}

void LegacyDownstreamPipeline::enqueueRelocalizationUpdate(
    const adagraph::localization::RelocalizationUpdate& update) {
  const bool accepted = global_alignment_mailbox_.submit(update);
  (void)accepted;
}

bool LegacyDownstreamPipeline::commitGlobalAlignment(
    const adagraph::geometry::StampedTransform3d& T_map_base,
    const adagraph::geometry::StampedTransform3d& T_odom_base,
    adagraph::system::GlobalAlignmentSource source,
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
    callbacks_.set_map_to_odom(
        toLegacyPose(alignment->T_map_odom.T_target_source));
  }
  return true;
}

void LegacyDownstreamPipeline::applyPendingRelocalizationUpdate() {
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
      adagraph::system::GlobalAlignmentSource::kRelocalization,
      result.fitness);
  (void)committed;
}

bool LegacyDownstreamPipeline::currentMapToOdom(SE3& T_map_odom) const {
  if (!global_alignment_state_) {
    return false;
  }
  const auto alignment = global_alignment_state_->snapshot();
  if (!alignment || !alignment->valid()) {
    return false;
  }
  T_map_odom = toLegacyPose(alignment->T_map_odom.T_target_source);
  return true;
}

void LegacyDownstreamPipeline::applyBackendAlignmentFromSnapshot() {
  if (!runtime_config_.backend_drive_map_to_odom ||
      !backend_config_.use_localization_factor ||
      !backend_scheduler_ || !prior_map_localization_ ||
      !prior_map_localization_->enabled()) {
    return;
  }

  const auto snapshot = backend_scheduler_->mapSnapshot();
  if (!snapshot || snapshot->version <= last_backend_alignment_snapshot_version_) {
    return;
  }

  // A snapshot published after the keyframe but before its absolute
  // relocalization constraint is applied is not a global correction.  Do not
  // interpret its odom-frame fallback pose as map alignment.
  const bool has_absolute_constraint = std::any_of(
      snapshot->constraints.begin(), snapshot->constraints.end(),
      [](const adagraph::backend::Constraint& constraint) {
        return constraint.type ==
                   adagraph::backend::ConstraintType::kRelocalization ||
               constraint.type == adagraph::backend::ConstraintType::kPrior;
      });
  if (!has_absolute_constraint) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
    return;
  }

  const adagraph::mapping::KeyFrameSnapshot* latest = nullptr;
  for (const auto& keyframe : snapshot->keyframes) {
    if (keyframe.session_id != session_id_ ||
        keyframe.trajectory_id != trajectory_id_) {
      continue;
    }
    if (latest == nullptr ||
        keyframe.T_odom_base.timestamp > latest->T_odom_base.timestamp) {
      latest = &keyframe;
    }
  }
  if (latest == nullptr || !latest->T_map_base.valid() ||
      !latest->T_odom_base.valid()) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
    return;
  }

  const auto candidate_T_map_odom =
      latest->T_map_base.compose(latest->T_odom_base.inverse());
  if (!candidate_T_map_odom) {
    last_backend_alignment_snapshot_version_ = snapshot->version;
    return;
  }

  const SE3 candidate = toLegacyPose(candidate_T_map_odom->T_target_source);
  const SE3 odom_base = toLegacyPose(latest->T_odom_base.T_target_source);
  const auto refined = prior_map_localization_->refineBackendMapToOdom(
      candidate, odom_base);
  last_backend_alignment_snapshot_version_ = snapshot->version;
  if (!refined) {
    return;
  }

  const adagraph::geometry::StampedTransform3d refined_T_map_odom{
      latest->T_odom_base.timestamp,
      adagraph::geometry::FrameId::kMap,
      adagraph::geometry::FrameId::kOdom,
      toDomainPose(*refined),
  };
  const auto T_map_base = refined_T_map_odom.compose(latest->T_odom_base);
  if (!T_map_base) {
    ++alignment_updates_rejected_;
    return;
  }
  const bool committed = commitGlobalAlignment(
      *T_map_base, latest->T_odom_base,
      adagraph::system::GlobalAlignmentSource::kBackendOptimization, 0.0);
  (void)committed;
}

LegacySuperBuffers LegacyDownstreamPipeline::acquireSuperBuffers() {
  {
    std::lock_guard<std::mutex> lock(super_buffer_pool_mutex_);
    if (!super_buffer_pool_.empty()) {
      LegacySuperBuffers buffers = std::move(super_buffer_pool_.back());
      super_buffer_pool_.pop_back();
      return buffers;
    }
  }

  LegacySuperBuffers buffers;
  buffers.full_body_cloud.reset(new PointCloudType());
  buffers.downsampled_body_cloud.reset(new PointCloudType());
  buffers.full_body_cloud->reserve(21'000);
  buffers.downsampled_body_cloud->reserve(21'000);
  buffers.observation_terms.reserve(1);
  return buffers;
}

void LegacyDownstreamPipeline::recycleSuperBuffers(
    LegacySuperFrame&& frame) {
  frame.full_body_cloud->clear();
  frame.downsampled_body_cloud->clear();
  frame.observation_terms.clear();

  LegacySuperBuffers buffers;
  buffers.full_body_cloud = std::move(frame.full_body_cloud);
  buffers.downsampled_body_cloud = std::move(frame.downsampled_body_cloud);
  buffers.observation_terms = std::move(frame.observation_terms);
  std::lock_guard<std::mutex> lock(super_buffer_pool_mutex_);
  super_buffer_pool_.push_back(std::move(buffers));
}

LegacySuperBuffers LegacyDownstreamPipeline::enqueueSuperFrame(
    LegacySuperFrame&& frame) {
  if (!initialized_ || frontend_kind_ != LegacyFrontendKind::kSuperLio) {
    throw std::logic_error(
        "Super-LIO downstream queue is not initialized");
  }
  if (!frame.valid()) {
    throw std::invalid_argument("invalid owning Super-LIO frame");
  }

  LegacySuperBuffers reusable = acquireSuperBuffers();
  {
    std::unique_lock<std::mutex> lock(super_queue_mutex_);
    if (super_queue_.size() >= super_queue_capacity_) {
      ++super_queue_waits_;
    }
    super_queue_not_full_cv_.wait(lock, [this] {
      return super_worker_stop_ ||
             super_queue_.size() < super_queue_capacity_;
    });
    if (super_worker_stop_) {
      throw std::runtime_error("Super-LIO downstream queue is stopping");
    }
    super_queue_.push_back(std::move(frame));
    ++super_frames_enqueued_;
    const auto depth = static_cast<std::uint64_t>(super_queue_.size());
    super_max_queue_depth_.store(
        std::max(super_max_queue_depth_.load(), depth));
  }
  super_queue_not_empty_cv_.notify_one();
  return reusable;
}

void LegacyDownstreamPipeline::superWorkerLoop() {
  for (;;) {
    LegacySuperFrame frame;
    {
      std::unique_lock<std::mutex> lock(super_queue_mutex_);
      super_queue_not_empty_cv_.wait(lock, [this] {
        return super_worker_stop_ || !super_queue_.empty();
      });
      if (super_worker_stop_ && super_queue_.empty()) {
        applyPendingRelocalizationUpdate();
        return;
      }
      frame = std::move(super_queue_.front());
      super_queue_.pop_front();
    }
    super_queue_not_full_cv_.notify_one();

    const auto begin = std::chrono::steady_clock::now();
    try {
      processSuperFrame(frame);
    } catch (const std::exception& error) {
      LOG(ERROR) << RED
                 << " ---> [LegacyDownstreamPipeline]: Super-LIO frame failed: "
                 << error.what() << RESET;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - begin);
    super_downstream_time_ns_.fetch_add(
        static_cast<std::uint64_t>(elapsed.count()));
    recycleSuperBuffers(std::move(frame));
    ++super_frames_processed_;
  }
}

void LegacyDownstreamPipeline::processPointFrame(
    LegacyPointFrame&& owned_frame,
    const IMUData& latest_imu) {
  if (!initialized_) {
    throw std::logic_error(
        "LegacyDownstreamPipeline must be initialized before processing");
  }
  if (frontend_kind_ != LegacyFrontendKind::kPointLio) {
    throw std::logic_error(
        "LegacyDownstreamPipeline received Point-LIO data in Super-LIO mode");
  }

  applyPendingRelocalizationUpdate();

  // The frame retains unique ownership for the whole call. No point vector is
  // copied when crossing from the frontend queue into this system boundary.
  const auto& frame = owned_frame;
  if (frame.dense_points_odom.empty() &&
      frame.matching_points_odom.empty()) {
    return;
  }

  const NavState& state = frame.state;
  const SE3 odom_body = state.GetSE3();

  CloudPtr body_cloud;
  const auto get_body_cloud = [&]() -> const CloudPtr& {
    if (!body_cloud) {
      body_cloud = toBodyCloud(frame.dense_points_odom, odom_body);
    }
    return body_cloud;
  };

  if (backend_scheduler_ && keyframe_policy_) {
    const auto domain_pose = toDomainPose(odom_body);
    if (keyframe_policy_->shouldAccept(domain_pose)) {
      const SE3 last_keyframe_pose =
          toLegacyPose(keyframe_policy_->lastAcceptedPose());
      const CloudPtr matching_cloud = frame.matching_points_odom.empty()
          ? get_body_cloud()
          : toBodyCloud(frame.matching_points_odom, odom_body);
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

  if (prior_map_localization_ && prior_map_localization_->enabled()) {
    if (callbacks_.consume_manual_relocalization_request &&
        callbacks_.consume_manual_relocalization_request()) {
      prior_map_localization_->requestGlobalRelocalization();
    }
    prior_map_localization_->submit(
        state.timestamp, get_body_cloud(), odom_body);
  }

  if (callbacks_.occupancy_grid_enabled &&
      callbacks_.occupancy_grid_enabled()) {
    const int grid_step = runtime_config_.grid_update_step;
    if ((point_grid_count_++ % grid_step) == 0) {
      if (callbacks_.update_occupancy_grid) {
        callbacks_.update_occupancy_grid(get_body_cloud(), state);
      }
    }
  }

  const int pub_step = runtime_config_.visualization_step;
  const bool publish_world =
      runtime_config_.visualize_map &&
      (point_visualization_count_++ % pub_step) == 0;
  if (publish_world) {
    const CloudPtr world_cloud = toCloud(frame.dense_points_odom);
    if (publish_world) {
      if (callbacks_.publish_cloud_world) {
        callbacks_.publish_cloud_world(world_cloud, state.timestamp);
      }
    }
  }
  applyPendingRelocalizationUpdate();
}

void LegacyDownstreamPipeline::processSuperFrame(
    const LegacySuperFrame& frame) {
  if (!initialized_) {
    throw std::logic_error(
        "LegacyDownstreamPipeline must be initialized before processing");
  }
  if (frontend_kind_ != LegacyFrontendKind::kSuperLio) {
    throw std::logic_error(
        "LegacyDownstreamPipeline received Super-LIO data in Point-LIO mode");
  }
  if (!frame.valid()) {
    throw std::invalid_argument("invalid legacy Super-LIO frame view");
  }

  applyPendingRelocalizationUpdate();

  if (backend_scheduler_ && keyframe_policy_) {
    const auto domain_pose = toDomainPose(frame.odom_pose);
    if (keyframe_policy_->shouldAccept(domain_pose)) {
      const FeatureFrame features = AdaGraphFeatureExtractor::build({
          frame.odom_pose,
          toLegacyPose(keyframe_policy_->lastAcceptedPose()),
          frame.observation_terms,
          frame.downsampled_body_cloud,
          frame.effective_constraint_count,
          frame.latest_imu});

      std::vector<MapObservation> map_observations;
      if (backend_config_.use_localization_factor &&
          prior_map_localization_ &&
          prior_map_localization_->enabled()) {
        prior_map_localization_->takeMapObservations(map_observations);
      }

      const bool submitted = submitGraphKeyFrame(
          frame.odom_pose, frame.state.timestamp,
          frame.downsampled_body_cloud, features,
          frame.observation_terms,
          frame.full_body_cloud ? frame.full_body_cloud->size() : 0U,
          frame.effective_constraint_count);
      submitMapObservations(map_observations);
      if (submitted) {
        keyframe_policy_->accept(domain_pose);
      } else {
        keyframe_policy_->reject();
      }

      applyBackendAlignmentFromSnapshot();
    } else {
      keyframe_policy_->reject();
    }
  }

  if (callbacks_.publish_odom) {
    callbacks_.publish_odom(frame.state, frame.angular_velocity);
  }

  if (prior_map_localization_ && prior_map_localization_->enabled()) {
    if (callbacks_.consume_manual_relocalization_request &&
        callbacks_.consume_manual_relocalization_request()) {
      prior_map_localization_->requestGlobalRelocalization();
    }
    prior_map_localization_->submit(
        frame.state.timestamp, frame.full_body_cloud, frame.odom_pose);
  }

  const int grid_step = runtime_config_.grid_update_step;
  ++super_grid_count_;
  if ((super_grid_count_ % grid_step) == 0) {
    super_grid_count_ = 0;
    const CloudPtr& grid_cloud = runtime_config_.visualize_dense
        ? frame.full_body_cloud
        : frame.downsampled_body_cloud;
    if (callbacks_.update_occupancy_grid) {
      callbacks_.update_occupancy_grid(grid_cloud, frame.state);
    }
  }

  const int pub_step = runtime_config_.visualization_step;
  ++super_visualization_count_;
  const bool publish_visualization =
      (super_visualization_count_ % pub_step) == 0;
  if (publish_visualization) {
    super_visualization_count_ = 0;
  }

  if (runtime_config_.visualize_map && publish_visualization) {
    const CloudPtr& body_cloud = runtime_config_.visualize_dense
        ? frame.full_body_cloud
        : frame.downsampled_body_cloud;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) =
        frame.state.R.R_.cast<float>();
    transformation.block<3, 1>(0, 3) = frame.state.p.cast<float>();
    CloudPtr world_cloud(new PointCloudType());
    pcl::transformPointCloud(*body_cloud, *world_cloud, transformation);
    if (callbacks_.publish_cloud_world) {
      callbacks_.publish_cloud_world(world_cloud, frame.state.timestamp);
    }
  }

  accumulateSuperMap(frame);
  applyPendingRelocalizationUpdate();
}

bool LegacyDownstreamPipeline::processPointBackend(
    const LegacyPointFrame& frame,
    const SE3& odom_body,
    const SE3& last_keyframe_pose,
    const CloudPtr& matching_cloud,
    const IMUData& latest_imu) {
  if (!backend_scheduler_) {
    return false;
  }

  std::vector<std::pair<M6, V6>> hessian_terms;
  if (frame.effective_point_updates > 0 &&
      frame.lidar_hessian.allFinite() &&
      frame.lidar_gradient.allFinite()) {
    hessian_terms.emplace_back(
        frame.lidar_hessian.cast<BASIC::scalar>(),
        frame.lidar_gradient.cast<BASIC::scalar>());
  }

  const FeatureFrame features = AdaGraphFeatureExtractor::build({
      odom_body,
      last_keyframe_pose,
      hessian_terms,
      matching_cloud,
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
  submitMapObservations(map_observations);
  applyBackendAlignmentFromSnapshot();
  return submitted;
}

void LegacyDownstreamPipeline::accumulateSuperMap(
    const LegacySuperFrame& frame) {
  if (!runtime_config_.save_map) {
    return;
  }
  if (!super_accumulated_map_ || !super_world_cloud_) {
    throw std::logic_error("Super-LIO map buffers were not initialized");
  }

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = frame.state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = frame.state.p.cast<float>();
  const CloudPtr& source = runtime_config_.filter_map
      ? frame.downsampled_body_cloud
      : frame.full_body_cloud;
  pcl::transformPointCloud(*source, *super_world_cloud_, transformation);

  if (!super_world_cloud_->empty()) {
    *super_accumulated_map_ += *super_world_cloud_;
    ++super_scan_wait_count_;
  }

  if (runtime_config_.pcd_save_interval < 0) {
    super_scan_wait_count_ = 0;
    return;
  }

  if (!prepareSuperPcdDirectory()) {
    return;
  }

  if (!super_accumulated_map_->empty() &&
      super_scan_wait_count_ >= runtime_config_.pcd_save_interval) {
    if (enqueueSuperFragmentSave()) {
      super_accumulated_map_->clear();
      super_scan_wait_count_ = 0;
    }
  }
}

bool LegacyDownstreamPipeline::prepareSuperPcdDirectory() {
  if (super_pcd_dir_prepared_) {
    return true;
  }
  if (!mapIoAvailable()) {
    return false;
  }

  namespace fs = std::filesystem;
  fs::path save_root =
      fs::path(runtime_config_.save_map_dir).lexically_normal();
  if (!save_root.has_filename()) {
    save_root = save_root.parent_path();
  }
  const fs::path pcd_dir = (save_root / "PCD").lexically_normal();
  if (runtime_config_.save_map_dir.empty() || !save_root.is_absolute() ||
      pcd_dir.parent_path() != save_root) {
    LOG(ERROR) << RED << " ---> Invalid save_map_dir '"
               << runtime_config_.save_map_dir
               << "', refusing to reset the PCD cache directory." << RESET;
    return false;
  }

  std::error_code error;
  fs::remove_all(pcd_dir, error);
  if (error) {
    LOG(ERROR) << RED << " ---> Failed to remove " << pcd_dir << ": "
               << error.message() << RESET;
    return false;
  }
  error.clear();
  fs::create_directories(pcd_dir, error);
  if (error) {
    LOG(ERROR) << RED << " ---> Failed to create " << pcd_dir << ": "
               << error.message() << RESET;
    return false;
  }

  super_pcd_paths_.clear();
  super_pcd_dir_prepared_ = true;
  return true;
}

bool LegacyDownstreamPipeline::enqueueSuperFragmentSave() {
  if (!super_accumulated_map_ || super_accumulated_map_->empty() ||
      !prepareSuperPcdDirectory()) {
    return false;
  }

  ++super_pcd_index_;
  const auto path = std::filesystem::path(runtime_config_.save_map_dir) / "PCD" /
                    ("scans_" + std::to_string(super_pcd_index_) + ".pcd");
  if (!enqueuePointCloudSave(super_accumulated_map_, path, 0, 0.0)) {
    --super_pcd_index_;
    return false;
  }
  super_pcd_paths_.push_back(path);
  LOG(INFO) << GREEN << " ---> queued current scan to /PCD/scans_"
            << super_pcd_index_ << "  size: " << super_accumulated_map_->size()
            << RESET;
  return true;
}

bool LegacyDownstreamPipeline::enqueueSuperCachedMapMerge() {
  if (super_pcd_paths_.empty()) {
    return false;
  }

  namespace fs = std::filesystem;
  const fs::path output_dir(runtime_config_.save_map_dir);
  const fs::path configured_name(runtime_config_.map_name);
  if (output_dir.empty() || !output_dir.is_absolute() ||
      configured_name.empty() || configured_name.is_absolute() ||
      configured_name.parent_path() != fs::path()) {
    LOG(ERROR) << RED
               << " ---> [MapSaveScheduler]: invalid fragment merge output: "
               << output_dir << "/" << configured_name << RESET;
    return false;
  }

  auto fragments = std::make_shared<adagraph::io::PcdFragmentSet>();
  fragments->input_paths = super_pcd_paths_;
  fragments->voxelize_each_fragment = runtime_config_.filter_map;
  fragments->fragment_voxel_leaf_size =
      std::max(0.0, runtime_config_.map_voxel_leaf_size);
  fragments->require_all_inputs = true;

  adagraph::io::MapSaveRequest request;
  request.fragment_set = std::move(fragments);
  request.output_path = output_dir / configured_name;
  request.voxel_leaf_size = runtime_config_.filter_map
                                ? std::max(0.0, runtime_config_.map_voxel_leaf_size)
                                : 0.0;
  const auto status = submitMapIoRequest(std::move(request));
  if (status != adagraph::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << RED
               << " ---> [MapSaveScheduler]: unable to enqueue fragment "
                  "merge (status="
               << static_cast<int>(status) << ")" << RESET;
    return false;
  }
  LOG(INFO) << GREEN << " ---> [MapSaveScheduler]: queued merge of "
            << super_pcd_paths_.size() << " PCD fragments to "
            << (output_dir / configured_name) << RESET;
  return true;
}

void LegacyDownstreamPipeline::saveSuperMap() {
  if (!runtime_config_.save_map) {
    return;
  }
  saveSnapshotMap();
  if (!super_accumulated_map_) {
    finishMapIo();
    return;
  }
  if (runtime_config_.pcd_save_interval > 0) {
    LOG(INFO) << YELLOW << " ---> Saving last cache ... " << RESET;
    if (!super_accumulated_map_->empty()) {
      if (enqueueSuperFragmentSave()) {
        super_accumulated_map_->clear();
      }
    }
    LOG(INFO) << YELLOW << " ---> Queue PCD fragment merge ... " << RESET;
    enqueueSuperCachedMapMerge();
    finishMapIo();
    return;
  }

  if (super_accumulated_map_->empty()) {
    finishMapIo();
    return;
  }
  const std::string map_name = runtime_config_.save_map_dir + "/" +
                               runtime_config_.map_name;
  enqueuePointCloudSave(super_accumulated_map_, map_name, 0,
                        runtime_config_.map_voxel_leaf_size);
  finishMapIo();
}

void LegacyDownstreamPipeline::savePointMap() {
  if (!runtime_config_.save_map) {
    return;
  }

  // Keep the explicit Snapshot comparison product while making the final
  // Point-LIO map itself Snapshot-derived as well.
  saveSnapshotMap();

  namespace fs = std::filesystem;
  const fs::path output_dir(runtime_config_.save_map_dir);
  const fs::path map_name(runtime_config_.map_name);
  if (output_dir.empty() || !output_dir.is_absolute()) {
    LOG(ERROR) << RED
               << " ---> [LegacyDownstreamPipeline]: refusing to save map to '"
                 << runtime_config_.save_map_dir
                 << "'; an absolute directory is required."
               << RESET;
    return;
  }
  if (map_name.empty() || map_name.is_absolute() ||
      map_name.parent_path() != fs::path()) {
    LOG(ERROR) << RED
               << " ---> [LegacyDownstreamPipeline]: refusing map filename '"
               << runtime_config_.map_name
               << "'; it must be a plain filename." << RESET;
    return;
  }

  const fs::path output_file = output_dir / map_name;
  const auto snapshot = backend_scheduler_ ? backend_scheduler_->mapSnapshot()
                                           : nullptr;
  if (!enqueueSnapshotMap(snapshot, output_file)) {
    LOG(WARNING) << YELLOW
                 << " ---> [SnapshotMapSave]: no final Point-LIO snapshot "
                    "was available; map was not written."
                 << RESET;
  }
  finishMapIo();
}

bool LegacyDownstreamPipeline::enqueueSnapshotMap(
    const std::shared_ptr<const adagraph::mapping::MapSnapshot>& snapshot,
    const std::filesystem::path& output_path) {
  if (!snapshot || !snapshot->consistent() || snapshot->keyframes.empty() ||
      output_path.empty()) {
    return false;
  }
  adagraph::io::MapSaveRequest request;
  request.snapshot = snapshot;
  request.output_path = output_path;
  request.voxel_leaf_size =
      std::max(0.01, runtime_config_.map_voxel_leaf_size);
  const auto status = submitMapIoRequest(std::move(request));
  if (status != adagraph::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << RED << " ---> [SnapshotMapSave]: unable to enqueue final "
               << output_path << " (status=" << static_cast<int>(status)
               << ")" << RESET;
    return false;
  }
  LOG(INFO) << GREEN << " ---> [SnapshotMapSave]: queued final version="
            << snapshot->version << ", file=" << output_path << RESET;
  return true;
}

void LegacyDownstreamPipeline::saveSnapshotMap() {
  if (!backend_scheduler_ || !mapIoAvailable()) {
    return;
  }
  const auto snapshot = backend_scheduler_->mapSnapshot();
  if (!snapshot || !snapshot->consistent() || snapshot->keyframes.empty()) {
    LOG(WARNING) << YELLOW
                 << " ---> [SnapshotMapSave]: no consistent map snapshot to save."
                 << RESET;
    return;
  }
  namespace fs = std::filesystem;
  const fs::path output_dir(runtime_config_.save_map_dir);
  const fs::path configured_name(runtime_config_.map_name);
  if (output_dir.empty() || !output_dir.is_absolute() ||
      configured_name.empty() || configured_name.is_absolute() ||
      configured_name.parent_path() != fs::path()) {
    LOG(ERROR) << RED
               << " ---> [SnapshotMapSave]: invalid output directory/name: "
               << output_dir << "/" << configured_name << RESET;
    return;
  }

  const fs::path snapshot_name =
      configured_name.stem().string() + "_snapshot" +
      configured_name.extension().string();
  const fs::path output_file = output_dir / snapshot_name;
  adagraph::io::MapSaveRequest request;
  request.snapshot = snapshot;
  request.output_path = output_file;
  request.voxel_leaf_size =
      std::max(0.01, runtime_config_.map_voxel_leaf_size);
  const auto status = submitMapIoRequest(std::move(request));
  if (status != adagraph::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << RED << " ---> [SnapshotMapSave]: unable to enqueue snapshot "
               << snapshot->version << " (status="
               << static_cast<int>(status) << ")" << RESET;
  } else {
    LOG(INFO) << GREEN << " ---> [SnapshotMapSave]: queued version="
              << snapshot->version << ", keyframes="
              << snapshot->keyframes.size() << ", file=" << output_file
              << RESET;
  }
}

bool LegacyDownstreamPipeline::enqueuePointCloudSave(
    const BASIC::CloudPtr& cloud, const std::filesystem::path& output_path,
    std::uint64_t snapshot_version, double voxel_leaf_size) {
  if (!cloud || output_path.empty()) {
    return false;
  }

  auto product =
      std::make_shared<adagraph::mapping::SnapshotPointCloudMap>();
  product->snapshot_version = snapshot_version;
  product->points.reserve(cloud->size());
  Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::infinity());
  Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
      -std::numeric_limits<float>::infinity());
  for (const auto& point : *cloud) {
    const Eigen::Vector3f xyz(point.x, point.y, point.z);
    if (!xyz.allFinite()) {
      continue;
    }
    product->points.push_back({point.x, point.y, point.z});
    minimum = minimum.cwiseMin(xyz);
    maximum = maximum.cwiseMax(xyz);
  }
  if (product->points.empty()) {
    LOG(WARNING) << YELLOW
                 << " ---> [MapSaveScheduler]: refusing to enqueue an empty "
                    "point-cloud product."
                 << RESET;
    return false;
  }
  product->minimum = minimum;
  product->maximum = maximum;

  adagraph::io::MapSaveRequest request;
  request.point_cloud = std::move(product);
  request.output_path = output_path;
  request.voxel_leaf_size = std::max(0.0, voxel_leaf_size);
  const auto status = submitMapIoRequest(std::move(request));
  if (status != adagraph::io::MapSaveSubmissionStatus::kAccepted) {
    LOG(ERROR) << RED << " ---> [MapSaveScheduler]: unable to enqueue "
               << output_path << " (status=" << static_cast<int>(status)
               << ")" << RESET;
    return false;
  }
  return true;
}

void LegacyDownstreamPipeline::finishMapIo() {
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
                     std::promise<adagraph::system::SystemStatus>> pending;
  {
    std::lock_guard<std::mutex> lock(map_save_mutex_);
    pending.swap(pending_map_saves_);
  }
  for (auto& [request_id, promise] : pending) {
    (void)request_id;
    promise.set_value(adagraph::system::SystemStatus::Error(
        adagraph::system::SystemStatusCode::kClosed,
        "map IO scheduler stopped before completing request"));
  }
  LOG(INFO) << GREEN
            << " ---> [MapSaveScheduler]: submitted/processed/failed="
            << stats.submitted << "/" << stats.processed << "/"
            << stats.failed << ", max_queue=" << stats.maximum_queue_depth
            << ", capacity_waits=" << stats.capacity_waits << RESET;
}

}  // namespace LI2Sup
