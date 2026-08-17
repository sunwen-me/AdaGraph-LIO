#include "lio/point_lio_frontend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "lio/frontend/point_lio_frame.h"
namespace LI2Sup {

namespace {

void PropagateImuState(NavState& state,
                       const Eigen::Vector3d& linear_acceleration_mps2,
                       const Eigen::Vector3d& angular_velocity_radps,
                       const Eigen::Vector3d& gravity_mps2,
                       const double timestamp_seconds) noexcept {
  const double dt = timestamp_seconds - state.timestamp;
  if (!std::isfinite(dt) || dt <= 0.0) {
    return;
  }
  const double angle = angular_velocity_radps.norm() * dt;
  Eigen::Quaterniond delta = Eigen::Quaterniond::Identity();
  if (angle > 1e-12) {
    delta = Eigen::Quaterniond(Eigen::AngleAxisd(
        angle, angular_velocity_radps.normalized()));
  }
  state.T_odom_imu.rotation =
      (state.T_odom_imu.rotation * delta).normalized();
  const Eigen::Vector3d acceleration_odom =
      state.T_odom_imu.rotation * linear_acceleration_mps2 + gravity_mps2;
  state.T_odom_imu.translation +=
      state.velocity_odom * dt + 0.5 * acceleration_odom * dt * dt;
  state.velocity_odom += acceleration_odom * dt;
  state.timestamp = timestamp_seconds;
}

small_point_lio::Parameters ToVendorParameters(
    const hybrid_slam::frontend::FrontendConfig& frontend,
    const hybrid_slam::frontend::PointLIOConfig& point_lio) {
  if (const auto error = frontend.validate()) {
    throw std::invalid_argument("invalid frontend config: " + *error);
  }
  if (const auto error = point_lio.validate()) {
    throw std::invalid_argument("invalid Point-LIO config: " + *error);
  }

  small_point_lio::Parameters parameters;
  parameters.point_filter_num = static_cast<int>(frontend.point_filter_stride);
  parameters.min_distance_squared =
      static_cast<float>(frontend.min_range_m * frontend.min_range_m);
  parameters.max_distance_squared =
      static_cast<float>(frontend.max_range_m * frontend.max_range_m);
  parameters.space_downsample = point_lio.space_downsample;
  parameters.space_downsample_leaf_size =
      static_cast<float>(frontend.voxel_filter_size_m);
  parameters.gravity = frontend.gravity_mps2;
  parameters.check_satu = point_lio.check_saturation;
  parameters.fix_gravity_direction = point_lio.fix_gravity_direction;
  parameters.satu_acc = point_lio.saturation_acceleration_mps2;
  parameters.satu_gyro = point_lio.saturation_angular_velocity_radps;
  parameters.acc_norm = point_lio.acceleration_norm_mps2;
  parameters.map_resolution = frontend.local_map_resolution_m;
  parameters.init_map_size = point_lio.initial_map_size;
  parameters.extrinsic_est_en = point_lio.estimate_extrinsic;
  parameters.extrinsic_T = frontend.T_imu_lidar.T_target_source.translation;
  parameters.extrinsic_R =
      frontend.T_imu_lidar.T_target_source.rotation.toRotationMatrix();
  parameters.laser_point_cov = point_lio.lidar_point_covariance;
  parameters.imu_meas_acc_cov =
      point_lio.imu_acceleration_measurement_covariance;
  parameters.imu_meas_omg_cov =
      point_lio.imu_angular_velocity_measurement_covariance;
  parameters.velocity_cov = point_lio.velocity_covariance;
  parameters.acceleration_cov = point_lio.acceleration_covariance;
  parameters.omg_cov = point_lio.angular_velocity_covariance;
  parameters.ba_cov = point_lio.accelerometer_bias_covariance;
  parameters.bg_cov = point_lio.gyroscope_bias_covariance;
  parameters.plane_threshold = point_lio.plane_threshold_m;
  parameters.match_sqaured = point_lio.matching_distance_squared_m2;
  parameters.publish_odometry_without_downsample =
      point_lio.publish_odometry_without_downsample;
  parameters.validate();
  return parameters;
}

}  // namespace

class PointLIOFrontend::Impl {
public:
  Impl(const hybrid_slam::frontend::FrontendConfig& frontend,
       const hybrid_slam::frontend::PointLIOConfig& point_lio)
      : point_lio(std::make_unique<small_point_lio::SmallPointLio>(
            ToVendorParameters(frontend, point_lio))) {}

  std::unique_ptr<small_point_lio::SmallPointLio> point_lio;
};

PointLIOFrontend::MemoryStats PointLIOFrontend::memoryStats() const noexcept {
  if (!impl_ || !impl_->point_lio) {
    return {};
  }
  const auto memory = impl_->point_lio->memory_stats();
  return {
      memory.prior_ivox.entry_count,
      memory.prior_ivox.bucket_count,
      memory.prior_ivox.capacity_limit,
      memory.prior_ivox.estimated_bytes,
      memory.ivox.entry_count,
      memory.ivox.bucket_count,
      memory.ivox.capacity_limit,
      memory.ivox.estimated_bytes,
      memory.preprocess.queued_point_count,
      memory.preprocess.queued_imu_count,
      memory.preprocess.vector_backing_bytes,
      memory.frame_vector_backing_bytes,
  };
}

class PointLIOFrontend::VendorFrame {
public:
  explicit VendorFrame(small_point_lio::SmallPointLio::FrontendFrame&& value)
      : value(std::move(value)) {}

  small_point_lio::SmallPointLio::FrontendFrame value;
};

PointLIOFrontend::PointLIOFrontend(Config config)
    : config_(std::move(config)) {}

PointLIOFrontend::~PointLIOFrontend() {
  finish();
}

void PointLIOFrontend::init() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (initialized_) {
    return;
  }
  input_enabled_.store(true, std::memory_order_release);
  if (!downstream_frame_callback_) {
    throw std::runtime_error(
        "PointLIOFrontend requires a downstream frame callback");
  }
  if (!odometry_callback_) {
    throw std::runtime_error(
        "PointLIOFrontend requires an odometry output callback");
  }

  if (config_.input_queue_capacity == 0 ||
      config_.downstream_queue_capacity == 0) {
    throw std::invalid_argument(
        "PointLIOFrontend queue capacities must be positive");
  }
  if (const auto error = config_.frontend.validate()) {
    throw std::invalid_argument(
        "invalid Point-LIO frontend config: " + *error);
  }
  input_queue_capacity_ = config_.input_queue_capacity;
  frame_queue_capacity_ = config_.downstream_queue_capacity;
  {
    std::lock_guard<std::mutex> lock(input_queue_mutex_);
    recovery_input_timestamp_floor_seconds_.reset();
    last_accepted_imu_timestamp_seconds_.reset();
    last_accepted_lidar_timestamp_seconds_.reset();
  }
  impl_ = std::make_unique<Impl>(config_.frontend, config_.point_lio);
  impl_->point_lio->set_map_write_enabled(
      local_map_write_enabled_.load(std::memory_order_acquire));
  frontend_config_ = config_.frontend;
  impl_->point_lio->set_odometry_callback(
      [this](const common::Odometry& odometry) {
        handleOdometry(odometry);
      });
  impl_->point_lio->set_frame_callback(
      [this](small_point_lio::SmallPointLio::FrontendFrame&& frame) {
        enqueueFrame(VendorFrame(std::move(frame)));
      });

  startFrameWorker();
  initialized_ = true;
  finished_ = false;
}

void PointLIOFrontend::setInputEnabled(const bool enabled) noexcept {
  input_enabled_.store(enabled, std::memory_order_release);
}

void PointLIOFrontend::setLocalMapWriteEnabled(const bool enabled) noexcept {
  local_map_write_enabled_.store(enabled, std::memory_order_release);
  // The vendor gate itself is atomic. `impl_` is constructed before any
  // downstream callback is installed and remains alive through finish(), so
  // this must not take lifecycle_mutex_: reset joins that worker.
  if (impl_ && impl_->point_lio) {
    impl_->point_lio->set_map_write_enabled(enabled);
  }
}

void PointLIOFrontend::setRecoveryQueryMode(const bool enabled) noexcept {
  std::lock_guard<std::mutex> lock(recovery_query_mutex_);
  if (!enabled) {
    recovery_propagation_ = {};
    recovery_propagation_samples_.clear();
    recovery_raw_frames_.clear();
    recovery_query_frames_.clear();
    return;
  }
  if (recovery_propagation_.active) {
    return;
  }

  recovery_propagation_ = {};
  recovery_propagation_samples_.clear();
  recovery_raw_frames_.clear();
  recovery_query_frames_.clear();
  recovery_propagation_.active = true;
  if (!latest_healthy_nav_state_ ||
      !std::isfinite(latest_healthy_nav_state_->timestamp) ||
      !latest_healthy_nav_state_->T_odom_imu.isFinite() ||
      !latest_healthy_nav_state_->velocity_odom.allFinite()) {
    return;
  }
  recovery_propagation_.valid = true;
  recovery_propagation_.state = *latest_healthy_nav_state_;
  recovery_propagation_.last_imu_timestamp_seconds =
      latest_healthy_nav_state_->timestamp;
  recovery_propagation_samples_.push_back({
      *latest_healthy_nav_state_, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero()});
}

void PointLIOFrontend::replacePriorMapSeed(
    std::vector<Eigen::Vector3f> points) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!initialized_ || !impl_ || !impl_->point_lio) {
    throw std::logic_error(
        "PointLIOFrontend must be initialized before loading a recovery seed");
  }
  if (input_enabled_.load(std::memory_order_acquire)) {
    throw std::logic_error(
        "PointLIOFrontend ingress must be disabled before loading a recovery seed");
  }
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  impl_->point_lio->set_prior_map(points);
}

void PointLIOFrontend::clearPriorMapSeed() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!initialized_ || !impl_ || !impl_->point_lio) {
    throw std::logic_error(
        "PointLIOFrontend must be initialized before clearing a recovery seed");
  }
  if (input_enabled_.load(std::memory_order_acquire)) {
    throw std::logic_error(
        "PointLIOFrontend ingress must be disabled before clearing a recovery seed");
  }
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  impl_->point_lio->clear_prior_map();
}

void PointLIOFrontend::clearOnlineLocalMap() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!initialized_ || !impl_ || !impl_->point_lio) {
    throw std::logic_error(
        "PointLIOFrontend must be initialized before clearing its online map");
  }
  if (input_enabled_.load(std::memory_order_acquire)) {
    throw std::logic_error(
        "PointLIOFrontend ingress must be disabled before clearing its online map");
  }
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  impl_->point_lio->clear_online_map();
}

void PointLIOFrontend::resetAndSeed(
    const RecoveryState& recovery_state,
    std::vector<Eigen::Vector3f> seed_cloud_odom) {
  if (!recovery_state.valid()) {
    throw std::invalid_argument("invalid Point-LIO recovery state");
  }
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!initialized_ || !impl_ || !impl_->point_lio) {
    throw std::logic_error(
        "PointLIOFrontend must be initialized before resetAndSeed");
  }
  if (input_enabled_.load(std::memory_order_acquire)) {
    throw std::logic_error(
        "PointLIOFrontend ingress must be disabled before resetAndSeed");
  }

  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  {
    std::lock_guard<std::mutex> lock(input_queue_mutex_);
    input_imu_queue_.clear();
    input_lidar_queue_.clear();
    recovery_input_timestamp_floor_seconds_ = recovery_state.timestamp_seconds;
    last_accepted_imu_timestamp_seconds_ = recovery_state.timestamp_seconds;
    last_accepted_lidar_timestamp_seconds_ = recovery_state.timestamp_seconds;
  }
  {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    latest_healthy_nav_state_.reset();
    recovery_propagation_ = {};
    recovery_propagation_samples_.clear();
    recovery_raw_frames_.clear();
    recovery_query_frames_.clear();
  }
  stopFrameWorker(false);

  const auto T_odom_imu = recovery_state.T_odom_base.compose(
      frontend_config_.T_base_imu.T_target_source);
  small_point_lio::SmallPointLio::RecoveryState vendor_state;
  vendor_state.timestamp = recovery_state.timestamp_seconds;
  vendor_state.position = T_odom_imu.translation;
  vendor_state.rotation = T_odom_imu.rotation.toRotationMatrix();
  vendor_state.velocity = recovery_state.velocity_odom;
  vendor_state.gyro_bias = recovery_state.gyro_bias;
  vendor_state.accel_bias = recovery_state.accel_bias;
  vendor_state.gravity = recovery_state.gravity_odom;
  vendor_state.covariance = recovery_state.covariance;
  vendor_state.use_covariance = recovery_state.use_covariance;
  vendor_state.reinitialize_imu = recovery_state.reinitialize_imu;
  impl_->point_lio->reset_and_seed(vendor_state, seed_cloud_odom);
  impl_->point_lio->set_map_write_enabled(
      local_map_write_enabled_.load(std::memory_order_acquire));

  latest_imu_ = IMUData{};
  processing_input_.store(false, std::memory_order_release);
  lidar_callbacks_.store(0, std::memory_order_release);
  imu_callbacks_.store(0, std::memory_order_release);
  frames_enqueued_.store(0, std::memory_order_release);
  frames_processed_.store(0, std::memory_order_release);
  frames_dropped_.store(0, std::memory_order_release);
  max_queue_depth_.store(0, std::memory_order_release);
  backend_time_ns_.store(0, std::memory_order_release);
  finished_ = false;
  startFrameWorker();
}

std::pair<std::size_t, std::size_t> PointLIOFrontend::discardQueuedInput() {
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  const std::pair<std::size_t, std::size_t> discarded{
      input_imu_queue_.size(), input_lidar_queue_.size()};
  input_imu_queue_.clear();
  input_lidar_queue_.clear();
  return discarded;
}

bool PointLIOFrontend::enqueueImu(hybrid_slam::sensor::ImuData imu) {
  if (!imu.valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  // Re-check while holding the queue mutex so Reset cannot clear the queue
  // and then receive a callback that passed the old enabled-state check.
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  const double timestamp = imu.timestamp.seconds();
  if ((recovery_input_timestamp_floor_seconds_ &&
       timestamp <= *recovery_input_timestamp_floor_seconds_) ||
      (last_accepted_imu_timestamp_seconds_ &&
       timestamp < *last_accepted_imu_timestamp_seconds_)) {
    return false;
  }
  if (input_imu_queue_.size() >= input_queue_capacity_) {
    return false;
  }
  last_accepted_imu_timestamp_seconds_ = timestamp;
  input_imu_queue_.push_back(std::move(imu));
  ++imu_callbacks_;
  return true;
}

bool PointLIOFrontend::enqueuePointCloud(
    hybrid_slam::sensor::PointCloudFrame&& cloud) {
  if (!cloud.valid() || cloud.points().empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  const double timestamp = cloud.startTime().seconds();
  if ((recovery_input_timestamp_floor_seconds_ &&
       timestamp <= *recovery_input_timestamp_floor_seconds_) ||
      (last_accepted_lidar_timestamp_seconds_ &&
       timestamp < *last_accepted_lidar_timestamp_seconds_)) {
    return false;
  }
  if (input_lidar_queue_.size() >= input_queue_capacity_) {
    return false;
  }
  last_accepted_lidar_timestamp_seconds_ = timestamp;
  input_lidar_queue_.push_back(std::move(cloud));
  ++lidar_callbacks_;
  return true;
}

void PointLIOFrontend::processQueuedImu(hybrid_slam::sensor::ImuData&& typed) {
  common::ImuMsg imu;
  imu.timestamp = typed.timestamp.seconds();
  imu.angular_velocity = typed.angular_velocity_radps;
  imu.linear_acceleration = typed.linear_acceleration_mps2;
  latest_imu_ = std::move(typed);
  updateRecoveryPropagation(latest_imu_);
  impl_->point_lio->on_imu_callback(imu);
  impl_->point_lio->handle_once();
}

void PointLIOFrontend::processQueuedPointCloud(
    hybrid_slam::sensor::PointCloudFrame&& typed) {
  std::vector<common::Point> pointcloud;
  const auto points = typed.points();
  const auto recovery_query = makeRecoveryQueryFrame();
  if (recovery_query) {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    recovery_query_frames_.push_back(std::move(*recovery_query));
  }
  pointcloud.reserve(points.size());
  const double start_time = typed.startTime().seconds();
  for (const auto& point : points) {
    common::Point converted;
    converted.timestamp =
        start_time + static_cast<double>(point.offset_nanoseconds) * 1e-9;
    converted.position << point.x, point.y, point.z;
    pointcloud.push_back(converted);
  }
  if (!pointcloud.empty()) {
    impl_->point_lio->on_point_cloud_callback(pointcloud);
    impl_->point_lio->handle_once();
  }
  {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    if (recovery_propagation_.active) {
      RecoveryRawFrame raw;
      raw.start_time = typed.startTime();
      raw.end_time = typed.endTime();
      raw.points.assign(points.begin(), points.end());
      recovery_raw_frames_.push_back(std::move(raw));
    }
  }
}

void PointLIOFrontend::updateRecoveryPropagation(
    const hybrid_slam::sensor::ImuData& imu) noexcept {
  if (!imu.valid() || imu.frame != hybrid_slam::geometry::FrameId::kImu) {
    return;
  }
  const double timestamp = imu.timestamp.seconds();
  std::lock_guard<std::mutex> lock(recovery_query_mutex_);
  auto& propagation = recovery_propagation_;
  if (!propagation.active || !propagation.valid ||
      !std::isfinite(timestamp) ||
      timestamp <= propagation.last_imu_timestamp_seconds) {
    return;
  }
  const double dt = timestamp - propagation.last_imu_timestamp_seconds;
  if (!std::isfinite(dt) || dt > 1.0) {
    propagation.valid = false;
    return;
  }

  auto& state = propagation.state;
  PropagateImuState(state, imu.linear_acceleration_mps2,
                    imu.angular_velocity_radps, frontend_config_.gravity_mps2,
                    timestamp);
  propagation.last_imu_timestamp_seconds = timestamp;
  propagation.last_linear_acceleration_mps2 = imu.linear_acceleration_mps2;
  propagation.last_angular_velocity_radps = imu.angular_velocity_radps;
  propagation.has_last_imu = true;
  propagation.valid = state.T_odom_imu.isFinite() &&
                      state.velocity_odom.allFinite();
  if (propagation.valid) {
    recovery_propagation_samples_.push_back({
        state, imu.linear_acceleration_mps2, imu.angular_velocity_radps});
  }
}

std::optional<RecoveryQueryFrame> PointLIOFrontend::makeRecoveryQueryFrame() {
  std::lock_guard<std::mutex> lock(recovery_query_mutex_);
  const auto& propagation = recovery_propagation_;
  if (!propagation.active || !propagation.valid ||
      recovery_raw_frames_.empty() || recovery_propagation_samples_.empty()) {
    return std::nullopt;
  }
  const auto& raw = recovery_raw_frames_.front();
  const double end_timestamp = raw.end_time.seconds();
  if (propagation.state.timestamp < end_timestamp) {
    return std::nullopt;
  }
  const auto timestamp = hybrid_slam::common::Timestamp::FromSeconds(end_timestamp);
  if (!timestamp) {
    return std::nullopt;
  }
  const auto state_at = [this](const double point_timestamp)
      -> std::optional<NavState> {
    const auto found = std::upper_bound(
        recovery_propagation_samples_.begin(),
        recovery_propagation_samples_.end(), point_timestamp,
        [](const double timestamp, const RecoveryPropagationSample& sample) {
          return timestamp < sample.state.timestamp;
        });
    if (found == recovery_propagation_samples_.begin()) {
      return std::nullopt;
    }
    const auto& sample = *std::prev(found);
    NavState state = sample.state;
    PropagateImuState(state, sample.linear_acceleration_mps2,
                      sample.angular_velocity_radps,
                      frontend_config_.gravity_mps2, point_timestamp);
    return state.T_odom_imu.isFinite() ? std::optional<NavState>(state)
                                        : std::nullopt;
  };
  const auto end_state = state_at(end_timestamp);
  if (!end_state) {
    return std::nullopt;
  }
  const hybrid_slam::geometry::StampedTransform3d T_odom_imu{
      *timestamp, hybrid_slam::geometry::FrameId::kOdom,
      hybrid_slam::geometry::FrameId::kImu, end_state->T_odom_imu};
  const auto T_odom_base = T_odom_imu.compose(frontend_config_.T_base_imu.inverse());
  if (!T_odom_base) {
    return std::nullopt;
  }

  RecoveryQueryFrame result;
  result.T_odom_base = *T_odom_base;
  result.points_odom.reserve(raw.points.size());
  for (const auto& point : raw.points) {
    const Eigen::Vector3d lidar_point(point.x, point.y, point.z);
    if (!lidar_point.allFinite()) {
      continue;
    }
    const double point_timestamp = raw.start_time.seconds() +
        static_cast<double>(point.offset_nanoseconds) * 1e-9;
    const auto point_state = state_at(point_timestamp);
    if (!point_state) {
      continue;
    }
    const Eigen::Vector3d point_odom = point_state->T_odom_imu.transformPoint(
        frontend_config_.T_imu_lidar.T_target_source.transformPoint(lidar_point));
    if (point_odom.allFinite()) {
      result.points_odom.push_back(point_odom.cast<float>());
    }
  }
  recovery_raw_frames_.pop_front();
  while (recovery_propagation_samples_.size() > 1 &&
         recovery_propagation_samples_[1].state.timestamp <= end_timestamp) {
    recovery_propagation_samples_.pop_front();
  }
  return result.valid() ? std::optional<RecoveryQueryFrame>(std::move(result))
                        : std::nullopt;
}

void PointLIOFrontend::processInput() {
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  bool expected = false;
  if (!processing_input_.compare_exchange_strong(expected, true)) {
    return;
  }
  for (;;) {
    std::optional<hybrid_slam::sensor::ImuData> imu;
    std::optional<hybrid_slam::sensor::PointCloudFrame> cloud;
    {
      std::lock_guard<std::mutex> lock(input_queue_mutex_);
      if (input_imu_queue_.empty() && input_lidar_queue_.empty()) {
        break;
      }
      const bool take_cloud =
          !input_lidar_queue_.empty() &&
          (input_imu_queue_.empty() ||
           input_lidar_queue_.front().startTime() <=
               input_imu_queue_.front().timestamp);
      if (take_cloud) {
        cloud.emplace(std::move(input_lidar_queue_.front()));
        input_lidar_queue_.pop_front();
      } else {
        imu.emplace(std::move(input_imu_queue_.front()));
        input_imu_queue_.pop_front();
      }
    }
    if (cloud) {
      processQueuedPointCloud(std::move(*cloud));
    } else {
      processQueuedImu(std::move(*imu));
    }
  }
  processing_input_.store(false, std::memory_order_release);
}

NavState PointLIOFrontend::toNavState(const common::Odometry& odometry) {
  NavState state;
  state.timestamp = odometry.timestamp;
  state.T_odom_imu = {
      odometry.orientation.normalized(), odometry.position};
  state.velocity_odom = odometry.velocity;
  return state;
}

void PointLIOFrontend::handleOdometry(const common::Odometry& odometry) {
  const NavState state = toNavState(odometry);
  {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    if (!recovery_propagation_.active) {
      latest_healthy_nav_state_ = state;
    }
  }
  odometry_callback_(state, odometry.angular_velocity);
}

void PointLIOFrontend::emitFrontendResult(const VendorFrame& frame) const {
  if (!frontend_result_callback_) {
    return;
  }

  using hybrid_slam::common::Timestamp;
  using hybrid_slam::frontend::FrontendResult;
  using hybrid_slam::frontend::PointCloudView;
  using hybrid_slam::frontend::Status;
  using hybrid_slam::geometry::FrameId;

  const NavState state = toNavState(frame.value.odometry);
  const auto timestamp = Timestamp::FromSeconds(state.timestamp);
  if (!timestamp) {
    return;
  }
  const auto& points = frame.value.dense_points_odom;

  std::size_t x_offset = 0;
  std::size_t y_offset = sizeof(float);
  std::size_t z_offset = 2 * sizeof(float);
  if (!points.empty()) {
    const auto* origin =
        reinterpret_cast<const std::byte*>(points.data());
    const auto* coordinates = reinterpret_cast<const std::byte*>(
        points.front().data());
    x_offset = static_cast<std::size_t>(coordinates - origin);
    y_offset = x_offset + sizeof(float);
    z_offset = y_offset + sizeof(float);
  }

  FrontendResult result;
  result.status = Status::kTracking;
  const hybrid_slam::geometry::StampedTransform3d T_odom_imu{
      *timestamp,
      FrameId::kOdom,
      FrameId::kImu,
      state.T_odom_imu,
  };
  const auto T_odom_base =
      T_odom_imu.compose(frontend_config_.T_base_imu.inverse());
  if (!T_odom_base) {
    return;
  }
  result.local_pose = *T_odom_base;
  // The vendored Point-LIO already accumulates its registered dense frame in
  // odom. Expose that storage directly; converting it back to IMU/base here
  // would add a full cloud transform/copy to the estimator callback.
  result.registered_cloud = PointCloudView::InterleavedFloatXyz(
      points.empty() ? nullptr : points.data(), points.size(),
      sizeof(Eigen::Vector3f), x_offset, y_offset, z_offset,
      FrameId::kOdom);
  result.matching.input_point_count = points.size();
  result.matching.registered_point_count = points.size();
  result.matching.matching_point_count =
      frame.value.matching_points_odom.size();
  result.matching.effective_constraint_count =
      frame.value.effective_point_updates;

  frontend_result_callback_(result);
}

void PointLIOFrontend::enqueueFrame(VendorFrame&& frame) {
  // Observe the frontend result before ownership of the vendor frame moves
  // into the bounded downstream queue. The callback is synchronous, so the
  // non-owning point view cannot outlive this call.
  emitFrontendResult(frame);

  PointLIOOutputFrame downstream_frame;
  downstream_frame.state = toNavState(frame.value.odometry);
  downstream_frame.dense_points_odom =
      std::move(frame.value.dense_points_odom);
  downstream_frame.matching_points_odom =
      std::move(frame.value.matching_points_odom);
  {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    if (!recovery_query_frames_.empty()) {
      downstream_frame.recovery_query =
          std::move(recovery_query_frames_.front());
      recovery_query_frames_.pop_front();
    }
  }
  downstream_frame.lidar_hessian = frame.value.lidar_hessian;
  downstream_frame.lidar_gradient = frame.value.lidar_gradient;
  downstream_frame.attempted_point_updates =
      frame.value.attempted_point_updates;
  downstream_frame.effective_point_updates =
      frame.value.effective_point_updates;

  std::lock_guard<std::mutex> lock(frame_queue_mutex_);
  if (frame_worker_stop_) {
    return;
  }
  if (frame_queue_.size() >= frame_queue_capacity_) {
    // Frontend state estimation must never wait for mapping/localization.
    // Retain the newest observations when downstream work is overloaded.
    frame_queue_.pop_front();
    ++frames_dropped_;
  }
  frame_queue_.push_back({std::move(downstream_frame), latest_imu_});
  ++frames_enqueued_;
  max_queue_depth_.store(
      std::max(max_queue_depth_.load(),
               static_cast<std::uint64_t>(frame_queue_.size())));
  frame_queue_cv_.notify_one();
}

void PointLIOFrontend::startFrameWorker() {
  std::lock_guard<std::mutex> lock(frame_queue_mutex_);
  if (frame_worker_started_) {
    return;
  }
  frame_worker_stop_ = false;
  frame_worker_started_ = true;
  frame_worker_ = std::thread(&PointLIOFrontend::frameWorkerLoop, this);
}

void PointLIOFrontend::stopFrameWorker(const bool drain) {
  {
    std::lock_guard<std::mutex> lock(frame_queue_mutex_);
    if (!frame_worker_started_) {
      return;
    }
    if (!drain) {
      frame_queue_.clear();
    }
    frame_worker_stop_ = true;
  }
  frame_queue_cv_.notify_all();
  if (frame_worker_.joinable()) {
    frame_worker_.join();
  }
  frame_worker_started_ = false;

  const std::uint64_t processed = frames_processed_.load();
  const double mean_backend_ms = processed == 0
      ? 0.0
      : static_cast<double>(backend_time_ns_.load()) /
            (1e6 * static_cast<double>(processed));
  (void)mean_backend_ms;
}

void PointLIOFrontend::frameWorkerLoop() {
  for (;;) {
    QueuedFrame queued;
    {
      std::unique_lock<std::mutex> lock(frame_queue_mutex_);
      frame_queue_cv_.wait(lock, [this] {
        return frame_worker_stop_ || !frame_queue_.empty();
      });
      if (frame_worker_stop_ && frame_queue_.empty()) {
        return;
      }
      queued = std::move(frame_queue_.front());
      frame_queue_.pop_front();
    }

    const auto begin = std::chrono::steady_clock::now();
    try {
      downstream_frame_callback_(
          std::move(queued.frame), queued.latest_imu);
    } catch (const std::exception&) {
      // The downstream owner reports its own failure. Keep the realtime
      // adapter free of the glog dependency.
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - begin);
    backend_time_ns_.fetch_add(
        static_cast<std::uint64_t>(elapsed.count()));
    ++frames_processed_;
  }
}

void PointLIOFrontend::finish() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (finished_) {
    return;
  }
  if (impl_) {
    input_enabled_.store(false, std::memory_order_release);
    processInput();
    impl_->point_lio->flush_frame();
  }
  stopFrameWorker();
  const auto memory = memoryStats();
  std::clog << "[PointLIOMemory]: prior_ivox(entries/buckets/capacity/estimated)="
            << memory.prior_ivox_entries << "/" << memory.prior_ivox_buckets
            << "/" << memory.prior_ivox_capacity_limit << "/"
            << memory.prior_ivox_estimated_bytes
            << ", online_ivox(entries/buckets/capacity/estimated)="
            << memory.ivox_entries << "/" << memory.ivox_buckets << "/"
            << memory.ivox_capacity_limit << "/" << memory.ivox_estimated_bytes
            << ", preprocess(queued_points/queued_imus/vectors)="
            << memory.preprocess_queued_points << "/"
            << memory.preprocess_queued_imus << "/"
            << memory.preprocess_vector_backing_bytes
            << ", frame_vectors=" << memory.frame_vector_backing_bytes
            << " bytes\n";
  finished_ = true;
}

void PointLIOFrontend::reset() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!initialized_ || !impl_) {
    throw std::logic_error("PointLIOFrontend must be initialized before reset");
  }

  input_enabled_.store(false, std::memory_order_release);
  // Wait for the application timer, if any, to leave the vendor before
  // resetting its mutable estimator and preprocessing queues.
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  {
    std::lock_guard<std::mutex> lock(input_queue_mutex_);
    input_imu_queue_.clear();
    input_lidar_queue_.clear();
    recovery_input_timestamp_floor_seconds_.reset();
    last_accepted_imu_timestamp_seconds_.reset();
    last_accepted_lidar_timestamp_seconds_.reset();
  }

  // A reset starts a new session; queued downstream frames from the previous
  // session must not be delivered after the system graph is rebuilt.  A
  // frame already executing in the worker is allowed to finish, then join.
  stopFrameWorker(false);
  impl_->point_lio->reset();
  latest_imu_ = IMUData{};
  {
    std::lock_guard<std::mutex> lock(recovery_query_mutex_);
    latest_healthy_nav_state_.reset();
    recovery_propagation_ = {};
    recovery_propagation_samples_.clear();
    recovery_raw_frames_.clear();
    recovery_query_frames_.clear();
  }
  processing_input_.store(false, std::memory_order_release);
  lidar_callbacks_.store(0, std::memory_order_release);
  imu_callbacks_.store(0, std::memory_order_release);
  frames_enqueued_.store(0, std::memory_order_release);
  frames_processed_.store(0, std::memory_order_release);
  frames_dropped_.store(0, std::memory_order_release);
  max_queue_depth_.store(0, std::memory_order_release);
  backend_time_ns_.store(0, std::memory_order_release);
  finished_ = false;
  startFrameWorker();
}

}  // namespace LI2Sup
