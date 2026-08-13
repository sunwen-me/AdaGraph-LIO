#include "lio/point_lio_frontend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "lio/frontend/point_lio_frame.h"
namespace LI2Sup {

using BASIC::SE3;
using BASIC::SO3;

namespace {

small_point_lio::Parameters ToVendorParameters(
    const adagraph::frontend::PointLIOConfig& config) {
  if (const auto error = config.validate()) {
    throw std::invalid_argument("invalid Point-LIO config: " + *error);
  }

  small_point_lio::Parameters parameters;
  parameters.point_filter_num = static_cast<int>(config.point_filter_stride);
  parameters.min_distance_squared =
      static_cast<float>(config.min_range_m * config.min_range_m);
  parameters.max_distance_squared =
      static_cast<float>(config.max_range_m * config.max_range_m);
  parameters.space_downsample = config.space_downsample;
  parameters.space_downsample_leaf_size =
      static_cast<float>(config.space_downsample_leaf_size_m);
  parameters.gravity = config.gravity_mps2;
  parameters.check_satu = config.check_saturation;
  parameters.fix_gravity_direction = config.fix_gravity_direction;
  parameters.satu_acc = config.saturation_acceleration_mps2;
  parameters.satu_gyro = config.saturation_angular_velocity_radps;
  parameters.acc_norm = config.acceleration_norm_mps2;
  parameters.map_resolution = config.local_map_resolution_m;
  parameters.init_map_size = config.initial_map_size;
  parameters.extrinsic_est_en = config.estimate_extrinsic;
  parameters.extrinsic_T = config.T_imu_lidar.T_target_source.translation;
  parameters.extrinsic_R =
      config.T_imu_lidar.T_target_source.rotation.toRotationMatrix();
  parameters.laser_point_cov = config.lidar_point_covariance;
  parameters.imu_meas_acc_cov = config.imu_acceleration_measurement_covariance;
  parameters.imu_meas_omg_cov =
      config.imu_angular_velocity_measurement_covariance;
  parameters.velocity_cov = config.velocity_covariance;
  parameters.acceleration_cov = config.acceleration_covariance;
  parameters.omg_cov = config.angular_velocity_covariance;
  parameters.ba_cov = config.accelerometer_bias_covariance;
  parameters.bg_cov = config.gyroscope_bias_covariance;
  parameters.plane_threshold = config.plane_threshold_m;
  parameters.match_sqaured = config.matching_distance_squared_m2;
  parameters.publish_odometry_without_downsample =
      config.publish_odometry_without_downsample;
  parameters.validate();
  return parameters;
}

}  // namespace

class PointLIOFrontend::Impl {
public:
  explicit Impl(const adagraph::frontend::PointLIOConfig& config)
      : point_lio(std::make_unique<small_point_lio::SmallPointLio>(
            ToVendorParameters(config))) {}

  std::unique_ptr<small_point_lio::SmallPointLio> point_lio;
};

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
  if (!config_.T_base_imu.valid() ||
      config_.T_base_imu.target_frame != adagraph::geometry::FrameId::kBase ||
      config_.T_base_imu.source_frame != adagraph::geometry::FrameId::kImu) {
    throw std::invalid_argument(
        "PointLIOFrontend requires a valid base-to-IMU transform");
  }
  input_queue_capacity_ = config_.input_queue_capacity;
  frame_queue_capacity_ = config_.downstream_queue_capacity;
  impl_ = std::make_unique<Impl>(config_.point_lio);
  frontend_config_.min_range_m = config_.point_lio.min_range_m;
  frontend_config_.max_range_m = config_.point_lio.max_range_m;
  frontend_config_.point_filter_stride = config_.point_lio.point_filter_stride;
  frontend_config_.voxel_filter_size_m =
      config_.point_lio.space_downsample_leaf_size_m;
  frontend_config_.local_map_resolution_m =
      config_.point_lio.local_map_resolution_m;
  frontend_config_.local_map_capacity = config_.point_lio.initial_map_size;
  frontend_config_.gravity_mps2 = config_.point_lio.gravity_mps2.norm();
  frontend_config_.T_imu_lidar = config_.point_lio.T_imu_lidar;
  frontend_config_.T_base_imu = config_.T_base_imu;
  if (const auto error = frontend_config_.validate()) {
    throw std::invalid_argument("invalid Point-LIO config: " + *error);
  }
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

std::pair<std::size_t, std::size_t> PointLIOFrontend::discardQueuedInput() {
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  const std::pair<std::size_t, std::size_t> discarded{
      input_imu_queue_.size(), input_lidar_queue_.size()};
  input_imu_queue_.clear();
  input_lidar_queue_.clear();
  return discarded;
}

bool PointLIOFrontend::enqueueImu(adagraph::sensor::ImuData imu) {
  if (!imu.valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  // Re-check while holding the queue mutex so Reset cannot clear the queue
  // and then receive a callback that passed the old enabled-state check.
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  if (input_imu_queue_.size() >= input_queue_capacity_) {
    return false;
  }
  input_imu_queue_.push_back(std::move(imu));
  ++imu_callbacks_;
  return true;
}

bool PointLIOFrontend::enqueuePointCloud(
    adagraph::sensor::PointCloudFrame&& cloud) {
  if (!cloud.valid() || cloud.points().empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(input_queue_mutex_);
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  if (input_lidar_queue_.size() >= input_queue_capacity_) {
    return false;
  }
  input_lidar_queue_.push_back(std::move(cloud));
  ++lidar_callbacks_;
  return true;
}

void PointLIOFrontend::processQueuedImu(adagraph::sensor::ImuData&& typed) {
  common::ImuMsg imu;
  imu.timestamp = typed.timestamp.seconds();
  imu.angular_velocity = typed.angular_velocity_radps;
  imu.linear_acceleration = typed.linear_acceleration_mps2;
  latest_imu_.secs = imu.timestamp;
  latest_imu_.gyr = imu.angular_velocity.cast<BASIC::scalar>();
  latest_imu_.acc = imu.linear_acceleration.cast<BASIC::scalar>();
  impl_->point_lio->on_imu_callback(imu);
  impl_->point_lio->handle_once();
}

void PointLIOFrontend::processQueuedPointCloud(
    adagraph::sensor::PointCloudFrame&& typed) {
  std::vector<common::Point> pointcloud;
  const auto points = typed.points();
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
}

void PointLIOFrontend::processInput() {
  std::unique_lock<std::mutex> processing_lock(processing_mutex_);
  bool expected = false;
  if (!processing_input_.compare_exchange_strong(expected, true)) {
    return;
  }
  for (;;) {
    std::optional<adagraph::sensor::ImuData> imu;
    std::optional<adagraph::sensor::PointCloudFrame> cloud;
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
  Eigen::Quaterniond q = odometry.orientation.normalized();
  state.R = SO3(q.toRotationMatrix().cast<BASIC::scalar>());
  state.p = odometry.position.cast<BASIC::scalar>();
  state.v = odometry.velocity.cast<BASIC::scalar>();
  return state;
}

void PointLIOFrontend::handleOdometry(const common::Odometry& odometry) {
  const NavState state = toNavState(odometry);
  odometry_callback_(
      state, odometry.angular_velocity.cast<BASIC::scalar>());
}

void PointLIOFrontend::emitFrontendResult(const VendorFrame& frame) const {
  if (!frontend_result_callback_) {
    return;
  }

  using adagraph::common::Timestamp;
  using adagraph::frontend::FrontendResult;
  using adagraph::frontend::Implementation;
  using adagraph::frontend::PointCloudView;
  using adagraph::frontend::Status;
  using adagraph::geometry::FrameId;
  using adagraph::geometry::RigidTransform3d;

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
  result.implementation = Implementation::kPointLio;
  result.status = Status::kTracking;
  const BASIC::SE3 legacy_pose = state.GetSE3();
  const adagraph::geometry::StampedTransform3d T_odom_imu{
      *timestamp,
      FrameId::kOdom,
      FrameId::kImu,
      RigidTransform3d{
          Eigen::Quaterniond(legacy_pose.R_.cast<double>()).normalized(),
          legacy_pose.t_.cast<double>(),
      },
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

  LegacyPointFrame downstream_frame;
  downstream_frame.state = toNavState(frame.value.odometry);
  downstream_frame.dense_points_odom =
      std::move(frame.value.dense_points_odom);
  downstream_frame.matching_points_odom =
      std::move(frame.value.matching_points_odom);
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
      // adapter free of the legacy glog dependency.
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
  }

  // A reset starts a new session; queued downstream frames from the previous
  // session must not be delivered after the system graph is rebuilt.  A
  // frame already executing in the worker is allowed to finish, then join.
  stopFrameWorker(false);
  impl_->point_lio->reset();
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

}  // namespace LI2Sup
