#ifndef LI2SUP_POINT_LIO_FRONTEND_H_
#define LI2SUP_POINT_LIO_FRONTEND_H_

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "common/navigation_state.h"
#include "hybrid_slam/frontend/frontend_config.h"
#include "hybrid_slam/frontend/point_lio_config.h"
#include "hybrid_slam/sensor/imu_data.h"
#include "hybrid_slam/sensor/point_cloud_frame.h"
#include "lio/frontend/frontend_result.h"
#include "lio/system/point_lio_output_frame.h"

namespace common {
struct Odometry;
}  // namespace common

namespace LI2Sup {

// In-process adapter around the local Point-LIO estimator and its bounded
// output queue. Backend, localization, mapping and map IO are application
// composition responsibilities and are not owned here.
class PointLIOFrontend {
public:
  // ROS-independent construction values. The ROS node/another transport
  // adapter owns parameter declaration and conversion; the frontend only
  // consumes these validated values.
  struct Config {
    hybrid_slam::frontend::FrontendConfig frontend;
    hybrid_slam::frontend::PointLIOConfig point_lio;
    std::size_t input_queue_capacity = 4096;
    std::size_t downstream_queue_capacity = 8;
  };

  // Snapshot of the explicit newest-only downstream contract.  These are
  // lifecycle diagnostics, not a second queue owner: after finish(), every
  // accepted downstream frame is accounted for as either delivered or
  // replaced under bounded backpressure.
  struct DownstreamStats {
    std::uint64_t enqueued = 0;
    std::uint64_t processed = 0;
    std::uint64_t dropped = 0;
    std::uint64_t max_queue_depth = 0;
  };

  // Explicit state required to start a new Point-LIO odom segment after a
  // validated global relocalization. `T_odom_base` and `seed_cloud_odom`
  // share the new local odom frame; map alignment remains outside the ESKF.
  struct RecoveryState {
    double timestamp_seconds = 0.0;
    hybrid_slam::geometry::RigidTransform3d T_odom_base =
        hybrid_slam::geometry::RigidTransform3d::Identity();
    Eigen::Vector3d velocity_odom = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_odom = Eigen::Vector3d(0.0, 0.0, -9.81);
    Eigen::Matrix<double, 30, 30> covariance =
        Eigen::Matrix<double, 30, 30>::Identity() * 0.01;
    bool use_covariance = false;
    // A suspected IMU/timestamp fault uses the normal bootstrap path instead
    // of inheriting velocity, biases and gravity from the old segment.
    bool reinitialize_imu = false;

    [[nodiscard]] bool valid() const noexcept {
      return std::isfinite(timestamp_seconds) && T_odom_base.isFinite() &&
             velocity_odom.allFinite() && gyro_bias.allFinite() &&
             accel_bias.allFinite() && gravity_odom.allFinite() &&
             gravity_odom.norm() > 1e-6 && covariance.allFinite();
    }
  };

  struct MemoryStats {
    std::size_t prior_ivox_entries = 0;
    std::size_t prior_ivox_buckets = 0;
    std::size_t prior_ivox_capacity_limit = 0;
    std::size_t prior_ivox_estimated_bytes = 0;
    std::size_t ivox_entries = 0;
    std::size_t ivox_buckets = 0;
    std::size_t ivox_capacity_limit = 0;
    std::size_t ivox_estimated_bytes = 0;
    std::size_t preprocess_queued_points = 0;
    std::size_t preprocess_queued_imus = 0;
    std::size_t preprocess_vector_backing_bytes = 0;
    std::size_t frame_vector_backing_bytes = 0;
  };

  using DownstreamFrameCallback =
      std::function<void(PointLIOOutputFrame&&, const IMUData&)>;
  using OdometryCallback =
      std::function<void(const NavState&, const Eigen::Vector3d&)>;

  explicit PointLIOFrontend(Config config);
  ~PointLIOFrontend();

  void init();
  void finish();
  // Reuse the same adapter/vendor instance for a new session.  Reset leaves
  // typed ingress disabled; the lifecycle owner enables it after downstream
  // state has been reconstructed.
  void reset();
  [[nodiscard]] bool enqueueImu(hybrid_slam::sensor::ImuData imu);
  [[nodiscard]] bool enqueuePointCloud(
      hybrid_slam::sensor::PointCloudFrame&& cloud);
  // Drain the typed ingress queue on the caller's worker/timer thread.
  // ROS callbacks never call the vendor estimator directly.
  void processInput();
  void setInputEnabled(bool enabled) noexcept;
  // Change future vendor local-map admission without blocking estimator
  // processing. The value is retained across frontend reset.
  void setLocalMapWriteEnabled(bool enabled) noexcept;
  // During LOST, recovery query geometry must be driven by an IMU-only state
  // captured before any new scan-to-map update can corrupt the odom segment.
  void setRecoveryQueryMode(bool enabled) noexcept;
  // Recovery-only local-map operations. The caller must disable typed ingress
  // first so a seed/map reset cannot race vendor estimator processing.
  void replacePriorMapSeed(std::vector<Eigen::Vector3f> points);
  void clearPriorMapSeed();
  void clearOnlineLocalMap();
  void resetAndSeed(const RecoveryState& state,
                    std::vector<Eigen::Vector3f> seed_cloud_odom);
  // Reset-only operation: discard packets that have not entered the vendor
  // estimator and return the number of discarded IMU/lidar packets.
  [[nodiscard]] std::pair<std::size_t, std::size_t> discardQueuedInput();
  [[nodiscard]] bool inputEnabled() const noexcept {
    return input_enabled_.load(std::memory_order_acquire);
  }
  [[nodiscard]] const hybrid_slam::frontend::FrontendConfig& frontendConfig()
      const noexcept {
    return frontend_config_;
  }
  [[nodiscard]] DownstreamStats downstreamStats() const noexcept {
    return {
        frames_enqueued_.load(std::memory_order_acquire),
        frames_processed_.load(std::memory_order_acquire),
        frames_dropped_.load(std::memory_order_acquire),
        max_queue_depth_.load(std::memory_order_acquire),
    };
  }
  [[nodiscard]] MemoryStats memoryStats() const noexcept;

  // Configure before init()/sensor processing. The callback is synchronous
  // and must not retain the borrowed cloud view.
  void setFrontendResultCallback(
      hybrid_slam::frontend::FrontendResultCallback callback) {
    frontend_result_callback_ = std::move(callback);
  }

  // Configure before init(). The callback consumes frames on the existing
  // bounded downstream worker, never on the estimator callback thread.
  void setDownstreamFrameCallback(DownstreamFrameCallback callback) {
    downstream_frame_callback_ = std::move(callback);
  }
  void setOdometryCallback(OdometryCallback callback) {
    odometry_callback_ = std::move(callback);
  }

private:
  class Impl;
  class VendorFrame;

  struct QueuedFrame {
    PointLIOOutputFrame frame;
    IMUData latest_imu;
  };

  void handleOdometry(const common::Odometry& odometry);
  void emitFrontendResult(const VendorFrame& frame) const;
  void enqueueFrame(VendorFrame&& frame);
  void startFrameWorker();
  void stopFrameWorker(bool drain = true);
  void frameWorkerLoop();

  static NavState toNavState(const common::Odometry& odometry);

  void processQueuedImu(hybrid_slam::sensor::ImuData&& imu);
  void processQueuedPointCloud(hybrid_slam::sensor::PointCloudFrame&& cloud);
  void updateRecoveryPropagation(const hybrid_slam::sensor::ImuData& imu) noexcept;
  [[nodiscard]] std::optional<RecoveryQueryFrame> makeRecoveryQueryFrame();

  Config config_;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> input_enabled_{true};
  std::atomic<bool> local_map_write_enabled_{true};

  std::mutex input_queue_mutex_;
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex processing_mutex_;
  std::deque<hybrid_slam::sensor::ImuData> input_imu_queue_;
  std::deque<hybrid_slam::sensor::PointCloudFrame> input_lidar_queue_;
  // A recovery query is used only to choose/reset the new trajectory. Packets
  // from the paused ingress window must not enter the restarted estimator and
  // apply that query scan a second time. These values are guarded by
  // input_queue_mutex_ with the typed input queues.
  std::optional<double> recovery_input_timestamp_floor_seconds_;
  std::optional<double> last_accepted_imu_timestamp_seconds_;
  std::optional<double> last_accepted_lidar_timestamp_seconds_;
  std::size_t input_queue_capacity_ = 4096;
  std::atomic<bool> processing_input_{false};
  IMUData latest_imu_;

  struct RecoveryPropagationState {
    bool active = false;
    bool valid = false;
    NavState state;
    double last_imu_timestamp_seconds = 0.0;
    Eigen::Vector3d last_linear_acceleration_mps2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_angular_velocity_radps = Eigen::Vector3d::Zero();
    bool has_last_imu = false;
  };
  struct RecoveryPropagationSample {
    NavState state;
    Eigen::Vector3d linear_acceleration_mps2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity_radps = Eigen::Vector3d::Zero();
  };
  struct RecoveryRawFrame {
    hybrid_slam::common::Timestamp start_time;
    hybrid_slam::common::Timestamp end_time;
    hybrid_slam::sensor::PointCloudBuffer points;
  };
  mutable std::mutex recovery_query_mutex_;
  std::optional<NavState> latest_healthy_nav_state_;
  RecoveryPropagationState recovery_propagation_;
  std::deque<RecoveryPropagationSample> recovery_propagation_samples_;
  std::deque<RecoveryRawFrame> recovery_raw_frames_;
  std::deque<RecoveryQueryFrame> recovery_query_frames_;

  std::mutex frame_queue_mutex_;
  std::condition_variable frame_queue_cv_;
  std::deque<QueuedFrame> frame_queue_;
  std::thread frame_worker_;
  std::size_t frame_queue_capacity_ = 8;
  bool frame_worker_stop_ = false;
  bool frame_worker_started_ = false;
  std::atomic<std::uint64_t> lidar_callbacks_{0};
  std::atomic<std::uint64_t> imu_callbacks_{0};
  std::atomic<std::uint64_t> frames_enqueued_{0};
  std::atomic<std::uint64_t> frames_processed_{0};
  std::atomic<std::uint64_t> frames_dropped_{0};
  std::atomic<std::uint64_t> max_queue_depth_{0};
  std::atomic<std::uint64_t> backend_time_ns_{0};
  hybrid_slam::frontend::FrontendResultCallback frontend_result_callback_;
  DownstreamFrameCallback downstream_frame_callback_;
  OdometryCallback odometry_callback_;
  hybrid_slam::frontend::FrontendConfig frontend_config_;
  bool initialized_ = false;
  bool finished_ = false;
};

}  // namespace LI2Sup

#endif  // LI2SUP_POINT_LIO_FRONTEND_H_
