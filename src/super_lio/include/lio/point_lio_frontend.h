#ifndef LI2SUP_POINT_LIO_FRONTEND_H_
#define LI2SUP_POINT_LIO_FRONTEND_H_

#include <atomic>
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

#include "common/legacy_state.h"
#include "adagraph/frontend/frontend_config.h"
#include "adagraph/frontend/point_lio_config.h"
#include "adagraph/sensor/imu_data.h"
#include "adagraph/sensor/point_cloud_frame.h"
#include "lio/frontend/frontend_result.h"
#include "lio/system/legacy_point_frame.h"

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
    adagraph::frontend::PointLIOConfig point_lio;
    adagraph::geometry::StaticTransform3d T_base_imu{
        adagraph::geometry::FrameId::kBase,
        adagraph::geometry::FrameId::kImu,
        adagraph::geometry::RigidTransform3d::Identity(),
    };
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

  using DownstreamFrameCallback =
      std::function<void(LegacyPointFrame&&, const IMUData&)>;
  using OdometryCallback =
      std::function<void(const NavState&, const BASIC::V3&)>;

  explicit PointLIOFrontend(Config config);
  ~PointLIOFrontend();

  void init();
  void finish();
  // Reuse the same adapter/vendor instance for a new session.  Reset leaves
  // typed ingress disabled; the lifecycle owner enables it after downstream
  // state has been reconstructed.
  void reset();
  [[nodiscard]] bool enqueueImu(adagraph::sensor::ImuData imu);
  [[nodiscard]] bool enqueuePointCloud(
      adagraph::sensor::PointCloudFrame&& cloud);
  // Drain the typed ingress queue on the caller's worker/timer thread.
  // ROS callbacks never call the vendor estimator directly.
  void processInput();
  void setInputEnabled(bool enabled) noexcept;
  // Reset-only operation: discard packets that have not entered the vendor
  // estimator and return the number of discarded IMU/lidar packets.
  [[nodiscard]] std::pair<std::size_t, std::size_t> discardQueuedInput();
  [[nodiscard]] bool inputEnabled() const noexcept {
    return input_enabled_.load(std::memory_order_acquire);
  }
  [[nodiscard]] const adagraph::frontend::FrontendConfig& frontendConfig()
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

  // Configure before init()/sensor processing. The callback is synchronous
  // and must not retain the borrowed cloud view.
  void setFrontendResultCallback(
      adagraph::frontend::FrontendResultCallback callback) {
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
    LegacyPointFrame frame;
    IMUData latest_imu;
  };

  void handleOdometry(const common::Odometry& odometry);
  void emitFrontendResult(const VendorFrame& frame) const;
  void enqueueFrame(VendorFrame&& frame);
  void startFrameWorker();
  void stopFrameWorker(bool drain = true);
  void frameWorkerLoop();

  static NavState toNavState(const common::Odometry& odometry);

  void processQueuedImu(adagraph::sensor::ImuData&& imu);
  void processQueuedPointCloud(adagraph::sensor::PointCloudFrame&& cloud);

  Config config_;
  std::unique_ptr<Impl> impl_;
  std::atomic<bool> input_enabled_{true};

  std::mutex input_queue_mutex_;
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex processing_mutex_;
  std::deque<adagraph::sensor::ImuData> input_imu_queue_;
  std::deque<adagraph::sensor::PointCloudFrame> input_lidar_queue_;
  std::size_t input_queue_capacity_ = 4096;
  std::atomic<bool> processing_input_{false};
  IMUData latest_imu_;

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
  adagraph::frontend::FrontendResultCallback frontend_result_callback_;
  DownstreamFrameCallback downstream_frame_callback_;
  OdometryCallback odometry_callback_;
  adagraph::frontend::FrontendConfig frontend_config_;
  bool initialized_ = false;
  bool finished_ = false;
};

}  // namespace LI2Sup

#endif  // LI2SUP_POINT_LIO_FRONTEND_H_
