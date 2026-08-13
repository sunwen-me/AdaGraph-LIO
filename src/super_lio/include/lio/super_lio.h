

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <memory>
#include <utility>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "adagraph/frontend/frontend_config.h"
#include "adagraph/sensor/imu_data.h"
#include "adagraph/sensor/point_cloud_frame.h"
#include "lio/frontend/frontend_result.h"
#include "lio/system/legacy_downstream_frame.h"

namespace LI2Sup{

class SuperLIO{
public:
  using LegacyMeasureProvider = std::function<bool(MeasureGroup&)>;
  using LatestImuProvider = std::function<IMUData()>;
  using DownstreamFrameCallback =
      std::function<LegacySuperBuffers(LegacySuperFrame&&)>;

  struct Config {
    adagraph::frontend::FrontendConfig frontend;
    ESKF::Options eskf;
    BASIC::M3 lidar_robot_yaw = BASIC::M3::Identity();
    bool ground_robot_planar_constraint = false;
    double ground_robot_base_z = 0.0;
    bool evaluate_timing = false;
    std::size_t input_queue_capacity = 4096;
  };

  SuperLIO();
  explicit SuperLIO(Config config);
  virtual ~SuperLIO();

  // Transitional compatibility input for external legacy callers. Standard
  // compositions use the typed ingress below.
  void setLegacyInputProvider(LegacyMeasureProvider measure_provider,
                              LatestImuProvider latest_imu_provider = {}) {
    legacy_measure_provider_ = std::move(measure_provider);
    latest_imu_provider_ = std::move(latest_imu_provider);
  }
  virtual void init();
  // Reinitialize this frontend instance without replacing its injected
  // callbacks or construction configuration.  Reset leaves typed ingress
  // disabled; the owning lifecycle handler enables it after the new session
  // has been assembled.
  virtual void reset();
  void process();

  [[nodiscard]] ESKF::Ptr estimator() const noexcept { return kf_; }

  // Typed sensor ingress used by the standard ROS adapter and dataset path.
  // The callback only moves values into a bounded queue; the estimator is
  // advanced by process() on the application-owned worker/timer.
  [[nodiscard]] bool enqueueImu(adagraph::sensor::ImuData imu);
  [[nodiscard]] bool enqueuePointCloud(
      adagraph::sensor::PointCloudFrame&& cloud);
  void setTypedInputEnabled(bool enabled) noexcept;
  // Reset-only operation: discard packets that have not entered the legacy
  // estimator and return the number of discarded IMU/lidar packets.
  [[nodiscard]] std::pair<std::size_t, std::size_t> discardTypedInput();
  // Stop accepting new packets and process every complete queued scan. Any
  // incomplete tail is reported by the caller's lifecycle diagnostics.
  void drainTypedInput();
  [[nodiscard]] bool typedInputEnabled() const noexcept {
    return typed_input_enabled_.load(std::memory_order_acquire);
  }
  void printTimeRecord();
  [[nodiscard]] const adagraph::frontend::FrontendConfig& frontendConfig()
      const noexcept {
    return frontend_config_;
  }

  // Configure before processing starts. The callback is synchronous and its
  // borrowed cloud view is valid only until the callback returns.
  void setFrontendResultCallback(
      adagraph::frontend::FrontendResultCallback callback) {
    frontend_result_callback_ = std::move(callback);
  }
  void setDownstreamFrameCallback(DownstreamFrameCallback callback) {
    downstream_frame_callback_ = std::move(callback);
  }

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  void ApplyGroundRobotConstraint();
  virtual void UpdateMap();
  virtual void Output();
  virtual void PostProcess() {}
  void emitFrontendResult() const;
  [[nodiscard]] bool takeTypedMeasure(MeasureGroup& measure);
  [[nodiscard]] static LidarData toLegacyLidar(
      adagraph::sensor::PointCloudFrame&& cloud);
  void initializeBaseState();

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  LegacyMeasureProvider legacy_measure_provider_;
  LatestImuProvider latest_imu_provider_;
  MeasureGroup measures_;

  mutable std::mutex typed_input_mutex_;
  // Serializes estimator execution with reset.  Sensor ingress remains
  // independently protected by typed_input_mutex_ so callbacks can be
  // disabled and drained before this lock is acquired.
  mutable std::mutex processing_mutex_;
  std::deque<adagraph::sensor::ImuData> typed_imu_queue_;
  std::deque<adagraph::sensor::PointCloudFrame> typed_lidar_queue_;
  std::size_t typed_input_queue_capacity_ = 4096;
  std::atomic<bool> typed_input_enabled_{false};
  std::atomic<bool> typed_input_mode_{false};
  IMUData latest_imu_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  // Initialization statistics belong to this frontend instance. Keeping
  // them as members prevents a reset or a second instance from inheriting
  // samples accumulated by a previous process-global function-local state.
  int kf_imu_count_ = 0;
  int kf_still_rejects_ = 0;
  BASIC::V3 kf_mean_gyro_ = BASIC::V3::Zero();
  BASIC::V3 kf_mean_acce_ = BASIC::V3::Zero();
  BASIC::V3 kf_m2_gyro_ = BASIC::V3::Zero();
  BASIC::V3 kf_m2_acce_ = BASIC::V3::Zero();

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  // Sized to the current downsampled scan in Observe(); uint8_t (not
  // std::vector<bool>) so concurrent TBB writes to distinct indices are safe.
  std::vector<uint8_t> effect_mask_;
  std::vector<uint8_t> effect_knn_mask_;
  std::vector<float> point_lengths_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> backend_observation_terms_;
  std::vector<std::array<double, 4>> abcd_vec_;
  Timer time_record_;

  adagraph::frontend::FrontendResultCallback frontend_result_callback_;
  DownstreamFrameCallback downstream_frame_callback_;
  Config config_;
  adagraph::frontend::FrontendConfig frontend_config_;
};

} // namespace END.

#endif
