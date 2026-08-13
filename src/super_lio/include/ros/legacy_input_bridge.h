#ifndef LI2SUP_LEGACY_INPUT_BRIDGE_H_
#define LI2SUP_LEGACY_INPUT_BRIDGE_H_

#include <atomic>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "basic/alias.h"
#include "common/ds.h"
#include "ros/sensor_bridge.h"

namespace LI2Sup {

// Compatibility-only ROS input adapter for external callers. It owns legacy
// synchronization buffers and ROS control subscriptions, but does not own an
// estimator, map, or publication state. Standard mapping nodes use
// SensorBridge directly and never instantiate this class.
class LegacyInputBridge final {
 public:
  using LegacyImuPredictor = std::function<bool(
      const IMUData&, DynamicState&, DynamicState&)>;
  using ImuPredictionCallback = std::function<void(
      const builtin_interfaces::msg::Time&, const DynamicState&,
      const DynamicState&)>;

  struct Config {
    SensorBridge::Config sensor;
    std::size_t imu_buffer_capacity = 2000;
    std::size_t lidar_buffer_capacity = 100;
    std::string initial_pose_topic = "/initialpose";
    std::string manual_relocalization_service =
        "/lio/relocalization/trigger";

    [[nodiscard]] static Config FromNode(rclcpp::Node& node);
    [[nodiscard]] std::optional<std::string> validate() const;
  };

  LegacyInputBridge(rclcpp::Node& node,
                    rclcpp::CallbackGroup::SharedPtr callback_group,
                    Config config);

  LegacyInputBridge(const LegacyInputBridge&) = delete;
  LegacyInputBridge& operator=(const LegacyInputBridge&) = delete;

  void start();

  void setInputEnabled(bool enabled) noexcept;
  // Clear buffered compatibility input without changing the enabled state.
  void clear() noexcept;
  [[nodiscard]] bool inputEnabled() const noexcept {
    return input_enabled_.load(std::memory_order_acquire);
  }

  [[nodiscard]] bool sync_measure(MeasureGroup& measure);
  [[nodiscard]] IMUData latestImuData() const;

  void setLegacyImuPredictor(LegacyImuPredictor predictor);
  void setImuPredictionCallback(ImuPredictionCallback callback);

  // Drain IMU samples that arrived through the ROS callback and run the
  // injected compatibility predictor from the application-owned processing
  // context.  ROS subscription callbacks only enqueue samples; they never
  // advance ESKF state or publish prediction output.
  [[nodiscard]] std::size_t processPendingImuPredictions();

  // Initial-pose capture is enabled only while relocation initialization is
  // waiting for a pose.  The pose is transferred as a value, so no ROS
  // subscription callback retains references into estimator state.
  void setInitialPoseCaptureEnabled(bool enabled) noexcept;
  [[nodiscard]] std::optional<BASIC::SE3> takeInitialPose();

  [[nodiscard]] bool consumeManualRelocalizationRequest() noexcept;

 private:
  void handleImuMeasurement(const adagraph::sensor::ImuData& data);
  void pushLidarData(LidarData data);
  void clearBuffers() noexcept;
  void handleInitialPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg);

  rclcpp::Node& node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Config config_;
  std::unique_ptr<SensorBridge> sensor_bridge_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      manual_relocalization_service_;

  mutable std::mutex buffer_mutex_;
  std::deque<IMUData> imu_buffer_;
  std::deque<IMUData> pending_prediction_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;
  IMUData latest_imu_;

  mutable std::mutex predictor_mutex_;
  LegacyImuPredictor imu_predictor_;
  ImuPredictionCallback imu_prediction_callback_;

  mutable std::mutex initial_pose_mutex_;
  std::optional<BASIC::SE3> pending_initial_pose_;
  std::atomic<bool> initial_pose_capture_enabled_{false};
  std::atomic<bool> manual_relocalization_requested_{false};
  std::atomic<bool> input_enabled_{true};
  bool started_ = false;
};

}  // namespace LI2Sup

#endif  // LI2SUP_LEGACY_INPUT_BRIDGE_H_
