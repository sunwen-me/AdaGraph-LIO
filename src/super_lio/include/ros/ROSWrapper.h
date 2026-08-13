
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include "basic/alias.h"
#include "common/ds.h"

namespace adagraph::mapping {
struct MapSnapshot;
}

namespace LI2Sup {
class OccupancyGridPublisher;
class TfBridge;
class LegacyInputBridge;
class PublicationBridge;
}

namespace LI2Sup{

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ROSWrapper(const std::string& node_name,
             bool enable_sensor_input,
             const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ROSWrapper(const std::string& node_name,
             bool enable_sensor_input,
             bool enable_legacy_output,
             const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper();
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  // Lifecycle adapter gate. The centralized compatibility bridge subscriptions
  // remain installed, but disabled input is discarded and buffered data is
  // cleared, so Stop or FinishTrajectory cannot feed stale scans into a later
  // lifecycle state.
  void setSensorInputEnabled(bool enabled);
  [[nodiscard]] bool sensorInputEnabled() const noexcept {
    return sensor_input_enabled_.load(std::memory_order_acquire);
  }

  // Compatibility-only hook for external callers. Standard mapping nodes feed
  // typed IMU data through SlamSystem and never ask ROSWrapper to predict an
  // estimator state. Keeping the predictor as an injected callback prevents
  // the ROS adapter from acquiring a link-time dependency on Super-LIO's
  // ESKF implementation.
  using LegacyImuPredictor = std::function<bool(
      const IMUData&, DynamicState&, DynamicState&)>;
  void setLegacyImuPredictor(LegacyImuPredictor predictor);
  using LegacyImuStateProvider = std::function<DynamicState()>;
  void setLegacyImuStateProvider(LegacyImuStateProvider provider);

  // Compatibility-only prediction pump.  The ROS IMU callback only records
  // samples; callers that intentionally retain the old ROSWrapper path must
  // invoke this from their serialized application/processing context.
  [[nodiscard]] std::size_t processPendingImuPredictions();

  // 获取最新IMU数据（用于AdaGraph特征提取）
  [[nodiscard]] IMUData getLatestImuData() const;

  void clear();

  // Clear ROS-side derived/session caches while keeping subscriptions and
  // publishers alive for a subsequent lifecycle session.
  void resetSessionState();

  void pub_odom(const NavState&);
  void pub_odom(const NavState&, const BASIC::V3& angular_velocity);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void update_occupancy_grid_from_body_cloud(const BASIC::CloudPtr& pc,
                                             const NavState& state);
  bool occupancy_grid_enabled() const;
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc,
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc,
                           const NavState& state);
  void pub_processing_time(double time, double current_time, double mean_time,
                           double std_time);

  void set_global_map(const BASIC::CloudPtr& global_map);
  void set_prior_map_localization_active(bool active);
  void set_map_to_odom(const BASIC::SE3& map_to_odom);
  bool get_map_to_odom(BASIC::SE3& map_to_odom) const;
  bool consume_manual_relocalization_request();

  void set_backend_map_snapshot(
      std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept;
  [[nodiscard]] std::shared_ptr<const adagraph::mapping::MapSnapshot>
  backend_map_snapshot() const noexcept;

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess,
                        bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

private:
  void publishLegacyImuPrediction(
      const builtin_interfaces::msg::Time& source_stamp,
      const DynamicState& imu_state,
      const DynamicState& robo_state);
  void setupIO(bool enable_sensor_input, bool enable_legacy_output);

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;

  std::atomic<bool> sensor_input_enabled_{true};

  mutable std::mutex legacy_imu_predictor_mutex_;
  // The predictor is stored by LegacyInputBridge and invoked only by the
  // explicit compatibility processing pump.  This state provider remains a
  // small output-side compatibility hook for pub_odom(state).
  LegacyImuStateProvider legacy_imu_state_provider_;
  std::unique_ptr<LegacyInputBridge> legacy_input_bridge_;
  std::unique_ptr<PublicationBridge> publication_bridge_;

};

} // namespace END.

#endif
