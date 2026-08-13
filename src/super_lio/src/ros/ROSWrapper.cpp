#include "ros/ROSWrapper.h"

#include <utility>

#include "ros/legacy_input_bridge.h"
#include "ros/publication_bridge.h"

namespace LI2Sup {

ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
    : ROSWrapper("super_lio", true, options) {}

ROSWrapper::~ROSWrapper() = default;

ROSWrapper::ROSWrapper(const std::string& node_name,
                       const bool enable_sensor_input,
                       const rclcpp::NodeOptions& options)
    : ROSWrapper(node_name, enable_sensor_input, true, options) {}

ROSWrapper::ROSWrapper(const std::string& node_name,
                       const bool enable_sensor_input,
                       const bool enable_legacy_output,
                       const rclcpp::NodeOptions& options)
    : rclcpp::Node(node_name, options),
      sensor_input_enabled_(enable_sensor_input) {
  setupIO(enable_sensor_input, enable_legacy_output);
}

void ROSWrapper::setSensorInputEnabled(const bool enabled) {
  sensor_input_enabled_.store(enabled, std::memory_order_release);
  if (legacy_input_bridge_) {
    legacy_input_bridge_->setInputEnabled(enabled);
  }
  if (!enabled) {
    clear();
  }
}

IMUData ROSWrapper::getLatestImuData() const {
  return legacy_input_bridge_ ? legacy_input_bridge_->latestImuData()
                              : IMUData{};
}

void ROSWrapper::clear() {
  if (legacy_input_bridge_) {
    legacy_input_bridge_->clear();
  }
}

bool ROSWrapper::occupancy_grid_enabled() const {
  return publication_bridge_ && publication_bridge_->occupancyGridEnabled();
}

void ROSWrapper::resetSessionState() {
  setSensorInputEnabled(false);
  if (publication_bridge_) {
    publication_bridge_->resetSession();
  }
  if (legacy_input_bridge_) {
    legacy_input_bridge_->setInitialPoseCaptureEnabled(false);
  }
}

void ROSWrapper::setLegacyImuPredictor(LegacyImuPredictor predictor) {
  if (legacy_input_bridge_) {
    legacy_input_bridge_->setLegacyImuPredictor(std::move(predictor));
  }
}

void ROSWrapper::setLegacyImuStateProvider(
    LegacyImuStateProvider provider) {
  std::lock_guard<std::mutex> lock(legacy_imu_predictor_mutex_);
  legacy_imu_state_provider_ = std::move(provider);
}

std::size_t ROSWrapper::processPendingImuPredictions() {
  return legacy_input_bridge_
             ? legacy_input_bridge_->processPendingImuPredictions()
             : 0;
}

void ROSWrapper::pub_odom(const NavState& state) {
  LegacyImuStateProvider provider;
  {
    std::lock_guard<std::mutex> lock(legacy_imu_predictor_mutex_);
    provider = legacy_imu_state_provider_;
  }
  const BASIC::V3 angular_velocity = provider ? provider().w
                                               : BASIC::V3::Zero();
  pub_odom(state, angular_velocity);
}

void ROSWrapper::pub_odom(const NavState& state,
                          const BASIC::V3& angular_velocity) {
  if (publication_bridge_) {
    publication_bridge_->publishOdom(state, angular_velocity);
  }
}

void ROSWrapper::pub_cloud_world(const BASIC::CloudPtr& cloud,
                                 const double timestamp) {
  if (publication_bridge_) {
    publication_bridge_->publishCloudWorld(cloud, timestamp);
  }
}

void ROSWrapper::update_occupancy_grid_from_body_cloud(
    const BASIC::CloudPtr& cloud, const NavState& state) {
  if (publication_bridge_) {
    publication_bridge_->updateOccupancyGridFromBodyCloud(cloud, state);
  }
}

void ROSWrapper::pub_cloud2planner(const BASIC::CloudPtr& cloud,
                                   const double timestamp) {
  if (publication_bridge_) {
    publication_bridge_->publishCloud2Planner(cloud, timestamp);
  }
}

void ROSWrapper::pub_cloud_world_pose(const BASIC::CloudPtr& cloud,
                                      const NavState& state) {
  if (publication_bridge_) {
    publication_bridge_->publishCloudWorldPose(cloud, state);
  }
}

void ROSWrapper::pub_cloud_body_pose(const BASIC::CloudPtr& cloud,
                                     const NavState& state) {
  if (publication_bridge_) {
    publication_bridge_->publishCloudBodyPose(cloud, state);
  }
}

void ROSWrapper::pub_processing_time(const double timestamp,
                                     const double current_time,
                                     const double mean_time,
                                     const double std_time) {
  if (publication_bridge_) {
    publication_bridge_->publishProcessingTime(
        timestamp, current_time, mean_time, std_time);
  }
}

void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map) {
  if (publication_bridge_) {
    publication_bridge_->setGlobalMap(global_map);
  }
}

void ROSWrapper::set_prior_map_localization_active(const bool active) {
  if (publication_bridge_) {
    publication_bridge_->setPriorMapLocalizationActive(active);
  }
}

void ROSWrapper::set_map_to_odom(const BASIC::SE3& map_to_odom) {
  if (publication_bridge_) {
    publication_bridge_->setMapToOdom(map_to_odom);
  }
}

bool ROSWrapper::get_map_to_odom(BASIC::SE3& map_to_odom) const {
  return publication_bridge_ &&
         publication_bridge_->getMapToOdom(map_to_odom);
}

void ROSWrapper::set_backend_map_snapshot(
    std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) noexcept {
  if (publication_bridge_) {
    publication_bridge_->setBackendMapSnapshot(std::move(snapshot));
  }
}

std::shared_ptr<const adagraph::mapping::MapSnapshot>
ROSWrapper::backend_map_snapshot() const noexcept {
  return publication_bridge_ ? publication_bridge_->backendMapSnapshot()
                             : nullptr;
}

bool ROSWrapper::consume_manual_relocalization_request() {
  return legacy_input_bridge_ &&
         legacy_input_bridge_->consumeManualRelocalizationRequest();
}

bool ROSWrapper::sync_measure(MeasureGroup& measure) {
  return legacy_input_bridge_ && legacy_input_bridge_->sync_measure(measure);
}

void ROSWrapper::set_initial_data(BASIC::SE3& init_pose,
                                  bool& flg_get_init_guess,
                                  const bool flg_finish_init) {
  if (!legacy_input_bridge_) {
    return;
  }
  if (flg_finish_init) {
    legacy_input_bridge_->setInitialPoseCaptureEnabled(false);
    return;
  }
  legacy_input_bridge_->setInitialPoseCaptureEnabled(true);
  if (const auto pose = legacy_input_bridge_->takeInitialPose()) {
    init_pose = *pose;
    flg_get_init_guess = true;
  }
}

void ROSWrapper::publishLegacyImuPrediction(
    const builtin_interfaces::msg::Time& stamp,
    const DynamicState& imu_state,
    const DynamicState& robot_state) {
  if (publication_bridge_) {
    publication_bridge_->publishLegacyImuPrediction(
        stamp, imu_state, robot_state);
  }
}

void ROSWrapper::setupIO(const bool enable_sensor_input,
                         const bool enable_legacy_output) {
  cb_sensor_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  if (enable_legacy_output) {
    publication_bridge_ = std::make_unique<PublicationBridge>(
        *this, cb_sensor_, PublicationBridge::Config::FromNode(*this));
    publication_bridge_->setup();
  }

  legacy_input_bridge_ = std::make_unique<LegacyInputBridge>(
      *this, cb_sensor_, LegacyInputBridge::Config::FromNode(*this));
  legacy_input_bridge_->setImuPredictionCallback(
      [this](const builtin_interfaces::msg::Time& stamp,
             const DynamicState& imu_state,
             const DynamicState& robot_state) {
        publishLegacyImuPrediction(stamp, imu_state, robot_state);
      });
  legacy_input_bridge_->start();
  legacy_input_bridge_->setInputEnabled(enable_sensor_input);
}

}  // namespace LI2Sup
