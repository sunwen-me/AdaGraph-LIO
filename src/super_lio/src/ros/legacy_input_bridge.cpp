#include "ros/legacy_input_bridge.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "basic/logs.h"

namespace LI2Sup {

namespace {

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, value);
  }
  node.get_parameter(name, value);
  return value;
}

builtin_interfaces::msg::Time ToRosTime(const double seconds) {
  builtin_interfaces::msg::Time time;
  time.sec = static_cast<std::int32_t>(std::floor(seconds));
  time.nanosec = static_cast<std::uint32_t>(
      std::max(0.0, seconds - static_cast<double>(time.sec)) * 1e9);
  return time;
}

}  // namespace

LegacyInputBridge::Config LegacyInputBridge::Config::FromNode(
    rclcpp::Node& node) {
  Config config;
  config.sensor.imu_topic = ReadParameter<std::string>(
      node, "lio.ros.imu_topic", "/imu");
  config.sensor.lidar_topic = ReadParameter<std::string>(
      node, "lio.ros.lidar_topic", "/lidar");
  config.sensor.lidar_type = ReadParameter<int>(
      node, "lio.sensor.lidar_type", LID_TYPE::SIM_XYZI);
  const double blind = ReadParameter<double>(
      node, "lio.sensor.blind", 0.0);
  const double max_range = ReadParameter<double>(
      node, "lio.sensor.maxrange", 100.0);
  config.sensor.blind_squared = blind * blind;
  config.sensor.max_range_squared = max_range * max_range;
  config.sensor.filter_rate = std::max(
      1, ReadParameter<int>(node, "lio.sensor.filter_rate", 1));

  const auto acc_scale = ReadParameter<std::vector<double>>(
      node, "lio.sensor.imu_acc_scale", std::vector<double>{1.0, 1.0, 1.0});
  const auto gyro_scale = ReadParameter<std::vector<double>>(
      node, "lio.sensor.imu_gyro_scale", std::vector<double>{1.0, 1.0, 1.0});
  if (acc_scale.size() != 3 || gyro_scale.size() != 3) {
    throw std::invalid_argument(
        "lio.sensor.imu_acc_scale and imu_gyro_scale must contain three values");
  }
  config.sensor.imu_acc_scale =
      BASIC::V3(acc_scale[0], acc_scale[1], acc_scale[2]);
  config.sensor.imu_gyro_scale =
      BASIC::V3(gyro_scale[0], gyro_scale[1], gyro_scale[2]);
  config.sensor.imu_time_offset = ReadParameter<double>(
      node, "lio.sensor.time_offset", 0.0);
  config.sensor.imu_queue_depth = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.input.imu_queue_depth", 500)));
  config.sensor.lidar_queue_depth = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.input.lidar_queue_depth", 20)));
  config.sensor.reliable = ReadParameter<bool>(
      node, "lio.input.reliable", false);

  config.imu_buffer_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.ros.imu_buffer_capacity", 2000)));
  config.lidar_buffer_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.ros.lidar_buffer_capacity", 100)));
  config.initial_pose_topic = ReadParameter<std::string>(
      node, "lio.relocation.initial_pose_topic", "/initialpose");
  config.manual_relocalization_service = ReadParameter<std::string>(
      node, "lio.lifecycle.manual_relocalization_service",
      "/lio/relocalization/trigger");
  return config;
}

std::optional<std::string> LegacyInputBridge::Config::validate() const {
  if (const auto error = sensor.validate()) {
    return error;
  }
  if (imu_buffer_capacity == 0 || lidar_buffer_capacity == 0) {
    return "legacy input buffer capacities must be positive";
  }
  if (initial_pose_topic.empty() || initial_pose_topic.front() != '/') {
    return "initial_pose_topic must be an absolute ROS name";
  }
  if (manual_relocalization_service.empty() ||
      manual_relocalization_service.front() != '/') {
    return "manual_relocalization_service must be an absolute ROS name";
  }
  return std::nullopt;
}

LegacyInputBridge::LegacyInputBridge(
    rclcpp::Node& node, rclcpp::CallbackGroup::SharedPtr callback_group,
    Config config)
    : node_(node),
      callback_group_(std::move(callback_group)),
      config_(std::move(config)) {}

void LegacyInputBridge::start() {
  if (started_) {
    return;
  }
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid LegacyInputBridge config: " + *error);
  }

  sensor_bridge_ = std::make_unique<SensorBridge>(
      node_, callback_group_, config_.sensor,
      [this](adagraph::sensor::ImuData data) {
        handleImuMeasurement(data);
      },
      [this](adagraph::sensor::PointCloudFrame&& cloud) {
        LidarData legacy;
        legacy.start_time = cloud.startTime().seconds();
        legacy.end_time = cloud.endTime().seconds();
        legacy.pc = std::make_shared<pcl::PointCloud<PointXTZIT>>();
        const auto points = std::move(cloud).releasePoints();
        legacy.pc->points.reserve(points.size());
        for (const auto& point : points) {
          legacy.pc->points.emplace_back(
              point.x, point.y, point.z, point.intensity,
              static_cast<double>(point.offset_nanoseconds) * 1e-9);
        }
        legacy.pc->width = static_cast<std::uint32_t>(legacy.pc->points.size());
        legacy.pc->height = 1;
        legacy.pc->is_dense = true;
        pushLidarData(std::move(legacy));
      });

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  initial_pose_subscription_ = node_.create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      config_.initial_pose_topic, rclcpp::QoS(1),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        handleInitialPose(msg);
      },
      subscription_options);

  manual_relocalization_service_ = node_.create_service<std_srvs::srv::Trigger>(
      config_.manual_relocalization_service,
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        manual_relocalization_requested_.store(true,
                                                std::memory_order_release);
        response->success = true;
        response->message = "manual relocalization request cached";
        LOG(INFO) << YELLOW
                  << " ---> [LegacyInputBridge]: manual relocalization "
                     "request cached."
                  << RESET;
      },
      rclcpp::ServicesQoS(), callback_group_);

  sensor_bridge_->start();
  started_ = true;
}

void LegacyInputBridge::setInputEnabled(const bool enabled) noexcept {
  input_enabled_.store(enabled, std::memory_order_release);
  if (sensor_bridge_) {
    sensor_bridge_->setInputEnabled(enabled);
  }
  if (!enabled) {
    clearBuffers();
  }
}

void LegacyInputBridge::clear() noexcept {
  clearBuffers();
}

void LegacyInputBridge::clearBuffers() noexcept {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  imu_buffer_.clear();
  pending_prediction_buffer_.clear();
  lidar_buffer_.clear();
  lidar_pushed_ = false;
  last_timestamp_imu_ = -1.0;
  last_timestamp_lidar_ = -1.0;
  latest_imu_ = IMUData{};
}

void LegacyInputBridge::setLegacyImuPredictor(
    LegacyImuPredictor predictor) {
  std::lock_guard<std::mutex> lock(predictor_mutex_);
  imu_predictor_ = std::move(predictor);
}

void LegacyInputBridge::setImuPredictionCallback(
    ImuPredictionCallback callback) {
  std::lock_guard<std::mutex> lock(predictor_mutex_);
  imu_prediction_callback_ = std::move(callback);
}

void LegacyInputBridge::handleImuMeasurement(
    const adagraph::sensor::ImuData& data) {
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return;
  }

  IMUData legacy;
  legacy.secs = data.timestamp.seconds();
  legacy.acc = data.linear_acceleration_mps2.cast<BASIC::scalar>();
  legacy.gyr = data.angular_velocity_radps.cast<BASIC::scalar>();
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (legacy.secs < last_timestamp_imu_) {
      LOG(WARNING) << "imu loop back, clear buffer";
      imu_buffer_.clear();
      pending_prediction_buffer_.clear();
    }
    imu_buffer_.push_back(legacy);
    pending_prediction_buffer_.push_back(legacy);
    latest_imu_ = legacy;
    last_timestamp_imu_ = legacy.secs;
    while (imu_buffer_.size() > config_.imu_buffer_capacity) {
      imu_buffer_.pop_front();
      RCLCPP_WARN_THROTTLE(
          node_.get_logger(), *node_.get_clock(), 5000,
          "imu buffer full (%zu samples), dropping oldest; "
          "is the lidar stream alive?",
          config_.imu_buffer_capacity);
    }
    while (pending_prediction_buffer_.size() > config_.imu_buffer_capacity) {
      pending_prediction_buffer_.pop_front();
    }
  }
}

std::size_t LegacyInputBridge::processPendingImuPredictions() {
  std::deque<IMUData> pending;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    pending.swap(pending_prediction_buffer_);
  }

  LegacyImuPredictor predictor;
  ImuPredictionCallback output;
  {
    std::lock_guard<std::mutex> lock(predictor_mutex_);
    predictor = imu_predictor_;
    output = imu_prediction_callback_;
  }
  if (!predictor) {
    return 0;
  }

  std::size_t processed = 0;
  for (const auto& imu : pending) {
    DynamicState imu_state;
    DynamicState robot_state;
    if (predictor(imu, imu_state, robot_state)) {
      ++processed;
      if (output) {
        output(ToRosTime(imu.secs), imu_state, robot_state);
      }
    }
  }
  return processed;
}

void LegacyInputBridge::pushLidarData(LidarData data) {
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return;
  }
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (last_timestamp_lidar_ > 0.0 &&
      data.end_time < last_timestamp_lidar_ - 1.0) {
    LOG(WARNING) << "lidar loop back, clear buffer";
    lidar_buffer_.clear();
    lidar_pushed_ = false;
    last_timestamp_lidar_ = -1.0;
  }
  lidar_buffer_.push_back(std::move(data));
  while (lidar_buffer_.size() > config_.lidar_buffer_capacity) {
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    RCLCPP_WARN_THROTTLE(
        node_.get_logger(), *node_.get_clock(), 5000,
        "lidar buffer full (%zu scans), dropping oldest; "
        "is the IMU stream alive?",
        config_.lidar_buffer_capacity);
  }
}

bool LegacyInputBridge::sync_measure(MeasureGroup& measure) {
  if (!input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }
  if (!lidar_pushed_) {
    measure.lidar = lidar_buffer_.front();
    lidar_pushed_ = true;
  }
  if (last_timestamp_lidar_ > measure.lidar.end_time) {
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }
  if (last_timestamp_imu_ < measure.lidar.end_time) {
    return false;
  }

  measure.imu.clear();
  while (!imu_buffer_.empty() &&
         imu_buffer_.front().secs <= measure.lidar.end_time) {
    measure.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  if (measure.imu.empty()) {
    last_timestamp_lidar_ = measure.lidar.end_time;
    lidar_buffer_.pop_front();
    lidar_pushed_ = false;
    return false;
  }
  last_timestamp_lidar_ = measure.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}

IMUData LegacyInputBridge::latestImuData() const {
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  return latest_imu_;
}

void LegacyInputBridge::setInitialPoseCaptureEnabled(const bool enabled) noexcept {
  initial_pose_capture_enabled_.store(enabled, std::memory_order_release);
  if (!enabled) {
    std::lock_guard<std::mutex> lock(initial_pose_mutex_);
    pending_initial_pose_.reset();
  }
}

std::optional<BASIC::SE3> LegacyInputBridge::takeInitialPose() {
  std::lock_guard<std::mutex> lock(initial_pose_mutex_);
  if (!pending_initial_pose_) {
    return std::nullopt;
  }
  auto pose = pending_initial_pose_;
  pending_initial_pose_.reset();
  return pose;
}

bool LegacyInputBridge::consumeManualRelocalizationRequest() noexcept {
  return manual_relocalization_requested_.exchange(
      false, std::memory_order_acq_rel);
}

void LegacyInputBridge::handleInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr& msg) {
  if (!msg ||
      !initial_pose_capture_enabled_.load(std::memory_order_acquire)) {
    return;
  }
  BASIC::V3 translation;
  translation << msg->pose.pose.position.x, msg->pose.pose.position.y, 0.2;
  const Eigen::Quaterniond rotation(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  if (!rotation.coeffs().allFinite()) {
    return;
  }
  const BASIC::SE3 pose(
      BASIC::SO3(rotation.normalized().toRotationMatrix().cast<BASIC::scalar>()),
      translation);
  {
    std::lock_guard<std::mutex> lock(initial_pose_mutex_);
    pending_initial_pose_ = pose;
  }
  LOG(INFO) << YELLOW << " ---> GET Initial guess: "
            << translation.transpose() << RESET;
}

}  // namespace LI2Sup
