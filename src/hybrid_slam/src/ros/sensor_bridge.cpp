#include "ros/sensor_bridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "hybrid_slam/adapter/ros_point_types.h"
#include "ros/livox_point_time.h"

namespace LI2Sup {

namespace {

bool finiteVector(const Eigen::Vector3d& value) {
  return value.allFinite();
}

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, value);
  }
  node.get_parameter(name, value);
  return value;
}

}  // namespace

SensorBridge::Config SensorBridge::FromNode(rclcpp::Node& node) {
  Config config;
  config.imu_topic = ReadParameter<std::string>(
      node, "lio.ros.imu_topic", "/imu");
  config.lidar_topic = ReadParameter<std::string>(
      node, "lio.ros.lidar_topic", "/lidar");
  config.lidar_type = ReadParameter<int>(node, "lio.sensor.lidar_type",
                                         LID_TYPE::SIM_XYZI);
  const double blind = ReadParameter<double>(node, "lio.sensor.blind", 0.0);
  const double max_range =
      ReadParameter<double>(node, "lio.sensor.maxrange", 100.0);
  config.blind_squared = blind * blind;
  config.max_range_squared = max_range * max_range;
  config.filter_rate = std::max(
      1, ReadParameter<int>(node, "lio.sensor.filter_rate", 1));

  const auto acc_scale = ReadParameter<std::vector<double>>(
      node, "lio.sensor.imu_acc_scale", std::vector<double>{1.0, 1.0, 1.0});
  const auto gyro_scale = ReadParameter<std::vector<double>>(
      node, "lio.sensor.imu_gyro_scale", std::vector<double>{1.0, 1.0, 1.0});
  if (acc_scale.size() != 3 || gyro_scale.size() != 3) {
    throw std::invalid_argument(
        "lio.sensor.imu_acc_scale and imu_gyro_scale must contain three values");
  }
  config.imu_acc_scale =
      Eigen::Vector3d(acc_scale[0], acc_scale[1], acc_scale[2]);
  config.imu_gyro_scale =
      Eigen::Vector3d(gyro_scale[0], gyro_scale[1], gyro_scale[2]);
  config.imu_time_offset = ReadParameter<double>(
      node, "lio.sensor.time_offset", 0.0);
  config.imu_queue_depth = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.input.imu_queue_depth", 500)));
  config.lidar_queue_depth = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.input.lidar_queue_depth", 20)));
  config.reliable = ReadParameter<bool>(
      node, "lio.input.reliable", false);
  return config;
}

std::optional<std::string> SensorBridge::Config::validate() const {
  if (imu_topic.empty()) {
    return "imu_topic must not be empty";
  }
  if (lidar_topic.empty()) {
    return "lidar_topic must not be empty";
  }
  if (!SensorBridge::isSupportedLidarType(lidar_type)) {
    return "unsupported lidar_type " + std::to_string(lidar_type);
  }
  if (!std::isfinite(blind_squared) || blind_squared < 0.0) {
    return "blind_squared must be finite and non-negative";
  }
  if (!std::isfinite(max_range_squared) || max_range_squared <= 0.0 ||
      max_range_squared <= blind_squared) {
    return "max_range_squared must exceed blind_squared";
  }
  if (filter_rate < 1) {
    return "filter_rate must be at least one";
  }
  if (!finiteVector(imu_acc_scale) || !finiteVector(imu_gyro_scale)) {
    return "IMU scale vectors must be finite";
  }
  if (!std::isfinite(imu_time_offset)) {
    return "imu_time_offset must be finite";
  }
  if (imu_queue_depth == 0 || lidar_queue_depth == 0) {
    return "subscription queue depths must be positive";
  }
  return std::nullopt;
}

SensorBridge::SensorBridge(rclcpp::Node& node,
                           rclcpp::CallbackGroup::SharedPtr callback_group,
                           Config config,
                           ImuCallback imu_callback,
                           LidarCallback lidar_callback)
    : node_(node),
      callback_group_(std::move(callback_group)),
      config_(std::move(config)),
      imu_callback_(std::move(imu_callback)),
      lidar_callback_(std::move(lidar_callback)) {}

bool SensorBridge::isSupportedLidarType(const int lidar_type) {
  switch (lidar_type) {
    case LID_TYPE::LIVOX:
    case LID_TYPE::HESAI16:
    case LID_TYPE::VELO16:
    case LID_TYPE::VELO32:
    case LID_TYPE::VEL_NCLT:
    case LID_TYPE::OUSTER:
    case LID_TYPE::SIM_XYZI:
    case LID_TYPE::LIVOX_POINTCLOUD2:
    case LID_TYPE::CUSTOM_MID360:
    case LID_TYPE::UNILIDAR:
      return true;
    default:
      return false;
  }
}

std::string SensorBridge::lidarTypeName(const int lidar_type) {
  if (lidar_type < 0 ||
      lidar_type >= static_cast<int>(LID_TYPE_NAMES.size())) {
    return "UNKNOWN";
  }
  return LID_TYPE_NAMES[static_cast<std::size_t>(lidar_type)];
}

int SensorBridge::lidarTypeFromName(const std::string_view name) {
  if (name == "livox_custom_msg") {
    return LID_TYPE::LIVOX;
  }
  if (name == "livox_pointcloud2") {
    return LID_TYPE::LIVOX_POINTCLOUD2;
  }
  if (name == "hesai16") {
    return LID_TYPE::HESAI16;
  }
  if (name == "velodyne16") {
    return LID_TYPE::VELO16;
  }
  if (name == "velodyne32" || name == "velodyne_pointcloud2") {
    return LID_TYPE::VELO32;
  }
  if (name == "nclt") {
    return LID_TYPE::VEL_NCLT;
  }
  if (name == "ouster") {
    return LID_TYPE::OUSTER;
  }
  if (name == "sim_xyzi") {
    return LID_TYPE::SIM_XYZI;
  }
  if (name == "custom_mid360_driver") {
    return LID_TYPE::CUSTOM_MID360;
  }
  if (name == "unilidar") {
    return LID_TYPE::UNILIDAR;
  }
  return -1;
}

void SensorBridge::start() {
  if (started_) {
    return;
  }
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid SensorBridge config: " + *error);
  }
  if (!imu_callback_ || !lidar_callback_) {
    throw std::invalid_argument(
        "SensorBridge requires IMU and lidar callbacks");
  }

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(config_.imu_queue_depth))
                     .durability_volatile();
  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(config_.lidar_queue_depth))
                       .durability_volatile();
  if (config_.reliable) {
    imu_qos.reliable();
    lidar_qos.reliable();
  } else {
    imu_qos.best_effort();
    lidar_qos.best_effort();
  }

  imu_subscription_ = node_.create_subscription<sensor_msgs::msg::Imu>(
      config_.imu_topic, imu_qos,
      std::bind(&SensorBridge::imuHandler, this, std::placeholders::_1),
      subscription_options);

  if (config_.lidar_type == LID_TYPE::LIVOX) {
    livox_subscription_ =
        node_.create_subscription<livox_ros_driver2::msg::CustomMsg>(
            config_.lidar_topic, lidar_qos,
            std::bind(&SensorBridge::livoxHandler, this,
                      std::placeholders::_1),
            subscription_options);
  } else {
    pointcloud_subscription_ =
        node_.create_subscription<sensor_msgs::msg::PointCloud2>(
            config_.lidar_topic, lidar_qos,
            std::bind(&SensorBridge::stdMsgHandler, this,
                      std::placeholders::_1),
            subscription_options);
  }
  started_ = true;
  RCLCPP_INFO(node_.get_logger(),
              "SensorBridge started: lidar=%s (%s), imu=%s, qos=%s, queues=%zu/%zu",
              config_.lidar_topic.c_str(),
              lidarTypeName(config_.lidar_type).c_str(),
              config_.imu_topic.c_str(), config_.reliable ? "reliable" : "best_effort",
              config_.lidar_queue_depth, config_.imu_queue_depth);
}

double SensorBridge::stampToSec(
    const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         static_cast<double>(stamp.nanosec) * 1e-9;
}

bool SensorBridge::validPoint(const double x, const double y,
                              const double z) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
    return false;
  }
  const double distance_squared = x * x + y * y + z * z;
  return distance_squared > config_.blind_squared &&
         distance_squared < config_.max_range_squared;
}

const sensor_msgs::msg::PointField* SensorBridge::findPointField(
    const sensor_msgs::msg::PointCloud2& msg, const std::string& name) {
  const auto it = std::find_if(
      msg.fields.begin(), msg.fields.end(),
      [&name](const sensor_msgs::msg::PointField& field) {
        return field.name == name;
      });
  return it == msg.fields.end() ? nullptr : &(*it);
}

bool SensorBridge::hasLivoxPointCloud2Layout(
    const sensor_msgs::msg::PointCloud2& msg, std::string& error) {
  struct RequiredField {
    const char* name;
    std::uint8_t datatype;
  };
  constexpr std::array<RequiredField, 6> required_fields = {{
      {"x", sensor_msgs::msg::PointField::FLOAT32},
      {"y", sensor_msgs::msg::PointField::FLOAT32},
      {"z", sensor_msgs::msg::PointField::FLOAT32},
      {"intensity", sensor_msgs::msg::PointField::FLOAT32},
      {"tag", sensor_msgs::msg::PointField::UINT8},
      {"timestamp", sensor_msgs::msg::PointField::FLOAT64},
  }};
  for (const auto& required : required_fields) {
    const auto* field = findPointField(msg, required.name);
    if (!field) {
      error = "missing field '" + std::string(required.name) + "'";
      return false;
    }
    if (field->datatype != required.datatype || field->count != 1) {
      error = "field '" + std::string(required.name) +
              "' has an incompatible datatype or count";
      return false;
    }
  }
  if (msg.point_step == 0 || msg.data.empty()) {
    error = "empty data or zero point_step";
    return false;
  }
  return true;
}

void SensorBridge::imuHandler(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!msg || !input_enabled_.load(std::memory_order_acquire)) {
    return;
  }
  const auto timestamp = hybrid_slam::common::Timestamp::FromSeconds(
      stampToSec(msg->header.stamp) + config_.imu_time_offset);
  if (!timestamp) {
    return;
  }
  hybrid_slam::sensor::ImuData data;
  data.timestamp = *timestamp;
  data.frame = hybrid_slam::geometry::FrameId::kImu;
  data.linear_acceleration_mps2 =
      Eigen::Vector3d(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z)
          .cwiseProduct(config_.imu_acc_scale.cast<double>());
  data.angular_velocity_radps =
      Eigen::Vector3d(msg->angular_velocity.x,
                      msg->angular_velocity.y,
                      msg->angular_velocity.z)
          .cwiseProduct(config_.imu_gyro_scale.cast<double>());
  if (data.valid()) {
    imu_callback_(std::move(data));
  }
}

void SensorBridge::emitLidar(std::vector<ParsedPoint>&& points,
                             const double header_time_seconds) {
  if (!input_enabled_.load(std::memory_order_acquire) || points.empty()) {
    return;
  }
  double minimum_offset = std::numeric_limits<double>::infinity();
  double maximum_offset = -std::numeric_limits<double>::infinity();
  for (const auto& point : points) {
    minimum_offset = std::min(minimum_offset, point.offset_seconds);
    maximum_offset = std::max(maximum_offset, point.offset_seconds);
  }
  if (!std::isfinite(minimum_offset) || !std::isfinite(maximum_offset)) {
    return;
  }
  const auto start_time = hybrid_slam::common::Timestamp::FromSeconds(
      header_time_seconds + minimum_offset);
  const auto end_time = hybrid_slam::common::Timestamp::FromSeconds(
      header_time_seconds + maximum_offset);
  if (!start_time || !end_time || *end_time < *start_time) {
    return;
  }

  hybrid_slam::sensor::PointCloudBuffer buffer;
  buffer.reserve(points.size());
  for (const auto& point : points) {
    const double normalized_offset = point.offset_seconds - minimum_offset;
    const auto offset_nanoseconds =
        hybrid_slam::common::Timestamp::FromSeconds(normalized_offset);
    if (!offset_nanoseconds) {
      continue;
    }
    buffer.push_back({point.x, point.y, point.z, point.intensity,
                      offset_nanoseconds->nanoseconds()});
  }
  if (buffer.empty()) {
    return;
  }
  hybrid_slam::sensor::PointCloudFrame frame(
      *start_time, *end_time, hybrid_slam::geometry::FrameId::kLidar,
      std::move(buffer));
  if (frame.valid()) {
    lidar_callback_(std::move(frame));
  }
}

void SensorBridge::livoxHandler(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  if (!msg || msg->point_num < 10) {
    return;
  }
  const std::size_t point_count = msg->point_num;
  const std::size_t filter_step = static_cast<std::size_t>(config_.filter_rate);
  std::vector<ParsedPoint> points;
  points.reserve(point_count / filter_step + 1);
  for (std::size_t index = 0; index < point_count; ++index) {
    const auto& point = msg->points[index];
    const double offset_time = point.offset_time * 1e-9;
    if (index % filter_step != 0) {
      continue;
    }
    const auto tag = static_cast<std::uint8_t>(point.tag & 0x30);
    if ((tag != 0x10 && tag != 0x00) ||
        !validPoint(point.x, point.y, point.z)) {
      continue;
    }
    points.push_back({point.x, point.y, point.z, point.reflectivity,
                      offset_time});
  }
  emitLidar(std::move(points), stampToSec(msg->header.stamp));
}

void SensorBridge::stdMsgHandler(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!msg || msg->data.empty() ||
      !input_enabled_.load(std::memory_order_acquire)) {
    return;
  }
  const std::size_t filter_step = static_cast<std::size_t>(config_.filter_rate);
  const double header_time = stampToSec(msg->header.stamp);
  std::vector<ParsedPoint> points;

  switch (config_.lidar_type) {
    case LID_TYPE::HESAI16: {
      pcl::PointCloud<hesai_ros::Point> source;
      pcl::fromROSMsg(*msg, source);
      if (source.empty()) return;
      points.reserve(source.size() / filter_step + 1);
      const double time_begin = source.front().timestamp;
      for (std::size_t i = 0; i < source.size(); ++i) {
        const auto& point = source[i];
        const double offset = point.timestamp - time_begin;
        if (i % filter_step == 0 && validPoint(point.x, point.y, point.z)) {
          points.push_back({point.x, point.y, point.z, point.intensity, offset});
        }
      }
      break;
    }
    case LID_TYPE::VEL_NCLT: {
      pcl::PointCloud<NCLT::Point> source;
      pcl::fromROSMsg(*msg, source);
      points.reserve(source.size() / filter_step + 1);
      for (std::size_t i = 0; i < source.size(); ++i) {
        const auto& point = source[i];
        const double offset = point.time * 1e-6;
        if (i % filter_step == 0 && validPoint(point.x, point.y, point.z)) {
          points.push_back({point.x, point.y, point.z, 1.0F, offset});
        }
      }
      break;
    }
    case LID_TYPE::VELO16:
    case LID_TYPE::VELO32: {
      pcl::PointCloud<velodyne_ros::Point> source;
      pcl::fromROSMsg(*msg, source);
      points.reserve(source.size() / filter_step + 1);
      for (std::size_t i = 0; i < source.size(); ++i) {
        const auto& point = source[i];
        const double offset = static_cast<double>(point.time);
        if (i % filter_step == 0 && validPoint(point.x, point.y, point.z)) {
          points.push_back({point.x, point.y, point.z, point.intensity, offset});
        }
      }
      break;
    }
    case LID_TYPE::OUSTER: {
      pcl::PointCloud<ouster_ros::Point> source;
      pcl::fromROSMsg(*msg, source);
      points.reserve(source.size() / filter_step + 1);
      for (std::size_t i = 0; i < source.size(); ++i) {
        const auto& point = source[i];
        const double offset = point.t * 1e-9;
        if (i % filter_step == 0 && validPoint(point.x, point.y, point.z)) {
          points.push_back({point.x, point.y, point.z, point.intensity, offset});
        }
      }
      break;
    }
    case LID_TYPE::SIM_XYZI: {
      pcl::PointCloud<pcl::PointXYZI> source;
      pcl::fromROSMsg(*msg, source);
      points.reserve(source.size() / filter_step + 1);
      for (std::size_t i = 0; i < source.size(); i += filter_step) {
        const auto& point = source[i];
        if (validPoint(point.x, point.y, point.z)) {
          points.push_back({point.x, point.y, point.z, point.intensity, 0.0});
        }
      }
      break;
    }
    case LID_TYPE::CUSTOM_MID360: {
      const auto* timestamp_field = findPointField(*msg, "timestamp");
      if (!timestamp_field ||
          timestamp_field->datatype != sensor_msgs::msg::PointField::FLOAT64) {
        RCLCPP_ERROR_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000,
            "Custom Mid-360 PointCloud2 rejected: expected FLOAT64 "
            "'timestamp' field");
        return;
      }
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<double> timestamp(*msg,
                                                               "timestamp");
      const std::size_t count = static_cast<std::size_t>(msg->width) *
                                msg->height;
      points.reserve(count / filter_step + 1);
      for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++timestamp) {
        if (i % filter_step != 0 || !validPoint(*x, *y, *z) ||
            !std::isfinite(*timestamp)) {
          continue;
        }
        const double timestamp_value = *timestamp;
        const double offset =
            std::isfinite(header_time) && std::abs(header_time) > 1.0 &&
                    std::abs(timestamp_value - header_time) < 10.0
                ? timestamp_value - header_time
                : timestamp_value;
        points.push_back({*x, *y, *z, 0.0F, offset});
      }
      break;
    }
    case LID_TYPE::UNILIDAR: {
      const auto* timestamp_field = findPointField(*msg, "timestamp");
      if (!timestamp_field ||
          timestamp_field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
        RCLCPP_ERROR_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000,
            "Unilidar PointCloud2 rejected: expected FLOAT32 'timestamp' "
            "field");
        return;
      }
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> timestamp(*msg,
                                                               "timestamp");
      const std::size_t count = static_cast<std::size_t>(msg->width) *
                                msg->height;
      points.reserve(count / filter_step + 1);
      for (std::size_t i = 0; i < count; ++i, ++x, ++y, ++z, ++timestamp) {
        if (i % filter_step != 0 || !validPoint(*x, *y, *z) ||
            !std::isfinite(*timestamp)) {
          continue;
        }
        points.push_back({*x, *y, *z, 0.0F,
                          static_cast<double>(*timestamp)});
      }
      break;
    }
    case LID_TYPE::LIVOX_POINTCLOUD2: {
      std::string layout_error;
      if (!hasLivoxPointCloud2Layout(*msg, layout_error)) {
        RCLCPP_ERROR_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000,
            "Livox PointCloud2 rejected: %s. Configure xfer_format=0.",
            layout_error.c_str());
        return;
      }
      sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> intensity(*msg, "intensity");
      sensor_msgs::PointCloud2ConstIterator<std::uint8_t> tag(*msg, "tag");
      sensor_msgs::PointCloud2ConstIterator<double> timestamp(*msg, "timestamp");
      const std::size_t count = static_cast<std::size_t>(msg->width) * msg->height;
      points.reserve(count / filter_step + 1);
      const double frame_start_ns =
          static_cast<double>(msg->header.stamp.sec) * 1e9 +
          static_cast<double>(msg->header.stamp.nanosec);
      for (std::size_t i = 0; i < count;
           ++i, ++x, ++y, ++z, ++intensity, ++tag, ++timestamp) {
        const auto offset = decodeLivoxPointOffsetSeconds(*timestamp,
                                                            frame_start_ns);
        if (!offset || i % filter_step != 0 || ((*tag & 0x30) != 0x00 &&
                                                 (*tag & 0x30) != 0x10) ||
            !validPoint(*x, *y, *z)) {
          continue;
        }
        points.push_back({*x, *y, *z, *intensity, *offset});
      }
      break;
    }
    default:
      return;
  }

  if (points.size() >= 1) {
    emitLidar(std::move(points), header_time);
  }
}

}  // namespace LI2Sup
