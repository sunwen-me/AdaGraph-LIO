#include <memory>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "basic/logs.h"
#include "adagraph/system/slam_system.h"
#include "lio/point_lio_frontend.h"
#include "lio/system/legacy_downstream_pipeline.h"
#include "ros/legacy_parameter_bridge.h"
#include "ros/publication_bridge.h"
#include "ros/sensor_bridge.h"
#include "ros/service_bridge.h"

namespace {

using LI2Sup::LID_TYPE;
using LI2Sup::PointLIOFrontend;
using LI2Sup::SensorBridge;

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T default_value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  T value = std::move(default_value);
  node.get_parameter(name, value);
  return value;
}

adagraph::frontend::PointLIOConfig ReadPointLioConfig(rclcpp::Node& node) {
  adagraph::frontend::PointLIOConfig config;
  config.point_filter_stride = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "point_filter_num", 1)));
  config.min_range_m = ReadParameter<double>(node, "min_distance", 0.5);
  config.max_range_m = ReadParameter<double>(node, "max_distance", 1000.0);
  config.space_downsample = ReadParameter<bool>(
      node, "space_downsample", config.space_downsample);
  config.space_downsample_leaf_size_m = ReadParameter<double>(
      node, "space_downsample_leaf_size",
      config.space_downsample_leaf_size_m);

  const auto gravity = ReadParameter<std::vector<double>>(
      node, "gravity", std::vector<double>{0.0, 0.0, -9.81});
  if (gravity.size() != 3) {
    throw std::invalid_argument("gravity must contain exactly three values");
  }
  config.gravity_mps2 = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  config.check_saturation =
      ReadParameter<bool>(node, "check_satu", config.check_saturation);
  config.fix_gravity_direction = ReadParameter<bool>(
      node, "fix_gravity_direction", config.fix_gravity_direction);
  config.saturation_acceleration_mps2 =
      ReadParameter<double>(node, "satu_acc", 3.0) * 0.99;
  config.saturation_angular_velocity_radps =
      ReadParameter<double>(node, "satu_gyro", 35.0) * 0.99;
  config.acceleration_norm_mps2 =
      ReadParameter<double>(node, "acc_norm", config.acceleration_norm_mps2);

  config.local_map_resolution_m =
      ReadParameter<double>(node, "map_resolution", 0.5);
  config.initial_map_size = static_cast<std::size_t>(std::max<std::int64_t>(
      1, ReadParameter<std::int64_t>(
             node, "init_map_size",
             static_cast<std::int64_t>(config.initial_map_size))));

  config.estimate_extrinsic = ReadParameter<bool>(
      node, "extrinsic_est_en", config.estimate_extrinsic);
  const auto extrinsic_t = ReadParameter<std::vector<double>>(
      node, "extrinsic_T", std::vector<double>{0.0, 0.0, 0.0});
  const auto extrinsic_r = ReadParameter<std::vector<double>>(
      node, "extrinsic_R", std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0,
                                                0.0, 0.0, 0.0, 1.0});
  if (extrinsic_t.size() != 3 || extrinsic_r.size() != 9) {
    throw std::invalid_argument(
        "extrinsic_T/extrinsic_R have invalid element counts");
  }
  Eigen::Matrix3d extrinsic_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      extrinsic_rotation(row, column) = extrinsic_r[row * 3 + column];
    }
  }
  config.T_imu_lidar = {
      adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar,
      {Eigen::Quaterniond(extrinsic_rotation).normalized(),
       Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2])},
  };

  config.lidar_point_covariance = ReadParameter<double>(
      node, "laser_point_cov", config.lidar_point_covariance);
  config.imu_acceleration_measurement_covariance = ReadParameter<double>(
      node, "imu_meas_acc_cov", config.imu_acceleration_measurement_covariance);
  config.imu_angular_velocity_measurement_covariance = ReadParameter<double>(
      node, "imu_meas_omg_cov", config.imu_angular_velocity_measurement_covariance);
  config.velocity_covariance = ReadParameter<double>(
      node, "velocity_cov", config.velocity_covariance);
  config.acceleration_covariance = ReadParameter<double>(
      node, "acceleration_cov", config.acceleration_covariance);
  config.angular_velocity_covariance =
      ReadParameter<double>(node, "omg_cov", config.angular_velocity_covariance);
  config.accelerometer_bias_covariance =
      ReadParameter<double>(node, "ba_cov", config.accelerometer_bias_covariance);
  config.gyroscope_bias_covariance =
      ReadParameter<double>(node, "bg_cov", config.gyroscope_bias_covariance);
  config.plane_threshold_m = ReadParameter<double>(
      node, "plane_threshold", config.plane_threshold_m);
  config.matching_distance_squared_m2 = ReadParameter<double>(
      node, "match_sqaured", config.matching_distance_squared_m2);
  config.publish_odometry_without_downsample = ReadParameter<bool>(
      node, "publish_odometry_without_downsample",
      config.publish_odometry_without_downsample);
  if (const auto error = config.validate()) {
    throw std::invalid_argument("invalid Point-LIO parameters: " + *error);
  }
  return config;
}

adagraph::geometry::StaticTransform3d ReadBaseImuTransform(
    rclcpp::Node& node) {
  constexpr double kPi = 3.14159265358979323846;
  const auto values = ReadParameter<std::vector<double>>(
      node, "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  if (values.size() != 6) {
    throw std::invalid_argument(
        "lio.extrinsic.odom_robo must contain exactly six values");
  }
  const Eigen::Vector3d translation(values[0], values[1], values[2]);
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(values[5] * kPi / 180.0,
                         Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(values[4] * kPi / 180.0,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(values[3] * kPi / 180.0,
                         Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return {
      adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu,
      {Eigen::Quaterniond(rotation).normalized(), translation},
  };
}

void DeclarePointLioTransportParameters(rclcpp::Node& node) {
  ReadParameter<std::string>(node, "lidar_topic", "/livox/lidar");
  ReadParameter<std::string>(node, "imu_topic", "/livox/imu");
  ReadParameter<std::string>(node, "lidar_type", "livox_custom_msg");
  ReadParameter<bool>(node, "point_lio.input.reliable", false);
  ReadParameter<int>(node, "point_lio.input.lidar_queue_depth", 64);
  ReadParameter<int>(node, "point_lio.input.imu_queue_depth", 2000);
  ReadParameter<int>(node, "point_lio.input.queue_capacity", 4096);
  ReadParameter<int>(node, "point_lio.backend.queue_capacity", 8);
}

PointLIOFrontend::Config MakeFrontendConfig(rclcpp::Node& node) {
  PointLIOFrontend::Config config;
  config.point_lio = ReadPointLioConfig(node);
  config.T_base_imu = ReadBaseImuTransform(node);
  config.input_queue_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "point_lio.input.queue_capacity", 4096)));
  config.downstream_queue_capacity = static_cast<std::size_t>(std::max(
      1,
      ReadParameter<int>(node, "point_lio.backend.queue_capacity", 8)));
  return config;
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BASIC::init_log();

  auto node = std::make_shared<rclcpp::Node>("adagraph_point_lio");
  auto callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  DeclarePointLioTransportParameters(*node);
  const auto runtime_config =
      LI2Sup::ReadLegacyRuntimeConfigFromRos(*node);
  auto publication = std::make_shared<LI2Sup::PublicationBridge>(
      *node, callback_group,
      LI2Sup::PublicationBridge::Config::FromNode(*node));
  publication->setup();
  auto frontend = std::make_shared<PointLIOFrontend>(MakeFrontendConfig(*node));
  auto manual_relocalization_requested =
      std::make_shared<std::atomic<bool>>(false);
  LI2Sup::LegacyDownstreamCallbacks downstream_callbacks;
  downstream_callbacks.publish_backend_snapshot =
      [publication](std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) {
        publication->setBackendMapSnapshot(std::move(snapshot));
      };
  downstream_callbacks.publish_global_map =
      [publication](const BASIC::CloudPtr& cloud) {
        publication->setGlobalMap(cloud);
      };
  downstream_callbacks.set_prior_map_localization_active =
      [publication](bool active) {
        publication->setPriorMapLocalizationActive(active);
      };
  downstream_callbacks.reset_session_state =
      [publication] { publication->resetSession(); };
  downstream_callbacks.set_map_to_odom =
      [publication](const BASIC::SE3& transform) {
        publication->setMapToOdom(transform);
      };
  downstream_callbacks.get_map_to_odom =
      [publication](BASIC::SE3& transform) {
        return publication->getMapToOdom(transform);
      };
  downstream_callbacks.consume_manual_relocalization_request =
      [manual_relocalization_requested] {
        return manual_relocalization_requested->exchange(
            false, std::memory_order_acq_rel);
      };
  downstream_callbacks.occupancy_grid_enabled =
      [publication] { return publication->occupancyGridEnabled(); };
  downstream_callbacks.update_occupancy_grid =
      [publication](const BASIC::CloudPtr& cloud, const LI2Sup::NavState& state) {
        publication->updateOccupancyGridFromBodyCloud(cloud, state);
      };
  downstream_callbacks.publish_odom =
      [publication](const LI2Sup::NavState& state,
                    const BASIC::V3& angular_velocity) {
        publication->publishOdom(state, angular_velocity);
      };
  downstream_callbacks.publish_cloud_world =
      [publication](const BASIC::CloudPtr& cloud, double timestamp) {
        publication->publishCloudWorld(cloud, timestamp);
      };
  auto downstream = std::make_shared<LI2Sup::LegacyDownstreamPipeline>(
      std::move(downstream_callbacks));

  try {
    frontend->setDownstreamFrameCallback(
        [downstream](LI2Sup::LegacyPointFrame&& frame,
                     const LI2Sup::IMUData& latest_imu) {
          downstream->processPointFrame(std::move(frame), latest_imu);
        });
    frontend->setOdometryCallback(
      [publication](const LI2Sup::NavState& state,
                  const BASIC::V3& angular_velocity) {
          publication->publishOdom(state, angular_velocity);
        });
    frontend->init();
    downstream->init(LI2Sup::LegacyFrontendKind::kPointLio,
                     frontend->frontendConfig(), runtime_config);
    frontend->setInputEnabled(false);

    auto system = std::make_shared<adagraph::system::SlamSystem>();
    auto rejected_imu = std::make_shared<std::atomic<std::uint64_t>>(0);
    auto rejected_cloud = std::make_shared<std::atomic<std::uint64_t>>(0);
    system->setImuSink([frontend](adagraph::sensor::ImuData imu) {
      return frontend->enqueueImu(std::move(imu))
                 ? adagraph::system::SystemStatus::Ok()
                 : adagraph::system::SystemStatus::Error(
                       adagraph::system::SystemStatusCode::kRejected,
                       "Point-LIO typed IMU queue is full or disabled");
    });
    system->setPointCloudSink(
        [frontend](adagraph::sensor::PointCloudFrame&& cloud) {
          return frontend->enqueuePointCloud(std::move(cloud))
                     ? adagraph::system::SystemStatus::Ok()
                     : adagraph::system::SystemStatus::Error(
                           adagraph::system::SystemStatusCode::kRejected,
                           "Point-LIO typed lidar queue is full or disabled");
        });
    system->setProcessHandler([frontend]() {
      frontend->processInput();
      return adagraph::system::SystemStatus::Ok();
    });
    system->setSnapshotProvider(
        [downstream] { return downstream->mapSnapshot(); });
    system->setAlignmentProvider(
        [downstream] { return downstream->alignmentSnapshot(); });
    system->setSaveMapHandler(
        [downstream](adagraph::io::MapSaveRequest request) {
          return downstream->saveMapAsync(std::move(request));
        });

    LI2Sup::SensorBridge::Config sensor_config =
        LI2Sup::SensorBridge::FromNode(*node);
    sensor_config.imu_topic = ReadParameter<std::string>(
        *node, "imu_topic", sensor_config.imu_topic);
    sensor_config.lidar_topic = ReadParameter<std::string>(
        *node, "lidar_topic", sensor_config.lidar_topic);
    sensor_config.lidar_type = SensorBridge::lidarTypeFromName(
        ReadParameter<std::string>(*node, "lidar_type", "sim_xyzi"));
    // Small Point-LIO owns its vendor-side range and point sampling policy.
    // Keep the ROS transport adapter lossless so the same vendor parameters
    // are applied exactly once, rather than inheriting Super-LIO's
    // lio.sensor.* filtering values from a shared YAML file.
    sensor_config.blind_squared = 0.0;
    sensor_config.max_range_squared = 1.0e12;
    sensor_config.filter_rate = 1;
    sensor_config.imu_queue_depth = static_cast<std::size_t>(std::max(
        1, ReadParameter<int>(*node, "point_lio.input.imu_queue_depth", 2000)));
    sensor_config.lidar_queue_depth = static_cast<std::size_t>(std::max(
        1,
        ReadParameter<int>(*node, "point_lio.input.lidar_queue_depth", 64)));
    sensor_config.reliable = ReadParameter<bool>(
        *node, "point_lio.input.reliable", false);
    auto sensor_bridge = std::make_unique<LI2Sup::SensorBridge>(
        *node, callback_group, std::move(sensor_config),
        [system, node, rejected_imu](adagraph::sensor::ImuData imu) {
          const auto status = system->AddImu(std::move(imu));
          if (!status.ok()) {
            const auto count = rejected_imu->fetch_add(1) + 1;
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 5000,
                "SensorBridge IMU packet rejected (%llu total): %s",
                static_cast<unsigned long long>(count),
                status.message.c_str());
          }
        },
        [system, node, rejected_cloud](
            adagraph::sensor::PointCloudFrame&& cloud) {
          const auto status = system->AddPointCloud(std::move(cloud));
          if (!status.ok()) {
            const auto count = rejected_cloud->fetch_add(1) + 1;
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 5000,
                "SensorBridge lidar packet rejected (%llu total): %s",
                static_cast<unsigned long long>(count),
                status.message.c_str());
          }
        });
    sensor_bridge->setInputEnabled(false);
    sensor_bridge->start();

    const auto start_input = [frontend, downstream, bridge = sensor_bridge.get()]
                              (const auto& options) {
      if (!downstream->acceptingInput()) {
        return adagraph::system::SystemStatus::Error(
            adagraph::system::SystemStatusCode::kClosed,
            "legacy Point-LIO pipeline is already finished");
      }
      downstream->setTrajectoryIdentity(options.session_id, options.trajectory_id);
      frontend->setInputEnabled(true);
      bridge->setInputEnabled(true);
      return adagraph::system::SystemStatus::Ok();
    };
    system->setMappingStartHandler(start_input);
    system->setLocalizationStartHandler(start_input);

    const auto finish_input = [frontend, downstream, bridge = sensor_bridge.get()] {
      bridge->setInputEnabled(false);
      frontend->setInputEnabled(false);
      frontend->finish();
      downstream->finish();
      return adagraph::system::SystemStatus::Ok();
    };
    system->setFinishTrajectoryHandler(finish_input);
    system->setStopHandler(finish_input);
    system->setResetHandler(
        [frontend, downstream, bridge = sensor_bridge.get(),
         manual_relocalization_requested] {
          bridge->setInputEnabled(false);
          frontend->setInputEnabled(false);
          manual_relocalization_requested->store(
              false, std::memory_order_release);
          frontend->reset();
          downstream->reset();
          return adagraph::system::SystemStatus::Ok();
        });

    LI2Sup::ServiceBridge service_bridge(
        *node, callback_group, *system, {},
        [downstream] { return downstream->localizationMap(); },
        [manual_relocalization_requested] {
          manual_relocalization_requested->store(
              true, std::memory_order_release);
          return adagraph::system::SystemStatus::Ok();
        });
    service_bridge.start();

    if (const auto status = system->StartMapping(); !status.ok()) {
      RCLCPP_FATAL(node->get_logger(), "failed to start mapping: %s",
                   status.message.c_str());
      rclcpp::shutdown();
      return 1;
    }

    auto process_timer = node->create_wall_timer(
        std::chrono::milliseconds(2),
        [system] { (void)system->ProcessPending(); },
        callback_group);
    (void)process_timer;
    rclcpp::spin(node);

    service_bridge.stop();
    (void)system->FinishTrajectory();
    downstream->savePointMap();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
