

#include <memory>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "basic/logs.h"
#include "lio/super_lio.h"
#include "lio/system/legacy_downstream_pipeline.h"
#include "adagraph/system/slam_system.h"
#include "ros/legacy_parameter_bridge.h"
#include "ros/publication_bridge.h"
#include "ros/sensor_bridge.h"
#include "ros/service_bridge.h"


using namespace LI2Sup;

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T default_value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  T value = std::move(default_value);
  node.get_parameter(name, value);
  return value;
}

adagraph::geometry::StaticTransform3d ReadStaticTransform(
    rclcpp::Node& node, const char* name, adagraph::geometry::FrameId target,
    adagraph::geometry::FrameId source, std::size_t count,
    const std::vector<double>& defaults) {
  const auto values = ReadParameter<std::vector<double>>(node, name, defaults);
  if (values.size() != count) {
    throw std::invalid_argument(std::string(name) + " has invalid length");
  }
  if (count == 12) {
    Eigen::Matrix3d rotation;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        rotation(row, column) = values[3 + row * 3 + column];
      }
    }
    return {target, source,
            {Eigen::Quaterniond(rotation).normalized(),
             Eigen::Vector3d(values[0], values[1], values[2])}};
  }
  constexpr double kPi = 3.14159265358979323846;
  const Eigen::Matrix3d rotation =
      (Eigen::AngleAxisd(values[5] * kPi / 180.0,
                         Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(values[4] * kPi / 180.0,
                         Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(values[3] * kPi / 180.0,
                         Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return {target, source,
          {Eigen::Quaterniond(rotation).normalized(),
           Eigen::Vector3d(values[0], values[1], values[2])}};
}

SuperLIO::Config MakeSuperLioConfig(rclcpp::Node& node) {
  SuperLIO::Config config;
  const double blind = ReadParameter<double>(node, "lio.sensor.blind", 0.0);
  const double max_range =
      ReadParameter<double>(node, "lio.sensor.maxrange", 100.0);
  config.frontend.min_range_m = std::max(0.0, blind);
  config.frontend.max_range_m = std::max(config.frontend.min_range_m + 1e-6,
                                         max_range);
  config.frontend.point_filter_stride = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.sensor.filter_rate", 1)));
  config.frontend.voxel_filter_size_m = ReadParameter<double>(
      node, "lio.sensor.voxel_fliter_size", 0.2);
  config.frontend.local_map_resolution_m =
      ReadParameter<double>(node, "lio.hash_map.vox_resolution", 0.5);
  config.frontend.local_map_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.hash_map.hash_capacity", 1000000)));
  config.frontend.gravity_mps2 =
      ReadParameter<double>(node, "lio.sensor.gravity_norm", 9.81);
  config.frontend.T_imu_lidar = ReadStaticTransform(
      node, "lio.extrinsic.lidar_imu", adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar, 12,
      std::vector<double>{0.0, 0.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0});
  const auto odom_robo_values = ReadParameter<std::vector<double>>(
      node, "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  config.frontend.T_base_imu = ReadStaticTransform(
      node, "lio.extrinsic.odom_robo", adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu, 6, std::vector<double>(6, 0.0));
  if (odom_robo_values.size() != 6) {
    throw std::invalid_argument(
        "lio.extrinsic.odom_robo must contain exactly six values");
  }

  config.eskf.gyro_var_ =
      ReadParameter<double>(node, "lio.sensor.imu_ng", 0.0);
  config.eskf.acce_var_ =
      ReadParameter<double>(node, "lio.sensor.imu_na", 0.0);
  config.eskf.bias_gyro_var_ =
      ReadParameter<double>(node, "lio.sensor.imu_nbg", 0.0);
  config.eskf.bias_acce_var_ =
      ReadParameter<double>(node, "lio.sensor.imu_nba", 0.0);
  config.eskf.num_iterations_ =
      ReadParameter<int>(node, "lio.kf.kf_max_iterations", 4);
  config.eskf.quit_eps_ =
      ReadParameter<double>(node, "lio.kf.kf_quit_eps", 1e-6);
  config.eskf.gravity_norm_ = config.frontend.gravity_mps2;
  constexpr double kPi = 3.14159265358979323846;
  config.lidar_robot_yaw =
      Eigen::AngleAxisd(odom_robo_values[5] * kPi / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix()
          .cast<BASIC::scalar>();
  config.ground_robot_planar_constraint = ReadParameter<bool>(
      node, "lio.ground_robot.planar_constraint", false);
  config.ground_robot_base_z =
      ReadParameter<double>(node, "lio.ground_robot.base_z", 0.0);
  config.evaluate_timing = ReadParameter<bool>(node, "lio.eva.timer", false);
  config.input_queue_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "lio.input.queue_capacity", 4096)));
  return config;
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  BASIC::init_log();

  auto node = std::make_shared<rclcpp::Node>("adagraph_super_lio");
  auto callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  const LegacyRuntimeConfig runtime_config =
      ReadLegacyRuntimeConfigFromRos(*node);

  auto publication = std::make_shared<PublicationBridge>(
      *node, callback_group, PublicationBridge::Config::FromNode(*node));
  publication->setup();
  auto manual_relocalization_requested =
      std::make_shared<std::atomic<bool>>(false);
  LegacyDownstreamCallbacks downstream_callbacks;
  downstream_callbacks.publish_backend_snapshot =
      [publication](std::shared_ptr<const adagraph::mapping::MapSnapshot>
                        snapshot) {
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
      [publication](const BASIC::CloudPtr& cloud, const NavState& state) {
        publication->updateOccupancyGridFromBodyCloud(cloud, state);
      };
  downstream_callbacks.publish_odom =
      [publication](const NavState& state, const BASIC::V3& angular_velocity) {
        publication->publishOdom(state, angular_velocity);
      };
  downstream_callbacks.publish_cloud_world =
      [publication](const BASIC::CloudPtr& cloud, double timestamp) {
        publication->publishCloudWorld(cloud, timestamp);
      };
  auto downstream = std::make_shared<LegacyDownstreamPipeline>(
      std::move(downstream_callbacks));
  auto lio = std::make_shared<SuperLIO>(MakeSuperLioConfig(*node));
  lio->setDownstreamFrameCallback(
      [downstream](LegacySuperFrame&& frame) {
        return downstream->enqueueSuperFrame(std::move(frame));
      });
  lio->init();
  lio->setTypedInputEnabled(false);
  downstream->init(LegacyFrontendKind::kSuperLio,
                   lio->frontendConfig(), runtime_config);

  auto slam_system = std::make_shared<adagraph::system::SlamSystem>();
  slam_system->setImuSink([lio](adagraph::sensor::ImuData imu) {
    return lio->enqueueImu(std::move(imu))
               ? adagraph::system::SystemStatus::Ok()
               : adagraph::system::SystemStatus::Error(
                     adagraph::system::SystemStatusCode::kRejected,
                     "Super-LIO typed IMU queue is full or disabled");
  });
  slam_system->setPointCloudSink(
      [lio](adagraph::sensor::PointCloudFrame&& cloud) {
        return lio->enqueuePointCloud(std::move(cloud))
                   ? adagraph::system::SystemStatus::Ok()
                   : adagraph::system::SystemStatus::Error(
                         adagraph::system::SystemStatusCode::kRejected,
                     "Super-LIO typed lidar queue is full or disabled");
      });
  slam_system->setProcessHandler([lio]() {
    lio->process();
    return adagraph::system::SystemStatus::Ok();
  });
  slam_system->setSnapshotProvider(
      [downstream] { return downstream->mapSnapshot(); });
  slam_system->setAlignmentProvider(
      [downstream] { return downstream->alignmentSnapshot(); });
  slam_system->setSaveMapHandler(
      [downstream](adagraph::io::MapSaveRequest request) {
        return downstream->saveMapAsync(std::move(request));
      });
  const auto start_input = [downstream, lio](
                               const auto& options) {
    if (!downstream->acceptingInput()) {
      return adagraph::system::SystemStatus::Error(
          adagraph::system::SystemStatusCode::kClosed,
          "legacy Super-LIO pipeline is already finished");
    }
    downstream->setTrajectoryIdentity(options.session_id, options.trajectory_id);
    lio->setTypedInputEnabled(true);
    return adagraph::system::SystemStatus::Ok();
  };
  const auto finish_input = [downstream, lio]() {
    lio->setTypedInputEnabled(false);
    lio->drainTypedInput();
    downstream->finish();
    return adagraph::system::SystemStatus::Ok();
  };
  auto rejected_imu = std::make_shared<std::atomic<std::uint64_t>>(0);
  auto rejected_cloud = std::make_shared<std::atomic<std::uint64_t>>(0);
  LI2Sup::ServiceBridge service_bridge(
      *node, callback_group, *slam_system, {},
      [downstream] { return downstream->localizationMap(); },
      [manual_relocalization_requested] {
        manual_relocalization_requested->store(true,
                                               std::memory_order_release);
        return adagraph::system::SystemStatus::Ok();
      });
  service_bridge.start();

  LI2Sup::SensorBridge::Config sensor_config =
      LI2Sup::SensorBridge::FromNode(*node);
  auto sensor_bridge = std::make_unique<LI2Sup::SensorBridge>(
      *node, callback_group, std::move(sensor_config),
      [slam_system, node, rejected_imu](adagraph::sensor::ImuData imu) {
        const auto status = slam_system->AddImu(std::move(imu));
        if (!status.ok()) {
          const auto count = rejected_imu->fetch_add(1) + 1;
          RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 5000,
              "SensorBridge IMU packet rejected (%llu total): %s",
              static_cast<unsigned long long>(count), status.message.c_str());
        }
      },
      [slam_system, node, rejected_cloud](
          adagraph::sensor::PointCloudFrame&& cloud) {
        const auto status = slam_system->AddPointCloud(std::move(cloud));
        if (!status.ok()) {
          const auto count = rejected_cloud->fetch_add(1) + 1;
          RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 5000,
              "SensorBridge lidar packet rejected (%llu total): %s",
              static_cast<unsigned long long>(count), status.message.c_str());
        }
      });
  sensor_bridge->setInputEnabled(false);
  sensor_bridge->start();

  slam_system->setResetHandler(
      [downstream, lio, bridge = sensor_bridge.get(),
       manual_relocalization_requested]() {
        bridge->setInputEnabled(false);
        lio->setTypedInputEnabled(false);
        manual_relocalization_requested->store(false,
                                               std::memory_order_release);
        lio->reset();
        downstream->reset();
        return adagraph::system::SystemStatus::Ok();
      });

  auto enable_sensor_bridge = [bridge = sensor_bridge.get()](bool enabled) {
    bridge->setInputEnabled(enabled);
  };
  // The system handler is installed after bridge construction so callbacks
  // cannot observe a partially assembled application graph.
  slam_system->setMappingStartHandler(
      [start_input, enable_sensor_bridge](const auto& options) {
        const auto status = start_input(options);
        if (status.ok()) enable_sensor_bridge(true);
        return status;
      });
  slam_system->setLocalizationStartHandler(
      [start_input, enable_sensor_bridge](const auto& options) {
        const auto status = start_input(options);
        if (status.ok()) enable_sensor_bridge(true);
        return status;
      });
  slam_system->setFinishTrajectoryHandler(
      [finish_input, enable_sensor_bridge]() {
        enable_sensor_bridge(false);
        return finish_input();
      });
  slam_system->setStopHandler([finish_input, enable_sensor_bridge]() {
    enable_sensor_bridge(false);
    return finish_input();
  });
  if (const auto status = slam_system->StartMapping(); !status.ok()) {
    RCLCPP_FATAL(node->get_logger(), "failed to start mapping: %s",
                 status.message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(2),
    [slam_system]() { (void)slam_system->ProcessPending(); },
    callback_group
  );

  rclcpp::spin(node);

  service_bridge.stop();
  (void)slam_system->FinishTrajectory();
  downstream->saveSuperMap();
  lio->printTimeRecord();

  rclcpp::shutdown();
  return 0;
}
