#include <memory>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "hybrid_slam/backend/loop_closure_scheduler.h"
#include "hybrid_slam/logging/logging.h"
#include "hybrid_slam/system/slam_system.h"
#include "lio/point_lio_frontend.h"
#include "lio/point_lio_recovery_coordinator.h"
#include "lio/loop_closure_plugin.h"
#include "lio/pcl_loop_closure.h"
#include "lio/pcl_submap_relocalization_scheduler.h"
#include "lio/system/point_lio_pipeline.h"
#include "ros/parameter_bridge.h"
#include "ros/publication_bridge.h"
#include "ros/sensor_bridge.h"
#include "ros/service_bridge.h"

namespace {

using LI2Sup::LID_TYPE;
using LI2Sup::PointLIOFrontend;
using LI2Sup::SensorBridge;

enum class StartupMode : std::uint8_t {
  kMapping,
  kLocalization,
};

hybrid_slam::geometry::StaticTransform3d ReadBaseImuTransform(
    rclcpp::Node& node);

template <typename T>
T ReadParameter(rclcpp::Node& node, const char* name, T default_value) {
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  T value = std::move(default_value);
  node.get_parameter(name, value);
  return value;
}

StartupMode ReadStartupMode(rclcpp::Node& node) {
  const auto value = ReadParameter<std::string>(
      node, "lio.lifecycle.mode", "mapping");
  if (value == "mapping") {
    return StartupMode::kMapping;
  }
  if (value == "localization") {
    return StartupMode::kLocalization;
  }
  throw std::invalid_argument(
      "lio.lifecycle.mode must be 'mapping' or 'localization', got '" +
      value + "'");
}

PointLIOFrontend::Config ReadPointLioConfig(rclcpp::Node& node) {
  PointLIOFrontend::Config config;
  auto& frontend = config.frontend;
  auto& point_lio = config.point_lio;
  frontend.point_filter_stride = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "point_filter_num", 1)));
  frontend.min_range_m = ReadParameter<double>(node, "min_distance", 0.5);
  frontend.max_range_m = ReadParameter<double>(node, "max_distance", 1000.0);
  point_lio.space_downsample = ReadParameter<bool>(
      node, "space_downsample", point_lio.space_downsample);
  frontend.voxel_filter_size_m = ReadParameter<double>(
      node, "space_downsample_leaf_size", frontend.voxel_filter_size_m);

  const auto gravity = ReadParameter<std::vector<double>>(
      node, "gravity", std::vector<double>{0.0, 0.0, -9.81});
  if (gravity.size() != 3) {
    throw std::invalid_argument("gravity must contain exactly three values");
  }
  frontend.gravity_mps2 = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  point_lio.check_saturation =
      ReadParameter<bool>(node, "check_satu", point_lio.check_saturation);
  point_lio.fix_gravity_direction = ReadParameter<bool>(
      node, "fix_gravity_direction", point_lio.fix_gravity_direction);
  point_lio.saturation_acceleration_mps2 =
      ReadParameter<double>(node, "satu_acc", 3.0) * 0.99;
  point_lio.saturation_angular_velocity_radps =
      ReadParameter<double>(node, "satu_gyro", 35.0) * 0.99;
  point_lio.acceleration_norm_mps2 =
      ReadParameter<double>(node, "acc_norm", point_lio.acceleration_norm_mps2);

  frontend.local_map_resolution_m =
      ReadParameter<double>(node, "map_resolution", 0.5);
  point_lio.initial_map_size = static_cast<std::size_t>(std::max<std::int64_t>(
      1, ReadParameter<std::int64_t>(
             node, "init_map_size",
             static_cast<std::int64_t>(point_lio.initial_map_size))));

  point_lio.estimate_extrinsic = ReadParameter<bool>(
      node, "extrinsic_est_en", point_lio.estimate_extrinsic);
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
  frontend.T_imu_lidar = {
      hybrid_slam::geometry::FrameId::kImu,
      hybrid_slam::geometry::FrameId::kLidar,
      {Eigen::Quaterniond(extrinsic_rotation).normalized(),
       Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2])},
  };

  point_lio.lidar_point_covariance = ReadParameter<double>(
      node, "laser_point_cov", point_lio.lidar_point_covariance);
  point_lio.imu_acceleration_measurement_covariance = ReadParameter<double>(
      node, "imu_meas_acc_cov", point_lio.imu_acceleration_measurement_covariance);
  point_lio.imu_angular_velocity_measurement_covariance = ReadParameter<double>(
      node, "imu_meas_omg_cov", point_lio.imu_angular_velocity_measurement_covariance);
  point_lio.velocity_covariance = ReadParameter<double>(
      node, "velocity_cov", point_lio.velocity_covariance);
  point_lio.acceleration_covariance = ReadParameter<double>(
      node, "acceleration_cov", point_lio.acceleration_covariance);
  point_lio.angular_velocity_covariance =
      ReadParameter<double>(node, "omg_cov", point_lio.angular_velocity_covariance);
  point_lio.accelerometer_bias_covariance =
      ReadParameter<double>(node, "ba_cov", point_lio.accelerometer_bias_covariance);
  point_lio.gyroscope_bias_covariance =
      ReadParameter<double>(node, "bg_cov", point_lio.gyroscope_bias_covariance);
  point_lio.plane_threshold_m = ReadParameter<double>(
      node, "plane_threshold", point_lio.plane_threshold_m);
  point_lio.matching_distance_squared_m2 = ReadParameter<double>(
      node, "match_sqaured", point_lio.matching_distance_squared_m2);
  point_lio.publish_odometry_without_downsample = ReadParameter<bool>(
      node, "publish_odometry_without_downsample",
      point_lio.publish_odometry_without_downsample);
  frontend.T_base_imu = ReadBaseImuTransform(node);
  if (const auto error = frontend.validate()) {
    throw std::invalid_argument("invalid frontend parameters: " + *error);
  }
  if (const auto error = point_lio.validate()) {
    throw std::invalid_argument("invalid Point-LIO parameters: " + *error);
  }
  config.input_queue_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "point_lio.input.queue_capacity", 4096)));
  config.downstream_queue_capacity = static_cast<std::size_t>(std::max(
      1, ReadParameter<int>(node, "point_lio.backend.queue_capacity", 8)));
  return config;
}

hybrid_slam::geometry::StaticTransform3d ReadBaseImuTransform(
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
      hybrid_slam::geometry::FrameId::kBase,
      hybrid_slam::geometry::FrameId::kImu,
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
  return ReadPointLioConfig(node);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  hybrid_slam::logging::InitializeConsole();

  auto node = std::make_shared<rclcpp::Node>("hybrid_slam_point_lio");
  auto callback_group = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  DeclarePointLioTransportParameters(*node);
  const auto runtime_config =
      LI2Sup::ReadPointLIORuntimeConfigFromRos(*node);
  auto publication = std::make_shared<LI2Sup::PublicationBridge>(
      *node, callback_group,
      LI2Sup::PublicationBridge::Config::FromNode(*node));
  publication->setup();
  auto frontend = std::make_shared<PointLIOFrontend>(MakeFrontendConfig(*node));
  using SubmapRecoveryScheduler =
      LI2Sup::PclSubmapRelocalizationScheduler;
  auto submap_recovery_scheduler_holder =
      std::make_shared<std::shared_ptr<SubmapRecoveryScheduler>>();
  using LoopClosureScheduler = hybrid_slam::backend::LoopClosureScheduler;
  auto loop_closure_scheduler_holder =
      std::make_shared<std::shared_ptr<LoopClosureScheduler>>();
  struct FinishedSubmapIdentity {
    std::uint64_t session = 0;
    std::uint64_t trajectory = 0;
    std::uint64_t submap = 0;

    bool operator==(const FinishedSubmapIdentity&) const = default;
  };
  auto last_loop_finished_submap =
      std::make_shared<std::optional<FinishedSubmapIdentity>>();
  auto last_loop_finished_submap_mutex = std::make_shared<std::mutex>();
  const bool loop_closure_enabled = runtime_config.loop_closure_enabled;
  auto recovery_in_progress = std::make_shared<std::atomic<bool>>(false);
  auto next_recovery_trajectory =
      std::make_shared<std::atomic<std::uint64_t>>(2);
  auto manual_relocalization_requested =
      std::make_shared<std::atomic<bool>>(false);
  LI2Sup::PointLIOPipelineCallbacks downstream_callbacks;
  downstream_callbacks.publication.publish_backend_snapshot =
      [publication, loop_closure_scheduler_holder, last_loop_finished_submap,
       loop_closure_enabled, last_loop_finished_submap_mutex](
          std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> snapshot) {
        publication->setBackendMapSnapshot(snapshot);
        if (!loop_closure_enabled || !snapshot || snapshot->keyframes.empty()) {
          return;
        }

        // A finished submap is the loop-closure rate boundary. Its final
        // keyframe receives any verified factor; the descriptor and VGICP
        // target are built from immutable submap geometry.
        const auto finished = std::find_if(
            snapshot->submaps.rbegin(), snapshot->submaps.rend(),
            [](const auto& submap) {
              return submap.state ==
                         hybrid_slam::mapping::SubmapState::kFinished &&
                     !submap.keyframe_ids.empty();
            });
        if (finished == snapshot->submaps.rend()) {
          return;
        }
        const FinishedSubmapIdentity identity{
            finished->session_id.value(), finished->trajectory_id.value(),
            finished->id.value()};
        {
          std::lock_guard<std::mutex> lock(*last_loop_finished_submap_mutex);
          if (last_loop_finished_submap->has_value() &&
              last_loop_finished_submap->value() == identity) {
            return;
          }
        }

        const auto query_keyframe_id = finished->keyframe_ids.back();
        const auto query = std::find_if(
            snapshot->keyframes.begin(), snapshot->keyframes.end(),
            [finished, query_keyframe_id](const auto& keyframe) {
              return keyframe.keyframe_id == query_keyframe_id &&
                     keyframe.submap_id == finished->id &&
                     keyframe.session_id == finished->session_id &&
                     keyframe.trajectory_id == finished->trajectory_id;
            });
        if (query == snapshot->keyframes.end() || query->node_id.value() == 0) {
          return;
        }
        const auto& scheduler = *loop_closure_scheduler_holder;
        if (!scheduler) {
          return;
        }
        std::lock_guard<std::mutex> lock(*last_loop_finished_submap_mutex);
        if (last_loop_finished_submap->has_value() &&
            last_loop_finished_submap->value() == identity) {
          return;
        }
        if (scheduler->submit(snapshot, query->node_id) ==
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted) {
          *last_loop_finished_submap = identity;
        }
      };
  downstream_callbacks.publication.publish_global_map =
      [publication](const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
        publication->setGlobalMap(cloud);
      };
  downstream_callbacks.alignment.set_prior_map_localization_active =
      [publication](bool active) {
        publication->setPriorMapLocalizationActive(active);
      };
  downstream_callbacks.publication.get_transform_publication_stats = [publication] {
    const auto stats = publication->transformPublicationStats();
    return LI2Sup::PointLIOPipelineCallbacks::TransformPublicationStats{
        stats.odom_base_published,
        stats.map_odom_published,
        stats.nonmonotonic_odom_base_stamps,
        stats.nonmonotonic_map_odom_stamps,
    };
  };
  downstream_callbacks.frontend.set_frontend_local_map_write_enabled =
      [frontend](bool enabled) {
        frontend->setLocalMapWriteEnabled(enabled);
      };
  downstream_callbacks.frontend.set_frontend_recovery_query_mode =
      [frontend](bool enabled) {
        frontend->setRecoveryQueryMode(enabled);
      };
  downstream_callbacks.frontend.clear_frontend_online_local_map =
      [frontend, node] {
        frontend->setInputEnabled(false);
        (void)frontend->discardQueuedInput();
        try {
          frontend->clearOnlineLocalMap();
        } catch (const std::exception& error) {
          RCLCPP_ERROR(
              node->get_logger(),
              "Failed to clear provisional Point-LIO online map: %s",
              error.what());
        } catch (...) {
          RCLCPP_ERROR(
              node->get_logger(),
              "Failed to clear provisional Point-LIO online map");
        }
        frontend->setInputEnabled(true);
      };
  downstream_callbacks.recovery.request_submap_recovery =
      [submap_recovery_scheduler_holder](
          const hybrid_slam::localization::RelocalizationQuery& query,
          std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map) {
        const auto& scheduler = *submap_recovery_scheduler_holder;
        if (scheduler) {
          (void)scheduler->submit(query, std::move(map));
        }
      };
  downstream_callbacks.session.reset_session_state =
      [publication] { publication->resetSession(); };
  downstream_callbacks.alignment.set_map_to_odom =
      [publication](const hybrid_slam::geometry::RigidTransform3d& transform) {
        publication->setMapToOdom(transform);
      };
  downstream_callbacks.alignment.get_map_to_odom =
      [publication](hybrid_slam::geometry::RigidTransform3d& transform) {
        return publication->getMapToOdom(transform);
      };
  downstream_callbacks.recovery.consume_manual_relocalization_request =
      [manual_relocalization_requested] {
        return manual_relocalization_requested->exchange(
            false, std::memory_order_acq_rel);
      };
  downstream_callbacks.publication.occupancy_grid_enabled =
      [publication] { return publication->occupancyGridEnabled(); };
  downstream_callbacks.publication.update_occupancy_grid =
      [publication](const hybrid_slam::sensor::ImmutablePointCloud& cloud,
                    const LI2Sup::NavState& state) {
        publication->updateOccupancyGridFromBodyCloud(cloud, state);
      };
  downstream_callbacks.publication.publish_odom =
      [publication](const LI2Sup::NavState& state,
                    const Eigen::Vector3d& angular_velocity) {
        publication->publishOdom(state, angular_velocity);
      };
  downstream_callbacks.publication.publish_cloud_world =
      [publication](const hybrid_slam::sensor::ImmutablePointCloud& cloud,
                    double timestamp) {
        publication->publishCloudWorld(cloud, timestamp);
      };
  auto downstream = std::make_shared<LI2Sup::PointLIOPipeline>(
      std::move(downstream_callbacks));

  try {
    const auto startup_mode = ReadStartupMode(*node);
    frontend->setDownstreamFrameCallback(
        [downstream](LI2Sup::PointLIOOutputFrame&& frame,
                     const LI2Sup::IMUData& latest_imu) {
          downstream->processPointFrame(std::move(frame), latest_imu);
        });
    frontend->setOdometryCallback(
      [publication](const LI2Sup::NavState& state,
                  const Eigen::Vector3d& angular_velocity) {
          publication->publishOdom(state, angular_velocity);
        });
    frontend->init();
    downstream->init(frontend->frontendConfig(), runtime_config);
    frontend->setInputEnabled(false);

    auto system = std::make_shared<hybrid_slam::system::SlamSystem>();
    auto rejected_imu = std::make_shared<std::atomic<std::uint64_t>>(0);
    auto rejected_cloud = std::make_shared<std::atomic<std::uint64_t>>(0);
    system->setImuSink([frontend](hybrid_slam::sensor::ImuData imu) {
      return frontend->enqueueImu(std::move(imu))
                 ? hybrid_slam::system::SystemStatus::Ok()
                 : hybrid_slam::system::SystemStatus::Error(
                       hybrid_slam::system::SystemStatusCode::kRejected,
                       "Point-LIO typed IMU queue is full or disabled");
    });
    system->setPointCloudSink(
        [frontend](hybrid_slam::sensor::PointCloudFrame&& cloud) {
          return frontend->enqueuePointCloud(std::move(cloud))
                     ? hybrid_slam::system::SystemStatus::Ok()
                     : hybrid_slam::system::SystemStatus::Error(
                           hybrid_slam::system::SystemStatusCode::kRejected,
                           "Point-LIO typed lidar queue is full or disabled");
        });
    system->setProcessHandler([frontend]() {
      frontend->processInput();
      return hybrid_slam::system::SystemStatus::Ok();
    });
    system->setSnapshotProvider(
        [downstream] { return downstream->mapSnapshot(); });
    system->setAlignmentProvider(
        [downstream] { return downstream->alignmentSnapshot(); });
    system->setSaveMapHandler(
        [downstream](hybrid_slam::io::MapSaveRequest request) {
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
    // Small Point-LIO owns range and sampling policy. Keep ROS transport
    // lossless so those values are applied exactly once by the frontend.
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
        [system, node, rejected_imu](hybrid_slam::sensor::ImuData imu) {
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
            hybrid_slam::sensor::PointCloudFrame&& cloud) {
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

    if (runtime_config.submap_recovery_enabled) {
      auto recovery_config = runtime_config.submap_recovery;
      recovery_config.kiss.enabled = true;
      auto scheduler = std::make_shared<SubmapRecoveryScheduler>(
          std::make_unique<LI2Sup::PclSubmapRelocalizer>(
              std::move(recovery_config)),
          [frontend, downstream, bridge = sensor_bridge.get(), node,
           recovery_in_progress, next_recovery_trajectory](
              std::uint64_t,
              const std::optional<
                  hybrid_slam::localization::SubmapRelocalizationResult>&
                  result,
              const hybrid_slam::localization::RelocalizationQuery& query,
              std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map) {
            if (recovery_in_progress->exchange(true,
                                               std::memory_order_acq_rel)) {
              return;
            }
            if (!result || !result->valid() || !map) {
              RCLCPP_WARN(node->get_logger(),
                          "Submap recovery candidate rejected or unavailable; "
                          "returning to LOST for a newer query");
              downstream->markRecoveryAttemptFailed();
              recovery_in_progress->store(false, std::memory_order_release);
              return;
            }

            // Retrieval runs while the old estimator is still alive. Freeze
            // both ingress sources only after a candidate has passed all
            // geometry gates, immediately before the reset/seed transaction.
            bridge->setInputEnabled(false);
            frontend->setInputEnabled(false);
            frontend->setRecoveryQueryMode(false);

            LI2Sup::PointLIORecoveryRequest request;
            request.relocalization = *result;
            request.new_trajectory_id = hybrid_slam::common::TrajectoryId(
                next_recovery_trajectory->fetch_add(
                    1, std::memory_order_acq_rel));
            RCLCPP_INFO(
                node->get_logger(),
                "Submap recovery candidate: target_session=%llu, "
                "target_trajectory=%llu, target_submap=%llu, "
                "new_trajectory=%llu, query_timestamp=%.9f",
                static_cast<unsigned long long>(result->session_id.value()),
                static_cast<unsigned long long>(result->trajectory_id.value()),
                static_cast<unsigned long long>(result->submap_id.value()),
                static_cast<unsigned long long>(request.new_trajectory_id.value()),
                query.T_odom_base.timestamp.seconds());
            request.frontend_state.timestamp_seconds =
                query.T_odom_base.timestamp.seconds();
            request.frontend_state.T_odom_base =
                query.T_odom_base.T_target_source;
            request.frontend_state.gravity_odom =
                frontend->frontendConfig().gravity_mps2;
            // A LiDAR dropout with a healthy IMU must restart from the
            // matched pose/seed immediately. Setting this unconditionally to
            // true would make Small Point-LIO wait for a fresh 200-IMU/initial
            // map bootstrap and fail the verification window. A future IMU or
            // timestamp fault policy may opt into full reinitialization.
            request.frontend_state.reinitialize_imu = false;

            const bool recovered =
                LI2Sup::PointLIORecoveryCoordinator::Recover(
                    request, *map, *frontend, *downstream);
            if (!recovered) {
              RCLCPP_WARN(node->get_logger(),
                          "Point-LIO submap recovery transaction failed; "
                          "returning to LOST for a newer query");
              downstream->markRecoveryAttemptFailed();
              frontend->setInputEnabled(true);
              bridge->setInputEnabled(true);
            } else {
              RCLCPP_INFO(node->get_logger(),
                          "Point-LIO submap recovery committed; entering "
                          "verification window");
              // Recovery owns only the reset/seed critical section. Resume
              // both ingress paths so the frontend can run its verification
              // window with the read-only prior layer active.
              frontend->setInputEnabled(true);
              bridge->setInputEnabled(true);
            }
            recovery_in_progress->store(false, std::memory_order_release);
          });
      *submap_recovery_scheduler_holder = std::move(scheduler);
    }

    if (runtime_config.loop_closure_enabled) {
      auto loop_config = runtime_config.loop_closure;
      auto plugin = LI2Sup::CreateLoopClosurePlugin(loop_config);
      auto scheduler = std::make_shared<LoopClosureScheduler>(
          std::move(plugin.detector), std::move(plugin.verifier),
          [downstream](hybrid_slam::backend::Constraint constraint) {
            (void)downstream->submitLoopClosureConstraint(std::move(constraint));
          });
      *loop_closure_scheduler_holder = std::move(scheduler);
    }

    const auto start_input = [frontend, downstream, bridge = sensor_bridge.get()]
                              (const auto& options) {
      if (!downstream->acceptingInput()) {
        return hybrid_slam::system::SystemStatus::Error(
            hybrid_slam::system::SystemStatusCode::kClosed,
            "Point-LIO pipeline is already finished");
      }
      downstream->setTrajectoryIdentity(options.session_id, options.trajectory_id);
      frontend->setInputEnabled(true);
      bridge->setInputEnabled(true);
      return hybrid_slam::system::SystemStatus::Ok();
    };
    system->setMappingStartHandler(start_input);
    system->setLocalizationStartHandler(start_input);

    const auto finish_input = [frontend, downstream, bridge = sensor_bridge.get()] {
      bridge->setInputEnabled(false);
      frontend->setInputEnabled(false);
      frontend->finish();
      downstream->finish();
      return hybrid_slam::system::SystemStatus::Ok();
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
          return hybrid_slam::system::SystemStatus::Ok();
        });

    LI2Sup::ServiceBridge service_bridge(
        *node, callback_group, *system, {},
        [downstream] { return downstream->localizationMap(); },
        [manual_relocalization_requested] {
          manual_relocalization_requested->store(
              true, std::memory_order_release);
          return hybrid_slam::system::SystemStatus::Ok();
        });
    service_bridge.start();

    hybrid_slam::system::SystemStatus startup_status;
    if (startup_mode == StartupMode::kMapping) {
      startup_status = system->StartMapping();
    } else {
      if (!runtime_config.prior_map.relocalization.enabled) {
        throw std::invalid_argument(
            "localization startup requires "
            "lio.prior_map_localization.enable=true");
      }
      hybrid_slam::system::LocalizationOptions options;
      options.map = downstream->localizationMap();
      options.allow_deferred_map = false;
      if (!options.map || !options.map->valid()) {
        throw std::invalid_argument(
            "localization startup requires a valid "
            "lio.prior_map_localization.map_file");
      }
      startup_status = system->StartLocalization(std::move(options));
    }
    if (!startup_status.ok()) {
      const char* mode = startup_mode == StartupMode::kMapping
                             ? "mapping"
                             : "localization";
      RCLCPP_FATAL(node->get_logger(), "failed to start %s: %s", mode,
                   startup_status.message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Point-LIO lifecycle started: %s",
                startup_mode == StartupMode::kMapping ? "mapping"
                                                      : "localization");

    auto process_timer = node->create_wall_timer(
        std::chrono::milliseconds(2),
        [system] { (void)system->ProcessPending(); },
        callback_group);
    auto frontend_watchdog_timer = node->create_wall_timer(
        std::chrono::milliseconds(100),
        [downstream] { downstream->observeFrontendTimeout(); },
        callback_group);
    (void)process_timer;
    (void)frontend_watchdog_timer;
    rclcpp::spin(node);

    // Commit the final submap while the backend can still accept its loop
    // constraint. The loop scheduler drains only after this snapshot exists.
    sensor_bridge->setInputEnabled(false);
    frontend->setInputEnabled(false);
    frontend->finish();
    if (!downstream->finishActiveSubmaps()) {
      RCLCPP_WARN(node->get_logger(),
                  "failed to finalize active submaps before loop shutdown");
    }
    if (*submap_recovery_scheduler_holder) {
      (*submap_recovery_scheduler_holder)->finish();
      submap_recovery_scheduler_holder->reset();
    }
    if (*loop_closure_scheduler_holder) {
      const auto scheduler = *loop_closure_scheduler_holder;
      scheduler->finishLatest();
      const auto stats = scheduler->stats();
      LOG(INFO) << " ---> [LoopClosureScheduler]: submitted/replaced/"
                   "cancelled/completed/published/callback_failures="
                << stats.submitted << "/" << stats.replaced_pending << "/"
                << stats.cancellation_requests << "/" << stats.completed
                << "/" << stats.constraints_published << "/"
                << stats.callback_failures;
      loop_closure_scheduler_holder->reset();
    }

    service_bridge.stop();
    (void)system->FinishTrajectory();
    downstream->saveMap();
    // Stop the asynchronous ROS publication worker while its node/context is
    // still alive. Its destructor otherwise runs after rclcpp::shutdown().
    publication->shutdown();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
