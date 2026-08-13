#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "basic/logs.h"
#include "adagraph/system/dataset_runner.h"
#include "adagraph/system/slam_system.h"
#include "lio/point_lio_frontend.h"
#include "lio/super_lio.h"
#include "lio/system/legacy_downstream_pipeline.h"

namespace {

using LI2Sup::LegacyDownstreamCallbacks;
using LI2Sup::LegacyDownstreamPipeline;
using LI2Sup::LegacyFrontendKind;
using LI2Sup::LegacyRuntimeConfig;
using LI2Sup::PointLIOFrontend;
using LI2Sup::SuperLIO;
using adagraph::system::DatasetRunMode;
using adagraph::system::DatasetRunOptions;
using adagraph::system::DatasetRunner;
using adagraph::system::SlamSystem;
using adagraph::system::SystemStatus;
using adagraph::system::SystemStatusCode;

struct Options {
  std::string frontend;
  std::filesystem::path dataset;
  std::filesystem::path params;
  std::filesystem::path output;
  std::uint64_t maximum_packets = 0;
  bool save_map = false;
};

struct Application {
  std::shared_ptr<SlamSystem> system;
  std::shared_ptr<LegacyDownstreamPipeline> downstream;
  std::shared_ptr<SuperLIO> super;
  std::shared_ptr<PointLIOFrontend> point;
  std::shared_ptr<std::ofstream> trajectory;
};

template <typename T>
T scalar(const YAML::Node& node, const std::string& key, T fallback) {
  const auto value = node[key];
  return value ? value.as<T>() : fallback;
}

YAML::Node parameters(const YAML::Node& root) {
  if (root["/**"] && root["/**"]["ros__parameters"]) {
    return root["/**"]["ros__parameters"];
  }
  if (root["ros__parameters"]) {
    return root["ros__parameters"];
  }
  return root;
}

std::vector<double> vectorParameter(const YAML::Node& node,
                                    const std::string& key,
                                    std::vector<double> fallback) {
  const auto value = node[key];
  if (!value) {
    return fallback;
  }
  return value.as<std::vector<double>>();
}

adagraph::geometry::StaticTransform3d readStaticTransform(
    const YAML::Node& node, const std::string& key,
    adagraph::geometry::FrameId target, adagraph::geometry::FrameId source,
    std::size_t count, std::vector<double> fallback) {
  const auto values = vectorParameter(node, key, std::move(fallback));
  if (values.size() != count) {
    throw std::invalid_argument(key + " must contain " +
                                std::to_string(count) + " values");
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
      (Eigen::AngleAxisd(values[5] * kPi / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(values[4] * kPi / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(values[3] * kPi / 180.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  return {target, source,
          {Eigen::Quaterniond(rotation).normalized(),
           Eigen::Vector3d(values[0], values[1], values[2])}};
}

BASIC::SE3 toBasic(const adagraph::geometry::StaticTransform3d& transform) {
  const BASIC::M3 rotation = transform.T_target_source.rotation
                                 .toRotationMatrix()
                                 .cast<BASIC::scalar>();
  const BASIC::V3 translation =
      transform.T_target_source.translation.cast<BASIC::scalar>();
  return BASIC::SE3(rotation, translation);
}

LegacyRuntimeConfig readRuntimeConfig(const YAML::Node& node,
                                      const std::filesystem::path& root,
                                      bool force_save_map,
                                      const std::filesystem::path& output) {
  LegacyRuntimeConfig config;
  config.save_map = force_save_map || scalar<bool>(node, "lio.map.save_map", false);
  config.filter_map = scalar<bool>(node, "lio.map.if_filter", false);
  config.save_map_dir = scalar<std::string>(node, "lio.map.save_map_dir", "");
  if (config.save_map_dir.empty() || !std::filesystem::path(config.save_map_dir).is_absolute()) {
    config.save_map_dir = (force_save_map ? output / "map"
                                          : root / config.save_map_dir)
                              .lexically_normal().string();
  }
  config.map_name = scalar<std::string>(node, "lio.map.map_name", "map.pcd");
  config.map_voxel_leaf_size = scalar<double>(node, "lio.map.ds_size", 0.3);
  config.pcd_save_interval = std::max(
      1, scalar<int>(node, "lio.map.save_interval", 500));
  config.grid_update_step = std::max(
      1, scalar<int>(node, "lio.output.grid_update_step", 1));
  config.visualization_step = std::max(
      1, scalar<int>(node, "lio.output.pub_step", 1));
  config.visualize_map = scalar<bool>(node, "lio.output.map", false);
  config.visualize_dense = scalar<bool>(node, "lio.output.dense", false);
  // There is no ROS publication sink in this executable.  Keep map saving
  // and graph/map processing enabled, but avoid constructing discarded world
  // visualization clouds on every scan.
  config.visualize_map = false;
  config.visualize_dense = false;
  config.backend.keyframe_translation_m = scalar<double>(
      node, "lio.backend.keyframe_translation_m",
      config.backend.keyframe_translation_m);
  config.backend.keyframe_rotation_rad = scalar<double>(
      node, "lio.backend.keyframe_rotation_rad",
      config.backend.keyframe_rotation_rad);
  config.backend.keyframe_max_frames = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.backend.keyframe_max_frames",
                     static_cast<int>(config.backend.keyframe_max_frames))));
  config.backend.submap_max_keyframes = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.backend.submap_max_keyframes",
                     static_cast<int>(config.backend.submap_max_keyframes))));
  config.backend.submission_queue_capacity = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.backend.submission_queue_capacity",
                     static_cast<int>(config.backend.submission_queue_capacity))));
  config.backend.use_localization_factor = scalar<bool>(
      node, "lio.backend.use_localization_factor", false);
  config.backend.localization_translation_sigma_m = scalar<double>(
      node, "lio.backend.loc_factor_sigma_t", 0.1);
  config.backend.localization_rotation_sigma_rad = scalar<double>(
      node, "lio.backend.loc_factor_sigma_r", 0.02);
  config.backend.localization_fitness_reference = scalar<double>(
      node, "lio.backend.loc_factor_fitness_ref", 0.1);
  config.backend.localization_max_time_difference_s = scalar<double>(
      node, "lio.backend.loc_factor_max_time_diff", 0.15);
  config.backend_drive_map_to_odom = scalar<bool>(
      node, "lio.backend.drive_map_to_odom", false);

  auto& localization = config.prior_map.relocalization;
  localization.enabled = scalar<bool>(
      node, "lio.prior_map_localization.enable", false);
  localization.use_global_descriptor = scalar<bool>(
      node, "lio.kiss_matcher.enable", false);
  localization.map_file = scalar<std::string>(
      node, "lio.prior_map_localization.map_file", "");
  localization.local_frame_count = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.local_frames", 8)));
  localization.minimum_global_search_frames = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.min_kiss_frames", 8)));
  localization.minimum_frame_interval = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.min_frame_interval", 5)));
  localization.source_voxel_size_m = scalar<double>(
      node, "lio.prior_map_localization.local_voxel_size", 0.5);
  localization.target_voxel_size_m = scalar<double>(
      node, "lio.prior_map_localization.map_voxel_size", 0.5);
  localization.crop_radius_m = scalar<double>(
      node, "lio.prior_map_localization.crop_radius", 50.0);
  localization.crop_grid_cell_size_m = scalar<double>(
      node, "lio.prior_map_localization.crop_grid_cell_size", 10.0);
  localization.max_source_points = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.max_source_points", 20'000)));
  localization.max_local_history_points = static_cast<std::size_t>(std::max(
      1, scalar<int>(node,
                     "lio.prior_map_localization.max_local_history_points",
                     static_cast<int>(localization.max_source_points))));
  localization.max_target_points = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.max_target_points", 100'000)));
  localization.max_registration_iterations = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.max_iterations", 64)));
  localization.registration_threads = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.num_threads", 1)));
  localization.max_fitness = scalar<double>(
      node, "lio.prior_map_localization.max_fitness", 1.0);
  localization.initial_max_fitness = scalar<double>(
      node, "lio.prior_map_localization.initial_max_fitness", 2.0);
  localization.lost_fitness = scalar<double>(
      node, "lio.prior_map_localization.lost_fitness", 2.0);
  localization.max_translation_m = scalar<double>(
      node, "lio.prior_map_localization.max_translation", 5.0);
  localization.max_rotation_deg = scalar<double>(
      node, "lio.prior_map_localization.max_rotation_deg", 30.0);
  localization.translation_smoothing_alpha = scalar<double>(
      node, "lio.prior_map_localization.smooth_translation_alpha", 1.0);
  localization.rotation_smoothing_alpha = scalar<double>(
      node, "lio.prior_map_localization.smooth_rotation_alpha", 1.0);
  localization.global_retry_cooldown_s = scalar<double>(
      node, "lio.prior_map_localization.kiss_retry_cooldown_sec", 5.0);
  localization.global_retry_max_backoff = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.kiss_retry_max_backoff", 6)));
  localization.max_delta_rejections = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.prior_map_localization.max_delta_rejections", 10)));
  auto& kiss = config.prior_map.kiss_matcher;
  kiss.enabled = scalar<bool>(node, "lio.kiss_matcher.enable", false);
  kiss.voxel_size = static_cast<float>(scalar<double>(
      node, "lio.kiss_matcher.voxel_size", 0.5));
  kiss.use_quatro = scalar<bool>(node, "lio.kiss_matcher.use_quatro", false);
  kiss.use_voxel_sampling = scalar<bool>(
      node, "lio.kiss_matcher.use_voxel_sampling", true);
  kiss.use_ratio_test = scalar<bool>(
      node, "lio.kiss_matcher.use_ratio_test", true);
  kiss.adaptive_ratio_test = scalar<bool>(
      node, "lio.kiss_matcher.adaptive_ratio_test", false);
  kiss.deterministic_voxel_sampling = scalar<bool>(
      node, "lio.kiss_matcher.deterministic_voxel_sampling", false);
  kiss.min_final_inliers = scalar<int>(
      node, "lio.kiss_matcher.min_final_inliers", 5);
  kiss.max_source_points = scalar<int>(
      node, "lio.kiss_matcher.max_source_points", 30'000);
  kiss.max_target_points = scalar<int>(
      node, "lio.kiss_matcher.max_target_points", 120'000);
  config.prior_map.root_dir = root.string() + "/";
  config.prior_map.fallback_map_file =
      (std::filesystem::path(config.save_map_dir) / config.map_name).string();
  config.prior_map.use_backend_localization_factor =
      config.backend.use_localization_factor;
  return config;
}

SuperLIO::Config readSuperConfig(const YAML::Node& node) {
  SuperLIO::Config config;
  const double blind = scalar<double>(node, "lio.sensor.blind", 0.0);
  const double max_range = scalar<double>(node, "lio.sensor.maxrange", 100.0);
  config.frontend.min_range_m = std::max(0.0, blind);
  config.frontend.max_range_m = std::max(config.frontend.min_range_m + 1e-6,
                                         max_range);
  config.frontend.point_filter_stride = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.sensor.filter_rate", 1)));
  config.frontend.voxel_filter_size_m = scalar<double>(
      node, "lio.sensor.voxel_fliter_size", 0.2);
  config.frontend.local_map_resolution_m = scalar<double>(
      node, "lio.hash_map.vox_resolution", 0.5);
  config.frontend.local_map_capacity = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.hash_map.hash_capacity", 1'000'000)));
  config.frontend.gravity_mps2 = scalar<double>(
      node, "lio.sensor.gravity_norm", 9.81);
  config.frontend.T_imu_lidar = readStaticTransform(
      node, "lio.extrinsic.lidar_imu", adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar, 12,
      {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  config.frontend.T_base_imu = readStaticTransform(
      node, "lio.extrinsic.odom_robo", adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu, 6, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  config.eskf.gyro_var_ = scalar<double>(node, "lio.sensor.imu_ng", 0.0);
  config.eskf.acce_var_ = scalar<double>(node, "lio.sensor.imu_na", 0.0);
  config.eskf.bias_gyro_var_ = scalar<double>(
      node, "lio.sensor.imu_nbg", 0.0);
  config.eskf.bias_acce_var_ = scalar<double>(
      node, "lio.sensor.imu_nba", 0.0);
  config.eskf.num_iterations_ = scalar<int>(
      node, "lio.kf.kf_max_iterations", 4);
  config.eskf.quit_eps_ = scalar<double>(
      node, "lio.kf.kf_quit_eps", 1e-6);
  config.eskf.gravity_norm_ = config.frontend.gravity_mps2;
  const auto extrinsic = vectorParameter(
      node, "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  constexpr double kPi = 3.14159265358979323846;
  config.lidar_robot_yaw =
      Eigen::AngleAxisd(extrinsic[5] * kPi / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix().cast<BASIC::scalar>();
  config.ground_robot_planar_constraint = scalar<bool>(
      node, "lio.ground_robot.planar_constraint", false);
  config.ground_robot_base_z = scalar<double>(
      node, "lio.ground_robot.base_z", 0.0);
  config.evaluate_timing = scalar<bool>(node, "lio.eva.timer", false);
  config.input_queue_capacity = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "lio.input.queue_capacity", 4096)));
  return config;
}

PointLIOFrontend::Config readPointConfig(const YAML::Node& node) {
  PointLIOFrontend::Config config;
  auto& point_lio = config.point_lio;
  point_lio.point_filter_stride = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "point_filter_num", 1)));
  point_lio.min_range_m = scalar<double>(
      node, "min_distance", point_lio.min_range_m);
  point_lio.max_range_m = scalar<double>(
      node, "max_distance", point_lio.max_range_m);
  point_lio.space_downsample = scalar<bool>(
      node, "space_downsample", point_lio.space_downsample);
  point_lio.space_downsample_leaf_size_m = scalar<double>(
      node, "space_downsample_leaf_size",
      point_lio.space_downsample_leaf_size_m);
  const auto gravity = vectorParameter(
      node, "gravity", {0.0, 0.0, -9.81});
  if (gravity.size() != 3) {
    throw std::invalid_argument("gravity must contain three values");
  }
  point_lio.gravity_mps2 = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]);
  point_lio.check_saturation = scalar<bool>(
      node, "check_satu", point_lio.check_saturation);
  point_lio.fix_gravity_direction = scalar<bool>(
      node, "fix_gravity_direction", point_lio.fix_gravity_direction);
  point_lio.saturation_acceleration_mps2 =
      scalar<double>(node, "satu_acc", 3.0) * 0.99;
  point_lio.saturation_angular_velocity_radps =
      scalar<double>(node, "satu_gyro", 35.0) * 0.99;
  point_lio.acceleration_norm_mps2 = scalar<double>(
      node, "acc_norm", point_lio.acceleration_norm_mps2);
  point_lio.local_map_resolution_m = scalar<double>(
      node, "map_resolution", point_lio.local_map_resolution_m);
  point_lio.initial_map_size = static_cast<std::size_t>(std::max<std::int64_t>(
      1, scalar<std::int64_t>(node, "init_map_size",
                              static_cast<std::int64_t>(point_lio.initial_map_size))));
  point_lio.estimate_extrinsic = scalar<bool>(
      node, "extrinsic_est_en", point_lio.estimate_extrinsic);
  const auto extrinsic_t = vectorParameter(node, "extrinsic_T", {0.0, 0.0, 0.0});
  const auto extrinsic_r = vectorParameter(
      node, "extrinsic_R", {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});
  if (extrinsic_t.size() != 3 || extrinsic_r.size() != 9) {
    throw std::invalid_argument("extrinsic_T/extrinsic_R have invalid lengths");
  }
  Eigen::Matrix3d extrinsic_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      extrinsic_rotation(row, column) = extrinsic_r[row * 3 + column];
    }
  }
  point_lio.T_imu_lidar = {
      adagraph::geometry::FrameId::kImu,
      adagraph::geometry::FrameId::kLidar,
      {Eigen::Quaterniond(extrinsic_rotation).normalized(),
       Eigen::Vector3d(extrinsic_t[0], extrinsic_t[1], extrinsic_t[2])},
  };
  point_lio.lidar_point_covariance = scalar<double>(
      node, "laser_point_cov", point_lio.lidar_point_covariance);
  point_lio.imu_acceleration_measurement_covariance = scalar<double>(
      node, "imu_meas_acc_cov",
      point_lio.imu_acceleration_measurement_covariance);
  point_lio.imu_angular_velocity_measurement_covariance = scalar<double>(
      node, "imu_meas_omg_cov",
      point_lio.imu_angular_velocity_measurement_covariance);
  point_lio.velocity_covariance = scalar<double>(
      node, "velocity_cov", point_lio.velocity_covariance);
  point_lio.acceleration_covariance = scalar<double>(
      node, "acceleration_cov", point_lio.acceleration_covariance);
  point_lio.angular_velocity_covariance = scalar<double>(
      node, "omg_cov", point_lio.angular_velocity_covariance);
  point_lio.accelerometer_bias_covariance = scalar<double>(
      node, "ba_cov", point_lio.accelerometer_bias_covariance);
  point_lio.gyroscope_bias_covariance = scalar<double>(
      node, "bg_cov", point_lio.gyroscope_bias_covariance);
  point_lio.plane_threshold_m = scalar<double>(
      node, "plane_threshold", point_lio.plane_threshold_m);
  point_lio.matching_distance_squared_m2 = scalar<double>(
      node, "match_sqaured", point_lio.matching_distance_squared_m2);
  point_lio.publish_odometry_without_downsample = scalar<bool>(
      node, "publish_odometry_without_downsample",
      point_lio.publish_odometry_without_downsample);
  if (const auto error = point_lio.validate()) {
    throw std::invalid_argument("invalid Point-LIO parameters: " + *error);
  }
  config.T_base_imu = readStaticTransform(
      node, "lio.extrinsic.odom_robo", adagraph::geometry::FrameId::kBase,
      adagraph::geometry::FrameId::kImu, 6, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  config.input_queue_capacity = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "point_lio.input.queue_capacity", 4096)));
  config.downstream_queue_capacity = static_cast<std::size_t>(std::max(
      1, scalar<int>(node, "point_lio.backend.queue_capacity", 8)));
  return config;
}

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view arg(argv[index]);
    const auto value = [&](std::string_view name) -> std::string {
      if (arg == name && index + 1 < argc) {
        return argv[++index];
      }
      throw std::invalid_argument("missing value for " + std::string(name));
    };
    if (arg == "--frontend") options.frontend = value(arg);
    else if (arg == "--dataset") options.dataset = value(arg);
    else if (arg == "--params") options.params = value(arg);
    else if (arg == "--output") options.output = value(arg);
    else if (arg == "--max-packets") options.maximum_packets = std::stoull(value(arg));
    else if (arg == "--save-map") options.save_map = true;
    else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: adagraph_dataset_runner --frontend super_lio|point_lio "
                   "--dataset typed.txt --params config.yaml --output dir "
                   "[--max-packets N] [--save-map]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown argument: " + std::string(arg));
    }
  }
  if ((options.frontend != "super_lio" && options.frontend != "point_lio") ||
      options.dataset.empty() || options.params.empty() || options.output.empty()) {
    throw std::invalid_argument(
        "--frontend, --dataset, --params and --output are required");
  }
  return options;
}

Application makeApplication(const Options& options) {
  const YAML::Node root = YAML::LoadFile(options.params.string());
  const YAML::Node node = parameters(root);
  std::filesystem::create_directories(options.output);
  auto runtime = readRuntimeConfig(
      node, std::filesystem::current_path(), options.save_map, options.output);
  const auto downstream = std::make_shared<LegacyDownstreamPipeline>();
  auto system = std::make_shared<SlamSystem>();
  auto trajectory = std::make_shared<std::ofstream>(
      options.output / "estimated.tum", std::ios::out | std::ios::trunc);
  if (!*trajectory) {
    throw std::runtime_error("failed to open offline trajectory output: " +
                             (options.output / "estimated.tum").string());
  }

  Application application{system, downstream, {}, {}, trajectory};
  const auto record_result = [trajectory](
                                 const adagraph::frontend::FrontendResult& result) {
    if (!result.local_pose.valid()) {
      return;
    }
    const auto& pose = result.local_pose.T_target_source;
    const auto& translation = pose.translation;
    const Eigen::Quaterniond rotation = pose.rotation.normalized();
    *trajectory << std::setprecision(17) << result.local_pose.timestamp.seconds()
                << ' ' << translation.x() << ' ' << translation.y() << ' '
                << translation.z() << ' ' << rotation.x() << ' '
                << rotation.y() << ' ' << rotation.z() << ' ' << rotation.w()
                << '\n';
  };
  if (options.frontend == "super_lio") {
    auto frontend = std::make_shared<SuperLIO>(readSuperConfig(node));
    frontend->setFrontendResultCallback(record_result);
    frontend->setDownstreamFrameCallback(
        [downstream](LI2Sup::LegacySuperFrame&& frame) {
          return downstream->enqueueSuperFrame(std::move(frame));
        });
    frontend->init();
    frontend->setTypedInputEnabled(false);
    downstream->init(LegacyFrontendKind::kSuperLio,
                     frontend->frontendConfig(), runtime);
    application.super = frontend;
    system->setImuSink([frontend](adagraph::sensor::ImuData imu) {
      return frontend->enqueueImu(std::move(imu))
                 ? SystemStatus::Ok()
                 : SystemStatus::Error(SystemStatusCode::kRejected,
                                       "Super-LIO IMU queue rejected input");
    });
    system->setPointCloudSink(
        [frontend](adagraph::sensor::PointCloudFrame&& cloud) {
          return frontend->enqueuePointCloud(std::move(cloud))
                     ? SystemStatus::Ok()
                     : SystemStatus::Error(
                           SystemStatusCode::kRejected,
                           "Super-LIO lidar queue rejected input");
        });
    system->setProcessHandler([frontend] {
      frontend->process();
      return SystemStatus::Ok();
    });
    system->setMappingStartHandler([frontend, downstream](const auto& options) {
      downstream->setTrajectoryIdentity(options.session_id, options.trajectory_id);
      frontend->setTypedInputEnabled(true);
      return SystemStatus::Ok();
    });
    system->setFinishTrajectoryHandler([frontend, downstream] {
      frontend->setTypedInputEnabled(false);
      frontend->drainTypedInput();
      downstream->finish();
      return SystemStatus::Ok();
    });
    system->setResetHandler([frontend, downstream] {
      frontend->reset();
      downstream->reset();
      return SystemStatus::Ok();
    });
  } else {
    auto frontend = std::make_shared<PointLIOFrontend>(readPointConfig(node));
    frontend->setFrontendResultCallback(record_result);
    frontend->setOdometryCallback([](const LI2Sup::NavState&, const BASIC::V3&) {});
    frontend->setDownstreamFrameCallback(
        [downstream](LI2Sup::LegacyPointFrame&& frame,
                     const LI2Sup::IMUData& latest_imu) {
          downstream->processPointFrame(std::move(frame), latest_imu);
        });
    frontend->init();
    frontend->setInputEnabled(false);
    downstream->init(LegacyFrontendKind::kPointLio,
                     frontend->frontendConfig(), runtime);
    application.point = frontend;
    system->setImuSink([frontend](adagraph::sensor::ImuData imu) {
      return frontend->enqueueImu(std::move(imu))
                 ? SystemStatus::Ok()
                 : SystemStatus::Error(SystemStatusCode::kRejected,
                                       "Point-LIO IMU queue rejected input");
    });
    system->setPointCloudSink(
        [frontend](adagraph::sensor::PointCloudFrame&& cloud) {
          return frontend->enqueuePointCloud(std::move(cloud))
                     ? SystemStatus::Ok()
                     : SystemStatus::Error(
                           SystemStatusCode::kRejected,
                           "Point-LIO lidar queue rejected input");
        });
    system->setProcessHandler([frontend] {
      frontend->processInput();
      return SystemStatus::Ok();
    });
    system->setMappingStartHandler([frontend, downstream](const auto& options) {
      downstream->setTrajectoryIdentity(options.session_id, options.trajectory_id);
      frontend->setInputEnabled(true);
      return SystemStatus::Ok();
    });
    system->setFinishTrajectoryHandler([frontend, downstream] {
      frontend->setInputEnabled(false);
      frontend->finish();
      downstream->finish();
      return SystemStatus::Ok();
    });
    system->setResetHandler([frontend, downstream] {
      frontend->reset();
      downstream->reset();
      return SystemStatus::Ok();
    });
  }
  system->setSnapshotProvider(
      [downstream] { return downstream->mapSnapshot(); });
  system->setAlignmentProvider(
      [downstream] { return downstream->alignmentSnapshot(); });
  system->setSaveMapHandler(
      [downstream](adagraph::io::MapSaveRequest request) {
        return downstream->saveMapAsync(std::move(request));
      });
  return application;
}

void writeSummary(const std::filesystem::path& path,
                  const adagraph::system::DatasetRunResult& result,
                  const Application& application) {
  const auto snapshot = application.system->Snapshot();
  std::ofstream output(path);
  if (!output) throw std::runtime_error("failed to write summary: " + path.string());
  output << "{\n"
         << "  \"status\": " << static_cast<int>(result.status.code) << ",\n"
         << "  \"status_message\": " << std::quoted(result.status.message) << ",\n"
         << "  \"reached_end_of_input\": " << (result.reached_end_of_input ? "true" : "false") << ",\n"
         << "  \"finished_trajectory\": " << (result.finished_trajectory ? "true" : "false") << ",\n"
         << "  \"packets_seen\": " << result.packets_seen << ",\n"
         << "  \"imu_packets\": " << result.imu_packets << ",\n"
         << "  \"point_cloud_packets\": " << result.point_cloud_packets << ",\n"
         << "  \"rejected_packets\": " << result.rejected_packets << ",\n"
         << "  \"final_lifecycle_state\": " << static_cast<int>(snapshot.state) << "\n"
         << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parseOptions(argc, argv);
    BASIC::init_log();
    const Application application = makeApplication(options);
    adagraph::system::TextDatasetInputSource source(options.dataset);
    DatasetRunOptions run_options;
    run_options.mode = DatasetRunMode::kMapping;
    run_options.maximum_packets = options.maximum_packets;
    DatasetRunner runner(*application.system);
    const auto result = runner.Run(source, run_options);
    // DatasetRunner closes the frontend/system lifecycle, but map products
    // are an explicit application-level side effect.  Keep this call outside
    // the generic runner so the same lifecycle loop remains reusable for
    // compositions that do not save maps.
    if (result.status.ok() && options.save_map) {
      if (options.frontend == "super_lio") {
        application.downstream->saveSuperMap();
      } else {
        application.downstream->savePointMap();
      }
    }
    if (application.trajectory) {
      application.trajectory->flush();
    }
    writeSummary(options.output / "summary.json", result, application);
    std::cout << "packets=" << result.packets_seen
              << " clouds=" << result.point_cloud_packets
              << " status=" << static_cast<int>(result.status.code)
              << " summary=" << (options.output / "summary.json") << "\n";
    return result.status.ok() ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "adagraph_dataset_runner: " << error.what() << "\n";
    return 2;
  }
}
