#include "ros/legacy_global_parameter_bridge.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "basic/alias.h"
#include "basic/logs.h"
#include "lio/params.h"

using namespace BASIC;

namespace LI2Sup {

void LoadLegacyParametersFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  // An absolute save_map_dir is honored as-is; a relative (or empty) one is
  // anchored at the source-tree ROOT, the legacy behavior.
  if (!std::filesystem::path(g_save_map_dir).is_absolute()) {
    g_save_map_dir = g_root_dir + g_save_map_dir;
  }

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);
  if (g_filter_rate < 1) {
    LOG(WARNING) << "lio.sensor.filter_rate must be >= 1; clamping to 1";
    g_filter_rate = 1;
  }

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  std::vector<double> imu_acc_scale;
  node.declare_parameter<std::vector<double>>(
      "lio.sensor.imu_acc_scale", std::vector<double>{1.0, 1.0, 1.0});
  node.get_parameter("lio.sensor.imu_acc_scale", imu_acc_scale);
  if (imu_acc_scale.size() == 3) {
    g_imu_acc_scale = V3(imu_acc_scale[0], imu_acc_scale[1], imu_acc_scale[2]);
  } else {
    LOG(WARNING) << "lio.sensor.imu_acc_scale must contain exactly 3 values; using [1, 1, 1]";
    g_imu_acc_scale = V3::Ones();
  }

  std::vector<double> imu_gyro_scale;
  node.declare_parameter<std::vector<double>>(
      "lio.sensor.imu_gyro_scale", std::vector<double>{1.0, 1.0, 1.0});
  node.get_parameter("lio.sensor.imu_gyro_scale", imu_gyro_scale);
  if (imu_gyro_scale.size() == 3) {
    g_imu_gyro_scale = V3(imu_gyro_scale[0], imu_gyro_scale[1], imu_gyro_scale[2]);
  } else {
    LOG(WARNING) << "lio.sensor.imu_gyro_scale must contain exactly 3 values; using [1, 1, 1]";
    g_imu_gyro_scale = V3::Ones();
  }

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);
  if (extrinsic_lidar_imu.size() != 12) {
    LOG(ERROR) << "lio.extrinsic.lidar_imu must contain exactly 12 values "
                  "(t + row-major R); using identity";
    extrinsic_lidar_imu.assign(12, 0.0);
    extrinsic_lidar_imu[3] = extrinsic_lidar_imu[7] = extrinsic_lidar_imu[11] = 1.0;
  }

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  M3 __R;
  // yaml lists the 3x3 rotation row by row.
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      __R(r, c) = static_cast<scalar>(extrinsic_lidar_imu[3 + 3 * r + c]);
    }
  }
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);
  if (extrinsic_odom_robo.size() != 6) {
    LOG(ERROR) << "lio.extrinsic.odom_robo must contain exactly 6 values "
                  "(t + rpy deg); using zeros";
    extrinsic_odom_robo.assign(6, 0.0);
  }

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.ground_robot.planar_constraint", false);
  node.get_parameter(
      "lio.ground_robot.planar_constraint", g_ground_robot_planar_constraint);

  node.declare_parameter<double>("lio.ground_robot.base_z", 0.0);
  node.get_parameter("lio.ground_robot.base_z", g_ground_robot_base_z);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  node.declare_parameter<int>("lio.output.grid_update_step", 1);
  node.get_parameter("lio.output.grid_update_step", g_grid_update_step);
  if (g_grid_update_step < 1) {
    LOG(WARNING) << "lio.output.grid_update_step must be >= 1; clamping to 1";
    g_grid_update_step = 1;
  }

  // ================= Nav2 TF =================
  node.declare_parameter<bool>("lio.nav2_tf.enable", false);
  node.get_parameter("lio.nav2_tf.enable", g_nav2_tf_enable);

  node.declare_parameter<bool>("lio.nav2_tf.publish_map_to_odom", false);
  node.get_parameter("lio.nav2_tf.publish_map_to_odom", g_nav2_publish_map_to_odom);

  node.declare_parameter<std::string>("lio.nav2_tf.map_frame", "map");
  node.get_parameter("lio.nav2_tf.map_frame", g_nav2_map_frame);

  node.declare_parameter<std::string>("lio.nav2_tf.odom_frame", "odom");
  node.get_parameter("lio.nav2_tf.odom_frame", g_nav2_odom_frame);

  node.declare_parameter<std::string>("lio.nav2_tf.base_frame", "base_link");
  node.get_parameter("lio.nav2_tf.base_frame", g_nav2_base_frame);

  node.declare_parameter<std::string>("lio.nav2_tf.imu_frame", "imu");
  node.get_parameter("lio.nav2_tf.imu_frame", g_nav2_imu_frame);

  // ================= KISS-Matcher =================
  node.declare_parameter<bool>("lio.kiss_matcher.enable", false);
  node.get_parameter("lio.kiss_matcher.enable", g_kiss_matcher_enable);

  node.declare_parameter<double>("lio.kiss_matcher.voxel_size", 0.5);
  double kiss_voxel_size = g_kiss_matcher_voxel_size;
  node.get_parameter("lio.kiss_matcher.voxel_size", kiss_voxel_size);
  g_kiss_matcher_voxel_size = static_cast<float>(kiss_voxel_size);

  node.declare_parameter<bool>("lio.kiss_matcher.use_quatro", false);
  node.get_parameter("lio.kiss_matcher.use_quatro", g_kiss_matcher_use_quatro);

  node.declare_parameter<bool>("lio.kiss_matcher.use_voxel_sampling", true);
  node.get_parameter(
      "lio.kiss_matcher.use_voxel_sampling", g_kiss_matcher_use_voxel_sampling);

  node.declare_parameter<bool>("lio.kiss_matcher.use_ratio_test", true);
  node.get_parameter(
      "lio.kiss_matcher.use_ratio_test", g_kiss_matcher_use_ratio_test);

  node.declare_parameter<bool>("lio.kiss_matcher.adaptive_ratio_test", false);
  node.get_parameter(
      "lio.kiss_matcher.adaptive_ratio_test",
      g_kiss_matcher_adaptive_ratio_test);

  node.declare_parameter<bool>(
      "lio.kiss_matcher.deterministic_voxel_sampling", false);
  node.get_parameter(
      "lio.kiss_matcher.deterministic_voxel_sampling",
      g_kiss_matcher_deterministic_voxel_sampling);

  node.declare_parameter<int>("lio.kiss_matcher.min_final_inliers", 5);
  node.get_parameter(
      "lio.kiss_matcher.min_final_inliers", g_kiss_matcher_min_final_inliers);

  node.declare_parameter<int>("lio.kiss_matcher.max_source_points", 30000);
  node.get_parameter(
      "lio.kiss_matcher.max_source_points", g_kiss_matcher_max_source_points);

  node.declare_parameter<int>("lio.kiss_matcher.max_target_points", 120000);
  node.get_parameter(
      "lio.kiss_matcher.max_target_points", g_kiss_matcher_max_target_points);

  // ================= Prior-map localization =================
  node.declare_parameter<bool>("lio.prior_map_localization.enable", false);
  node.get_parameter("lio.prior_map_localization.enable", g_prior_map_loc_enable);

  node.declare_parameter<std::string>("lio.prior_map_localization.map_file", "");
  node.get_parameter("lio.prior_map_localization.map_file", g_prior_map_loc_map_file);

  double prior_map_voxel_size = g_prior_map_loc_map_voxel_size;
  node.declare_parameter<double>(
      "lio.prior_map_localization.map_voxel_size", prior_map_voxel_size);
  node.get_parameter(
      "lio.prior_map_localization.map_voxel_size", prior_map_voxel_size);
  g_prior_map_loc_map_voxel_size = static_cast<float>(prior_map_voxel_size);

  double prior_local_voxel_size = g_prior_map_loc_local_voxel_size;
  node.declare_parameter<double>(
      "lio.prior_map_localization.local_voxel_size", prior_local_voxel_size);
  node.get_parameter(
      "lio.prior_map_localization.local_voxel_size", prior_local_voxel_size);
  g_prior_map_loc_local_voxel_size = static_cast<float>(prior_local_voxel_size);

  node.declare_parameter<int>(
      "lio.prior_map_localization.local_frames", g_prior_map_loc_local_frames);
  node.get_parameter(
      "lio.prior_map_localization.local_frames", g_prior_map_loc_local_frames);

  node.declare_parameter<int>(
      "lio.prior_map_localization.min_kiss_frames",
      g_prior_map_loc_min_kiss_frames);
  node.get_parameter(
      "lio.prior_map_localization.min_kiss_frames",
      g_prior_map_loc_min_kiss_frames);

  node.declare_parameter<int>(
      "lio.prior_map_localization.min_frame_interval",
      g_prior_map_loc_min_frame_interval);
  node.get_parameter(
      "lio.prior_map_localization.min_frame_interval",
      g_prior_map_loc_min_frame_interval);

  double prior_crop_radius = g_prior_map_loc_crop_radius;
  node.declare_parameter<double>(
      "lio.prior_map_localization.crop_radius", prior_crop_radius);
  node.get_parameter(
      "lio.prior_map_localization.crop_radius", prior_crop_radius);
  g_prior_map_loc_crop_radius = static_cast<float>(prior_crop_radius);

  node.declare_parameter<int>(
      "lio.prior_map_localization.max_target_points",
      g_prior_map_loc_max_target_points);
  node.get_parameter(
      "lio.prior_map_localization.max_target_points",
      g_prior_map_loc_max_target_points);

  node.declare_parameter<int>(
      "lio.prior_map_localization.max_source_points",
      g_prior_map_loc_max_source_points);
  node.get_parameter(
      "lio.prior_map_localization.max_source_points",
      g_prior_map_loc_max_source_points);

  node.declare_parameter<int>(
      "lio.prior_map_localization.max_iterations",
      g_prior_map_loc_max_iterations);
  node.get_parameter(
      "lio.prior_map_localization.max_iterations",
      g_prior_map_loc_max_iterations);

  node.declare_parameter<int>(
      "lio.prior_map_localization.num_threads",
      g_prior_map_loc_num_threads);
  node.get_parameter(
      "lio.prior_map_localization.num_threads",
      g_prior_map_loc_num_threads);
  g_prior_map_loc_num_threads = std::max(1, g_prior_map_loc_num_threads);

  node.declare_parameter<double>(
      "lio.prior_map_localization.max_fitness", g_prior_map_loc_max_fitness);
  node.get_parameter(
      "lio.prior_map_localization.max_fitness", g_prior_map_loc_max_fitness);

  node.declare_parameter<double>(
      "lio.prior_map_localization.initial_max_fitness",
      g_prior_map_loc_initial_max_fitness);
  node.get_parameter(
      "lio.prior_map_localization.initial_max_fitness",
      g_prior_map_loc_initial_max_fitness);

  node.declare_parameter<double>(
      "lio.prior_map_localization.max_translation",
      g_prior_map_loc_max_translation);
  node.get_parameter(
      "lio.prior_map_localization.max_translation",
      g_prior_map_loc_max_translation);

  node.declare_parameter<double>(
      "lio.prior_map_localization.max_rotation_deg",
      g_prior_map_loc_max_rotation_deg);
  node.get_parameter(
      "lio.prior_map_localization.max_rotation_deg",
      g_prior_map_loc_max_rotation_deg);

  node.declare_parameter<double>(
      "lio.prior_map_localization.lost_fitness", g_prior_map_loc_lost_fitness);
  node.get_parameter(
      "lio.prior_map_localization.lost_fitness", g_prior_map_loc_lost_fitness);

  node.declare_parameter<double>(
      "lio.prior_map_localization.smooth_translation_alpha",
      g_prior_map_loc_smooth_translation_alpha);
  node.get_parameter(
      "lio.prior_map_localization.smooth_translation_alpha",
      g_prior_map_loc_smooth_translation_alpha);

  node.declare_parameter<double>(
      "lio.prior_map_localization.smooth_rotation_alpha",
      g_prior_map_loc_smooth_rotation_alpha);
  node.get_parameter(
      "lio.prior_map_localization.smooth_rotation_alpha",
      g_prior_map_loc_smooth_rotation_alpha);

  node.declare_parameter<double>(
      "lio.prior_map_localization.kiss_retry_cooldown_sec",
      g_prior_map_loc_kiss_retry_cooldown_sec);
  node.get_parameter(
      "lio.prior_map_localization.kiss_retry_cooldown_sec",
      g_prior_map_loc_kiss_retry_cooldown_sec);

  node.declare_parameter<int>(
      "lio.prior_map_localization.kiss_retry_max_backoff",
      g_prior_map_loc_kiss_retry_max_backoff);
  node.get_parameter(
      "lio.prior_map_localization.kiss_retry_max_backoff",
      g_prior_map_loc_kiss_retry_max_backoff);

  node.declare_parameter<int>(
      "lio.prior_map_localization.max_delta_rejections",
      g_prior_map_loc_max_delta_rejections);
  node.get_parameter(
      "lio.prior_map_localization.max_delta_rejections",
      g_prior_map_loc_max_delta_rejections);

  double prior_crop_grid_cell_size = g_prior_map_loc_crop_grid_cell_size;
  node.declare_parameter<double>(
      "lio.prior_map_localization.crop_grid_cell_size",
      prior_crop_grid_cell_size);
  node.get_parameter(
      "lio.prior_map_localization.crop_grid_cell_size",
      prior_crop_grid_cell_size);
  g_prior_map_loc_crop_grid_cell_size =
      static_cast<float>(prior_crop_grid_cell_size);

  // ================= AdaGraph backend =================
  node.declare_parameter<bool>(
      "lio.backend.use_localization_factor", g_backend_use_localization_factor);
  node.get_parameter(
      "lio.backend.use_localization_factor", g_backend_use_localization_factor);

  node.declare_parameter<bool>(
      "lio.backend.drive_map_to_odom", g_backend_drive_map_to_odom);
  node.get_parameter(
      "lio.backend.drive_map_to_odom", g_backend_drive_map_to_odom);

  node.declare_parameter<double>(
      "lio.backend.loc_factor_sigma_t", g_backend_loc_factor_sigma_t);
  node.get_parameter(
      "lio.backend.loc_factor_sigma_t", g_backend_loc_factor_sigma_t);

  node.declare_parameter<double>(
      "lio.backend.loc_factor_sigma_r", g_backend_loc_factor_sigma_r);
  node.get_parameter(
      "lio.backend.loc_factor_sigma_r", g_backend_loc_factor_sigma_r);

  node.declare_parameter<double>(
      "lio.backend.loc_factor_fitness_ref", g_backend_loc_factor_fitness_ref);
  node.get_parameter(
      "lio.backend.loc_factor_fitness_ref", g_backend_loc_factor_fitness_ref);

  node.declare_parameter<double>(
      "lio.backend.loc_factor_max_time_diff",
      g_backend_loc_factor_max_time_diff);
  node.get_parameter(
      "lio.backend.loc_factor_max_time_diff",
      g_backend_loc_factor_max_time_diff);

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);
  if (init_pose.size() != 6) {
    LOG(ERROR) << "lio.relocation.init_pose must contain exactly 6 values "
                  "(xyz + rpy); using zeros";
    init_pose.assign(6, 0.0);
  }

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}

void LoadParamFromRos(rclcpp::Node& node) {
  LoadLegacyParametersFromRos(node);
}


}  // namespace LI2Sup
