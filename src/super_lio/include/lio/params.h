/**
 * @file parameters.hpp
 * @author WangLiansheng (lswang@mail.ecust.edu.cn)
 * @date 2023-03-14
 * @copyright Copyright (c) 2023
 */


#ifndef PARAMETERS_HPP_
#define PARAMETERS_HPP_


#include <atomic>
#include <string>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "basic/alias.h"
#include "basic/Manifold.h"


namespace LI2Sup{
  
  extern const std::string g_root_dir;
  extern std::atomic<bool> g_flag_run;
  extern bool g_flg_map_init;

  /// evaluation
  extern bool g_time_eva;

  extern bool g_save_map;
  extern bool g_if_filter;
  extern std::string g_map_name;
  extern std::string g_save_map_dir;
  extern float g_map_ds_size;
  extern int   g_pcd_save_interval;
  
  extern std::string g_imu_topic;
  extern std::string g_lidar_topic;

  extern int   g_lidar_type;       // 1: Livox CustomMsg, 2-7: other sensors, 8: sim XYZI, 9: Livox PointCloud2
  extern float g_blind2;
  extern float g_maxrange2;
  extern int   g_filter_rate;
  extern float g_voxel_fliter_size;

  extern BASIC::V3 g_imu_acc_scale;
  extern BASIC::V3 g_imu_gyro_scale;
  extern double g_gravity_norm;
  extern double g_imu_na;
  extern double g_imu_ng;
  extern double g_imu_nba;
  extern double g_imu_nbg;

  extern BASIC::SE3 g_lidar_imu;      // lidar in imu frame
  extern BASIC::SE3 g_odom_robo;      // lidar in robot frame
  extern BASIC::M3  g_lidar_robo_yaw; // lidar in robot frame rotation only yaw

  /// hash_map
  extern std::size_t g_ivox_capacity;
  extern float       g_ivox_resolution;
  
  /// kf
  extern int g_kf_max_iterations;
  extern double g_kf_quit_eps;

  /// output
  extern bool g_2_robot;
  extern bool g_visual_map;
  extern bool g_visual_dense;
  extern int  g_pub_step;
  extern int  g_grid_update_step;  // occupancy grid update cadence in frames (1 = every frame)

  /// Optional ground robot constraint for 2D navigation/simulation.
  extern bool g_ground_robot_planar_constraint;
  extern double g_ground_robot_base_z;

  /// for relocation
  extern bool g_update_map;
  extern double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// Nav2 compatible TF tree
  extern bool g_nav2_tf_enable;
  extern bool g_nav2_publish_map_to_odom;
  extern std::string g_nav2_map_frame;
  extern std::string g_nav2_odom_frame;
  extern std::string g_nav2_base_frame;
  extern std::string g_nav2_imu_frame;

  /// KISS-Matcher coarse global relocalization
  extern bool g_kiss_matcher_enable;
  extern float g_kiss_matcher_voxel_size;
  extern bool g_kiss_matcher_use_quatro;
  extern bool g_kiss_matcher_use_voxel_sampling;
  extern int g_kiss_matcher_min_final_inliers;
  extern int g_kiss_matcher_max_source_points;
  extern int g_kiss_matcher_max_target_points;

  /// Prior-map continuous localization inside the main Super-LIO node.
  extern bool g_prior_map_loc_enable;
  extern std::string g_prior_map_loc_map_file;
  extern float g_prior_map_loc_map_voxel_size;
  extern float g_prior_map_loc_local_voxel_size;
  extern int g_prior_map_loc_local_frames;
  extern int g_prior_map_loc_min_kiss_frames;
  extern int g_prior_map_loc_min_frame_interval;
  extern float g_prior_map_loc_crop_radius;
  extern int g_prior_map_loc_max_target_points;
  extern int g_prior_map_loc_max_source_points;
  extern int g_prior_map_loc_max_iterations;
  extern int g_prior_map_loc_num_threads;
  extern double g_prior_map_loc_max_fitness;
  extern double g_prior_map_loc_max_translation;
  extern double g_prior_map_loc_max_rotation_deg;
  extern double g_prior_map_loc_lost_fitness;
  extern double g_prior_map_loc_smooth_translation_alpha;
  extern double g_prior_map_loc_smooth_rotation_alpha;
  extern double g_prior_map_loc_kiss_retry_cooldown_sec;
  extern int g_prior_map_loc_kiss_retry_max_backoff;
  extern int g_prior_map_loc_max_delta_rejections;
  extern float g_prior_map_loc_crop_grid_cell_size;

  /// AdaGraph backend: prior-map localization results as unary factors.
  /// Experimental, not yet validated on a real robot.
  extern bool g_backend_use_localization_factor;
  extern bool g_backend_drive_map_to_odom;
  extern double g_backend_loc_factor_sigma_t;
  extern double g_backend_loc_factor_sigma_r;
  extern double g_backend_loc_factor_fitness_ref;
  extern double g_backend_loc_factor_max_time_diff;

}

#endif
