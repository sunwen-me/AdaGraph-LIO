

#include "lio/params.h"

using namespace std;
using namespace BASIC;

namespace LI2Sup{

  const std::string g_root_dir = std::string(ROOT);
  std::atomic<bool> g_flag_run = true; 
  bool g_flg_map_init = true;

  /// evaluation
  bool g_time_eva = false;

  bool   g_save_map;
  bool   g_if_filter; 
  string g_save_map_dir;
  string g_map_name;
  float  g_map_ds_size;
  int    g_pcd_save_interval;
  
  string g_imu_topic;
  string g_lidar_topic;

  int    g_lidar_type;
  float  g_blind2;
  float  g_maxrange2;
  int    g_filter_rate;
  float  g_voxel_fliter_size;

  V3     g_imu_acc_scale = V3::Ones();
  V3     g_imu_gyro_scale = V3::Ones();
  double g_gravity_norm = 9.7946;
  double g_imu_na;
  double g_imu_ng;
  double g_imu_nba;
  double g_imu_nbg;

  SE3 g_lidar_imu;
  SE3 g_odom_robo;
  M3  g_lidar_robo_yaw;

  /// hash_map
  std::size_t g_ivox_capacity = 100000;
  float       g_ivox_resolution = 0.5;

  /// kf
  int g_kf_max_iterations = 4;
  double g_kf_quit_eps;

  /// output
  bool g_2_robot    = false;
  bool g_visual_map = true;
  bool g_visual_dense = false;
  int  g_pub_step;
  int  g_grid_update_step = 1;

  bool g_ground_robot_planar_constraint = false;
  double g_ground_robot_base_z = 0.0;

  /// for relocation
  bool g_update_map = false;
  double g_init_px, g_init_py, g_init_pz, g_init_roll, g_init_pitch, g_init_yaw;

  /// Nav2 compatible TF tree
  bool g_nav2_tf_enable = false;
  bool g_nav2_publish_map_to_odom = false;
  string g_nav2_map_frame = "map";
  string g_nav2_odom_frame = "odom";
  string g_nav2_base_frame = "base_link";
  string g_nav2_imu_frame = "imu";

  /// KISS-Matcher coarse global relocalization
  bool g_kiss_matcher_enable = false;
  float g_kiss_matcher_voxel_size = 0.5f;
  bool g_kiss_matcher_use_quatro = false;
  bool g_kiss_matcher_use_voxel_sampling = true;
  bool g_kiss_matcher_use_ratio_test = true;
  bool g_kiss_matcher_adaptive_ratio_test = false;
  bool g_kiss_matcher_deterministic_voxel_sampling = false;
  int g_kiss_matcher_min_final_inliers = 5;
  int g_kiss_matcher_max_source_points = 30000;
  int g_kiss_matcher_max_target_points = 120000;

  /// Prior-map continuous localization inside the main Super-LIO node.
  bool g_prior_map_loc_enable = false;
  string g_prior_map_loc_map_file;
  float g_prior_map_loc_map_voxel_size = 0.5f;
  float g_prior_map_loc_local_voxel_size = 0.5f;
  int g_prior_map_loc_local_frames = 8;
  int g_prior_map_loc_min_kiss_frames = 8;
  int g_prior_map_loc_min_frame_interval = 5;
  float g_prior_map_loc_crop_radius = 40.0f;
  int g_prior_map_loc_max_target_points = 120000;
  int g_prior_map_loc_max_source_points = 50000;
  int g_prior_map_loc_max_iterations = 30;
  int g_prior_map_loc_num_threads = 4;
  double g_prior_map_loc_max_fitness = 1.0;
  double g_prior_map_loc_initial_max_fitness = 0.1;
  double g_prior_map_loc_max_translation = 2.0;
  double g_prior_map_loc_max_rotation_deg = 20.0;
  double g_prior_map_loc_lost_fitness = 2.0;
  double g_prior_map_loc_smooth_translation_alpha = 0.1;
  double g_prior_map_loc_smooth_rotation_alpha = 0.1;
  double g_prior_map_loc_kiss_retry_cooldown_sec = 5.0;
  int g_prior_map_loc_kiss_retry_max_backoff = 6;
  int g_prior_map_loc_max_delta_rejections = 10;
  float g_prior_map_loc_crop_grid_cell_size = 10.0f;

  /// AdaGraph backend: prior-map localization results as unary factors.
  bool g_backend_use_localization_factor = false;
  bool g_backend_drive_map_to_odom = false;
  double g_backend_loc_factor_sigma_t = 0.1;
  double g_backend_loc_factor_sigma_r = 0.02;
  double g_backend_loc_factor_fitness_ref = 0.1;
  double g_backend_loc_factor_max_time_diff = 0.15;

}
