
#include "lio/super_lio.h"
#include "lio/adagraph_backend.h"
#include "lio/adagraph_feature_extractor.h"

#include <algorithm>
#include <cmath>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>


using namespace BASIC;

namespace LI2Sup{

SuperLIO::SuperLIO() = default;

SuperLIO::~SuperLIO() {
  if (prior_map_localization_) {
    prior_map_localization_->shutdown();
  }
}

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.reserve(20000);
  effect_knn_idxs_.reserve(20000);
  effect_mask_.reserve(20000);
  effect_knn_mask_.reserve(20000);
  point_lengths_.reserve(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  adagraph_backend_ = std::make_unique<AdaGraphBackend>();
  adagraph_backend_->init();

  prior_map_localization_ = std::make_unique<PriorMapLocalization>();
  prior_map_localization_->init(
      [wrapper = data_wrapper_](const SE3& map_to_odom) {
        if (wrapper) {
          wrapper->set_map_to_odom(map_to_odom);
        }
      });

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
}


// Stationarity gates for IMU initialization: bias and imu_scale estimated from
// a moving/vibrating window would corrupt the whole run.
constexpr double kInitMaxGyroVar = 0.01;      // summed per-axis variance, (rad/s)^2
constexpr double kInitMaxAcceVarRel = 4e-3;   // accel variance relative to |mean_acce|^2 (unit-free)
constexpr double kInitMinAcceNorm = 0.1;      // below this the accel stream carries no gravity signal
// A platform that never passes the gates (engine idle vibration, hand-held
// start) must still initialize eventually: after this many rejected windows
// (~10 s at 100 Hz) accept the window anyway with degraded bias/scale.
constexpr int kInitMaxStillRejects = 20;

bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();
  static V3 m2_gyro = V3::Zero();
  static V3 m2_acce = V3::Zero();

  auto reset_stats = [&](){
    imu_cout = 0;
    mean_gyro.setZero();
    mean_acce.setZero();
    m2_gyro.setZero();
    m2_acce.setZero();
  };

  for(auto& imu: measures_.imu){
    imu_cout ++;
    const V3 d_gyro = imu.gyr - mean_gyro;
    const V3 d_acce = imu.acc - mean_acce;
    mean_gyro += d_gyro / imu_cout;
    mean_acce += d_acce / imu_cout;
    m2_gyro += d_gyro.cwiseProduct(imu.gyr - mean_gyro);
    m2_acce += d_acce.cwiseProduct(imu.acc - mean_acce);
  }

  /// ~50 samples: 0.5 s at 100 Hz.
  if(imu_cout < 50){
    return false;
  }

  if (mean_acce.norm() < kInitMinAcceNorm) {
    LOG(ERROR) << RED << " ---> [SuperLIO]: mean accel norm " << mean_acce.norm()
               << " is near zero, cannot align gravity. Check the IMU stream." << RESET;
    reset_stats();
    return false;
  }

  const double gyro_var = m2_gyro.sum() / (imu_cout - 1);
  const double acce_var_rel =
      m2_acce.sum() / (imu_cout - 1) / mean_acce.squaredNorm();
  static int still_rejects = 0;
  if (gyro_var > kInitMaxGyroVar || acce_var_rel > kInitMaxAcceVarRel) {
    still_rejects++;
    if (still_rejects < kInitMaxStillRejects) {
      LOG(WARNING) << YELLOW << " ---> [SuperLIO]: IMU not stationary (gyro var "
                   << gyro_var << ", rel accel var " << acce_var_rel
                   << "), waiting for a still window (" << still_rejects << "/"
                   << kInitMaxStillRejects << ")." << RESET;
      reset_stats();
      return false;
    }
    LOG(WARNING) << YELLOW << " ---> [SuperLIO]: no still window after "
                 << still_rejects
                 << " attempts, initializing from a moving window; gyro bias and"
                    " imu_scale estimates may be degraded." << RESET;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    ApplyGroundRobotConstraint();
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    ApplyGroundRobotConstraint();
    UpdateMap();
  }
  Output();
  caceData();
}

void SuperLIO::ApplyGroundRobotConstraint(){
  if (!g_ground_robot_planar_constraint) {
    return;
  }

  auto state = kf_->GetSysState();
  // Level the robot base frame (state.R * g_odom_robo.R_), not the sensor
  // frame, so a non-identity lidar-robot mounting rotation stays intact.
  const M3 base_R = state.R.R_ * g_odom_robo.R_;
  const scalar yaw = std::atan2(base_R(1, 0), base_R(0, 0));
  const M3 yaw_R =
      Eigen::AngleAxis<scalar>(yaw, V3::UnitZ()).toRotationMatrix();

  state.R = SO3(M3(yaw_R * g_odom_robo.R_.transpose()));
  state.v.z() = 0.0;

  const V3 base_offset = state.R.R_ * (-g_odom_robo.R_ * g_odom_robo.t_);
  state.p.z() = static_cast<scalar>(g_ground_robot_base_z) - base_offset.z();
  kf_->SetX(state);
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    namespace fs = std::filesystem;
    // lexically_normal keeps a trailing separator ("/x/maps/" stays as-is),
    // which would fail the parent check below; strip the empty final element.
    fs::path save_root = fs::path(g_save_map_dir).lexically_normal();
    if (!save_root.has_filename()) {
      save_root = save_root.parent_path();
    }
    const fs::path pcd_dir = (save_root / "PCD").lexically_normal();
    // Only ever remove the "PCD" directory directly under the configured
    // save_map root; refuse empty or relative roots (a mis-set save_map_dir
    // must not delete an unrelated tree).
    if (g_save_map_dir.empty() || !save_root.is_absolute() ||
        pcd_dir.parent_path() != save_root) {
      LOG(ERROR) << RED << " ---> Invalid save_map_dir '" << g_save_map_dir
                 << "', refusing to reset the PCD cache directory." << RESET;
    } else {
      std::error_code ec;
      fs::remove_all(pcd_dir, ec);
      if (ec) {
        LOG(ERROR) << RED << " ---> Failed to remove " << pcd_dir << ": "
                   << ec.message() << RESET;
      }
      fs::create_directories(pcd_dir, ec);
      if (ec) {
        LOG(ERROR) << RED << " ---> Failed to create " << pcd_dir << ": "
                   << ec.message() << RESET;
      }
    }
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  if (!fs::exists(pcd_folder)) {
    LOG(ERROR) << RED << " ---> PCD fragment folder does not exist: " << pcd_folder << RESET;
    return;
  }

  PointCloudType::Ptr merged_map(new PointCloudType());

  // Downsample each fragment before merging so peak memory stays bounded on
  // long runs; the final voxel pass below restores a consistent leaf size.
  pcl::VoxelGrid<PointType> fragment_filter;
  fragment_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        if (g_if_filter) {
          PointCloudType ds_cloud;
          fragment_filter.setInputCloud(tmp_cloud);
          fragment_filter.filter(ds_cloud);
          *merged_map += ds_cloud;
        } else {
          *merged_map += *tmp_cloud;
        }
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string()
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.empty()) {
    LOG(ERROR) << RED << " ---> Merged map is empty, nothing to save." << RESET;
    return;
  }
  filtered_map.width = filtered_map.size();
  filtered_map.height = 1;
  filtered_map.is_dense = false;

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    // Only record states from samples that actually advanced the filter, so
    // the interpolation below never sees duplicate timestamps.
    if (kf_->Predict(imu)) {
      propagate_states_.emplace_back(kf_->GetDynamicState());
    }
  }
  if (kf_->PredictToObsTime()) {
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;
  const bool has_intervals = propagate_states_.size() >= 2;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize);

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;
      if (!has_intervals || query_time > propagate_states_.back().time) {
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = TLI_R * raw + TLI_t;
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = propagate_states_.begin();
      for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double tau = query_time - match_iter->time;
      double s = dt > 0.0 ? std::clamp(tau / dt, 0.0, 1.0) : 0.0;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 p_i = p_h + v_h * tau + 0.5 * acc_t * tau * tau;
      V3 t_ei = p_i - T_end_t;
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};

// Lidar point-to-plane information weight = 1/sigma^2, sigma ~= 0.032 m.
constexpr double kLidarPointWeight = 1000.0;


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  backend_observation_terms_.clear();

  points_body_v3_.resize(ptsize);
  point_lengths_.resize(ptsize);
  abcd_vec_.resize(ptsize);
  effect_mask_.resize(ptsize);
  effect_knn_mask_.resize(ptsize);
  effect_knn_idxs_.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.end(), 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    point_lengths_[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, point_lengths_[idx], error);
          if(!effect_mask_[idx]) continue;

          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;

            local_acc.HTVH += J * kLidarPointWeight * J.transpose();
            local_acc.HTVr -= J * kLidarPointWeight * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();
    if (HTVH.norm() > 0 || HTVr.norm() > 0) {
      backend_observation_terms_.clear();
      backend_observation_terms_.emplace_back(HTVH, HTVr);
    }

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  frame_num_++;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  
  if (adagraph_backend_ && adagraph_backend_->initialized()) {
    BASIC::SE3 current_pose = kf_->GetSE3();
    
    if (adagraph_backend_->isKeyframe(current_pose)) {
      FeatureFrame frame = AdaGraphFeatureExtractor::build({
          current_pose,
          adagraph_backend_->lastKeyframePose(),
          backend_observation_terms_,
          ds_undistort_,
          effect_knn_num_,
          data_wrapper_->getLatestImuData()});

      std::vector<MapObservation> map_observations;
      SE3 map_to_odom;
      bool has_map_to_odom = false;
      if (g_backend_use_localization_factor && prior_map_localization_ &&
          prior_map_localization_->enabled()) {
        prior_map_localization_->takeMapObservations(map_observations);
        has_map_to_odom = data_wrapper_->get_map_to_odom(map_to_odom);
      }

      // The result must not be written back into the ESKF: UpdateMap() has
      // already committed this scan to the odom-frame voxel map at the
      // pre-optimization pose, so feeding an optimized pose back would desync
      // the filter from the map. Backend corrections exist only in the map
      // layer (optimized path, optional map->odom refinement below).
      BackendResult backend_result =
          adagraph_backend_->processKeyframe(
              frame, current_pose, backend_observation_terms_, state.timestamp,
              map_observations, has_map_to_odom ? &map_to_odom : nullptr);

      if (g_backend_use_localization_factor &&
          !backend_result.optimized_trajectory.empty()) {
        data_wrapper_->pub_backend_optimized_path(
            backend_result.optimized_trajectory);
      }

      if (g_backend_drive_map_to_odom && backend_result.optimized &&
          prior_map_localization_ && prior_map_localization_->enabled()) {
        // T_map_odom = T_map_keyframe * T_odom_keyframe^-1, fed through the
        // localization module's existing jump gate and smoothing (no second
        // TF publish path).
        const SE3 derived_map_to_odom =
            backend_result.optimized_pose * current_pose.inverse();
        prior_map_localization_->submitBackendMapToOdom(derived_map_to_odom,
                                                        current_pose);
      }
    } else {
      adagraph_backend_->noteNonKeyframe();
    }
  }
  
  // Publish odometry (original or optimized)
  data_wrapper_->pub_odom(state);

  if (prior_map_localization_ && prior_map_localization_->enabled()) {
    if (data_wrapper_->consume_manual_relocalization_request()) {
      prior_map_localization_->requestGlobalRelocalization();
    }
    const SE3 odom_lidar(state.R.R_, state.p);
    prior_map_localization_->submit(state.timestamp, ds_undistort_, odom_lidar);
  }

  // The occupancy grid feeds navigation: keep its cadence and input cloud
  // independent of the visualization throttling below.
  static int grid_count = -1;
  const int grid_step = std::max(1, g_grid_update_step);
  grid_count++;
  if(grid_count % grid_step == 0){
    grid_count = 0;
    // With dense output configured, feed the full undistorted scan: at fine
    // grid resolutions the voxel-downsampled cloud leaves gaps between hits
    // that raycast decay would erode into observed surfaces.
    const CloudPtr& grid_pc =
        g_visual_dense ? scan_undistort_full_ : ds_undistort_;
    data_wrapper_->update_occupancy_grid_from_body_cloud(grid_pc, state);
  }

  static int count = -1;
  const int pub_step = std::max(1, g_pub_step);
  count++;
  if(count % pub_step != 0){
    return;
  }
  count = 0;

  if(g_visual_map){
    const CloudPtr& body_pc =
        g_visual_dense ? scan_undistort_full_ : ds_undistort_;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
    transformation.block<3, 1>(0, 3) = state.p.cast<float>();
    CloudPtr world_pc(new PointCloudType());
    pcl::transformPointCloud(*body_pc, *world_pc, transformation);
    data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}


} // namespace END.
