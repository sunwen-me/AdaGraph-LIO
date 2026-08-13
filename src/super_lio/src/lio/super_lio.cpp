
#include "lio/super_lio.h"

#include "basic/logs.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/common/transforms.h>


using namespace BASIC;

namespace LI2Sup{

SuperLIO::SuperLIO() : SuperLIO(Config{}) {}

SuperLIO::SuperLIO(Config config)
    : config_(std::move(config)), frontend_config_(config_.frontend) {}

SuperLIO::~SuperLIO() = default;

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
  initializeBaseState();
}

void SuperLIO::initializeBaseState(){
  kf_imu_count_ = 0;
  kf_still_rejects_ = 0;
  kf_mean_gyro_.setZero();
  kf_mean_acce_.setZero();
  kf_m2_gyro_.setZero();
  kf_m2_acce_.setZero();
  frontend_config_ = config_.frontend;
  if (const auto error = frontend_config_.validate()) {
    throw std::invalid_argument("invalid Super-LIO config: " + *error);
  }
  if (!config_.lidar_robot_yaw.allFinite() ||
      config_.input_queue_capacity == 0) {
    throw std::invalid_argument("invalid Super-LIO construction config");
  }
  typed_input_queue_capacity_ = config_.input_queue_capacity;

  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{
      static_cast<float>(frontend_config_.local_map_resolution_m),
      frontend_config_.local_map_capacity}));
  kf_.reset(new ESKF(config_.eskf));
  const auto T_imu_base = frontend_config_.T_base_imu.inverse().T_target_source;
  kf_->SetLegacyRobotExtrinsic(
      T_imu_base.rotation.toRotationMatrix().cast<scalar>(),
      T_imu_base.translation.cast<scalar>());
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.reserve(20000);
  effect_knn_idxs_.reserve(20000);
  effect_mask_.reserve(20000);
  effect_knn_mask_.reserve(20000);
  point_lengths_.reserve(20000);
  voxel_grid_fliter_.setLeafSize(
      static_cast<float>(frontend_config_.voxel_filter_size_m));

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}

void SuperLIO::reset() {
  // Disable ingress before waiting for an in-flight estimator call.  The
  // queue mutex re-check in enqueue* closes the race with callbacks that
  // observed the previous enabled value.
  typed_input_enabled_.store(false, std::memory_order_release);
  std::lock_guard<std::mutex> processing_lock(processing_mutex_);
  {
    std::lock_guard<std::mutex> input_lock(typed_input_mutex_);
    typed_imu_queue_.clear();
    typed_lidar_queue_.clear();
  }

  measures_ = MeasureGroup{};
  latest_imu_ = IMUData{};
  propagate_states_.clear();
  points_world_v3_.clear();
  points_body_v3_.clear();
  effect_mask_.clear();
  effect_knn_mask_.clear();
  point_lengths_.clear();
  effect_knn_idxs_.clear();
  backend_observation_terms_.clear();
  abcd_vec_.clear();
  effect_knn_num_ = 0;
  flg_init_ = false;
  flg_first_scan_ = true;
  frame_num_ = 0;
  sys_init_pose_ = BASIC::SE3{};
  last_pose_ = BASIC::SE3{};
  time_record_.Clear();

  // initializeBaseState() reconstructs the estimator and local voxel map,
  // while preserving config_, typed-input mode and injected callbacks.
  initializeBaseState();
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
  std::lock_guard<std::mutex> processing_lock(processing_mutex_);
  if (typed_input_mode_.load(std::memory_order_acquire)) {
    if (!takeTypedMeasure(measures_)) {
      return;
    }
  } else if (!legacy_measure_provider_ ||
             !legacy_measure_provider_(measures_)) {
    return;
  }
  (this->*state_fn_)();
}

bool SuperLIO::enqueueImu(adagraph::sensor::ImuData imu) {
  if (!imu.valid()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(typed_input_mutex_);
  // Re-check under the queue mutex. A callback may have observed the old
  // enabled value just before Reset disabled the ingress and acquired this
  // mutex to clear stale packets.
  if (!typed_input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  if (typed_imu_queue_.size() >= typed_input_queue_capacity_) {
    return false;
  }
  typed_imu_queue_.push_back(std::move(imu));
  return true;
}

bool SuperLIO::enqueuePointCloud(adagraph::sensor::PointCloudFrame&& cloud) {
  if (!cloud.valid() || cloud.points().empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(typed_input_mutex_);
  if (!typed_input_enabled_.load(std::memory_order_acquire)) {
    return false;
  }
  if (typed_lidar_queue_.size() >= typed_input_queue_capacity_) {
    return false;
  }
  typed_lidar_queue_.push_back(std::move(cloud));
  return true;
}

void SuperLIO::setTypedInputEnabled(const bool enabled) noexcept {
  typed_input_mode_.store(true, std::memory_order_release);
  typed_input_enabled_.store(enabled, std::memory_order_release);
}

std::pair<std::size_t, std::size_t> SuperLIO::discardTypedInput() {
  std::lock_guard<std::mutex> lock(typed_input_mutex_);
  const std::pair<std::size_t, std::size_t> discarded{
      typed_imu_queue_.size(), typed_lidar_queue_.size()};
  typed_imu_queue_.clear();
  typed_lidar_queue_.clear();
  return discarded;
}

void SuperLIO::drainTypedInput() {
  for (;;) {
    std::size_t imu_before = 0;
    std::size_t lidar_before = 0;
    {
      std::lock_guard<std::mutex> lock(typed_input_mutex_);
      imu_before = typed_imu_queue_.size();
      lidar_before = typed_lidar_queue_.size();
    }
    if (imu_before == 0 && lidar_before == 0) {
      return;
    }

    process();

    std::lock_guard<std::mutex> lock(typed_input_mutex_);
    if (typed_imu_queue_.size() == imu_before &&
        typed_lidar_queue_.size() == lidar_before) {
      return;
    }
  }
}

LidarData SuperLIO::toLegacyLidar(
    adagraph::sensor::PointCloudFrame&& cloud) {
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
  return legacy;
}

bool SuperLIO::takeTypedMeasure(MeasureGroup& measure) {
  std::lock_guard<std::mutex> lock(typed_input_mutex_);
  if (typed_lidar_queue_.empty() || typed_imu_queue_.empty()) {
    return false;
  }

  const auto& cloud = typed_lidar_queue_.front();
  const double end_time = cloud.endTime().seconds();
  if (typed_imu_queue_.back().timestamp.seconds() < end_time) {
    return false;
  }

  measure = MeasureGroup{};
  measure.lidar = toLegacyLidar(std::move(typed_lidar_queue_.front()));
  typed_lidar_queue_.pop_front();
  while (!typed_imu_queue_.empty() &&
         typed_imu_queue_.front().timestamp.seconds() <= end_time) {
    const auto typed_imu = std::move(typed_imu_queue_.front());
    typed_imu_queue_.pop_front();
    IMUData legacy_imu;
    legacy_imu.secs = typed_imu.timestamp.seconds();
    legacy_imu.acc = typed_imu.linear_acceleration_mps2.cast<scalar>();
    legacy_imu.gyr = typed_imu.angular_velocity_radps.cast<scalar>();
    latest_imu_ = legacy_imu;
    measure.imu.push_back(std::move(legacy_imu));
  }
  if (measure.imu.empty()) {
    return false;
  }
  return true;
}


// Stationarity gates for IMU initialization: bias and imu_scale estimated from
// a moving/vibrating window would corrupt the whole run.
constexpr double kInitMaxGyroVar = 0.01;      // summed per-axis variance, (rad/s)^2
constexpr double kInitMaxAcceVarRel = 4e-3;   // accel variance relative to |mean_acce|^2 (unit-free)
constexpr double kInitMinAcceNorm = 0.1;      // below this the accel stream carries no gravity signal
constexpr int kInitStillWarningInterval = 20;

bool SuperLIO::kf_init(){
  auto reset_stats = [&](){
    kf_imu_count_ = 0;
    kf_mean_gyro_.setZero();
    kf_mean_acce_.setZero();
    kf_m2_gyro_.setZero();
    kf_m2_acce_.setZero();
  };

  for(auto& imu: measures_.imu){
    ++kf_imu_count_;
    const V3 d_gyro = imu.gyr - kf_mean_gyro_;
    const V3 d_acce = imu.acc - kf_mean_acce_;
    kf_mean_gyro_ += d_gyro / kf_imu_count_;
    kf_mean_acce_ += d_acce / kf_imu_count_;
    kf_m2_gyro_ += d_gyro.cwiseProduct(imu.gyr - kf_mean_gyro_);
    kf_m2_acce_ += d_acce.cwiseProduct(imu.acc - kf_mean_acce_);
  }

  /// ~50 samples: 0.5 s at 100 Hz.
  if(kf_imu_count_ < 50){
    return false;
  }

  if (kf_mean_acce_.norm() < kInitMinAcceNorm) {
    LOG(ERROR) << RED << " ---> [SuperLIO]: mean accel norm " << kf_mean_acce_.norm()
               << " is near zero, cannot align gravity. Check the IMU stream." << RESET;
    reset_stats();
    return false;
  }

  const double gyro_var = kf_m2_gyro_.sum() / (kf_imu_count_ - 1);
  const double acce_var_rel =
      kf_m2_acce_.sum() / (kf_imu_count_ - 1) /
      kf_mean_acce_.squaredNorm();
  if (gyro_var > kInitMaxGyroVar || acce_var_rel > kInitMaxAcceVarRel) {
    ++kf_still_rejects_;
    if (kf_still_rejects_ == 1 ||
        kf_still_rejects_ % kInitStillWarningInterval == 0) {
      LOG(WARNING) << YELLOW << " ---> [SuperLIO]: IMU not stationary (gyro var "
                   << gyro_var << ", rel accel var " << acce_var_rel
                   << "), waiting for a still window; rejected "
                   << kf_still_rejects_ << " window(s)." << RESET;
    }
    reset_stats();
    return false;
  }
  kf_still_rejects_ = 0;

  V3 gravity = - kf_mean_acce_ * frontend_config_.gravity_mps2 /
               kf_mean_acce_.norm();
  V3 ref_gravity(0, 0, -frontend_config_.gravity_mps2);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = config_.lidar_robot_yaw * R_yaw_inv * init_rot;

  ESKF::Options options = config_.eskf;

  float imu_scale = static_cast<float>(frontend_config_.gravity_mps2 /
                                       kf_mean_acce_.norm());
  kf_->SetInitialConditions(options, kf_mean_gyro_, V3::Zero(), imu_scale,
                            ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = frontend_config_.T_base_imu.T_target_source.translation
                .cast<scalar>();
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const auto& T_imu_lidar = frontend_config_.T_imu_lidar.T_target_source;
  const SE3 transform(
      SO3(T_imu_lidar.rotation.toRotationMatrix().cast<scalar>()),
      T_imu_lidar.translation.cast<scalar>());
  const SE3 map_transform = sys_init_pose_ * transform;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = map_transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(config_.evaluate_timing){
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
  emitFrontendResult();
  Output();
  PostProcess();
}

void SuperLIO::ApplyGroundRobotConstraint(){
  if (!config_.ground_robot_planar_constraint) {
    return;
  }

  auto state = kf_->GetSysState();
  // Level the robot base frame (state.R * g_odom_robo.R_), not the sensor
  // frame, so a non-identity lidar-robot mounting rotation stays intact.
  const M3 base_R = state.R.R_ *
      frontend_config_.T_base_imu.T_target_source.rotation
          .toRotationMatrix().cast<scalar>();
  const scalar yaw = std::atan2(base_R(1, 0), base_R(0, 0));
  const M3 yaw_R =
      Eigen::AngleAxis<scalar>(yaw, V3::UnitZ()).toRotationMatrix();

  const M3 R_base_imu = frontend_config_.T_base_imu.T_target_source.rotation
                            .toRotationMatrix().cast<scalar>();
  state.R = SO3(M3(yaw_R * R_base_imu.transpose()));
  state.v.z() = 0.0;

  const V3 base_translation = frontend_config_.T_base_imu.T_target_source
                                  .translation.cast<scalar>();
  const V3 base_offset = state.R.R_ * (-R_base_imu * base_translation);
  state.p.z() = static_cast<scalar>(config_.ground_robot_base_z) -
                base_offset.z();
  kf_->SetX(state);
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

  const M3 TLI_R = frontend_config_.T_imu_lidar.T_target_source.rotation
                       .toRotationMatrix().cast<scalar>();
  const V3 TLI_t = frontend_config_.T_imu_lidar.T_target_source.translation
                       .cast<scalar>();
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


void SuperLIO::emitFrontendResult() const {
  if (!frontend_result_callback_) {
    return;
  }

  using adagraph::common::Timestamp;
  using adagraph::frontend::FrontendResult;
  using adagraph::frontend::Implementation;
  using adagraph::frontend::PointCloudView;
  using adagraph::frontend::Status;
  using adagraph::geometry::FrameId;
  using adagraph::geometry::RigidTransform3d;

  const NavState state = kf_->GetNavState();
  const auto timestamp = Timestamp::FromSeconds(state.timestamp);
  if (!timestamp) {
    return;
  }
  const auto& points = scan_undistort_full_->points;

  std::size_t x_offset = 0;
  std::size_t y_offset = sizeof(float);
  std::size_t z_offset = 2 * sizeof(float);
  if (!points.empty()) {
    const auto* origin =
        reinterpret_cast<const std::byte*>(points.data());
    x_offset = static_cast<std::size_t>(
        reinterpret_cast<const std::byte*>(&points.front().x) - origin);
    y_offset = static_cast<std::size_t>(
        reinterpret_cast<const std::byte*>(&points.front().y) - origin);
    z_offset = static_cast<std::size_t>(
        reinterpret_cast<const std::byte*>(&points.front().z) - origin);
  }

  FrontendResult result;
  result.implementation = Implementation::kSuperLio;
  result.status = Status::kTracking;
  const BASIC::SE3 legacy_pose = state.GetSE3();
  const adagraph::geometry::StampedTransform3d T_odom_imu{
      *timestamp,
      FrameId::kOdom,
      FrameId::kImu,
      RigidTransform3d{
          Eigen::Quaterniond(legacy_pose.R_.cast<double>()).normalized(),
          legacy_pose.t_.cast<double>(),
      },
  };
  const auto T_odom_base =
      T_odom_imu.compose(frontend_config_.T_base_imu.inverse());
  if (!T_odom_base) {
    return;
  }
  result.local_pose = *T_odom_base;
  // Propagation_Undistort applies T_lidar_imu before expressing the scan at
  // the scan-end state, so this borrowed cloud is in the IMU tracking frame.
  result.registered_cloud = PointCloudView::InterleavedFloatXyz(
      points.empty() ? nullptr : points.data(),
      points.size(), sizeof(PointType), x_offset, y_offset, z_offset,
      FrameId::kImu);
  result.matching.input_point_count =
      measures_.lidar.pc ? measures_.lidar.pc->size() : 0;
  result.matching.registered_point_count = points.size();
  result.matching.matching_point_count = ds_undistort_->size();
  result.matching.effective_constraint_count = effect_knn_num_;

  frontend_result_callback_(result);
}


void SuperLIO::Output(){
  if (!downstream_frame_callback_) {
    throw std::logic_error(
        "SuperLIO requires a downstream frame callback");
  }

  LegacySuperFrame frame;
  frame.state = kf_->GetNavState();
  frame.odom_pose = kf_->GetSE3();
  frame.angular_velocity = kf_->GetDynamicState().w;
  frame.full_body_cloud = std::move(scan_undistort_full_);
  frame.downsampled_body_cloud = std::move(ds_undistort_);
  frame.observation_terms = std::move(backend_observation_terms_);
  frame.effective_constraint_count = effect_knn_num_;
  frame.latest_imu = typedInputEnabled()
                         ? latest_imu_
                         : (latest_imu_provider_ ? latest_imu_provider_()
                                                 : IMUData{});

  LegacySuperBuffers reusable =
      downstream_frame_callback_(std::move(frame));
  if (!reusable.full_body_cloud || !reusable.downsampled_body_cloud) {
    throw std::runtime_error(
        "Super-LIO downstream returned invalid reusable buffers");
  }
  scan_undistort_full_ = std::move(reusable.full_body_cloud);
  ds_undistort_ = std::move(reusable.downsampled_body_cloud);
  backend_observation_terms_ = std::move(reusable.observation_terms);
}

void SuperLIO::printTimeRecord(){
  if(!config_.evaluate_timing) return;
  time_record_.PrintAll();
}


} // namespace END.
