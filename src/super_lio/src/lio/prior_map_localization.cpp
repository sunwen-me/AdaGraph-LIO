#include "lio/prior_map_localization.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

#include "basic/logs.h"
#include "lio/params.h"

using namespace BASIC;

namespace LI2Sup {
namespace {

Eigen::Matrix4f toMatrix4f(const SE3& pose)
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  mat.block<3, 3>(0, 0) = pose.R_.cast<float>();
  mat.block<3, 1>(0, 3) = pose.t_.cast<float>();
  return mat;
}

SE3 fromMatrix4f(const Eigen::Matrix4f& mat)
{
  const M3 R = mat.block<3, 3>(0, 0).cast<scalar>();
  const V3 t = mat.block<3, 1>(0, 3).cast<scalar>();
  return SE3(R, t);
}

}  // namespace

PriorMapLocalization::PriorMapLocalization() = default;

PriorMapLocalization::~PriorMapLocalization()
{
  shutdown();
}

bool PriorMapLocalization::init(MapToOdomCallback map_to_odom_callback)
{
  enabled_ = g_prior_map_loc_enable;
  if (!enabled_) {
    return false;
  }

  map_to_odom_callback_ = std::move(map_to_odom_callback);
  if (!map_to_odom_callback_) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: disabled, missing map->odom callback."
                 << RESET;
    enabled_ = false;
    return false;
  }

  if (!loadPriorMap()) {
    enabled_ = false;
    return false;
  }

  stop_worker_ = false;
  worker_ = std::thread(&PriorMapLocalization::workerLoop, this);
  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: initialized in Super-LIO node."
            << RESET;
  return true;
}

void PriorMapLocalization::shutdown()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_worker_ = true;
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void PriorMapLocalization::submit(double timestamp,
                                  const CloudPtr& registered_cloud,
                                  const SE3& odom_lidar)
{
  if (!enabled_ || !registered_cloud || registered_cloud->empty()) {
    return;
  }

  const int interval = std::max(1, g_prior_map_loc_min_frame_interval);
  if ((submitted_frames_++ % static_cast<std::size_t>(interval)) != 0) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_worker_ || worker_busy_ || has_pending_frame_) {
      return;
    }
  }

  CloudPtr cloud_snapshot(new PointCloudType());
  *cloud_snapshot = *registered_cloud;

  Frame frame;
  frame.timestamp = timestamp;
  frame.cloud = std::move(cloud_snapshot);
  frame.odom_lidar = odom_lidar;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stop_worker_ || worker_busy_ || has_pending_frame_) {
      return;
    }
    pending_frame_ = std::move(frame);
    has_pending_frame_ = true;
  }
  cv_.notify_one();
}

void PriorMapLocalization::requestGlobalRelocalization()
{
  if (!enabled_) {
    return;
  }
  manual_kiss_requested_.store(true, std::memory_order_release);
  LOG(INFO) << YELLOW
            << " ---> [PriorMapLocalization]: manual KISS relocalization requested."
            << RESET;
}

std::string PriorMapLocalization::resolveMapPath() const
{
  if (!g_prior_map_loc_map_file.empty()) {
    const std::filesystem::path configured(g_prior_map_loc_map_file);
    if (configured.is_absolute()) {
      return configured.string();
    }
    return (std::filesystem::path(g_root_dir) / configured).string();
  }

  return (std::filesystem::path(g_save_map_dir) / g_map_name).string();
}

bool PriorMapLocalization::loadPriorMap()
{
  const std::string map_path = resolveMapPath();
  CloudPtr raw_map(new PointCloudType());
  if (pcl::io::loadPCDFile<PointType>(map_path, *raw_map) != 0 || raw_map->empty()) {
    LOG(ERROR) << RED
               << " ---> [PriorMapLocalization]: failed to load prior map: "
               << map_path << RESET;
    return false;
  }

  prior_map_ = downsampleCloud(raw_map, g_prior_map_loc_map_voxel_size);
  prior_map_->is_dense = false;
  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: loaded prior map: " << map_path
            << ", raw points: " << raw_map->size()
            << ", filtered points: " << prior_map_->size() << RESET;
#ifdef HAS_FAST_GICP
  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: using fast_gicp FastVGICP backend."
            << RESET;
#else
  LOG(INFO) << YELLOW
            << " ---> [PriorMapLocalization]: fast_gicp not found, using PCL GICP fallback."
            << RESET;
#endif
  return true;
}

void PriorMapLocalization::workerLoop()
{
  while (true) {
    Frame frame;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() {
        return stop_worker_ || has_pending_frame_;
      });

      if (stop_worker_ && !has_pending_frame_) {
        return;
      }

      frame = std::move(*pending_frame_);
      pending_frame_.reset();
      has_pending_frame_ = false;
      worker_busy_ = true;
    }

    process(std::move(frame));

    {
      std::lock_guard<std::mutex> lock(mutex_);
      worker_busy_ = false;
    }
  }
}

void PriorMapLocalization::process(Frame&& frame)
{
  appendLocalFrame(frame);

  CloudPtr local_submap = buildLocalSubmap();
  if (!local_submap || local_submap->empty()) {
    return;
  }

  SE3 predicted_map_odom = has_map_to_odom_ ? map_to_odom_ : SE3();
  bool used_kiss_initial_guess = false;
  bool force_jump = false;
  bool should_run_kiss = false;
  const bool manual_kiss_requested =
      manual_kiss_requested_.exchange(false, std::memory_order_acq_rel);

  if (g_kiss_matcher_enable) {
    const int min_kiss_frames = std::max(
        1, std::min(g_prior_map_loc_min_kiss_frames, g_prior_map_loc_local_frames));
    const bool has_enough_kiss_frames =
        local_frames_odom_.size() >= static_cast<std::size_t>(min_kiss_frames);

    if (!has_enough_kiss_frames &&
        (manual_kiss_requested ||
         (state_ == State::UNINITIALIZED && !startup_kiss_attempted_) ||
         (state_ == State::LOST && !lost_kiss_attempted_))) {
      if (manual_kiss_requested) {
        manual_kiss_requested_.store(true, std::memory_order_release);
      }
      LOG(WARNING) << YELLOW
                   << " ---> [PriorMapLocalization]: waiting for KISS local submap, frames: "
                   << local_frames_odom_.size() << "/" << min_kiss_frames
                   << RESET;
    } else if (manual_kiss_requested) {
      should_run_kiss = true;
      force_jump = true;
    } else if (state_ == State::UNINITIALIZED && !startup_kiss_attempted_) {
      startup_kiss_attempted_ = true;
      should_run_kiss = true;
    } else if (state_ == State::LOST && !lost_kiss_attempted_) {
      lost_kiss_attempted_ = true;
      should_run_kiss = true;
    }
  }

  if (should_run_kiss) {
    SE3 kiss_map_odom;
    if (kiss_relocalizer_.estimate(local_submap, prior_map_, kiss_map_odom)) {
      predicted_map_odom = kiss_map_odom;
      used_kiss_initial_guess = true;
      LOG(INFO) << GREEN
                << " ---> [PriorMapLocalization]: KISS-Matcher supplied coarse map->odom."
                << RESET;
    } else if (!has_map_to_odom_) {
      LOG(WARNING) << YELLOW
                   << " ---> [PriorMapLocalization]: KISS-Matcher one-shot initial pose failed; waiting for manual trigger."
                   << RESET;
      return;
    }
  } else if (!has_map_to_odom_ && state_ == State::UNINITIALIZED) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: no initial map->odom; waiting for manual trigger."
                 << RESET;
    return;
  }

  const SE3 predicted_map_lidar = predicted_map_odom * frame.odom_lidar;
  CloudPtr cropped_map = cropPriorMap(predicted_map_lidar);
  if (!cropped_map || cropped_map->empty()) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: no prior-map points in crop window."
                 << RESET;
    enterLost();
    return;
  }

  RegistrationResult result =
      runRegistration(local_submap, cropped_map, predicted_map_odom, frame.odom_lidar);

  if (!passQualityGate(result)) {
    if (result.fitness > g_prior_map_loc_lost_fitness) {
      enterLost();
    }
    return;
  }

  const bool jump = force_jump || state_ == State::UNINITIALIZED || state_ == State::LOST;
  const SE3 output_map_odom = smoothMapToOdom(result.map_odom, jump);
  map_to_odom_ = output_map_odom;
  has_map_to_odom_ = true;
  state_ = State::TRACKING;
  lost_kiss_attempted_ = false;
  map_to_odom_callback_(output_map_odom);

  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: accepted map->odom, fitness: "
            << result.fitness << (jump ? " (jump)" : " (smooth)")
            << (used_kiss_initial_guess ? " [KISS init]" : "") << RESET;
}

void PriorMapLocalization::appendLocalFrame(const Frame& frame)
{
  CloudPtr odom_cloud(new PointCloudType());
  pcl::transformPointCloud(*frame.cloud, *odom_cloud, toMatrix4f(frame.odom_lidar));
  local_frames_odom_.push_back(std::move(odom_cloud));

  const int max_frames = std::max(1, g_prior_map_loc_local_frames);
  while (local_frames_odom_.size() > static_cast<std::size_t>(max_frames)) {
    local_frames_odom_.pop_front();
  }
}

CloudPtr PriorMapLocalization::buildLocalSubmap() const
{
  CloudPtr submap(new PointCloudType());
  std::size_t total_points = 0;
  for (const auto& frame_cloud : local_frames_odom_) {
    if (frame_cloud) {
      total_points += frame_cloud->size();
    }
  }
  submap->reserve(total_points);

  for (const auto& frame_cloud : local_frames_odom_) {
    if (frame_cloud && !frame_cloud->empty()) {
      *submap += *frame_cloud;
    }
  }

  submap = downsampleCloud(submap, g_prior_map_loc_local_voxel_size);
  return limitCloudSize(submap, g_prior_map_loc_max_source_points);
}

CloudPtr PriorMapLocalization::cropPriorMap(const SE3& predicted_map_lidar) const
{
  CloudPtr cropped(new PointCloudType());
  if (!prior_map_ || prior_map_->empty()) {
    return cropped;
  }

  const double radius = std::max(1.0f, g_prior_map_loc_crop_radius);
  const double radius2 = radius * radius;
  const V3 center = predicted_map_lidar.t_;
  cropped->reserve(std::min<std::size_t>(
      prior_map_->size(), static_cast<std::size_t>(std::max(1, g_prior_map_loc_max_target_points))));

  for (const auto& point : prior_map_->points) {
    const double dx = static_cast<double>(point.x) - center.x();
    const double dy = static_cast<double>(point.y) - center.y();
    const double dz = static_cast<double>(point.z) - center.z();
    if (dx * dx + dy * dy + dz * dz <= radius2) {
      cropped->push_back(point);
    }
  }

  cropped->width = cropped->size();
  cropped->height = 1;
  cropped->is_dense = false;
  return limitCloudSize(cropped, g_prior_map_loc_max_target_points);
}

CloudPtr PriorMapLocalization::downsampleCloud(const CloudPtr& cloud, float leaf_size) const
{
  if (!cloud || cloud->empty() || leaf_size <= 0.0f) {
    return cloud;
  }

  CloudPtr filtered(new PointCloudType());
  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_filter.setInputCloud(cloud);
  voxel_filter.filter(*filtered);
  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = false;
  return filtered;
}

CloudPtr PriorMapLocalization::limitCloudSize(const CloudPtr& cloud, int max_points) const
{
  if (!cloud || max_points <= 0 ||
      cloud->size() <= static_cast<std::size_t>(max_points)) {
    return cloud;
  }

  CloudPtr limited(new PointCloudType());
  limited->reserve(static_cast<std::size_t>(max_points));
  const double step =
      static_cast<double>(cloud->size()) / static_cast<double>(max_points);
  for (int i = 0; i < max_points; ++i) {
    const std::size_t idx = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(i * step)), cloud->size() - 1);
    limited->push_back(cloud->points[idx]);
  }
  limited->width = limited->size();
  limited->height = 1;
  limited->is_dense = false;
  return limited;
}

PriorMapLocalization::RegistrationResult PriorMapLocalization::runRegistration(
    const CloudPtr& source_odom,
    const CloudPtr& target_map,
    const SE3& initial_map_odom,
    const SE3& odom_lidar) const
{
  RegistrationResult result;
  if (!source_odom || source_odom->empty() || !target_map || target_map->empty()) {
    return result;
  }

  PointCloudType aligned;
  Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();

#ifdef HAS_FAST_GICP
  fast_gicp::FastVGICP<PointType, PointType> vgicp;
  vgicp.setInputSource(source_odom);
  vgicp.setInputTarget(target_map);
  vgicp.setMaximumIterations(std::max(1, g_prior_map_loc_max_iterations));
  vgicp.setMaxCorrespondenceDistance(
      std::max(1.0f, g_prior_map_loc_local_voxel_size * 8.0f));
  vgicp.setTransformationEpsilon(1e-4);
  vgicp.setEuclideanFitnessEpsilon(1e-4);
  vgicp.setResolution(std::max(0.1f, g_prior_map_loc_local_voxel_size));
  vgicp.setNumThreads(std::max(1, g_prior_map_loc_num_threads));
  vgicp.align(aligned, toMatrix4f(initial_map_odom));
  result.converged = vgicp.hasConverged();
  result.fitness = vgicp.getFitnessScore();
  final_transformation = vgicp.getFinalTransformation();
#else
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setInputSource(source_odom);
  gicp.setInputTarget(target_map);
  gicp.setMaximumIterations(std::max(1, g_prior_map_loc_max_iterations));
  gicp.setMaxCorrespondenceDistance(
      std::max(1.0f, g_prior_map_loc_local_voxel_size * 8.0f));
  gicp.setTransformationEpsilon(1e-4);
  gicp.setEuclideanFitnessEpsilon(1e-4);
  gicp.align(aligned, toMatrix4f(initial_map_odom));
  result.converged = gicp.hasConverged();
  result.fitness = gicp.getFitnessScore();
  final_transformation = gicp.getFinalTransformation();
#endif

  if (!result.converged) {
    return result;
  }

  const SE3 candidate_map_odom = fromMatrix4f(final_transformation);
  const SE3 T_map_lidar_vgicp = candidate_map_odom * odom_lidar;
  result.map_odom = T_map_lidar_vgicp * odom_lidar.inverse();
  return result;
}

bool PriorMapLocalization::passQualityGate(const RegistrationResult& result) const
{
  if (!result.converged || !std::isfinite(result.fitness) ||
      result.fitness > g_prior_map_loc_max_fitness) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: rejected by fitness, converged: "
                 << result.converged << ", fitness: " << result.fitness << RESET;
    return false;
  }

  if (state_ != State::TRACKING || !has_map_to_odom_) {
    return true;
  }

  const SE3 delta = map_to_odom_.inverse() * result.map_odom;
  const double trans = delta.t_.norm();
  const double rot_deg = rotationAngleDeg(delta);
  if (trans > g_prior_map_loc_max_translation ||
      rot_deg > g_prior_map_loc_max_rotation_deg) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: rejected by jump gate, dt: "
                 << trans << ", drot_deg: " << rot_deg << RESET;
    return false;
  }

  return true;
}

SE3 PriorMapLocalization::smoothMapToOdom(const SE3& candidate, bool jump) const
{
  if (jump || !has_map_to_odom_) {
    return candidate;
  }

  const double trans_alpha = clamp01(g_prior_map_loc_smooth_translation_alpha);
  const double rot_alpha = clamp01(g_prior_map_loc_smooth_rotation_alpha);

  const V3 t =
      (1.0 - trans_alpha) * map_to_odom_.t_ + trans_alpha * candidate.t_;

  Quat q_old(map_to_odom_.R_);
  Quat q_new(candidate.R_);
  q_old.normalize();
  q_new.normalize();
  if (q_old.dot(q_new) < 0.0) {
    q_new.coeffs() *= -1.0;
  }
  const Quat q = q_old.slerp(rot_alpha, q_new).normalized();
  return SE3(q.toRotationMatrix(), t);
}

void PriorMapLocalization::enterLost()
{
  if (state_ != State::LOST) {
    state_ = State::LOST;
    lost_kiss_attempted_ = false;
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: state changed to LOST."
                 << RESET;
    return;
  }
  state_ = State::LOST;
}

double PriorMapLocalization::rotationAngleDeg(const SE3& delta)
{
  Eigen::AngleAxis<scalar> aa(delta.R_);
  return std::abs(static_cast<double>(aa.angle())) * 180.0 / M_PI;
}

double PriorMapLocalization::clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

}  // namespace LI2Sup
