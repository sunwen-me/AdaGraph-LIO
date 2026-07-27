#include "lio/prior_map_localization.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
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

std::uint64_t voxelKey(int cx, int cy, int cz)
{
  /// 21 bits per axis with a +2^20 offset covers +-2^20 cells.
  const auto pack = [](int c) {
    return static_cast<std::uint64_t>(static_cast<std::int64_t>(c) + (1 << 20)) &
           0x1FFFFF;
  };
  return pack(cx) | (pack(cy) << 21) | (pack(cz) << 42);
}

std::optional<int> voxelCoordinate(double coordinate, double cell_size)
{
  constexpr int kMinCell = -(1 << 20);
  constexpr int kMaxCell = (1 << 20) - 1;
  const double cell = std::floor(coordinate / cell_size);
  if (!std::isfinite(cell) ||
      cell < static_cast<double>(kMinCell) ||
      cell > static_cast<double>(kMaxCell)) {
    return std::nullopt;
  }
  return static_cast<int>(cell);
}

/// Backend observation queue bound; overflow drops the oldest entry.
constexpr std::size_t kMapObservationQueueCap = 8;

}  // namespace

PriorMapLocalization::PriorMapLocalization() = default;

PriorMapLocalization::~PriorMapLocalization()
{
  shutdown();
}

bool PriorMapLocalization::init(MapToOdomCallback map_to_odom_callback,
                                PriorMapCallback prior_map_callback)
{
  enabled_ = g_prior_map_loc_enable;
  if (!enabled_) {
    return false;
  }

  lost_kiss_retry_cooldown_sec_ =
      std::max(0.0, g_prior_map_loc_kiss_retry_cooldown_sec);
  lost_kiss_retry_max_backoff_ = std::max(1, g_prior_map_loc_kiss_retry_max_backoff);
  max_delta_gate_rejections_ = std::max(1, g_prior_map_loc_max_delta_rejections);
  prior_map_grid_cell_size_ = std::max(1.0f, g_prior_map_loc_crop_grid_cell_size);

  map_to_odom_callback_ = std::move(map_to_odom_callback);
  prior_map_callback_ = std::move(prior_map_callback);
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

void PriorMapLocalization::takeMapObservations(std::vector<MapObservation>& out)
{
  out.clear();
  std::lock_guard<std::mutex> lock(observation_mutex_);
  out.assign(map_observations_.begin(), map_observations_.end());
  map_observations_.clear();
}

bool PriorMapLocalization::submitBackendMapToOdom(const SE3& candidate,
                                                  const SE3& odom_lidar)
{
  if (!enabled_) {
    return false;
  }

  SE3 current_map_odom;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    if (!has_map_to_odom_) {
      /// The backend may only refine an existing map->odom; initialization
      /// stays with the KISS/VGICP pipeline.
      return false;
    }
    current_map_odom = map_to_odom_;
  }

  /// Same delta gate as passQualityGate, measured at the current robot pose.
  /// Rejections are not counted towards the LOST transition: that logic
  /// belongs to the registration pipeline on the worker thread.
  const SE3 delta =
      (current_map_odom * odom_lidar).inverse() * (candidate * odom_lidar);
  const double trans = delta.t_.norm();
  const double rot_deg = rotationAngleDeg(delta);
  if (trans > g_prior_map_loc_max_translation ||
      rot_deg > g_prior_map_loc_max_rotation_deg) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: backend map->odom rejected"
                    " by jump gate, dt: " << trans
                 << ", drot_deg: " << rot_deg << RESET;
    return false;
  }

  const SE3 smoothed = smoothMapToOdom(candidate, false);
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    map_to_odom_ = smoothed;
  }
  map_to_odom_callback_(smoothed);
  return true;
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

  CloudPtr finite_map(new PointCloudType());
  finite_map->reserve(raw_map->size());
  for (const auto& point : raw_map->points) {
    if (std::isfinite(point.x) &&
        std::isfinite(point.y) &&
        std::isfinite(point.z)) {
      finite_map->push_back(point);
    }
  }
  if (finite_map->empty()) {
    LOG(ERROR) << RED
               << " ---> [PriorMapLocalization]: prior map contains no finite points: "
               << map_path << RESET;
    return false;
  }
  finite_map->width = finite_map->size();
  finite_map->height = 1;
  finite_map->is_dense = true;
  raw_map = std::move(finite_map);

  prior_map_ = downsampleCloud(raw_map, g_prior_map_loc_map_voxel_size);
  prior_map_->is_dense = false;
  buildPriorMapGrid();
  if (prior_map_callback_) {
    // Publish/reset consumers from the original map before releasing it.
    // Registration keeps only the downsampled copy above.
    prior_map_callback_(raw_map);
  }
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

void PriorMapLocalization::buildPriorMapGrid()
{
  prior_map_grid_.clear();
  if (!prior_map_ || prior_map_->empty()) {
    return;
  }

  const double cell = std::max(1.0f, prior_map_grid_cell_size_);
  for (std::uint32_t i = 0; i < prior_map_->size(); ++i) {
    const auto& point = prior_map_->points[i];
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    const auto cx = voxelCoordinate(point.x, cell);
    const auto cy = voxelCoordinate(point.y, cell);
    const auto cz = voxelCoordinate(point.z, cell);
    if (!cx || !cy || !cz) {
      continue;
    }
    prior_map_grid_[voxelKey(*cx, *cy, *cz)].push_back(i);
  }
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

  SE3 predicted_map_odom;
  bool had_map_to_odom = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    had_map_to_odom = has_map_to_odom_;
    predicted_map_odom = has_map_to_odom_ ? map_to_odom_ : SE3();
  }
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
    const bool lost_kiss_due =
        state_ == State::LOST && lostKissRetryDue(frame.timestamp);

    if (!has_enough_kiss_frames &&
        (manual_kiss_requested ||
         (state_ == State::UNINITIALIZED && !startup_kiss_attempted_) ||
         lost_kiss_due)) {
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
    } else if (lost_kiss_due) {
      lost_kiss_attempted_ = true;
      last_lost_kiss_time_ = frame.timestamp;
      should_run_kiss = true;
      LOG(INFO) << YELLOW
                << " ---> [PriorMapLocalization]: LOST, retrying KISS relocalization"
                << " (consecutive failures: " << lost_kiss_failures_ << ")."
                << RESET;
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
    } else {
      if (state_ == State::LOST) {
        ++lost_kiss_failures_;
      }
      if (!had_map_to_odom) {
        LOG(WARNING) << YELLOW
                     << " ---> [PriorMapLocalization]: KISS-Matcher one-shot initial pose failed; waiting for manual trigger."
                     << RESET;
        return;
      }
    }
  } else if (!had_map_to_odom && state_ == State::UNINITIALIZED) {
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
      runRegistration(local_submap, cropped_map, predicted_map_odom);

  if (!passQualityGate(result, frame.odom_lidar, force_jump)) {
    if (result.fitness > g_prior_map_loc_lost_fitness) {
      enterLost();
    }
    return;
  }

  const bool jump = force_jump || state_ == State::UNINITIALIZED || state_ == State::LOST;
  const SE3 output_map_odom = smoothMapToOdom(result.map_odom, jump);
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    map_to_odom_ = output_map_odom;
    has_map_to_odom_ = true;
  }
  state_ = State::TRACKING;
  lost_kiss_attempted_ = false;
  lost_kiss_failures_ = 0;
  delta_gate_rejections_ = 0;
  has_rejected_candidate_ = false;
  map_to_odom_callback_(output_map_odom);

  if (g_backend_use_localization_factor) {
    /// Queue the raw (unsmoothed) registration for the backend: the unary
    /// factor wants the measured absolute pose, smoothing only serves TF.
    MapObservation obs;
    obs.timestamp = frame.timestamp;
    obs.map_lidar = result.map_odom * frame.odom_lidar;
    obs.fitness = result.fitness;
    std::lock_guard<std::mutex> lock(observation_mutex_);
    map_observations_.push_back(obs);
    while (map_observations_.size() > kMapObservationQueueCap) {
      map_observations_.pop_front();
      if ((++overflow_dropped_observations_ % 32) == 1) {
        LOG(WARNING) << YELLOW
                     << " ---> [PriorMapLocalization]: backend observation"
                        " queue overflow, total dropped: "
                     << overflow_dropped_observations_ << RESET;
      }
    }
  }

  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: accepted map->odom, fitness: "
            << result.fitness << (jump ? " (jump)" : " (smooth)")
            << (used_kiss_initial_guess ? " [KISS init]" : "")
            << ", stamp: " << std::fixed << std::setprecision(3)
            << frame.timestamp << RESET;
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
  if (!center.allFinite()) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: non-finite predicted pose; "
                    "skipping prior-map crop."
                 << RESET;
    return cropped;
  }
  cropped->reserve(std::min<std::size_t>(
      prior_map_->size(), static_cast<std::size_t>(std::max(1, g_prior_map_loc_max_target_points))));

  const double cell = std::max(1.0f, prior_map_grid_cell_size_);
  const auto min_cx = voxelCoordinate(center.x() - radius, cell);
  const auto max_cx = voxelCoordinate(center.x() + radius, cell);
  const auto min_cy = voxelCoordinate(center.y() - radius, cell);
  const auto max_cy = voxelCoordinate(center.y() + radius, cell);
  const auto min_cz = voxelCoordinate(center.z() - radius, cell);
  const auto max_cz = voxelCoordinate(center.z() + radius, cell);
  if (!min_cx || !max_cx || !min_cy || !max_cy || !min_cz || !max_cz) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: predicted crop exceeds "
                    "the prior-map grid coordinate range."
                 << RESET;
    return cropped;
  }

  for (int cx = *min_cx; cx <= *max_cx; ++cx) {
    for (int cy = *min_cy; cy <= *max_cy; ++cy) {
      for (int cz = *min_cz; cz <= *max_cz; ++cz) {
        const auto it = prior_map_grid_.find(voxelKey(cx, cy, cz));
        if (it == prior_map_grid_.end()) {
          continue;
        }
        for (const std::uint32_t idx : it->second) {
          const auto& point = prior_map_->points[idx];
          const double dx = static_cast<double>(point.x) - center.x();
          const double dy = static_cast<double>(point.y) - center.y();
          const double dz = static_cast<double>(point.z) - center.z();
          if (dx * dx + dy * dy + dz * dz <= radius2) {
            cropped->push_back(point);
          }
        }
      }
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
    const SE3& initial_map_odom) const
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
  /// FastVGICP ignores setMaxCorrespondenceDistance; correspondences are
  /// gated by the voxel resolution set below.
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

  /// The source cloud is already expressed in the odom frame, so the aligned
  /// transformation is directly map->odom.
  result.map_odom = fromMatrix4f(final_transformation);
  return result;
}

bool PriorMapLocalization::passQualityGate(const RegistrationResult& result,
                                           const SE3& odom_lidar,
                                           bool force_jump)
{
  if (!result.converged || !std::isfinite(result.fitness) ||
      result.fitness > g_prior_map_loc_max_fitness) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: rejected by fitness, converged: "
                 << result.converged << ", fitness: " << result.fitness << RESET;
    return false;
  }

  SE3 current_map_odom;
  bool has_current_map_odom = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    has_current_map_odom = has_map_to_odom_;
    current_map_odom = map_to_odom_;
  }

  if (state_ != State::TRACKING || !has_current_map_odom) {
    return true;
  }

  if (force_jump) {
    /// Manual/KISS-initialized attempts exist to correct arbitrarily large
    /// errors; only the fitness gate applies to them.
    return true;
  }

  /// Measure the correction at the current robot pose instead of at the odom
  /// origin, so a small heading correction far from the origin is not read as
  /// a large translation.
  const SE3 delta =
      (current_map_odom * odom_lidar).inverse() * (result.map_odom * odom_lidar);
  const double trans = delta.t_.norm();
  const double rot_deg = rotationAngleDeg(delta);
  if (trans > g_prior_map_loc_max_translation ||
      rot_deg > g_prior_map_loc_max_rotation_deg) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: rejected by jump gate, dt: "
                 << trans << ", drot_deg: " << rot_deg << RESET;
    noteDeltaGateRejection(result.map_odom, odom_lidar);
    return false;
  }

  return true;
}

void PriorMapLocalization::noteDeltaGateRejection(const SE3& candidate_map_odom,
                                                 const SE3& odom_lidar)
{
  bool consistent = false;
  if (has_rejected_candidate_) {
    const SE3 delta = (last_rejected_map_odom_ * odom_lidar).inverse() *
                      (candidate_map_odom * odom_lidar);
    consistent = delta.t_.norm() <= g_prior_map_loc_max_translation &&
                 rotationAngleDeg(delta) <= g_prior_map_loc_max_rotation_deg;
  }

  delta_gate_rejections_ = consistent ? delta_gate_rejections_ + 1 : 1;
  last_rejected_map_odom_ = candidate_map_odom;
  has_rejected_candidate_ = true;

  if (delta_gate_rejections_ >= max_delta_gate_rejections_) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: " << delta_gate_rejections_
                 << " consecutive consistent jump-gate rejections; assuming the"
                    " localization reference drifted, entering LOST to force"
                    " KISS relocalization."
                 << RESET;
    delta_gate_rejections_ = 0;
    has_rejected_candidate_ = false;
    enterLost();
  }
}

bool PriorMapLocalization::lostKissRetryDue(double now_sec) const
{
  if (!lost_kiss_attempted_) {
    return true;
  }
  /// Repeated failures stretch the cooldown instead of stopping retries.
  const int backoff =
      std::min(lost_kiss_failures_ + 1, std::max(1, lost_kiss_retry_max_backoff_));
  return now_sec - last_lost_kiss_time_ >=
         lost_kiss_retry_cooldown_sec_ * static_cast<double>(backoff);
}

SE3 PriorMapLocalization::smoothMapToOdom(const SE3& candidate, bool jump) const
{
  SE3 previous;
  bool has_previous = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    has_previous = has_map_to_odom_;
    previous = map_to_odom_;
  }

  if (jump || !has_previous) {
    return candidate;
  }

  const double trans_alpha = clamp01(g_prior_map_loc_smooth_translation_alpha);
  const double rot_alpha = clamp01(g_prior_map_loc_smooth_rotation_alpha);

  const V3 t =
      (1.0 - trans_alpha) * previous.t_ + trans_alpha * candidate.t_;

  Quat q_old(previous.R_);
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
    lost_kiss_failures_ = 0;
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: state changed to LOST."
                 << RESET;
  }
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
