#include "lio/prior_map_localization.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>
#include <unordered_set>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>

#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

#include "basic/logs.h"
#include "lio/pcl_relocalizer_adapters.h"

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

SE3 fromDomainPose(const adagraph::geometry::RigidTransform3d& pose)
{
  return SE3(
      BASIC::SO3(
          pose.rotation.toRotationMatrix().cast<BASIC::scalar>()),
      pose.translation.cast<BASIC::scalar>());
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

class FunctionRelocalizer final
    : public adagraph::localization::Relocalizer {
 public:
  using Function = std::function<adagraph::localization::RelocalizationResult(
      const adagraph::localization::RelocalizationQuery&,
      const adagraph::localization::LocalizationMap&,
      std::stop_token)>;

  explicit FunctionRelocalizer(Function function)
      : function_(std::move(function)) {}

  adagraph::localization::RelocalizationResult Relocalize(
      const adagraph::localization::RelocalizationQuery& query,
      const adagraph::localization::LocalizationMap& map,
      std::stop_token stop_token) override {
    return function_(query, map, stop_token);
  }

 private:
  Function function_;
};

adagraph::sensor::ImmutablePointCloud SharePclCloud(
    const CloudPtr& cloud, adagraph::geometry::FrameId frame) {
  if (!cloud || cloud->empty()) {
    return {};
  }
  std::shared_ptr<const PointCloudType> frozen = cloud;
  const auto* origin =
      reinterpret_cast<const std::byte*>(frozen->data());
  const auto& first = frozen->front();
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      frozen->data(), frozen->size(), sizeof(PointType),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.x) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.y) - origin),
      static_cast<std::size_t>(
          reinterpret_cast<const std::byte*>(&first.z) - origin),
      frame);
  return adagraph::sensor::ImmutablePointCloud::Share(
      std::move(frozen), view,
      cloud->points.capacity() * sizeof(PointType));
}

}  // namespace

PriorMapLocalization::PriorMapLocalization() = default;

PriorMapLocalization::QueryScratchGuard::~QueryScratchGuard() {
  owner_.active_local_submap_backing_bytes_ = 0;
  owner_.active_crop_backing_bytes_ = 0;
}

PriorMapLocalization::~PriorMapLocalization()
{
  shutdown();
}

bool PriorMapLocalization::init(
    PriorMapCallback prior_map_callback,
    adagraph::geometry::StaticTransform3d T_base_imu,
    ResultCallback result_callback,
    adagraph::localization::RelocalizationConfig config,
    KISSMatcherRelocalizer::Config kiss_config,
    const bool use_backend_localization_factor,
    std::string root_dir,
    std::string fallback_map_file)
{
  config_ = std::move(config);
  kiss_config_ = std::move(kiss_config);
  use_backend_localization_factor_ = use_backend_localization_factor;
  root_dir_ = std::move(root_dir);
  fallback_map_file_ = std::move(fallback_map_file);
  // The compatibility map filename is an explicit input to this overload,
  // rather than a process-wide global.  Materialize it into the snapshot so
  // validation and path resolution observe the same effective configuration.
  if (config_.enabled && config_.map_file.empty() &&
      !fallback_map_file_.empty()) {
    config_.map_file = fallback_map_file_;
  }
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid prior-map localization config: " +
                                *error);
  }

  enabled_ = config_.enabled;
  if (!enabled_) {
    return false;
  }

  lost_kiss_retry_cooldown_sec_ = config_.global_retry_cooldown_s;
  lost_kiss_retry_max_backoff_ =
      static_cast<int>(config_.global_retry_max_backoff);
  max_delta_gate_rejections_ =
      static_cast<int>(config_.max_delta_rejections);
  prior_map_grid_cell_size_ =
      static_cast<float>(config_.crop_grid_cell_size_m);

  prior_map_callback_ = std::move(prior_map_callback);
  T_base_imu_ = std::move(T_base_imu);
  result_callback_ = std::move(result_callback);
  if (!result_callback_) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: disabled, missing typed result callback."
                 << RESET;
    enabled_ = false;
    return false;
  }

  if (!loadPriorMap()) {
    enabled_ = false;
    return false;
  }

  scheduler_ =
      std::make_unique<adagraph::localization::RelocalizationScheduler>(
          std::make_unique<FunctionRelocalizer>(
              [this](
                  const adagraph::localization::RelocalizationQuery& query,
                  const adagraph::localization::LocalizationMap& map,
                  std::stop_token stop_token) {
                return processQuery(query, map, stop_token);
              }),
          [this](
              std::uint64_t,
              const adagraph::localization::RelocalizationResult& result,
              const adagraph::geometry::StampedTransform3d& T_odom_base) {
            adagraph::localization::RelocalizationUpdate update{
                result, T_odom_base};
            if (update.valid()) {
              result_callback_(update);
            }
          },
          adagraph::localization::ActiveRelocalizationPolicy::
              kFinishActiveAndPublish);
  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: initialized with typed"
               " latest-only scheduler."
            << RESET;
  return true;
}

void PriorMapLocalization::shutdown()
{
  if (!scheduler_) {
    return;
  }
  scheduler_->finish();
  if (!scheduler_stats_logged_) {
    scheduler_stats_logged_ = true;
    const auto stats = scheduler_->stats();
    LOG(INFO) << GREEN
              << " ---> [RelocalizationScheduler]: submitted/replaced/"
                 "cancelled/completed/published/callback_failures="
              << stats.submitted << "/" << stats.replaced_pending << "/"
              << stats.cancellation_requests << "/" << stats.completed
              << "/" << stats.published << "/"
              << stats.callback_failures
              << ", timing samples=" << stats.timing_samples
              << ", queue_wait mean/max="
              << (stats.timing_samples == 0
                      ? 0.0
                      : static_cast<double>(stats.queue_wait_total_ns) /
                            static_cast<double>(stats.timing_samples) /
                            1.0e6)
              << "/"
              << (static_cast<double>(stats.queue_wait_max_ns) / 1.0e6)
              << " ms, processing mean/max="
              << (stats.timing_samples == 0
                      ? 0.0
                      : static_cast<double>(stats.processing_total_ns) /
                            static_cast<double>(stats.timing_samples) /
                            1.0e6)
              << "/"
              << (static_cast<double>(stats.processing_max_ns) / 1.0e6)
              << " ms, end_to_end mean/max="
              << (stats.timing_samples == 0
                      ? 0.0
                      : static_cast<double>(stats.end_to_end_total_ns) /
                            static_cast<double>(stats.timing_samples) /
                            1.0e6)
              << "/"
              << (static_cast<double>(stats.end_to_end_max_ns) / 1.0e6)
              << " ms" << RESET;
    LOG(INFO) << GREEN
              << " ---> [PriorMapLocalization]: diagnostics startup_kiss "
                 "attempts/successes/failures="
              << startup_kiss_attempts_total_ << "/"
              << startup_kiss_successes_total_ << "/"
              << startup_kiss_failures_total_
              << ", lost_recovery attempts/successes/failures="
              << lost_kiss_attempts_total_ << "/"
              << lost_kiss_successes_total_ << "/"
              << lost_kiss_failures_total_ << ", first_success_delay_s="
              << (first_success_relocalization_delay_sec_.has_value()
                      ? *first_success_relocalization_delay_sec_
                      : -1.0)
              << RESET;
    refreshRetainedMemoryStats();
    const auto memory = memoryStats();
    LOG(INFO) << GREEN
              << " ---> [PriorMapLocalizationMemory]: prior(points/backing)="
              << memory.prior_map_points << "/"
              << memory.prior_map_backing_bytes
              << ", crop_index(cells/indices/payload)="
              << memory.crop_index_cell_count << "/"
              << memory.crop_index_point_indices << "/"
              << memory.crop_index_index_payload_bytes
              << ", local_frames(count/points/backing)="
              << memory.local_frame_count << "/"
              << memory.local_frame_points << "/"
              << memory.local_frame_backing_bytes
              << ", peak_local_submap(points/backing)="
              << memory.peak_local_submap_points << "/"
              << memory.peak_local_submap_backing_bytes
              << ", peak_crop(points/backing)="
              << memory.peak_crop_points << "/"
              << memory.peak_crop_backing_bytes
              << ", peak_pcl(source_odom/aligned)="
              << memory.peak_pcl_odom_conversion_backing_bytes << "/"
              << memory.peak_pcl_registration_aligned_backing_bytes
              << ", peak_runtime(kiss_rss/allocated/retained)="
              << memory.peak_kiss_resident_kib << "/"
              << memory.peak_kiss_glibc_allocated_bytes << "/"
              << memory.peak_kiss_glibc_retained_bytes
              << ", registration_rss/allocated/retained="
              << memory.peak_registration_resident_kib << "/"
              << memory.peak_registration_glibc_allocated_bytes << "/"
              << memory.peak_registration_glibc_retained_bytes
              << ", peak_explicit_working_set="
              << memory.peak_explicit_working_set_bytes << " bytes"
              << RESET;
  }
}

PriorMapLocalization::MemoryStats PriorMapLocalization::memoryStats() const
    noexcept {
  return memory_stats_;
}

void PriorMapLocalization::submit(double timestamp,
                                  const CloudPtr& registered_cloud,
                                  const SE3& odom_lidar)
{
  if (!enabled_ || !registered_cloud || registered_cloud->empty()) {
    return;
  }

  bool has_map_to_odom = false;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    has_map_to_odom = has_map_to_odom_;
  }
  // Build the stationary startup submap quickly so odom jitter cannot smear
  // scans collected over several seconds. Tracking keeps the configured
  // interval to bound registration CPU usage.
  const int interval = has_map_to_odom
      ? static_cast<int>(config_.minimum_frame_interval)
      : 1;
  if ((submitted_frames_++ % static_cast<std::size_t>(interval)) != 0) {
    return;
  }

  const auto typed_timestamp =
      adagraph::common::Timestamp::FromSeconds(timestamp);
  if (!typed_timestamp || !T_base_imu_.valid() || !scheduler_ ||
      !localization_map_) {
    return;
  }
  const adagraph::geometry::StampedTransform3d T_odom_imu{
      *typed_timestamp,
      adagraph::geometry::FrameId::kOdom,
      adagraph::geometry::FrameId::kImu,
      {Eigen::Quaterniond(odom_lidar.R_.cast<double>()).normalized(),
       odom_lidar.t_.cast<double>()},
  };
  const auto T_odom_base =
      T_odom_imu.compose(T_base_imu_.inverse());
  if (!T_odom_base) {
    return;
  }

  CloudPtr cloud_base(new PointCloudType());
  pcl::transformPointCloud(
      *registered_cloud, *cloud_base,
      toMatrix4f(fromDomainPose(T_base_imu_.T_target_source)));
  adagraph::localization::RelocalizationQuery query;
  query.T_odom_base = *T_odom_base;
  query.current_scan_base = SharePclCloud(
      cloud_base, adagraph::geometry::FrameId::kBase);
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    if (has_map_to_odom_) {
      const SE3 map_base =
          map_to_odom_ * fromDomainPose(T_odom_base->T_target_source);
      query.prior_T_map_base = adagraph::geometry::StampedTransform3d{
          *typed_timestamp,
          adagraph::geometry::FrameId::kMap,
          adagraph::geometry::FrameId::kBase,
          {Eigen::Quaterniond(map_base.R_.cast<double>()).normalized(),
           map_base.t_.cast<double>()},
      };
    }
  }
  const auto status = scheduler_->submit(
      std::move(query), localization_map_);
  if (status ==
      adagraph::localization::RelocalizationSubmissionStatus::kInvalid) {
    LOG(ERROR) << RED
               << " ---> [PriorMapLocalization]: invalid typed query."
               << RESET;
  }
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

std::optional<SE3> PriorMapLocalization::refineBackendMapToOdom(
    const SE3& candidate, const SE3& odom_lidar)
{
  if (!enabled_) {
    return std::nullopt;
  }

  SE3 current_map_odom;
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    if (!has_map_to_odom_) {
      /// The backend may only refine an existing map->odom; initialization
      /// stays with the KISS/VGICP pipeline.
      return std::nullopt;
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
  if (trans > config_.max_translation_m ||
      rot_deg > config_.max_rotation_deg) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: backend map->odom rejected"
                    " by jump gate, dt: " << trans
                 << ", drot_deg: " << rot_deg << RESET;
    return std::nullopt;
  }

  const SE3 smoothed = smoothMapToOdom(candidate, false);
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    map_to_odom_ = smoothed;
  }
  return smoothed;
}

std::string PriorMapLocalization::resolveMapPath() const
{
  if (!config_.map_file.empty()) {
    const std::filesystem::path configured(config_.map_file);
    if (configured.is_absolute()) {
      return configured.string();
    }
    return (std::filesystem::path(root_dir_) / configured).string();
  }

  return fallback_map_file_;
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

  CloudPtr filtered_map = downsampleCloud(
      raw_map, static_cast<float>(config_.target_voxel_size_m));
  if (!filtered_map || filtered_map->empty()) {
    LOG(ERROR) << RED
               << " ---> [PriorMapLocalization]: downsampled prior map is empty: "
               << map_path << RESET;
    return false;
  }
  filtered_map->is_dense = false;
  auto immutable_map = SharePclCloud(
      filtered_map, adagraph::geometry::FrameId::kMap);
  auto localization_map =
      std::make_shared<const adagraph::localization::LocalizationMap>(
          1, std::move(immutable_map));
  if (!localization_map->valid() ||
      !localization_map->cloudMap().ownerAs<PointCloudType>()) {
    LOG(ERROR) << RED
               << " ---> [PriorMapLocalization]: failed to freeze typed"
                  " localization map."
               << RESET;
    return false;
  }
  localization_map_ = std::move(localization_map);
  buildPriorMapGrid();
  global_relocalizer_ =
      std::make_unique<adagraph::localization::ComposedRelocalizer>(
          std::make_unique<PclKissCandidateRetriever>(
              kiss_config_, [this](const PclRelocalizationTemporaryMemory& sample) {
                notePclTemporaryMemory(sample);
              }),
          std::make_unique<PclRegistrationVerifier>(
              config_,
              [this](const SE3& initial_T_map_base) {
                return BASIC::PointCloudType::ConstPtr(
                    cropPriorMap(initial_T_map_base));
              }, [this](const PclRelocalizationTemporaryMemory& sample) {
                notePclTemporaryMemory(sample);
              }));
  if (prior_map_callback_) {
    // Publish/reset consumers from the original map before releasing it.
    // Registration keeps only the downsampled copy above.
    prior_map_callback_(raw_map);
  }
  LOG(INFO) << GREEN
            << " ---> [PriorMapLocalization]: loaded prior map: " << map_path
            << ", raw points: " << raw_map->size()
            << ", filtered points: " << localizationMapCloud()->size()
            << RESET;
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

BASIC::PointCloudType::ConstPtr
PriorMapLocalization::localizationMapCloud() const noexcept
{
  if (!localization_map_ || !localization_map_->valid()) {
    return {};
  }
  return localization_map_->cloudMap().ownerAs<PointCloudType>();
}

void PriorMapLocalization::buildPriorMapGrid()
{
  prior_map_grid_.clear();
  const auto map = localizationMapCloud();
  if (!map || map->empty()) {
    return;
  }

  const double cell = std::max(1.0f, prior_map_grid_cell_size_);
  for (std::uint32_t i = 0; i < map->size(); ++i) {
    const auto& point = map->points[i];
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
  refreshRetainedMemoryStats();
}

adagraph::localization::RelocalizationResult
PriorMapLocalization::processQuery(
    const adagraph::localization::RelocalizationQuery& query,
    const adagraph::localization::LocalizationMap& map,
    std::stop_token stop_token)
{
  adagraph::localization::RelocalizationResult output;
  QueryScratchGuard scratch_guard(*this);
  if (!query.valid() || !map.valid() || stop_token.stop_requested()) {
    output.status = stop_token.stop_requested()
        ? adagraph::localization::RelocalizationStatus::kCancelled
        : adagraph::localization::RelocalizationStatus::kInvalidInput;
    return output;
  }
  const auto cloud_base =
      query.current_scan_base.ownerAs<PointCloudType>();
  if (!cloud_base || cloud_base->empty()) {
    output.status =
        adagraph::localization::RelocalizationStatus::kInvalidInput;
    return output;
  }
  const double timestamp_seconds = query.T_odom_base.timestamp.seconds();
  if (!first_query_timestamp_sec_.has_value()) {
    first_query_timestamp_sec_ = timestamp_seconds;
  }
  const SE3 T_odom_base =
      fromDomainPose(query.T_odom_base.T_target_source);
  const SE3 T_odom_imu =
      T_odom_base * fromDomainPose(T_base_imu_.T_target_source);
  appendLocalFrame(cloud_base, T_odom_base);

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
  bool startup_kiss_attempt = false;
  bool lost_kiss_attempt = false;
  std::optional<RegistrationResult> kiss_registration_result;
  const bool manual_kiss_requested =
      manual_kiss_requested_.exchange(false, std::memory_order_acq_rel);

  if (config_.use_global_descriptor) {
    const int min_kiss_frames = std::max(
        1, std::min(static_cast<int>(config_.minimum_global_search_frames),
                    static_cast<int>(config_.local_frame_count)));
    const bool has_enough_kiss_frames =
        local_frames_base_.size() >= static_cast<std::size_t>(min_kiss_frames);
    const bool startup_kiss_due =
        state_ == State::UNINITIALIZED &&
        (!startup_kiss_attempted_ ||
         timestamp_seconds - last_startup_kiss_time_ >=
             lost_kiss_retry_cooldown_sec_);
    const bool lost_kiss_due =
        state_ == State::LOST && lostKissRetryDue(timestamp_seconds);

    if (!has_enough_kiss_frames &&
        (manual_kiss_requested ||
         startup_kiss_due ||
         lost_kiss_due)) {
      if (manual_kiss_requested) {
        manual_kiss_requested_.store(true, std::memory_order_release);
      }
      LOG(WARNING) << YELLOW
                   << " ---> [PriorMapLocalization]: waiting for KISS local submap, frames: "
                   << local_frames_base_.size() << "/" << min_kiss_frames
                   << RESET;
    } else if (manual_kiss_requested) {
      should_run_kiss = true;
      force_jump = true;
      if (state_ == State::UNINITIALIZED) {
        startup_kiss_attempt = true;
        ++startup_kiss_attempts_total_;
      } else if (state_ == State::LOST) {
        lost_kiss_attempt = true;
        ++lost_kiss_attempts_total_;
      }
    } else if (startup_kiss_due) {
      startup_kiss_attempted_ = true;
      last_startup_kiss_time_ = timestamp_seconds;
      startup_kiss_attempt = true;
      ++startup_kiss_attempts_total_;
      should_run_kiss = true;
    } else if (lost_kiss_due) {
      lost_kiss_attempted_ = true;
      last_lost_kiss_time_ = timestamp_seconds;
      lost_kiss_attempt = true;
      ++lost_kiss_attempts_total_;
      should_run_kiss = true;
      LOG(INFO) << YELLOW
                << " ---> [PriorMapLocalization]: LOST, retrying KISS relocalization"
                << " (consecutive failures: " << lost_kiss_failures_ << ")."
                << RESET;
    }
  }

  if (!should_run_kiss && !had_map_to_odom) {
    if ((waiting_for_initial_queries_++ % 50U) == 0U) {
      LOG(WARNING) << YELLOW
                   << " ---> [PriorMapLocalization]: no initial map->odom;"
                      " waiting for next KISS retry."
                   << RESET;
    }
    output.status =
        adagraph::localization::RelocalizationStatus::kNoCandidate;
    return output;
  }
  // LOST is a recovery state, not a normal tracking state. Keep ingesting
  // bounded history so the next KISS attempt has a fresh local submap, but do
  // not rebuild/downsample/register that submap on every query during the
  // retry cooldown.
  if (state_ == State::LOST && !should_run_kiss) {
    output.status =
        adagraph::localization::RelocalizationStatus::kNoCandidate;
    return output;
  }
  if (stop_token.stop_requested()) {
    output.status = adagraph::localization::RelocalizationStatus::kCancelled;
    return output;
  }
  CloudPtr local_submap = buildLocalSubmap(T_odom_base);
  if (!local_submap || local_submap->empty()) {
    output.status =
        adagraph::localization::RelocalizationStatus::kInvalidInput;
    return output;
  }

  if (should_run_kiss) {
    adagraph::localization::RelocalizationQuery global_query;
    if (global_relocalizer_) {
      global_query.T_odom_base = {
          query.T_odom_base.timestamp,
          adagraph::geometry::FrameId::kOdom,
          adagraph::geometry::FrameId::kBase,
          {Eigen::Quaterniond(T_odom_base.R_.cast<double>()).normalized(),
           T_odom_base.t_.cast<double>()},
      };
      global_query.current_scan_base = SharePclCloud(
          local_submap, adagraph::geometry::FrameId::kBase);
      if (had_map_to_odom) {
        const SE3 prior_map_base = predicted_map_odom * T_odom_base;
        global_query.prior_T_map_base =
            adagraph::geometry::StampedTransform3d{
            query.T_odom_base.timestamp,
            adagraph::geometry::FrameId::kMap,
            adagraph::geometry::FrameId::kBase,
            {Eigen::Quaterniond(prior_map_base.R_.cast<double>()).normalized(),
             prior_map_base.t_.cast<double>()},
        };
      }
    }

    const auto relocalization =
        global_query.valid() && global_relocalizer_
            ? global_relocalizer_->Relocalize(
                  global_query, map, stop_token)
            : adagraph::localization::RelocalizationResult{};
    if (relocalization.success() && relocalization.T_map_base) {
      RegistrationResult accepted;
      accepted.converged = true;
      accepted.fitness = relocalization.fitness;
      accepted.map_odom =
          fromDomainPose(
              relocalization.T_map_base->T_target_source) *
          T_odom_base.inverse();
      // The typed verifier reports registration quality only. This legacy
      // policy gate depends on the current tracking state, odom pose and
      // manual/startup mode, so it remains in this stateful adapter rather
      // than leaking back into the generic relocalizer.
      if (!passQualityGate(accepted, T_odom_imu, force_jump)) {
        if (!stop_token.stop_requested()) {
          if (startup_kiss_attempt) {
            ++startup_kiss_failures_total_;
          }
          if (lost_kiss_attempt) {
            ++lost_kiss_failures_total_;
          }
        }
        if (had_map_to_odom &&
            accepted.fitness > config_.lost_fitness) {
          enterLost();
        }
        output.status =
            adagraph::localization::RelocalizationStatus::kRegistrationRejected;
        return output;
      }
      kiss_registration_result = std::move(accepted);
      predicted_map_odom = kiss_registration_result->map_odom;
      used_kiss_initial_guess = true;
      LOG(INFO) << GREEN
                << " ---> [PriorMapLocalization]: typed KISS/VGICP"
                   " relocalizer selected candidate [fitness="
                << kiss_registration_result->fitness << "]."
                << RESET;
    } else {
      if (!stop_token.stop_requested()) {
        if (startup_kiss_attempt) {
          ++startup_kiss_failures_total_;
        }
        if (lost_kiss_attempt) {
          ++lost_kiss_failures_total_;
        }
      }
      if (state_ == State::LOST) {
        ++lost_kiss_failures_;
      } else if (state_ == State::UNINITIALIZED) {
        ++startup_kiss_failures_;
      }
      if (!had_map_to_odom) {
        LOG(WARNING) << YELLOW
                     << " ---> [PriorMapLocalization]: no KISS candidate passed"
                        " VGICP; retry "
                     << startup_kiss_failures_ << " scheduled automatically."
                     << RESET;
        output.status = stop_token.stop_requested()
            ? adagraph::localization::RelocalizationStatus::kCancelled
            : adagraph::localization::RelocalizationStatus::kNoCandidate;
        return output;
      }
    }
  }

  RegistrationResult result;
  bool registration_prevalidated = false;
  if (kiss_registration_result) {
    result = std::move(*kiss_registration_result);
    registration_prevalidated = true;
  } else {
    const SE3 predicted_map_base = predicted_map_odom * T_odom_base;
    CloudPtr cropped_map = cropPriorMap(predicted_map_base);
    if (!cropped_map || cropped_map->empty()) {
      LOG(WARNING) << YELLOW
                   << " ---> [PriorMapLocalization]: no prior-map points in crop window."
                   << RESET;
      enterLost();
      output.status =
          adagraph::localization::RelocalizationStatus::kRegistrationRejected;
      return output;
    }
    result = runRegistration(local_submap, cropped_map, predicted_map_odom,
                             T_odom_base);
  }

  if (!registration_prevalidated &&
      !passQualityGate(result, T_odom_imu, force_jump)) {
    if (had_map_to_odom &&
        result.fitness > config_.lost_fitness) {
      enterLost();
    }
    output.status =
        adagraph::localization::RelocalizationStatus::kRegistrationRejected;
    return output;
  }

  if (stop_token.stop_requested()) {
    output.status = adagraph::localization::RelocalizationStatus::kCancelled;
    return output;
  }

  const bool jump = force_jump || state_ == State::UNINITIALIZED || state_ == State::LOST;
  const SE3 output_map_odom = smoothMapToOdom(result.map_odom, jump);
  {
    std::lock_guard<std::mutex> lock(map_to_odom_state_mutex_);
    map_to_odom_ = output_map_odom;
    has_map_to_odom_ = true;
  }
  state_ = State::TRACKING;
  if (startup_kiss_attempt) {
    ++startup_kiss_successes_total_;
  }
  if (lost_kiss_attempt) {
    ++lost_kiss_successes_total_;
  }
  // This metric describes the startup/recovery relocalizer, not the first
  // ordinary local registration after a map->odom already exists.  Keeping
  // that distinction makes the acceptance gate meaningful when the global
  // descriptor path is disabled for a compatibility configuration.
  if ((startup_kiss_attempt || lost_kiss_attempt) &&
      !first_success_relocalization_delay_sec_.has_value() &&
      first_query_timestamp_sec_.has_value()) {
    first_success_relocalization_delay_sec_ = std::max(
        0.0, timestamp_seconds - *first_query_timestamp_sec_);
  }
  waiting_for_initial_queries_ = 0;
  startup_kiss_failures_ = 0;
  lost_kiss_attempted_ = false;
  lost_kiss_failures_ = 0;
  delta_gate_rejections_ = 0;
  has_rejected_candidate_ = false;
  const SE3 output_map_base = output_map_odom * T_odom_base;
  output.status = adagraph::localization::RelocalizationStatus::kSuccess;
  output.T_map_base = adagraph::geometry::StampedTransform3d{
      query.T_odom_base.timestamp,
      adagraph::geometry::FrameId::kMap,
      adagraph::geometry::FrameId::kBase,
      {Eigen::Quaterniond(output_map_base.R_.cast<double>()).normalized(),
       output_map_base.t_.cast<double>()},
  };
  output.fitness = result.fitness;
  output.inlier_ratio = 0.0;
  output.confidence = adagraph::localization::Confidence::kUnknown;

  if (use_backend_localization_factor_) {
    /// Queue the raw (unsmoothed) registration for the backend: the unary
    /// factor wants the measured absolute pose, smoothing only serves TF.
    MapObservation obs;
    obs.timestamp = timestamp_seconds;
    obs.map_lidar = result.map_odom * T_odom_imu;
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
            << timestamp_seconds << RESET;
  return output;
}

void PriorMapLocalization::appendLocalFrame(
    const PointCloudType::ConstPtr& cloud_base,
    const SE3& T_odom_base)
{
  local_frames_base_.push_back({cloud_base, T_odom_base});

  const int max_frames = static_cast<int>(config_.local_frame_count);
  while (local_frames_base_.size() > static_cast<std::size_t>(max_frames)) {
    local_frames_base_.pop_front();
  }
  refreshRetainedMemoryStats();
}

CloudPtr PriorMapLocalization::buildLocalSubmap(
    const SE3& T_odom_base)
{
  CloudPtr submap(new PointCloudType());
  std::size_t total_points = 0;
  for (const auto& frame : local_frames_base_) {
    if (frame.cloud_base) {
      total_points += frame.cloud_base->size();
    }
  }
  submap->reserve(total_points);

  for (const auto& frame : local_frames_base_) {
    if (!frame.cloud_base || frame.cloud_base->empty()) {
      continue;
    }
    for (const auto& point : frame.cloud_base->points) {
      const V3 point_odom =
          frame.T_odom_base * V3(point.x, point.y, point.z);
      PointType transformed = point;
      transformed.x = static_cast<float>(point_odom.x());
      transformed.y = static_cast<float>(point_odom.y());
      transformed.z = static_cast<float>(point_odom.z());
      submap->push_back(std::move(transformed));
    }
  }

  submap = downsampleCloud(
      submap, static_cast<float>(config_.source_voxel_size_m));
  submap = limitCloudSize(
      submap, static_cast<int>(config_.max_source_points));
  // Reuse the already allocated/filtered buffer. This makes the typed query
  // cloud genuinely base-frame without another large cloud allocation.
  const M3 R_base_odom = T_odom_base.R_.transpose();
  const V3 t_odom_base = T_odom_base.t_;
  for (auto& point : submap->points) {
    const V3 point_odom(point.x, point.y, point.z);
    const V3 point_base = R_base_odom * (point_odom - t_odom_base);
    point.x = static_cast<float>(point_base.x());
    point.y = static_cast<float>(point_base.y());
    point.z = static_cast<float>(point_base.z());
  }
  noteLocalSubmap(submap);
  return submap;
}

CloudPtr PriorMapLocalization::cropPriorMap(const SE3& predicted_map_lidar)
{
  CloudPtr cropped(new PointCloudType());
  const auto map = localizationMapCloud();
  if (!map || map->empty()) {
    return cropped;
  }

  const double radius = config_.crop_radius_m;
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
      map->size(), config_.max_target_points));

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
          if (idx >= map->size()) {
            continue;
          }
          const auto& point = map->points[idx];
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
  cropped = limitCloudSize(
      cropped, static_cast<int>(config_.max_target_points));
  noteCrop(cropped);
  return cropped;
}

void PriorMapLocalization::refreshRetainedMemoryStats() {
  const auto map = localizationMapCloud();
  memory_stats_.prior_map_points = map ? map->size() : 0;
  memory_stats_.prior_map_backing_bytes =
      map ? map->points.capacity() * sizeof(PointType) : 0;
  memory_stats_.crop_index_cell_count = prior_map_grid_.size();
  memory_stats_.crop_index_point_indices = 0;
  for (const auto& [unused_key, indices] : prior_map_grid_) {
    (void)unused_key;
    memory_stats_.crop_index_point_indices += indices.size();
  }
  memory_stats_.crop_index_index_payload_bytes =
      memory_stats_.crop_index_point_indices * sizeof(std::uint32_t);
  memory_stats_.local_frame_count = local_frames_base_.size();
  memory_stats_.local_frame_points = 0;
  memory_stats_.local_frame_backing_bytes = 0;
  std::unordered_set<const void*> unique_local_frames;
  for (const auto& frame : local_frames_base_) {
    if (!frame.cloud_base) {
      continue;
    }
    memory_stats_.local_frame_points += frame.cloud_base->size();
    const auto* storage = frame.cloud_base->points.data();
    if (storage != nullptr && unique_local_frames.insert(storage).second) {
      memory_stats_.local_frame_backing_bytes +=
          frame.cloud_base->points.capacity() * sizeof(PointType);
    }
  }
}

void PriorMapLocalization::noteLocalSubmap(const CloudPtr& cloud) {
  const std::size_t points = cloud ? cloud->size() : 0;
  const std::size_t backing =
      cloud ? cloud->points.capacity() * sizeof(PointType) : 0;
  active_local_submap_backing_bytes_ = backing;
  memory_stats_.peak_local_submap_points = std::max(
      memory_stats_.peak_local_submap_points, points);
  memory_stats_.peak_local_submap_backing_bytes = std::max(
      memory_stats_.peak_local_submap_backing_bytes, backing);
  memory_stats_.peak_explicit_working_set_bytes = std::max(
      memory_stats_.peak_explicit_working_set_bytes,
      memory_stats_.prior_map_backing_bytes +
          memory_stats_.crop_index_index_payload_bytes +
          memory_stats_.local_frame_backing_bytes +
          active_local_submap_backing_bytes_ + active_crop_backing_bytes_);
}

void PriorMapLocalization::noteCrop(const CloudPtr& cloud) {
  const std::size_t points = cloud ? cloud->size() : 0;
  const std::size_t backing =
      cloud ? cloud->points.capacity() * sizeof(PointType) : 0;
  active_crop_backing_bytes_ = backing;
  memory_stats_.peak_crop_points = std::max(memory_stats_.peak_crop_points,
                                             points);
  memory_stats_.peak_crop_backing_bytes = std::max(
      memory_stats_.peak_crop_backing_bytes, backing);
  memory_stats_.peak_explicit_working_set_bytes = std::max(
      memory_stats_.peak_explicit_working_set_bytes,
      memory_stats_.prior_map_backing_bytes +
          memory_stats_.crop_index_index_payload_bytes +
          memory_stats_.local_frame_backing_bytes +
          active_local_submap_backing_bytes_ + active_crop_backing_bytes_);
}

void PriorMapLocalization::notePclTemporaryMemory(
    const PclRelocalizationTemporaryMemory& sample) {
  memory_stats_.peak_pcl_odom_conversion_backing_bytes = std::max(
      memory_stats_.peak_pcl_odom_conversion_backing_bytes,
      sample.source_odom_backing_bytes);
  memory_stats_.peak_pcl_registration_aligned_backing_bytes = std::max(
      memory_stats_.peak_pcl_registration_aligned_backing_bytes,
      sample.registration_aligned_backing_bytes);
  if (sample.stage == PclRelocalizationTemporaryMemory::Stage::kKiss) {
    memory_stats_.peak_kiss_resident_kib = std::max(
        memory_stats_.peak_kiss_resident_kib, sample.runtime.resident_kib);
    memory_stats_.peak_kiss_glibc_allocated_bytes = std::max(
        memory_stats_.peak_kiss_glibc_allocated_bytes,
        sample.runtime.glibc_allocated_bytes);
    memory_stats_.peak_kiss_glibc_retained_bytes = std::max(
        memory_stats_.peak_kiss_glibc_retained_bytes,
        sample.runtime.glibc_retained_bytes);
  } else {
    noteTrackingRegistrationRuntimeMemory(sample.runtime);
  }
  memory_stats_.peak_explicit_working_set_bytes = std::max(
      memory_stats_.peak_explicit_working_set_bytes,
      memory_stats_.prior_map_backing_bytes +
          memory_stats_.crop_index_index_payload_bytes +
          memory_stats_.local_frame_backing_bytes +
          active_local_submap_backing_bytes_ + active_crop_backing_bytes_ +
          sample.source_odom_backing_bytes +
          sample.registration_aligned_backing_bytes);
}

void PriorMapLocalization::noteTrackingRegistrationRuntimeMemory(
    const RuntimeMemorySample sample) {
  memory_stats_.peak_registration_resident_kib = std::max(
      memory_stats_.peak_registration_resident_kib, sample.resident_kib);
  memory_stats_.peak_registration_glibc_allocated_bytes = std::max(
      memory_stats_.peak_registration_glibc_allocated_bytes,
      sample.glibc_allocated_bytes);
  memory_stats_.peak_registration_glibc_retained_bytes = std::max(
      memory_stats_.peak_registration_glibc_retained_bytes,
      sample.glibc_retained_bytes);
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
    const CloudPtr& source_base,
    const CloudPtr& target_map,
    const SE3& initial_map_odom,
    const SE3& T_odom_base)
{
  RegistrationResult result;
  if (!source_base || source_base->empty() || !target_map ||
      target_map->empty()) {
    return result;
  }

  const SE3 initial_map_base = initial_map_odom * T_odom_base;

  PointCloudType aligned;
  Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();

#ifdef HAS_FAST_GICP
  fast_gicp::FastVGICP<PointType, PointType> vgicp;
  vgicp.setInputSource(source_base);
  vgicp.setInputTarget(target_map);
  vgicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  /// FastVGICP ignores setMaxCorrespondenceDistance; correspondences are
  /// gated by the voxel resolution set below.
  vgicp.setTransformationEpsilon(1e-4);
  vgicp.setEuclideanFitnessEpsilon(1e-4);
  vgicp.setResolution(
      static_cast<float>(config_.source_voxel_size_m));
  vgicp.setNumThreads(static_cast<int>(config_.registration_threads));
  vgicp.align(aligned, toMatrix4f(initial_map_base));
  result.converged = vgicp.hasConverged();
  result.fitness = vgicp.getFitnessScore();
  final_transformation = vgicp.getFinalTransformation();
#else
  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
  gicp.setInputSource(source_base);
  gicp.setInputTarget(target_map);
  gicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  gicp.setMaxCorrespondenceDistance(
      std::max(1.0, config_.source_voxel_size_m * 8.0));
  gicp.setTransformationEpsilon(1e-4);
  gicp.setEuclideanFitnessEpsilon(1e-4);
  gicp.align(aligned, toMatrix4f(initial_map_base));
  result.converged = gicp.hasConverged();
  result.fitness = gicp.getFitnessScore();
  final_transformation = gicp.getFinalTransformation();
#endif

  notePclTemporaryMemory(
      {PclRelocalizationTemporaryMemory::Stage::kRegistration,
       0, aligned.points.capacity() * sizeof(PointType),
       CaptureRuntimeMemory()});

  if (!result.converged) {
    return result;
  }

  const SE3 optimized_map_base = fromMatrix4f(final_transformation);
  result.map_odom = optimized_map_base * T_odom_base.inverse();
  return result;
}

bool PriorMapLocalization::passQualityGate(const RegistrationResult& result,
                                           const SE3& odom_lidar,
                                           bool force_jump)
{
  const double fitness_limit =
      (state_ != State::TRACKING || force_jump)
          ? config_.initial_max_fitness
          : config_.max_fitness;
  if (!result.converged || !std::isfinite(result.fitness) ||
      result.fitness > fitness_limit) {
    LOG(WARNING) << YELLOW
                 << " ---> [PriorMapLocalization]: rejected by fitness, converged: "
                 << result.converged << ", fitness: " << result.fitness
                 << ", limit: " << fitness_limit << RESET;
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
  if (trans > config_.max_translation_m ||
      rot_deg > config_.max_rotation_deg) {
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
    consistent = delta.t_.norm() <= config_.max_translation_m &&
                 rotationAngleDeg(delta) <= config_.max_rotation_deg;
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

  const double trans_alpha = config_.translation_smoothing_alpha;
  const double rot_alpha = config_.rotation_smoothing_alpha;

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
