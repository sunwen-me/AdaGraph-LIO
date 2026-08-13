#ifndef PRIOR_MAP_LOCALIZATION_H_
#define PRIOR_MAP_LOCALIZATION_H_

#include <cstdint>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "adagraph/localization/relocalization_config.h"
#include "adagraph/localization/localization_map.h"
#include "adagraph/localization/composed_relocalizer.h"
#include "adagraph/localization/relocalization_scheduler.h"
#include "adagraph/localization/relocalization_update.h"
#include "lio/adagraph_types.h"
#include "lio/kiss_matcher_relocalizer.h"
#include "lio/pcl_relocalizer_adapters.h"

namespace LI2Sup {

class PriorMapLocalization {
public:
  using PriorMapCallback = std::function<void(const BASIC::CloudPtr&)>;
  using ResultCallback = std::function<void(
      const adagraph::localization::RelocalizationUpdate&)>;

  struct MemoryStats {
    std::size_t prior_map_points = 0;
    std::size_t prior_map_backing_bytes = 0;
    std::size_t crop_index_cell_count = 0;
    std::size_t crop_index_point_indices = 0;
    std::size_t crop_index_index_payload_bytes = 0;
    std::size_t local_frame_count = 0;
    std::size_t local_frame_points = 0;
    std::size_t local_frame_backing_bytes = 0;
    std::size_t peak_local_submap_points = 0;
    std::size_t peak_local_submap_backing_bytes = 0;
    std::size_t peak_crop_points = 0;
    std::size_t peak_crop_backing_bytes = 0;
    std::size_t peak_pcl_odom_conversion_backing_bytes = 0;
    std::size_t peak_pcl_registration_aligned_backing_bytes = 0;
    std::size_t peak_kiss_resident_kib = 0;
    std::size_t peak_kiss_glibc_allocated_bytes = 0;
    std::size_t peak_kiss_glibc_retained_bytes = 0;
    std::size_t peak_registration_resident_kib = 0;
    std::size_t peak_registration_glibc_allocated_bytes = 0;
    std::size_t peak_registration_glibc_retained_bytes = 0;
    std::size_t peak_explicit_working_set_bytes = 0;
  };

  PriorMapLocalization();
  ~PriorMapLocalization();

  bool init(PriorMapCallback prior_map_callback = {},
            adagraph::geometry::StaticTransform3d T_base_imu = {},
            ResultCallback result_callback = {});
  bool init(
      PriorMapCallback prior_map_callback,
      adagraph::geometry::StaticTransform3d T_base_imu,
      ResultCallback result_callback,
      adagraph::localization::RelocalizationConfig config,
      KISSMatcherRelocalizer::Config kiss_config,
      bool use_backend_localization_factor = false,
      std::string root_dir = {},
      std::string fallback_map_file = {});
  void shutdown();
  bool enabled() const { return enabled_; }
  [[nodiscard]] std::shared_ptr<const adagraph::localization::LocalizationMap>
  localizationMap() const noexcept {
    return localization_map_;
  }
  [[nodiscard]] const adagraph::localization::RelocalizationConfig& config()
      const noexcept {
    return config_;
  }
  // Valid after scheduler shutdown, or while the caller otherwise owns the
  // relocalization worker. Values describe visible container storage only.
  [[nodiscard]] MemoryStats memoryStats() const noexcept;

  void submit(double timestamp,
              const BASIC::CloudPtr& registered_cloud,
              const BASIC::SE3& odom_lidar);
  void requestGlobalRelocalization();

  /// Drains the queue of accepted registrations collected for the backend
  /// (filled only while lio.backend.use_localization_factor is set).
  void takeMapObservations(std::vector<MapObservation>& out);

  /// Feeds a backend-derived map->odom candidate through the existing delta
  /// gate and smoothing channel. Only refines an already-initialized
  /// map->odom (never initializes it) and never changes the tracking state.
  [[nodiscard]] std::optional<BASIC::SE3> refineBackendMapToOdom(
      const BASIC::SE3& candidate,
      const BASIC::SE3& odom_lidar);

private:
  enum class State {
    UNINITIALIZED,
    TRACKING,
    LOST
  };

  struct RegistrationResult {
    bool converged = false;
    double fitness = std::numeric_limits<double>::infinity();
    BASIC::SE3 map_odom;
  };

  struct LocalFrameBase {
    BASIC::PointCloudType::ConstPtr cloud_base;
    BASIC::SE3 T_odom_base;
  };

  class QueryScratchGuard {
   public:
    explicit QueryScratchGuard(PriorMapLocalization& owner) noexcept
        : owner_(owner) {}
    ~QueryScratchGuard();

   private:
    PriorMapLocalization& owner_;
  };

  std::string resolveMapPath() const;
  bool loadPriorMap();
  void buildPriorMapGrid();
  [[nodiscard]] BASIC::PointCloudType::ConstPtr localizationMapCloud()
      const noexcept;
  [[nodiscard]] adagraph::localization::RelocalizationResult processQuery(
      const adagraph::localization::RelocalizationQuery& query,
      const adagraph::localization::LocalizationMap& map,
      std::stop_token stop_token);
  void appendLocalFrame(
      const BASIC::PointCloudType::ConstPtr& cloud_base,
      const BASIC::SE3& T_odom_base);
  BASIC::CloudPtr buildLocalSubmap(
      const BASIC::SE3& T_odom_base);
  BASIC::CloudPtr cropPriorMap(const BASIC::SE3& predicted_map_lidar);
  BASIC::CloudPtr downsampleCloud(
      const BASIC::PointCloudType::ConstPtr& cloud, float leaf_size) const;
  BASIC::CloudPtr limitCloudSize(BASIC::CloudPtr cloud, int max_points) const;
  RegistrationResult runRegistration(const BASIC::CloudPtr& source_base,
                                     const BASIC::CloudPtr& target_map,
                                     const BASIC::SE3& initial_map_odom,
                                     const BASIC::SE3& T_odom_base);
  bool passQualityGate(const RegistrationResult& result,
                       const BASIC::SE3& odom_lidar,
                       bool force_jump);
  void noteDeltaGateRejection(const BASIC::SE3& candidate_map_odom,
                              const BASIC::SE3& odom_lidar);
  bool lostKissRetryDue(double now_sec) const;
  BASIC::SE3 smoothMapToOdom(const BASIC::SE3& candidate, bool jump) const;
  void enterLost();
  static double rotationAngleDeg(const BASIC::SE3& delta);
  static double clamp01(double value);
  void refreshRetainedMemoryStats();
  void noteLocalSubmap(const BASIC::CloudPtr& cloud);
  void noteCrop(const BASIC::CloudPtr& cloud);
  void notePclTemporaryMemory(
      const PclRelocalizationTemporaryMemory& sample);
  void noteTrackingRegistrationRuntimeMemory(
      RuntimeMemorySample sample);

  bool enabled_ = false;
  std::size_t submitted_frames_ = 0;
  std::unique_ptr<adagraph::localization::RelocalizationScheduler>
      scheduler_;
  bool scheduler_stats_logged_ = false;

  std::shared_ptr<const adagraph::localization::LocalizationMap>
      localization_map_;
  std::unique_ptr<adagraph::localization::ComposedRelocalizer>
      global_relocalizer_;
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> prior_map_grid_;
  std::deque<LocalFrameBase> local_frames_base_;
  MemoryStats memory_stats_;
  std::size_t active_local_submap_backing_bytes_ = 0;
  std::size_t active_crop_backing_bytes_ = 0;
  State state_ = State::UNINITIALIZED;
  bool startup_kiss_attempted_ = false;
  double last_startup_kiss_time_ = 0.0;
  int startup_kiss_failures_ = 0;
  std::uint64_t startup_kiss_attempts_total_ = 0;
  std::uint64_t startup_kiss_successes_total_ = 0;
  std::uint64_t startup_kiss_failures_total_ = 0;
  bool lost_kiss_attempted_ = false;
  double last_lost_kiss_time_ = 0.0;
  int lost_kiss_failures_ = 0;
  std::uint64_t lost_kiss_attempts_total_ = 0;
  std::uint64_t lost_kiss_successes_total_ = 0;
  std::uint64_t lost_kiss_failures_total_ = 0;
  std::optional<double> first_query_timestamp_sec_;
  std::optional<double> first_success_relocalization_delay_sec_;
  int delta_gate_rejections_ = 0;
  std::size_t waiting_for_initial_queries_ = 0;
  bool has_rejected_candidate_ = false;
  BASIC::SE3 last_rejected_map_odom_;
  std::atomic<bool> manual_kiss_requested_{false};
  /// map_to_odom_/has_map_to_odom_ are shared between the worker thread and
  /// refineBackendMapToOdom() on the system/downstream thread; every access goes through
  /// map_to_odom_state_mutex_. All other state above is worker-thread-only.
  mutable std::mutex map_to_odom_state_mutex_;
  bool has_map_to_odom_ = false;
  BASIC::SE3 map_to_odom_;
  PriorMapCallback prior_map_callback_;
  adagraph::geometry::StaticTransform3d T_base_imu_;
  ResultCallback result_callback_;

  /// Accepted registrations queued for the backend. Bounded; overflow drops
  /// the oldest entry. Guarded by observation_mutex_ (lock holds only the
  /// queue edit, never the registration itself).
  mutable std::mutex observation_mutex_;
  std::deque<MapObservation> map_observations_;
  std::size_t overflow_dropped_observations_ = 0;

  /// Tunables with in-class defaults; not yet exposed through the ROS
  /// parameter loading path.
  float prior_map_grid_cell_size_ = 10.0f;
  double lost_kiss_retry_cooldown_sec_ = 5.0;
  int lost_kiss_retry_max_backoff_ = 6;
  int max_delta_gate_rejections_ = 10;
  adagraph::localization::RelocalizationConfig config_;
  KISSMatcherRelocalizer::Config kiss_config_;
  bool use_backend_localization_factor_ = false;
  std::string root_dir_;
  std::string fallback_map_file_;
};

}  // namespace LI2Sup

#endif
