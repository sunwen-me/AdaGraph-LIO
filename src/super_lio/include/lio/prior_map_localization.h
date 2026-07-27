#ifndef PRIOR_MAP_LOCALIZATION_H_
#define PRIOR_MAP_LOCALIZATION_H_

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <atomic>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "lio/adagraph_types.h"
#include "lio/kiss_matcher_relocalizer.h"

namespace LI2Sup {

class PriorMapLocalization {
public:
  using MapToOdomCallback = std::function<void(const BASIC::SE3&)>;
  using PriorMapCallback = std::function<void(const BASIC::CloudPtr&)>;

  PriorMapLocalization();
  ~PriorMapLocalization();

  bool init(MapToOdomCallback map_to_odom_callback,
            PriorMapCallback prior_map_callback = {});
  void shutdown();
  bool enabled() const { return enabled_; }

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
  bool submitBackendMapToOdom(const BASIC::SE3& candidate,
                              const BASIC::SE3& odom_lidar);

private:
  enum class State {
    UNINITIALIZED,
    TRACKING,
    LOST
  };

  struct Frame {
    double timestamp = 0.0;
    BASIC::CloudPtr cloud;
    BASIC::SE3 odom_lidar;
  };

  struct RegistrationResult {
    bool converged = false;
    double fitness = std::numeric_limits<double>::infinity();
    BASIC::SE3 map_odom;
  };

  std::string resolveMapPath() const;
  bool loadPriorMap();
  void buildPriorMapGrid();
  void workerLoop();
  void process(Frame&& frame);
  void appendLocalFrame(const Frame& frame);
  BASIC::CloudPtr buildLocalSubmap() const;
  BASIC::CloudPtr cropPriorMap(const BASIC::SE3& predicted_map_lidar) const;
  BASIC::CloudPtr downsampleCloud(const BASIC::CloudPtr& cloud, float leaf_size) const;
  BASIC::CloudPtr limitCloudSize(const BASIC::CloudPtr& cloud, int max_points) const;
  RegistrationResult runRegistration(const BASIC::CloudPtr& source_odom,
                                     const BASIC::CloudPtr& target_map,
                                     const BASIC::SE3& initial_map_odom) const;
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

  bool enabled_ = false;
  bool stop_worker_ = false;
  bool worker_busy_ = false;
  bool has_pending_frame_ = false;
  std::size_t submitted_frames_ = 0;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<Frame> pending_frame_;
  std::thread worker_;

  BASIC::CloudPtr prior_map_;
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> prior_map_grid_;
  std::deque<BASIC::CloudPtr> local_frames_odom_;
  KISSMatcherRelocalizer kiss_relocalizer_;
  State state_ = State::UNINITIALIZED;
  bool startup_kiss_attempted_ = false;
  bool lost_kiss_attempted_ = false;
  double last_lost_kiss_time_ = 0.0;
  int lost_kiss_failures_ = 0;
  int delta_gate_rejections_ = 0;
  bool has_rejected_candidate_ = false;
  BASIC::SE3 last_rejected_map_odom_;
  std::atomic<bool> manual_kiss_requested_{false};
  /// map_to_odom_/has_map_to_odom_ are shared between the worker thread and
  /// submitBackendMapToOdom() on the LIO thread; every access goes through
  /// map_to_odom_state_mutex_. All other state above is worker-thread-only.
  mutable std::mutex map_to_odom_state_mutex_;
  bool has_map_to_odom_ = false;
  BASIC::SE3 map_to_odom_;
  MapToOdomCallback map_to_odom_callback_;
  PriorMapCallback prior_map_callback_;

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
};

}  // namespace LI2Sup

#endif
