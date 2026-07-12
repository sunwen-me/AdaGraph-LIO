#ifndef PRIOR_MAP_LOCALIZATION_H_
#define PRIOR_MAP_LOCALIZATION_H_

#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <atomic>

#include "basic/Manifold.h"
#include "basic/alias.h"
#include "lio/kiss_matcher_relocalizer.h"

namespace LI2Sup {

class PriorMapLocalization {
public:
  using MapToOdomCallback = std::function<void(const BASIC::SE3&)>;

  PriorMapLocalization();
  ~PriorMapLocalization();

  bool init(MapToOdomCallback map_to_odom_callback);
  void shutdown();
  bool enabled() const { return enabled_; }

  void submit(double timestamp,
              const BASIC::CloudPtr& registered_cloud,
              const BASIC::SE3& odom_lidar);
  void requestGlobalRelocalization();

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
  void workerLoop();
  void process(Frame&& frame);
  void appendLocalFrame(const Frame& frame);
  BASIC::CloudPtr buildLocalSubmap() const;
  BASIC::CloudPtr cropPriorMap(const BASIC::SE3& predicted_map_lidar) const;
  BASIC::CloudPtr downsampleCloud(const BASIC::CloudPtr& cloud, float leaf_size) const;
  BASIC::CloudPtr limitCloudSize(const BASIC::CloudPtr& cloud, int max_points) const;
  RegistrationResult runRegistration(const BASIC::CloudPtr& source_odom,
                                     const BASIC::CloudPtr& target_map,
                                     const BASIC::SE3& initial_map_odom,
                                     const BASIC::SE3& odom_lidar) const;
  bool passQualityGate(const RegistrationResult& result) const;
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
  std::deque<BASIC::CloudPtr> local_frames_odom_;
  KISSMatcherRelocalizer kiss_relocalizer_;
  State state_ = State::UNINITIALIZED;
  bool startup_kiss_attempted_ = false;
  bool lost_kiss_attempted_ = false;
  std::atomic<bool> manual_kiss_requested_{false};
  bool has_map_to_odom_ = false;
  BASIC::SE3 map_to_odom_;
  MapToOdomCallback map_to_odom_callback_;
};

}  // namespace LI2Sup

#endif
