#ifndef LI2SUP_SYSTEM_FRAME_HEALTH_GATE_H_
#define LI2SUP_SYSTEM_FRAME_HEALTH_GATE_H_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>

#include "hybrid_slam/frontend/frontend_health.h"

namespace LI2Sup {

const char* FrontendHealthStateName(
    hybrid_slam::frontend::FrontendHealthState state) noexcept;

// Frontend health state and the rules that decide when tracking is lost,
// grouped out of PointLIOPipeline where they were interleaved with the recovery
// transaction and the graph identity.
//
// THIS CLASS HAS NO LOCK OF ITS OWN. Every method requires the caller to already
// hold the pipeline's recovery transition mutex. That is not an oversight: a
// recovery commit has to change health state and graph identity atomically, or a
// frame could observe a new trajectory id against stale health and write a
// keyframe into the wrong trajectory. Giving this class its own mutex would
// split that transaction and create two paths that take the two locks in
// opposite orders - the commit path locking pipeline then gate, the observation
// path locking gate then reaching the frontend.
//
// The watchdog's obligation is therefore enforced at the caller: it must attempt
// the recovery mutex with try_to_lock and give up rather than wait, because the
// ROS timer that drives it shares a mutually exclusive callback group with
// sensor ingress. Blocking there stalls IMU and LiDAR delivery behind whatever
// the frontend worker is doing, including a backend submission waiting on queue
// capacity.
//
// Extracting observation itself was deliberately left out: turning a
// PointLIOOutputFrame into a FrontendHealthObservation needs the vendor frame's
// Hessian, gradient and matching-point layout, so that belongs to the Point-LIO
// adapter. This class consumes the already-typed observation.
class FrameHealthGate {
 public:
  // Invoked while the gate's lock is held, so each must be cheap and must never
  // wait for backend, localization or registration work.
  struct FrontendControls {
    std::function<void(bool)> set_local_map_write_enabled;
    std::function<void(bool)> set_recovery_query_mode;
    std::function<void()> clear_online_local_map;
  };

  // Carried only so a verification rollback or commit can name the trajectory
  // it applies to. The gate never interprets these values.
  struct RecoveryEvidence {
    std::uint64_t session = 0;
    std::uint64_t trajectory = 0;
    double timestamp_seconds = 0.0;
    bool first_durable_keyframe_pending = false;
  };

  struct TimeoutOutcome {
    hybrid_slam::frontend::FrontendHealthDecision decision;
    hybrid_slam::frontend::FrontendHealthState previous_state{};
    double elapsed_seconds = 0.0;
    bool forced_loss = false;
  };

  FrameHealthGate(hybrid_slam::frontend::FrontendHealthConfig config,
                  FrontendControls controls);

  FrameHealthGate(const FrameHealthGate&) = delete;
  FrameHealthGate& operator=(const FrameHealthGate&) = delete;

  [[nodiscard]] const hybrid_slam::frontend::FrontendHealthConfig& config()
      const noexcept {
    return config_;
  }

  // Evaluates one frame's observation, applies the decision to the frontend and
  // returns it. timestamp_seconds drives the continuity check.
  [[nodiscard]] hybrid_slam::frontend::FrontendHealthDecision observeFrame(
      hybrid_slam::frontend::FrontendHealthObservation observation,
      double timestamp_seconds);

  // Called from the ROS watchdog after it has acquired the recovery mutex.
  // Returns nullopt when the elapsed gap is still within budget or a report was
  // already made for this dropout.
  [[nodiscard]] std::optional<TimeoutOutcome> observeTimeout();

  // Counts watchdog ticks that declined to wait for the recovery mutex. Owned
  // here because this is the only consumer of that evidence.
  void noteTimeoutObservationSkipped() noexcept {
    timeout_observations_skipped_.fetch_add(1, std::memory_order_relaxed);
  }

  // Establishes the wall-clock baseline without judging continuity. Used after a
  // recovery commit, where the first frame may legitimately be separated from
  // the relocalization query by the dropout that triggered recovery.
  void resetTimingBaseline();
  void beginVerifying();

  void setRecoveryEvidence(std::optional<RecoveryEvidence> evidence);
  [[nodiscard]] std::optional<RecoveryEvidence> recoveryEvidence() const;
  void clearFirstDurableKeyframePending();

  [[nodiscard]] std::size_t retryCooldownFrames() const;
  void decrementRetryCooldown();
  void clearRetryCooldown();
  void beginRetryCooldown();

  [[nodiscard]] std::uint64_t timeoutObservationsSkipped() const noexcept {
    return timeout_observations_skipped_.load(std::memory_order_relaxed);
  }

  [[nodiscard]] hybrid_slam::frontend::FrontendHealthState state() const;

 private:
  void applyDecisionLocked(
      hybrid_slam::frontend::FrontendHealthState previous_state,
      const hybrid_slam::frontend::FrontendHealthDecision& decision);

  const hybrid_slam::frontend::FrontendHealthConfig config_;
  const FrontendControls controls_;

  hybrid_slam::frontend::FrontendHealthMonitor monitor_;
  std::optional<double> last_timestamp_seconds_;
  std::optional<std::chrono::steady_clock::time_point> last_wall_time_;
  std::optional<std::chrono::steady_clock::time_point>
      last_timeout_observation_;
  std::optional<RecoveryEvidence> recovery_evidence_;
  std::size_t retry_cooldown_frames_ = 0;
  std::atomic<std::uint64_t> timeout_observations_skipped_{0};
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_FRAME_HEALTH_GATE_H_
