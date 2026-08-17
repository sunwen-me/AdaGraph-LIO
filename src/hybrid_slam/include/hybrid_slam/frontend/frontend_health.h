#ifndef HYBRID_SLAM_FRONTEND_FRONTEND_HEALTH_H_
#define HYBRID_SLAM_FRONTEND_FRONTEND_HEALTH_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>

namespace hybrid_slam::frontend {

enum class FrontendHealthState {
  kTracking = 0,
  kSuspect,
  kLost,
  kRelocalizing,
  kRestarting,
  kVerifying,
  kRecovered,
};

struct FrontendHealthConfig {
  // Thresholds are deliberately opt-in until they are calibrated against the
  // deployed sensor/time-synchronization setup and fault-injection data.
  bool enabled = false;
  std::size_t suspect_bad_frames = 3;
  std::size_t lost_bad_frames = 10;
  std::size_t recover_good_frames = 5;
  // A freshly reset frontend may need a small number of frames to consume
  // the recovery seed before its normal Hessian/inlier diagnostics stabilize.
  std::size_t verification_grace_frames = 2;
  std::size_t recovery_cooldown_frames = 20;
  double min_inlier_ratio = 0.30;
  double max_condition_number = 10'000.0;
  double max_timestamp_gap_s = 1.0;
  // A complete LiDAR silence is a stronger signal than several degraded
  // frames. Once this wall-clock timeout expires, the watchdog enters LOST
  // directly; the next fresh LiDAR frame builds the recovery query.
  double lidar_dropout_lost_timeout_s = 2.0;
  std::size_t min_effective_rank = 4;
  std::size_t min_effective_points = 50;

  [[nodiscard]] std::optional<std::string> validate() const {
    if (suspect_bad_frames == 0 || lost_bad_frames == 0 ||
        recover_good_frames == 0 || recovery_cooldown_frames == 0) {
      return "frontend health frame thresholds must be positive";
    }
    if (suspect_bad_frames > lost_bad_frames) {
      return "frontend health suspect threshold must not exceed lost threshold";
    }
    if (!std::isfinite(min_inlier_ratio) || min_inlier_ratio < 0.0 ||
        min_inlier_ratio > 1.0) {
      return "frontend health minimum inlier ratio must be within [0, 1]";
    }
    if (!std::isfinite(max_condition_number) || max_condition_number <= 0.0) {
      return "frontend health maximum condition number must be positive";
    }
    if (!std::isfinite(max_timestamp_gap_s) || max_timestamp_gap_s <= 0.0) {
      return "frontend health maximum timestamp gap must be positive";
    }
    if (!std::isfinite(lidar_dropout_lost_timeout_s) ||
        lidar_dropout_lost_timeout_s <= 0.0) {
      return "frontend health LiDAR dropout timeout must be positive";
    }
    if (min_effective_rank == 0 || min_effective_rank > 6) {
      return "frontend health effective rank must be within [1, 6]";
    }
    if (min_effective_points == 0) {
      return "frontend health minimum effective points must be positive";
    }
    return std::nullopt;
  }
};

struct FrontendHealthObservation {
  bool has_information = false;
  bool imu_valid = false;
  bool timestamp_continuous = true;
  double inlier_ratio = 0.0;
  double condition_number = 0.0;
  std::size_t effective_rank = 0;
  std::size_t effective_point_count = 0;

  [[nodiscard]] bool good(const FrontendHealthConfig& config) const noexcept {
    return has_information && imu_valid && timestamp_continuous &&
           std::isfinite(inlier_ratio) &&
           inlier_ratio >= config.min_inlier_ratio &&
           std::isfinite(condition_number) &&
           condition_number <= config.max_condition_number &&
           effective_rank >= config.min_effective_rank &&
           effective_point_count >= config.min_effective_points;
  }
};

struct FrontendHealthDecision {
  FrontendHealthState state = FrontendHealthState::kTracking;
  bool transitioned = false;
  bool observation_good = false;
  bool local_map_write_enabled = true;
  bool global_keyframe_write_enabled = true;
  std::size_t bad_frames = 0;
  std::size_t good_frames = 0;
  std::size_t verification_grace_frames_remaining = 0;
  std::size_t cooldown_frames = 0;
};

// Small, instance-owned hysteresis state machine. It deliberately has no ROS,
// PCL or vendor dependency so it can be exercised independently of the
// realtime estimator.
class FrontendHealthMonitor {
 public:
  explicit FrontendHealthMonitor(FrontendHealthConfig config = {})
      : config_(std::move(config)) {}

  [[nodiscard]] const FrontendHealthConfig& config() const noexcept {
    return config_;
  }

  [[nodiscard]] FrontendHealthState state() const noexcept { return state_; }

  void reset() noexcept {
    state_ = FrontendHealthState::kTracking;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ = 0;
    cooldown_frames_ = config_.recovery_cooldown_frames;
  }

  [[nodiscard]] FrontendHealthDecision observe(
      const FrontendHealthObservation& observation) noexcept {
    FrontendHealthDecision decision;
    decision.observation_good =
        !config_.enabled || observation.good(config_);
    if (!config_.enabled && state_ != FrontendHealthState::kVerifying) {
      state_ = FrontendHealthState::kTracking;
      bad_frames_ = 0;
      good_frames_ = 0;
      cooldown_frames_ = config_.recovery_cooldown_frames;
      return makeDecision(decision);
    }

    // Verification is an explicit recovery lifecycle phase. It must remain
    // active even when normal health hysteresis is disabled, otherwise merely
    // enabling recovery would silently bypass the read-only window.
    if (!config_.enabled && state_ == FrontendHealthState::kVerifying) {
      ++good_frames_;
      bad_frames_ = 0;
      if (good_frames_ >= config_.recover_good_frames) {
        state_ = FrontendHealthState::kRecovered;
        cooldown_frames_ = config_.recovery_cooldown_frames;
      }
      decision.transitioned = true;
      return makeDecision(decision);
    }

    if (decision.observation_good) {
      ++good_frames_;
      bad_frames_ = 0;
    } else {
      ++bad_frames_;
      good_frames_ = 0;
    }

    const auto previous = state_;
    switch (state_) {
      case FrontendHealthState::kTracking:
        if (bad_frames_ >= config_.suspect_bad_frames) {
          state_ = FrontendHealthState::kSuspect;
        }
        break;
      case FrontendHealthState::kSuspect:
        if (bad_frames_ >= config_.lost_bad_frames) {
          state_ = FrontendHealthState::kLost;
        } else if (good_frames_ >= config_.recover_good_frames) {
          state_ = FrontendHealthState::kTracking;
        }
        break;
      case FrontendHealthState::kVerifying:
        if (!decision.observation_good &&
            verification_grace_frames_remaining_ > 0) {
          --verification_grace_frames_remaining_;
          bad_frames_ = 0;
          good_frames_ = 0;
        } else if (!decision.observation_good) {
          state_ = FrontendHealthState::kLost;
        } else if (good_frames_ >= config_.recover_good_frames) {
          state_ = FrontendHealthState::kRecovered;
          cooldown_frames_ = config_.recovery_cooldown_frames;
        }
        break;
      case FrontendHealthState::kRecovered:
        if (!decision.observation_good) {
          state_ = FrontendHealthState::kSuspect;
        }
        break;
      case FrontendHealthState::kLost:
      case FrontendHealthState::kRelocalizing:
      case FrontendHealthState::kRestarting:
        // Recovery orchestration owns these transitions explicitly.
        break;
    }
    decision.transitioned = previous != state_;
    return makeDecision(decision);
  }

  void beginRelocalizing() noexcept {
    state_ = FrontendHealthState::kRelocalizing;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ = 0;
    cooldown_frames_ = 0;
  }

  void beginRestarting() noexcept {
    state_ = FrontendHealthState::kRestarting;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ = 0;
    cooldown_frames_ = 0;
  }

  void beginVerifying() noexcept {
    state_ = FrontendHealthState::kVerifying;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ =
        config_.verification_grace_frames;
    cooldown_frames_ = 0;
  }

  void markRecovered() noexcept {
    state_ = FrontendHealthState::kRecovered;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ = 0;
    cooldown_frames_ = config_.recovery_cooldown_frames;
  }

  void markLost() noexcept {
    state_ = FrontendHealthState::kLost;
    bad_frames_ = 0;
    good_frames_ = 0;
    verification_grace_frames_remaining_ = 0;
    cooldown_frames_ = 0;
  }

 private:
  [[nodiscard]] FrontendHealthDecision makeDecision(
      FrontendHealthDecision decision) noexcept {
    decision.state = state_;
    // The online layer is provisional while verifying. It is explicitly
    // cleared if verification fails; the read-only prior layer is never
    // written. Global keyframes remain blocked until recovery is stable.
    decision.local_map_write_enabled =
        state_ == FrontendHealthState::kTracking ||
        state_ == FrontendHealthState::kVerifying ||
        state_ == FrontendHealthState::kRecovered;
    decision.global_keyframe_write_enabled =
        state_ == FrontendHealthState::kTracking ||
        (state_ == FrontendHealthState::kRecovered &&
         cooldown_frames_ == 0);
    decision.bad_frames = bad_frames_;
    decision.good_frames = good_frames_;
    decision.verification_grace_frames_remaining =
        verification_grace_frames_remaining_;
    decision.cooldown_frames = cooldown_frames_;
    if (state_ == FrontendHealthState::kRecovered && cooldown_frames_ > 0) {
      --cooldown_frames_;
    }
    return decision;
  }

  FrontendHealthConfig config_;
  FrontendHealthState state_ = FrontendHealthState::kTracking;
  std::size_t bad_frames_ = 0;
  std::size_t good_frames_ = 0;
  std::size_t verification_grace_frames_remaining_ = 0;
  std::size_t cooldown_frames_ = 0;
};

}  // namespace hybrid_slam::frontend

#endif  // HYBRID_SLAM_FRONTEND_FRONTEND_HEALTH_H_
