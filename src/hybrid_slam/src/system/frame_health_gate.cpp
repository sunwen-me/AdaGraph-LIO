#include "lio/system/frame_health_gate.h"

#include <cmath>
#include <limits>
#include <utility>

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {

FrameHealthGate::FrameHealthGate(
    hybrid_slam::frontend::FrontendHealthConfig config,
    FrontendControls controls)
    : config_(std::move(config)),
      controls_(std::move(controls)),
      monitor_(config_) {}

hybrid_slam::frontend::FrontendHealthState FrameHealthGate::state() const {
  return monitor_.state();
}

hybrid_slam::frontend::FrontendHealthDecision FrameHealthGate::observeFrame(
    hybrid_slam::frontend::FrontendHealthObservation observation,
    const double timestamp_seconds) {

  // Continuity is judged here rather than by the caller so the previous
  // timestamp and the decision that consumes it stay under one lock.
  observation.timestamp_continuous = std::isfinite(timestamp_seconds);
  double timestamp_delta = std::numeric_limits<double>::quiet_NaN();
  const auto previous_timestamp = last_timestamp_seconds_;
  if (observation.timestamp_continuous && last_timestamp_seconds_) {
    timestamp_delta = timestamp_seconds - *last_timestamp_seconds_;
    observation.timestamp_continuous =
        timestamp_delta > 0.0 &&
        timestamp_delta <= config_.max_timestamp_gap_s;
  }
  if (std::isfinite(timestamp_seconds)) {
    last_timestamp_seconds_ = timestamp_seconds;
  } else {
    last_timestamp_seconds_.reset();
  }
  last_wall_time_ = std::chrono::steady_clock::now();

  const auto previous_state = monitor_.state();
  const auto decision = monitor_.observe(observation);

  if (previous_state ==
      hybrid_slam::frontend::FrontendHealthState::kVerifying) {
    LOG(INFO) << " ---> [PointLIORecovery]: verification_frame timestamp="
              << timestamp_seconds << ", good=" << decision.observation_good
              << ", resulting_state=" << FrontendHealthStateName(decision.state)
              << ", good_frames=" << decision.good_frames
              << ", bad_frames=" << decision.bad_frames
              << ", grace_remaining="
              << decision.verification_grace_frames_remaining
              << ", effective_points=" << observation.effective_point_count
              << ", inlier_ratio=" << observation.inlier_ratio
              << ", rank=" << observation.effective_rank
              << ", condition_number=" << observation.condition_number;
    if (decision.state == hybrid_slam::frontend::FrontendHealthState::kLost) {
      LOG(WARNING) << " ---> [FrontendHealth]: verification failed source=frame"
                   << ", timestamp=" << timestamp_seconds
                   << ", previous_timestamp="
                   << (previous_timestamp
                           ? *previous_timestamp
                           : std::numeric_limits<double>::quiet_NaN())
                   << ", timestamp_delta=" << timestamp_delta
                   << ", effective_points="
                   << observation.effective_point_count
                   << ", inlier_ratio=" << observation.inlier_ratio
                   << ", rank=" << observation.effective_rank
                   << ", condition_number=" << observation.condition_number
                   << hybrid_slam::logging::kReset;
    }
  }

  applyDecisionLocked(previous_state, decision);
  return decision;
}

std::optional<FrameHealthGate::TimeoutOutcome>
FrameHealthGate::observeTimeout() {
  if (!config_.enabled || !last_wall_time_) {
    return std::nullopt;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto elapsed =
      std::chrono::duration<double>(now - *last_wall_time_).count();
  if (!std::isfinite(elapsed) || elapsed < config_.max_timestamp_gap_s) {
    return std::nullopt;
  }
  if (last_timeout_observation_ &&
      std::chrono::duration<double>(now - *last_timeout_observation_).count() <
          config_.max_timestamp_gap_s) {
    return std::nullopt;
  }
  last_timeout_observation_ = now;

  const auto previous_state = monitor_.state();
  hybrid_slam::frontend::FrontendHealthObservation observation;
  observation.has_information = false;
  observation.imu_valid = false;
  observation.timestamp_continuous = false;
  auto decision = monitor_.observe(observation);

  const bool can_force_loss =
      previous_state == hybrid_slam::frontend::FrontendHealthState::kTracking ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kSuspect ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kRecovered ||
      previous_state == hybrid_slam::frontend::FrontendHealthState::kVerifying;
  bool forced_loss = false;
  if (can_force_loss && elapsed >= config_.lidar_dropout_lost_timeout_s &&
      decision.state != hybrid_slam::frontend::FrontendHealthState::kLost) {
    monitor_.markLost();
    decision = monitor_.observe(observation);
    decision.transitioned = true;
    forced_loss = true;
    LOG(WARNING) << " ---> [FrontendHealth]: LiDAR dropout forced LOST"
                 << " elapsed_s=" << elapsed << ", timeout_s="
                 << config_.lidar_dropout_lost_timeout_s
                 << hybrid_slam::logging::kReset;
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost) {
    LOG(WARNING) << " ---> [FrontendHealth]: verification failed "
                 << "source=watchdog, elapsed_s=" << elapsed
                 << ", max_gap_s=" << config_.max_timestamp_gap_s
                 << hybrid_slam::logging::kReset;
  }

  applyDecisionLocked(previous_state, decision);
  return TimeoutOutcome{decision, previous_state, elapsed, forced_loss};
}

void FrameHealthGate::applyDecisionLocked(
    const hybrid_slam::frontend::FrontendHealthState previous_state,
    const hybrid_slam::frontend::FrontendHealthDecision& decision) {
  if (controls_.set_local_map_write_enabled) {
    controls_.set_local_map_write_enabled(decision.local_map_write_enabled);
  }
  if (controls_.set_recovery_query_mode) {
    controls_.set_recovery_query_mode(
        decision.state == hybrid_slam::frontend::FrontendHealthState::kLost);
  }

  const bool verification_failed =
      previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state == hybrid_slam::frontend::FrontendHealthState::kLost;
  if (verification_failed) {
    if (controls_.clear_online_local_map) {
      controls_.clear_online_local_map();
    }
    // A failed verification is a recovery attempt failure, not an ordinary
    // tracking frame. Wait for a materially newer query before submitting
    // another expensive candidate.
    retry_cooldown_frames_ = config_.recovery_cooldown_frames;
    if (recovery_evidence_) {
      LOG(WARNING) << " ---> [PointLIORecovery]: verification_rollback "
                   << "trajectory=" << recovery_evidence_->trajectory
                   << ", first_durable_keyframe_pending="
                   << recovery_evidence_->first_durable_keyframe_pending;
    }
  }
  if (previous_state ==
          hybrid_slam::frontend::FrontendHealthState::kVerifying &&
      decision.state ==
          hybrid_slam::frontend::FrontendHealthState::kRecovered &&
      recovery_evidence_) {
    LOG(INFO) << " ---> [PointLIORecovery]: verification_committed trajectory="
              << recovery_evidence_->trajectory
              << ", session=" << recovery_evidence_->session
              << ", timestamp=" << recovery_evidence_->timestamp_seconds;
  }
  if (decision.transitioned) {
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [FrontendHealth]: "
              << FrontendHealthStateName(previous_state) << " -> "
              << FrontendHealthStateName(decision.state)
              << ", good/bad/cooldown=" << decision.good_frames << "/"
              << decision.bad_frames << "/" << decision.cooldown_frames
              << ", local_map_write=" << decision.local_map_write_enabled
              << ", global_keyframe_write="
              << decision.global_keyframe_write_enabled
              << hybrid_slam::logging::kReset;
  }
}

void FrameHealthGate::resetTimingBaseline() {
  last_wall_time_ = std::chrono::steady_clock::now();
  last_timeout_observation_.reset();
  last_timestamp_seconds_.reset();
}

void FrameHealthGate::beginVerifying() {
  monitor_.beginVerifying();
}

void FrameHealthGate::setRecoveryEvidence(
    std::optional<RecoveryEvidence> evidence) {
  recovery_evidence_ = std::move(evidence);
}

std::optional<FrameHealthGate::RecoveryEvidence>
FrameHealthGate::recoveryEvidence() const {
  return recovery_evidence_;
}

void FrameHealthGate::clearFirstDurableKeyframePending() {
  if (recovery_evidence_) {
    recovery_evidence_->first_durable_keyframe_pending = false;
  }
}

std::size_t FrameHealthGate::retryCooldownFrames() const {
  return retry_cooldown_frames_;
}

void FrameHealthGate::decrementRetryCooldown() {
  if (retry_cooldown_frames_ > 0) {
    --retry_cooldown_frames_;
  }
}

void FrameHealthGate::clearRetryCooldown() {
  retry_cooldown_frames_ = 0;
}

void FrameHealthGate::beginRetryCooldown() {
  retry_cooldown_frames_ = config_.recovery_cooldown_frames;
}

}  // namespace LI2Sup
