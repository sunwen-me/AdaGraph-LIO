#include "lio/system/frame_health_gate.h"

#include <optional>
#include <vector>

#include <gtest/gtest.h>

namespace {

using LI2Sup::FrameHealthGate;
using hybrid_slam::frontend::FrontendHealthConfig;
using hybrid_slam::frontend::FrontendHealthObservation;
using hybrid_slam::frontend::FrontendHealthState;

FrontendHealthConfig MakeConfig() {
  FrontendHealthConfig config;
  config.enabled = true;
  config.suspect_bad_frames = 1;
  config.lost_bad_frames = 2;
  config.recover_good_frames = 2;
  config.recovery_cooldown_frames = 3;
  config.max_timestamp_gap_s = 0.5;
  config.lidar_dropout_lost_timeout_s = 1.0;
  return config;
}

FrontendHealthObservation GoodObservation() {
  FrontendHealthObservation observation;
  observation.has_information = true;
  observation.imu_valid = true;
  observation.timestamp_continuous = true;
  observation.inlier_ratio = 1.0;
  observation.condition_number = 1.0;
  observation.effective_rank = 6;
  observation.effective_point_count = 10000;
  return observation;
}

FrontendHealthObservation BadObservation() {
  FrontendHealthObservation observation = GoodObservation();
  observation.has_information = false;
  observation.effective_point_count = 0;
  return observation;
}

// Records what the gate asked of the frontend, which is the only externally
// visible effect of a health decision.
struct ControlLog {
  std::vector<bool> local_map_write;
  std::vector<bool> recovery_query_mode;
  int clear_online_calls = 0;

  FrameHealthGate::FrontendControls controls() {
    return {
        [this](bool enabled) { local_map_write.push_back(enabled); },
        [this](bool enabled) { recovery_query_mode.push_back(enabled); },
        [this] { ++clear_online_calls; },
    };
  }
};

TEST(FrameHealthGate, TracksHealthyFramesAndKeepsWritesEnabled) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  double timestamp = 100.0;
  for (int index = 0; index < 3; ++index, timestamp += 0.1) {
    const auto decision = gate.observeFrame(GoodObservation(), timestamp);
    EXPECT_EQ(decision.state, FrontendHealthState::kTracking);
    EXPECT_TRUE(decision.local_map_write_enabled);
  }
  EXPECT_EQ(gate.state(), FrontendHealthState::kTracking);
  ASSERT_FALSE(log.local_map_write.empty());
  EXPECT_TRUE(log.local_map_write.back());
  // Recovery query mode belongs to LOST only; leaking it on while tracking would
  // make the frontend build queries nobody consumes.
  ASSERT_FALSE(log.recovery_query_mode.empty());
  EXPECT_FALSE(log.recovery_query_mode.back());
}

TEST(FrameHealthGate, ReachesLostAndTurnsOnRecoveryQueries) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  gate.observeFrame(BadObservation(), 100.1);
  const auto decision = gate.observeFrame(BadObservation(), 100.2);
  EXPECT_EQ(decision.state, FrontendHealthState::kLost);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_TRUE(log.recovery_query_mode.back());
}

TEST(FrameHealthGate, JudgesTimestampContinuityItself) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  // A gap beyond max_timestamp_gap_s is a fault even when the observation
  // itself claims continuity: the gate owns the previous timestamp, so the
  // caller cannot be trusted to have compared them.
  auto observation = GoodObservation();
  observation.timestamp_continuous = true;
  const auto decision = gate.observeFrame(observation, 105.0);
  EXPECT_FALSE(decision.observation_good);
}

TEST(FrameHealthGate, TreatsNonFiniteTimestampsAsDiscontinuous) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  const auto decision = gate.observeFrame(
      GoodObservation(), std::numeric_limits<double>::quiet_NaN());
  EXPECT_FALSE(decision.observation_good);
}

TEST(FrameHealthGate, RejectsBackwardsTimestamps) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  // Replayed or reordered frames must not look continuous.
  const auto decision = gate.observeFrame(GoodObservation(), 99.9);
  EXPECT_FALSE(decision.observation_good);
}

TEST(FrameHealthGate, ReportsNoTimeoutBeforeAnyFrame) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  // Without a baseline there is no elapsed gap to judge, so an idle system
  // must not be declared lost before it ever tracked.
  EXPECT_FALSE(gate.observeTimeout().has_value());
}

TEST(FrameHealthGate, CountsSkippedWatchdogTicks) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  EXPECT_EQ(gate.timeoutObservationsSkipped(), 0U);
  // The caller reports a tick it declined to wait for; the count is the only
  // evidence that frame processing is outlasting the health timer.
  gate.noteTimeoutObservationSkipped();
  gate.noteTimeoutObservationSkipped();
  EXPECT_EQ(gate.timeoutObservationsSkipped(), 2U);
}

TEST(FrameHealthGate, VerificationFailureClearsProvisionalMapAndStartsCooldown) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  gate.beginVerifying();
  EXPECT_EQ(gate.state(), FrontendHealthState::kVerifying);
  gate.setRecoveryEvidence(FrameHealthGate::RecoveryEvidence{1, 4, 100.0, true});

  double timestamp = 100.1;
  for (int index = 0; index < 4 && gate.state() != FrontendHealthState::kLost;
       ++index, timestamp += 0.1) {
    gate.observeFrame(BadObservation(), timestamp);
  }
  ASSERT_EQ(gate.state(), FrontendHealthState::kLost);
  // A failed verification must roll back the provisional layer and hold off the
  // next expensive candidate, otherwise recovery retries once per frame.
  EXPECT_EQ(log.clear_online_calls, 1);
  EXPECT_EQ(gate.retryCooldownFrames(),
            MakeConfig().recovery_cooldown_frames);
}

TEST(FrameHealthGate, CooldownDecrementsAndFloorsAtZero) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.beginRetryCooldown();
  EXPECT_EQ(gate.retryCooldownFrames(), 3U);
  gate.decrementRetryCooldown();
  EXPECT_EQ(gate.retryCooldownFrames(), 2U);
  gate.clearRetryCooldown();
  EXPECT_EQ(gate.retryCooldownFrames(), 0U);
  // Unsigned underflow here would turn a cleared cooldown into a very long one.
  gate.decrementRetryCooldown();
  EXPECT_EQ(gate.retryCooldownFrames(), 0U);
}

TEST(FrameHealthGate, CarriesRecoveryEvidenceWithoutInterpretingIt) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  EXPECT_FALSE(gate.recoveryEvidence().has_value());
  gate.setRecoveryEvidence(
      FrameHealthGate::RecoveryEvidence{7, 9, 123.5, true});
  auto evidence = gate.recoveryEvidence();
  ASSERT_TRUE(evidence.has_value());
  EXPECT_EQ(evidence->session, 7U);
  EXPECT_EQ(evidence->trajectory, 9U);
  EXPECT_TRUE(evidence->first_durable_keyframe_pending);

  gate.clearFirstDurableKeyframePending();
  evidence = gate.recoveryEvidence();
  ASSERT_TRUE(evidence.has_value());
  EXPECT_FALSE(evidence->first_durable_keyframe_pending);
  // Clearing the pending flag must not discard the identity it belongs to.
  EXPECT_EQ(evidence->trajectory, 9U);
}

TEST(FrameHealthGate, TimingBaselineResetDropsStaleContinuity) {
  ControlLog log;
  FrameHealthGate gate(MakeConfig(), log.controls());
  gate.observeFrame(GoodObservation(), 100.0);
  // After a recovery commit the next frame may legitimately sit far past the
  // dropout, so the first post-reset frame must not be judged discontinuous.
  gate.resetTimingBaseline();
  const auto decision = gate.observeFrame(GoodObservation(), 500.0);
  EXPECT_TRUE(decision.observation_good);
}

TEST(FrameHealthGate, DisabledConfigKeepsWritesEnabled) {
  auto config = MakeConfig();
  config.enabled = false;
  ControlLog log;
  FrameHealthGate gate(config, log.controls());
  const auto decision = gate.observeFrame(BadObservation(), 100.0);
  EXPECT_TRUE(decision.local_map_write_enabled);
  EXPECT_EQ(decision.state, FrontendHealthState::kTracking);
  // A disabled monitor must not arm the dropout watchdog either.
  EXPECT_FALSE(gate.observeTimeout().has_value());
}

}  // namespace
