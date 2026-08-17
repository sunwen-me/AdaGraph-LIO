#include <gtest/gtest.h>

#include "hybrid_slam/frontend/frontend_health.h"

namespace {

using hybrid_slam::frontend::FrontendHealthConfig;
using hybrid_slam::frontend::FrontendHealthMonitor;
using hybrid_slam::frontend::FrontendHealthObservation;
using hybrid_slam::frontend::FrontendHealthState;

FrontendHealthObservation Good() {
  FrontendHealthObservation observation;
  observation.has_information = true;
  observation.imu_valid = true;
  observation.inlier_ratio = 0.8;
  observation.condition_number = 100.0;
  observation.effective_rank = 6;
  observation.effective_point_count = 100;
  return observation;
}

FrontendHealthObservation Bad() {
  auto observation = Good();
  observation.inlier_ratio = 0.05;
  observation.effective_rank = 2;
  return observation;
}

TEST(FrontendHealth, UsesHysteresisAndDisablesWritesAfterLoss) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.suspect_bad_frames = 2;
  config.lost_bad_frames = 4;
  config.recover_good_frames = 2;
  config.recovery_cooldown_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);

  EXPECT_EQ(monitor.observe(Bad()).state,
            FrontendHealthState::kTracking);
  auto decision = monitor.observe(Bad());
  EXPECT_EQ(decision.state, FrontendHealthState::kSuspect);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);

  EXPECT_EQ(monitor.observe(Bad()).state,
            FrontendHealthState::kSuspect);
  decision = monitor.observe(Bad());
  EXPECT_EQ(decision.state, FrontendHealthState::kLost);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);
}

TEST(FrontendHealth, RequiresStableVerificationAndCooldownBeforeCommit) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.recover_good_frames = 2;
  config.recovery_cooldown_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);
  monitor.markLost();
  monitor.beginVerifying();

  EXPECT_EQ(monitor.observe(Good()).state,
            FrontendHealthState::kVerifying);
  auto decision = monitor.observe(Good());
  EXPECT_EQ(decision.state, FrontendHealthState::kRecovered);
  EXPECT_TRUE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);

  EXPECT_FALSE(monitor.observe(Good()).global_keyframe_write_enabled);
  EXPECT_TRUE(monitor.observe(Good()).global_keyframe_write_enabled);
}

TEST(FrontendHealth, VerificationRemainsActiveWhenNormalHealthIsDisabled) {
  FrontendHealthConfig config;
  config.enabled = false;
  config.recover_good_frames = 2;
  config.recovery_cooldown_frames = 2;
  FrontendHealthMonitor monitor(config);
  monitor.beginVerifying();

  auto decision = monitor.observe(Good());
  EXPECT_EQ(decision.state, FrontendHealthState::kVerifying);
  EXPECT_TRUE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);

  decision = monitor.observe(Good());
  EXPECT_EQ(decision.state, FrontendHealthState::kRecovered);
  EXPECT_TRUE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);
}

TEST(FrontendHealth, VerificationFailureDisablesProvisionalWrites) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.verification_grace_frames = 0;
  config.lost_bad_frames = 1;
  config.recover_good_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);
  monitor.beginVerifying();

  const auto decision = monitor.observe(Bad());
  EXPECT_EQ(decision.state, FrontendHealthState::kLost);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);
}

TEST(FrontendHealth, VerificationGraceAllowsSeedWarmupButNotPersistentFailure) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.verification_grace_frames = 1;
  config.recover_good_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);
  monitor.beginVerifying();

  const auto first = monitor.observe(Bad());
  EXPECT_EQ(first.state, FrontendHealthState::kVerifying);
  EXPECT_EQ(first.verification_grace_frames_remaining, 0U);
  EXPECT_EQ(monitor.observe(Bad()).state,
            FrontendHealthState::kLost);
}

TEST(FrontendHealth, ContinuousImuFaultFreezesWritesWithHysteresis) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.suspect_bad_frames = 1;
  config.lost_bad_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);

  ASSERT_EQ(monitor.observe(Good()).state,
            FrontendHealthState::kTracking);
  auto imu_fault = Good();
  imu_fault.imu_valid = false;
  auto decision = monitor.observe(imu_fault);
  EXPECT_EQ(decision.state, FrontendHealthState::kSuspect);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);

  decision = monitor.observe(imu_fault);
  EXPECT_EQ(decision.state, FrontendHealthState::kLost);
  EXPECT_FALSE(decision.local_map_write_enabled);
  EXPECT_FALSE(decision.global_keyframe_write_enabled);
}

TEST(FrontendHealth, TimestampDiscontinuityIsAHealthFault) {
  FrontendHealthConfig config;
  config.enabled = true;
  config.suspect_bad_frames = 1;
  config.lost_bad_frames = 2;
  config.min_effective_points = 1;
  FrontendHealthMonitor monitor(config);

  ASSERT_EQ(monitor.observe(Good()).state,
            FrontendHealthState::kTracking);
  auto discontinuity = Good();
  discontinuity.timestamp_continuous = false;
  EXPECT_EQ(monitor.observe(discontinuity).state,
            FrontendHealthState::kSuspect);
  EXPECT_EQ(monitor.observe(discontinuity).state,
            FrontendHealthState::kLost);
}

TEST(FrontendHealth, DisabledMonitorKeepsNormalAdmission) {
  FrontendHealthConfig config;
  config.enabled = false;
  FrontendHealthMonitor monitor(config);

  const auto decision = monitor.observe(Bad());
  EXPECT_EQ(decision.state, FrontendHealthState::kTracking);
  EXPECT_TRUE(decision.local_map_write_enabled);
  EXPECT_TRUE(decision.global_keyframe_write_enabled);
}

TEST(FrontendHealth, RejectsInvalidConfiguration) {
  FrontendHealthConfig config;
  config.suspect_bad_frames = 5;
  config.lost_bad_frames = 2;
  EXPECT_TRUE(config.validate().has_value());
}

TEST(FrontendHealth, RejectsInvalidLidarDropoutTimeout) {
  FrontendHealthConfig config;
  config.lidar_dropout_lost_timeout_s = 0.0;
  EXPECT_TRUE(config.validate().has_value());
}

}  // namespace
