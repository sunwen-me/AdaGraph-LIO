#include <limits>

#include <gtest/gtest.h>

#include "ros/odometry_rate_limiter.h"

namespace {

TEST(OdometryRateLimiter, UnlimitedRatePublishesEveryFiniteTimestamp) {
  LI2Sup::OdometryRateLimiter limiter;

  EXPECT_TRUE(limiter.shouldPublish(1.0));
  EXPECT_TRUE(limiter.shouldPublish(1.000001));
  EXPECT_TRUE(limiter.shouldPublish(1.000002));
}

TEST(OdometryRateLimiter, CapsPointLevelOutputAtConfiguredRate) {
  LI2Sup::OdometryRateLimiter limiter(100.0);

  EXPECT_TRUE(limiter.shouldPublish(10.0));
  EXPECT_FALSE(limiter.shouldPublish(10.009));
  EXPECT_TRUE(limiter.shouldPublish(10.01));
  EXPECT_FALSE(limiter.shouldPublish(10.019));
  EXPECT_TRUE(limiter.shouldPublish(10.02));
}

TEST(OdometryRateLimiter, AllowsNewSessionTimestampAndReset) {
  LI2Sup::OdometryRateLimiter limiter(100.0);

  EXPECT_TRUE(limiter.shouldPublish(100.0));
  EXPECT_FALSE(limiter.shouldPublish(100.001));
  EXPECT_TRUE(limiter.shouldPublish(1.0));
  EXPECT_FALSE(limiter.shouldPublish(1.001));
  limiter.reset();
  EXPECT_TRUE(limiter.shouldPublish(1.001));
}

TEST(OdometryRateLimiter, InvalidRateAndTimestampDoNotSuppressOutput) {
  LI2Sup::OdometryRateLimiter limiter(-1.0);
  EXPECT_TRUE(limiter.shouldPublish(1.0));
  EXPECT_TRUE(limiter.shouldPublish(1.000001));

  LI2Sup::OdometryRateLimiter finite_limiter(100.0);
  EXPECT_TRUE(finite_limiter.shouldPublish(
      std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
