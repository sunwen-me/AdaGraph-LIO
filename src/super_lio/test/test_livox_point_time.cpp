#include <gtest/gtest.h>

#include <limits>

#include "ros/livox_point_time.h"

namespace {

TEST(LivoxPointTime, DecodesDriverOffsetNanoseconds) {
  const auto offset =
      LI2Sup::decodeLivoxPointOffsetSeconds(12'500'000.0, 1.8e18);
  ASSERT_TRUE(offset.has_value());
  EXPECT_NEAR(*offset, 0.0125, 1e-12);
}

TEST(LivoxPointTime, DecodesAbsoluteNanoseconds) {
  constexpr double frame_start_ns = 1'800'000'000'000'000'000.0;
  const auto offset = LI2Sup::decodeLivoxPointOffsetSeconds(
      frame_start_ns + 25'000'000.0, frame_start_ns);
  ASSERT_TRUE(offset.has_value());
  // Epoch-scale nanoseconds stored as double are quantized to roughly
  // hundreds of nanoseconds.
  EXPECT_NEAR(*offset, 0.025, 1e-6);
}

TEST(LivoxPointTime, DecodesAbsoluteNanosecondsWithLowSimulationClock) {
  constexpr double frame_start_ns = 100'000'000'000.0;
  const auto offset = LI2Sup::decodeLivoxPointOffsetSeconds(
      frame_start_ns + 25'000'000.0, frame_start_ns);
  ASSERT_TRUE(offset.has_value());
  EXPECT_NEAR(*offset, 0.025, 1e-9);
}

TEST(LivoxPointTime, ClampsSmallNegativeRoundoff) {
  const auto offset =
      LI2Sup::decodeLivoxPointOffsetSeconds(-500'000.0, 1.8e18);
  ASSERT_TRUE(offset.has_value());
  EXPECT_DOUBLE_EQ(*offset, 0.0);
}

TEST(LivoxPointTime, RejectsInvalidOrImplausibleOffsets) {
  EXPECT_FALSE(
      LI2Sup::decodeLivoxPointOffsetSeconds(2.0e9, 1.8e18).has_value());
  EXPECT_FALSE(LI2Sup::decodeLivoxPointOffsetSeconds(
                   std::numeric_limits<double>::quiet_NaN(), 1.8e18)
                   .has_value());
}

}  // namespace
