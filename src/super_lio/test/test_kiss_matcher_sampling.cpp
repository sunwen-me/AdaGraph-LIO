#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include <Eigen/Core>

#include <kiss_matcher/points/downsampling.hpp>

namespace {

TEST(KissMatcherSampling, DeterministicVoxelSamplingHasStableOrderAndValues) {
  std::vector<Eigen::Vector3f> input;
  input.reserve(4099);
  for (int i = 0; i < 4099; ++i) {
    // Deliberately cross the old parallel block boundary with multiple points
    // per voxel.  The stable source-index tie break defines both accumulation
    // and emitted-voxel order regardless of TBB worker scheduling.
    input.emplace_back(
        static_cast<float>(i % 17) * 0.21F,
        static_cast<float>((i / 17) % 11) * 0.19F,
        static_cast<float>((i / 187) % 7) * 0.23F);
  }

  const auto baseline =
      kiss_matcher::VoxelgridSamplingDeterministic(input, 0.5);
  ASSERT_FALSE(baseline.empty());
  for (int repeat = 0; repeat < 8; ++repeat) {
    const auto sampled =
        kiss_matcher::VoxelgridSamplingDeterministic(input, 0.5);
    ASSERT_EQ(sampled.size(), baseline.size());
    for (std::size_t index = 0; index < baseline.size(); ++index) {
      EXPECT_EQ(sampled[index], baseline[index]) << "index=" << index;
    }
  }
}

}  // namespace
