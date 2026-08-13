#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "lio/frontend/frontend_result.h"

namespace {

using adagraph::frontend::FrontendResult;
using adagraph::frontend::Implementation;
using adagraph::frontend::PointCloudView;
using adagraph::frontend::Status;
using adagraph::geometry::FrameId;

struct PaddedPoint {
  std::uint32_t prefix = 0;
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

static_assert(std::is_trivially_copyable_v<PointCloudView>);

TEST(FrontendPointCloudView, BorrowsOriginalStorageWithoutCopy) {
  std::vector<PaddedPoint> points{
      {7, 1.0F, 2.0F, 3.0F, 10.0F},
      {8, -4.0F, 5.5F, 6.0F, 20.0F},
  };

  const PointCloudView view = PointCloudView::InterleavedFloatXyz(
      points.data(), points.size(), sizeof(PaddedPoint),
      offsetof(PaddedPoint, x), offsetof(PaddedPoint, y),
      offsetof(PaddedPoint, z), FrameId::kImu);

  EXPECT_TRUE(view.valid());
  EXPECT_EQ(view.storageIdentity(), points.data());
  EXPECT_EQ(view.size(), points.size());
  EXPECT_EQ(view.strideBytes(), sizeof(PaddedPoint));
  EXPECT_EQ(view.frame(), FrameId::kImu);

  std::array<float, 3> xyz{};
  ASSERT_TRUE(view.xyz(1, xyz));
  EXPECT_FLOAT_EQ(xyz[0], -4.0F);
  EXPECT_FLOAT_EQ(xyz[1], 5.5F);
  EXPECT_FLOAT_EQ(xyz[2], 6.0F);

  // Mutating the source proves the view did not retain a copied buffer.
  points[1].y = 9.0F;
  ASSERT_TRUE(view.xyz(1, xyz));
  EXPECT_FLOAT_EQ(xyz[1], 9.0F);
}

TEST(FrontendPointCloudView, EmptyAndInvalidViewsAreSafe) {
  const PointCloudView empty = PointCloudView::InterleavedFloatXyz(
      nullptr, 0, 0, 0, 0, 0, FrameId::kOdom);
  EXPECT_TRUE(empty.empty());
  EXPECT_TRUE(empty.valid());

  std::array<float, 3> xyz{};
  EXPECT_FALSE(empty.xyz(0, xyz));

  const float one_coordinate = 1.0F;
  const PointCloudView invalid = PointCloudView::InterleavedFloatXyz(
      &one_coordinate, 1, sizeof(float), 0, sizeof(float),
      2 * sizeof(float), FrameId::kOdom);
  EXPECT_FALSE(invalid.valid());
  EXPECT_FALSE(invalid.xyz(0, xyz));
}

TEST(FrontendResult, CarriesExplicitLocalPoseAndCloudFrames) {
  FrontendResult result;
  result.implementation = Implementation::kPointLio;
  result.status = Status::kTracking;
  const auto timestamp = adagraph::common::Timestamp::FromSeconds(123.5);
  ASSERT_TRUE(timestamp.has_value());
  result.local_pose = {
      *timestamp,
      FrameId::kOdom,
      FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0)},
  };

  EXPECT_EQ(result.implementation, Implementation::kPointLio);
  EXPECT_EQ(result.status, Status::kTracking);
  EXPECT_DOUBLE_EQ(result.local_pose.timestamp.seconds(), 123.5);
  EXPECT_EQ(result.local_pose.target_frame, FrameId::kOdom);
  EXPECT_EQ(result.local_pose.source_frame, FrameId::kBase);
  EXPECT_TRUE(result.local_pose.T_target_source.translation.isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(result.local_pose.valid());
}

}  // namespace
