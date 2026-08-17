#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <numbers>
#include <vector>

#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "lio/scan_context_loop_detector.h"

namespace {

using hybrid_slam::backend::LoopDetectionSnapshot;
using hybrid_slam::common::KeyFrameId;
using hybrid_slam::common::NodeId;
using hybrid_slam::common::SessionId;
using hybrid_slam::common::SubmapId;
using hybrid_slam::common::Timestamp;
using hybrid_slam::common::TrajectoryId;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::mapping::KeyFrameSnapshot;
using hybrid_slam::mapping::MapSnapshot;
using hybrid_slam::mapping::SubmapSnapshot;
using hybrid_slam::mapping::SubmapState;
using hybrid_slam::sensor::FreezePointCloudBuffer;
using hybrid_slam::sensor::PointCloudBuffer;

hybrid_slam::sensor::ImmutablePointCloud MakeCloud(
    FrameId frame, const std::vector<Eigen::Vector3f>& points) {
  auto storage = std::make_shared<PointCloudBuffer>();
  storage->reserve(points.size());
  for (const auto& point : points) {
    storage->push_back({point.x(), point.y(), point.z(), 0.0F, 0});
  }
  return FreezePointCloudBuffer(std::move(storage), frame);
}

KeyFrameSnapshot MakeKeyFrame(const std::uint64_t id,
                              const std::uint64_t submap_id,
                              const double timestamp_s) {
  const auto timestamp = Timestamp::FromSeconds(timestamp_s);
  EXPECT_TRUE(timestamp);
  const auto cloud = MakeCloud(
      FrameId::kBase, {Eigen::Vector3f(1.0F, 0.0F, 1.0F)});
  return {NodeId(id), KeyFrameId(id), TrajectoryId(1), SessionId(1),
          SubmapId(submap_id),
          {*timestamp, FrameId::kOdom, FrameId::kBase,
           RigidTransform3d::Identity()},
          {*timestamp, FrameId::kMap, FrameId::kBase,
           RigidTransform3d::Identity()},
          cloud, {}};
}

SubmapSnapshot MakeSubmap(const std::uint64_t id,
                          const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
  SubmapSnapshot submap;
  submap.id = SubmapId(id);
  submap.trajectory_id = TrajectoryId(1);
  submap.session_id = SessionId(1);
  submap.state = SubmapState::kFinished;
  submap.keyframe_ids = {KeyFrameId(id)};
  submap.anchor_keyframe_id = KeyFrameId(id);
  submap.cloud_submap = cloud;
  submap.seed_cloud_submap = cloud;
  submap.descriptor = {1.0F};
  submap.geometry_ready = true;
  return submap;
}

TEST(ScanContextLoopDetector, ReturnsQueryToMatchYawForRotatedSubmaps) {
  std::vector<Eigen::Vector3f> match_points;
  for (int sector = 0; sector < 72; ++sector) {
    const float angle = static_cast<float>(
        2.0 * std::numbers::pi * static_cast<double>(sector) / 72.0);
    for (int ring = 0; ring < 3; ++ring) {
      const float radius = 4.0F + 8.0F * static_cast<float>(ring);
      const float height = static_cast<float>(
          1.0 + ((sector * 17 + ring * 13) % 37) / 10.0);
      match_points.emplace_back(radius * std::cos(angle),
                                radius * std::sin(angle), height);
    }
  }
  std::vector<Eigen::Vector3f> query_points;
  query_points.reserve(match_points.size());
  for (const auto& point : match_points) {
    query_points.emplace_back(-point.y(), point.x(), point.z());
  }

  auto map = std::make_shared<MapSnapshot>();
  map->version = 1;
  map->trajectories.push_back({TrajectoryId(1), SessionId(1),
                               {SubmapId(1), SubmapId(2)}});
  map->keyframes.push_back(MakeKeyFrame(1, 1, 0.0));
  map->keyframes.push_back(MakeKeyFrame(2, 2, 100.0));
  map->submaps.push_back(MakeSubmap(
      1, MakeCloud(FrameId::kSubmap, match_points)));
  map->submaps.push_back(MakeSubmap(
      2, MakeCloud(FrameId::kSubmap, query_points)));
  ASSERT_TRUE(map->consistent());

  LI2Sup::ScanContextConfig config;
  config.ring_count = 10;
  config.sector_count = 72;
  config.ring_key_candidates = 1;
  config.max_radius_m = 30.0;
  LI2Sup::ScanContextLoopDetector detector(
      {config, 1, 10.0});
  const auto candidates = detector.detect({{}, map, {}}, NodeId(2));

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().match_node, NodeId(1));
  ASSERT_TRUE(candidates.front().coarse_yaw_rad);
  EXPECT_NEAR(*candidates.front().coarse_yaw_rad,
              -std::numbers::pi / 2.0, 1e-9);
}

TEST(ScanContextYaw, EstimatesQueryToCandidateYawWithoutRetrieval) {
  std::vector<Eigen::Vector3f> candidate_points;
  for (int sector = 0; sector < 72; ++sector) {
    const float angle = static_cast<float>(
        2.0 * std::numbers::pi * static_cast<double>(sector) / 72.0);
    for (int ring = 0; ring < 3; ++ring) {
      const float radius = 4.0F + 8.0F * static_cast<float>(ring);
      const float height = static_cast<float>(
          1.0 + ((sector * 17 + ring * 13) % 37) / 10.0);
      candidate_points.emplace_back(radius * std::cos(angle),
                                    radius * std::sin(angle), height);
    }
  }
  std::vector<Eigen::Vector3f> query_points;
  query_points.reserve(candidate_points.size());
  for (const auto& point : candidate_points) {
    query_points.emplace_back(-point.y(), point.x(), point.z());
  }

  LI2Sup::ScanContextConfig config;
  config.ring_count = 10;
  config.sector_count = 72;
  config.ring_key_candidates = 1;
  config.max_radius_m = 30.0;
  const auto yaw = LI2Sup::EstimateScanContextYaw(
      MakeCloud(FrameId::kSubmap, query_points),
      MakeCloud(FrameId::kSubmap, candidate_points), config);

  ASSERT_TRUE(yaw);
  EXPECT_NEAR(*yaw, -std::numbers::pi / 2.0, 1e-9);
}

TEST(ScanContextLoopDetector, ExcludesItsOwnAndTemporallyAdjacentSubmaps) {
  const auto cloud = MakeCloud(
      FrameId::kSubmap, {Eigen::Vector3f(8.0F, 0.0F, 1.0F)});
  auto map = std::make_shared<MapSnapshot>();
  map->version = 1;
  map->trajectories.push_back({TrajectoryId(1), SessionId(1),
                               {SubmapId(1), SubmapId(2)}});
  map->keyframes.push_back(MakeKeyFrame(1, 1, 0.0));
  map->keyframes.push_back(MakeKeyFrame(2, 2, 5.0));
  map->submaps.push_back(MakeSubmap(1, cloud));
  map->submaps.push_back(MakeSubmap(2, cloud));
  ASSERT_TRUE(map->consistent());

  LI2Sup::ScanContextLoopDetector detector({{}, 5, 10.0});
  EXPECT_TRUE(detector.detect({{}, map, {}}, NodeId(2)).empty());
}

TEST(ScanContextLoopDetector, ReturnsRankedTopKWithoutAbsoluteDistanceGate) {
  const auto match_cloud = MakeCloud(
      FrameId::kSubmap,
      {Eigen::Vector3f(8.0F, 0.0F, 1.0F),
       Eigen::Vector3f(18.0F, 0.0F, 10.0F)});
  const auto query_cloud = MakeCloud(
      FrameId::kSubmap,
      {Eigen::Vector3f(8.0F, 0.0F, 10.0F),
       Eigen::Vector3f(18.0F, 0.0F, 1.0F)});
  auto map = std::make_shared<MapSnapshot>();
  map->version = 1;
  map->trajectories.push_back({TrajectoryId(1), SessionId(1),
                               {SubmapId(1), SubmapId(2)}});
  map->keyframes.push_back(MakeKeyFrame(1, 1, 0.0));
  map->keyframes.push_back(MakeKeyFrame(2, 2, 100.0));
  map->submaps.push_back(MakeSubmap(1, match_cloud));
  map->submaps.push_back(MakeSubmap(2, query_cloud));
  ASSERT_TRUE(map->consistent());

  LI2Sup::ScanContextLoopDetector detector({{}, 1, 10.0});
  const auto candidates = detector.detect({{}, map, {}}, NodeId(2));

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().match_node, NodeId(1));
  EXPECT_LT(candidates.front().retrieval_score, 0.9);
}

}  // namespace
