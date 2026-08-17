#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/localization/relocalizer.h"
#include "hybrid_slam/mapping/submap_index.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"
#include "lio/pcl_loop_closure.h"
#include "lio/pcl_submap_relocalizer.h"
#include "lio/pcl_submap_relocalization_scheduler.h"

namespace {

using hybrid_slam::common::SessionId;
using hybrid_slam::common::Timestamp;
using hybrid_slam::common::TrajectoryId;
using hybrid_slam::common::NodeId;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::geometry::StampedTransform3d;
using hybrid_slam::mapping::MapSnapshot;
using hybrid_slam::mapping::SubmapSnapshot;
using hybrid_slam::mapping::SubmapState;
using hybrid_slam::mapping::KeyFrameSnapshot;
using hybrid_slam::sensor::FreezePointCloudBuffer;
using hybrid_slam::sensor::PointCloudBuffer;
using hybrid_slam::sensor::PointXYZIT;

KeyFrameSnapshot MakeLoopKeyFrame(
    std::uint64_t node_id, std::uint64_t keyframe_id,
    TrajectoryId trajectory_id, double timestamp_seconds);

hybrid_slam::sensor::ImmutablePointCloud MakeCloud(
    const FrameId frame, const std::vector<Eigen::Vector3f>& points) {
  auto storage = std::make_shared<PointCloudBuffer>();
  storage->reserve(points.size());
  for (const auto& point : points) {
    storage->push_back({point.x(), point.y(), point.z(), 0.0F, 0});
  }
  return FreezePointCloudBuffer(std::move(storage), frame);
}

Eigen::Quaterniond QuaternionFromRpy(const double roll, const double pitch,
                                     const double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .normalized();
}

std::vector<Eigen::Vector3f> MakeNonPlanarLoopScene() {
  std::vector<Eigen::Vector3f> points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -12; y <= 12; ++y) {
      points.emplace_back(
          static_cast<float>(x), static_cast<float>(y),
          static_cast<float>(0.08 * std::sin(0.23 * x) +
                             0.05 * std::cos(0.31 * y)));
    }
  }
  for (int x = -20; x <= 20; ++x) {
    for (int z = 0; z <= 10; ++z) {
      points.emplace_back(static_cast<float>(x), -11.5F,
                          static_cast<float>(z));
    }
  }
  for (int y = -12; y <= 12; ++y) {
    for (int z = 0; z <= 7; ++z) {
      points.emplace_back(13.5F, static_cast<float>(y),
                          static_cast<float>(z));
    }
  }
  for (int index = 0; index < 80; ++index) {
    const float angle = static_cast<float>(index) * 0.17F;
    points.emplace_back(-8.0F + 2.0F * std::cos(angle),
                        4.0F + 2.0F * std::sin(angle),
                        0.13F * static_cast<float>(index));
  }
  return points;
}

std::vector<Eigen::Vector3f> TransformPoints(
    const std::vector<Eigen::Vector3f>& points,
    const RigidTransform3d& transform) {
  std::vector<Eigen::Vector3f> transformed;
  transformed.reserve(points.size());
  for (const auto& point : points) {
    transformed.push_back(
        transform.transformPoint(point.cast<double>()).cast<float>());
  }
  return transformed;
}

SubmapSnapshot MakeLoopGeometrySubmap(
    const std::uint64_t id, const std::uint64_t anchor_keyframe_id,
    const RigidTransform3d& T_map_submap,
    const std::vector<Eigen::Vector3f>& points_submap) {
  SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(id);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = SubmapState::kFinished;
  submap.anchor_keyframe_id =
      hybrid_slam::common::KeyFrameId(anchor_keyframe_id);
  submap.keyframe_ids = {submap.anchor_keyframe_id};
  submap.T_map_submap = T_map_submap;
  submap.cloud_submap = MakeCloud(FrameId::kSubmap, points_submap);
  submap.seed_cloud_submap = submap.cloud_submap;
  submap.descriptor = {1.0F};
  submap.geometry_ready = true;
  return submap;
}

KeyFrameSnapshot MakeGeometryKeyFrame(
    const std::uint64_t node_id, const std::uint64_t submap_id,
    const double timestamp_seconds, const RigidTransform3d& T_map_base,
    const std::vector<Eigen::Vector3f>& cloud_base) {
  auto keyframe = MakeLoopKeyFrame(node_id, node_id, TrajectoryId(1),
                                   timestamp_seconds);
  keyframe.submap_id = hybrid_slam::common::SubmapId(submap_id);
  keyframe.T_map_base.T_target_source = T_map_base;
  keyframe.T_odom_base.T_target_source = T_map_base;
  keyframe.cloud_base = MakeCloud(FrameId::kBase, cloud_base);
  return keyframe;
}

LI2Sup::PclLoopClosureConfig MakeSyntheticLoopConfig() {
  LI2Sup::PclLoopClosureConfig config;
  config.neighbor_submaps = 1;
  config.registration_crop_radius_m = 100.0;
  config.minimum_source_points = 50;
  config.minimum_target_points = 50;
  config.minimum_inlier_ratio = 0.5;
  config.minimum_overlap = 0.3;
  config.minimum_bidirectional_overlap = 0.3;
  config.minimum_mutual_inlier_ratio = 0.3;
  config.maximum_bidirectional_inlier_rmse_m = 0.1;
  config.minimum_support_extent_m = 10.0;
  config.minimum_support_z_extent_m = 1.5;
  config.minimum_support_eigenvalue_ratio = 1e-5;
  config.minimum_support_voxels = 10;
  config.support_voxel_size_m = 4.0;
  config.maximum_initial_translation_m = 30.0;
  config.maximum_coarse_translation_disagreement_m = 3.0;
  config.maximum_coarse_yaw_disagreement_rad = 0.25;
  config.maximum_pose_free_graph_correction_m = 1.0;
  config.maximum_pose_free_graph_correction_rad = 0.10;
  config.maximum_relative_z_m = 5.0;
  config.maximum_relative_roll_pitch_rad = 0.35;
  config.maximum_solution_disagreement_m = 0.1;
  config.maximum_solution_disagreement_rad = 0.02;
  config.uniqueness_probe_translation_m = 3.0;
  config.minimum_uniqueness_probe_consensus = 1;
  config.registration.source_voxel_size_m = 0.35;
  config.registration.target_voxel_size_m = 0.35;
  config.registration.coarse_registration_voxel_size_m = 1.5;
  config.registration.max_source_points = 10000;
  config.registration.max_target_points = 10000;
  config.registration.max_registration_iterations = 64;
  config.registration.max_fitness = 0.5;
  config.registration.minimum_inlier_ratio = 0.5;
  config.registration.minimum_overlap = 0.3;
  return config;
}

SubmapSnapshot MakeSubmap(
    const std::uint64_t id, const Eigen::Vector3d& map_translation,
    const std::vector<float>& descriptor) {
  SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(id);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = SubmapState::kFinished;
  submap.T_map_submap = {Eigen::Quaterniond::Identity(), map_translation};
  submap.cloud_submap = MakeCloud(
      FrameId::kSubmap,
      {Eigen::Vector3f(1.0F, 0.0F, 0.0F),
       Eigen::Vector3f(2.0F, 1.0F, 0.0F),
       Eigen::Vector3f(3.0F, 0.0F, 1.0F)});
  submap.seed_cloud_submap = submap.cloud_submap;
  submap.descriptor = descriptor;
  submap.geometry_ready = true;
  return submap;
}

KeyFrameSnapshot MakeLoopKeyFrame(
    const std::uint64_t node_id, const std::uint64_t keyframe_id,
    const TrajectoryId trajectory_id, const double timestamp_seconds) {
  const auto timestamp = Timestamp::FromSeconds(timestamp_seconds);
  KeyFrameSnapshot keyframe;
  keyframe.node_id = hybrid_slam::common::NodeId(node_id);
  keyframe.keyframe_id = hybrid_slam::common::KeyFrameId(keyframe_id);
  keyframe.session_id = SessionId(1);
  keyframe.trajectory_id = trajectory_id;
  keyframe.submap_id = hybrid_slam::common::SubmapId(42);
  keyframe.T_odom_base = {
      *timestamp, FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}};
  keyframe.T_map_base = {
      *timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}};
  keyframe.cloud_base = MakeCloud(
      FrameId::kBase,
      {Eigen::Vector3f(1.0F, 0.0F, 0.0F),
       Eigen::Vector3f(2.0F, 1.0F, 0.0F),
       Eigen::Vector3f(3.0F, 0.0F, 1.0F)});
  return keyframe;
}

hybrid_slam::localization::RelocalizationQuery MakeQuery(
    const Timestamp timestamp) {
  hybrid_slam::localization::RelocalizationQuery query;
  query.T_odom_base = {
      timestamp, FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}};
  query.current_scan_base = MakeCloud(
      FrameId::kBase,
      {Eigen::Vector3f(1.0F, 0.0F, 0.0F),
       Eigen::Vector3f(2.0F, 1.0F, 0.0F),
       Eigen::Vector3f(3.0F, 0.0F, 1.0F)});
  return query;
}

TEST(PclSubmapRelocalizer, BindsAcceptedPoseToVerifiedSubmapIdentity) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp);
  const auto query = MakeQuery(*timestamp);
  const auto descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);

  MapSnapshot map;
  map.version = 7;
  map.submaps.push_back(MakeSubmap(42, Eigen::Vector3d(5.0, 0.0, 0.0),
                                   descriptor));
  map.submaps.push_back(MakeSubmap(43, Eigen::Vector3d(50.0, 0.0, 0.0),
                                   std::vector<float>(8, 0.0F)));
  ASSERT_TRUE(map.valid());

  hybrid_slam::mapping::SubmapDescriptorIndex index;
  index.rebuild(map);

  bool target_was_map_frame = false;
  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [&target_was_map_frame](
                            const auto& candidate_query, const auto& target,
                            std::stop_token) {
    target_was_map_frame = target.valid() &&
                           target.cloudMap().frame() == FrameId::kMap &&
                           target.cloudMap().size() == 6U;
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}},
         1.0}};
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, 0.05, 0.9,
        hybrid_slam::localization::Confidence::kHigh, 0.8, true};
  };

  LI2Sup::PclSubmapRelocalizationConfig config;
  LI2Sup::PclSubmapRelocalizer relocalizer(config, std::move(operations));
  const auto result = relocalizer.Relocalize(query, map, index);

  ASSERT_TRUE(result);
  EXPECT_TRUE(target_was_map_frame);
  EXPECT_EQ(result->session_id, SessionId(1));
  EXPECT_EQ(result->trajectory_id, TrajectoryId(1));
  EXPECT_EQ(result->submap_id, hybrid_slam::common::SubmapId(42));
  EXPECT_TRUE(result->T_map_base.T_target_source.translation.isApprox(
      Eigen::Vector3d(5.0, 0.0, 0.0)));
}

TEST(PclLoopClosureConfig, RejectsInvalidGraphFactorUncertainty) {
  LI2Sup::PclLoopClosureConfig config;
  config.translation_sigma_m = 0.0;
  EXPECT_TRUE(config.validate().has_value());

  config.translation_sigma_m = 0.35;
  config.rotation_sigma_rad = 0.0;
  EXPECT_TRUE(config.validate().has_value());

  config.rotation_sigma_rad = 0.10;
  config.maximum_initial_translation_m = 0.0;
  EXPECT_TRUE(config.validate().has_value());

  config.maximum_initial_translation_m = 15.0;
  EXPECT_FALSE(config.validate().has_value());

  config.minimum_source_points = 0;
  EXPECT_TRUE(config.validate().has_value());

  config.minimum_source_points = 1000;
  config.minimum_target_points = 0;
  EXPECT_TRUE(config.validate().has_value());

  config.minimum_target_points = 1000;
  EXPECT_FALSE(config.validate().has_value());

  config.maximum_bidirectional_inlier_rmse_m = 0.0;
  EXPECT_TRUE(config.validate().has_value());

  config.maximum_bidirectional_inlier_rmse_m = 0.75;
  EXPECT_FALSE(config.validate().has_value());

  config.minimum_mutual_inlier_ratio = 0.0;
  EXPECT_TRUE(config.validate().has_value());

  config.minimum_mutual_inlier_ratio = 0.30;
  EXPECT_FALSE(config.validate().has_value());

  config.minimum_uniqueness_probe_consensus = 5;
  EXPECT_TRUE(config.validate().has_value());

  config.minimum_uniqueness_probe_consensus = 1;
  EXPECT_FALSE(config.validate().has_value());

  config.registration.coarse_registration_voxel_size_m = -1.0;
  EXPECT_TRUE(config.validate().has_value());
}

TEST(BevPlace2Config, BoundsTheNumberOfSubmapViewDescriptors) {
  LI2Sup::BevPlace2Config config;
  config.representative_keyframes_per_submap = 0;
  EXPECT_TRUE(config.validate().has_value());

  config.representative_keyframes_per_submap = 10;
  EXPECT_TRUE(config.validate().has_value());

  config.representative_keyframes_per_submap = 3;
  EXPECT_FALSE(config.validate().has_value());
}

TEST(PclLoopClosureVerifier, RecoversKnownMatchFromQuerySixDofTransform) {
  const auto timestamp = Timestamp::FromSeconds(100.0);
  ASSERT_TRUE(timestamp);
  const auto target_points = MakeNonPlanarLoopScene();
  const RigidTransform3d T_match_query{
      QuaternionFromRpy(0.08, -0.06, 0.35),
      Eigen::Vector3d(4.0, -3.0, 1.8)};
  const RigidTransform3d T_map_match{
      QuaternionFromRpy(-0.03, 0.04, -0.20),
      Eigen::Vector3d(20.0, -7.0, 3.0)};
  const RigidTransform3d T_map_query = T_map_match.compose(T_match_query);
  const auto query_points =
      TransformPoints(target_points, T_match_query.inverse());
  const std::vector<Eigen::Vector3f> target_anchor_points(
      target_points.begin(), target_points.begin() + 20);
  const std::vector<Eigen::Vector3f> target_neighbor_points(
      target_points.begin() + 20, target_points.end());
  const std::vector<Eigen::Vector3f> query_anchor_points(
      query_points.begin(), query_points.begin() + 20);
  const std::vector<Eigen::Vector3f> query_neighbor_points(
      query_points.begin() + 20, query_points.end());
  const RigidTransform3d T_match_target_neighbor{
      QuaternionFromRpy(0.01, -0.02, 0.04),
      Eigen::Vector3d(2.0, 1.0, 0.2)};
  const RigidTransform3d T_query_query_neighbor{
      QuaternionFromRpy(-0.02, 0.01, -0.03),
      Eigen::Vector3d(-1.5, 2.5, -0.1)};
  const RigidTransform3d T_odom_match{
      QuaternionFromRpy(0.02, 0.01, -0.10),
      Eigen::Vector3d(100.0, 20.0, -4.0)};
  const RigidTransform3d T_odom_query{
      QuaternionFromRpy(-0.01, 0.02, 0.15),
      Eigen::Vector3d(112.0, 17.0, -12.0)};
  const RigidTransform3d warped_target_neighbor_map{
      QuaternionFromRpy(0.4, -0.2, 1.1),
      Eigen::Vector3d(400.0, -300.0, 80.0)};
  const RigidTransform3d warped_query_neighbor_map{
      QuaternionFromRpy(-0.3, 0.25, -0.9),
      Eigen::Vector3d(-350.0, 280.0, -60.0)};

  auto map = std::make_shared<MapSnapshot>();
  map->version = 15;
  map->submaps.push_back(MakeLoopGeometrySubmap(
      10, 1, T_map_match, target_anchor_points));
  map->submaps.push_back(MakeLoopGeometrySubmap(
      100, 2, T_map_query, query_anchor_points));
  map->submaps.push_back(MakeLoopGeometrySubmap(
      11, 3, warped_target_neighbor_map,
      TransformPoints(target_neighbor_points,
                      T_match_target_neighbor.inverse())));
  map->submaps.push_back(MakeLoopGeometrySubmap(
      101, 4, warped_query_neighbor_map,
      TransformPoints(query_neighbor_points,
                      T_query_query_neighbor.inverse())));
  map->keyframes.push_back(
      MakeGeometryKeyFrame(1, 10, 1.0, T_map_match, target_points));
  map->keyframes.push_back(
      MakeGeometryKeyFrame(2, 100, 100.0, T_map_query, query_points));
  map->keyframes.push_back(MakeGeometryKeyFrame(
      3, 11, 2.0, warped_target_neighbor_map, target_neighbor_points));
  map->keyframes.push_back(MakeGeometryKeyFrame(
      4, 101, 101.0, warped_query_neighbor_map, query_neighbor_points));
  map->keyframes[0].T_odom_base.T_target_source = T_odom_match;
  map->keyframes[1].T_odom_base.T_target_source = T_odom_query;
  map->keyframes[2].T_odom_base.T_target_source =
      T_odom_match.compose(T_match_target_neighbor);
  map->keyframes[3].T_odom_base.T_target_source =
      T_odom_query.compose(T_query_query_neighbor);
  ASSERT_TRUE(map->valid());

  auto config = MakeSyntheticLoopConfig();
  LI2Sup::PclLoopClosureVerifier verifier(config);
  verifier.prepare({{}, map, {}});
  const RigidTransform3d graph_error{
      QuaternionFromRpy(0.02, -0.01, 0.03),
      Eigen::Vector3d(0.4, -0.3, 0.2)};
  const RigidTransform3d graph_initial = T_match_query.compose(graph_error);
  hybrid_slam::backend::LoopCandidate candidate{NodeId(2), NodeId(1), 0.95};
  candidate.coarse_T_match_query = T_match_query;
  const hybrid_slam::backend::LoopVerificationInput input{
      candidate, *timestamp, map->keyframes[1].cloud_base,
      map->keyframes[0].cloud_base, graph_initial};

  const auto measurement = verifier.verify(input);

  ASSERT_TRUE(measurement);
  EXPECT_EQ(measurement->from_node, NodeId(1));
  EXPECT_EQ(measurement->to_node, NodeId(2));
  EXPECT_NEAR(measurement->T_from_to.translation.x(), 4.0, 0.08);
  EXPECT_NEAR(measurement->T_from_to.translation.y(), -3.0, 0.08);
  EXPECT_NEAR(measurement->T_from_to.translation.z(), 1.8, 0.08);
  const double rotation_error = Eigen::AngleAxisd(
      T_match_query.rotation.inverse() * measurement->T_from_to.rotation)
                                    .angle();
  EXPECT_LT(rotation_error, 0.015);
  EXPECT_NEAR(std::sqrt(measurement->covariance(3, 3)),
              config.translation_sigma_m, 1e-9);
  EXPECT_NEAR(std::sqrt(measurement->covariance(0, 0)),
              config.rotation_sigma_rad, 1e-9);

  // The norm of T_match_query is the physical separation between endpoints,
  // not registration error.  A graph initializer must remain usable even when
  // that norm exceeds the descriptor-initializer safety limit.
  config.maximum_initial_translation_m = 1.0;
  LI2Sup::PclLoopClosureVerifier graph_only_verifier(config);
  graph_only_verifier.prepare({{}, map, {}});
  hybrid_slam::backend::LoopCandidate graph_only_candidate{
      NodeId(2), NodeId(1), 0.95};
  const hybrid_slam::backend::LoopVerificationInput graph_only_input{
      graph_only_candidate, *timestamp, map->keyframes[1].cloud_base,
      map->keyframes[0].cloud_base, graph_initial};
  const auto graph_only_measurement = graph_only_verifier.verify(
      graph_only_input);
  ASSERT_TRUE(graph_only_measurement);
  EXPECT_NEAR(graph_only_measurement->T_from_to.translation.z(), 1.8, 0.08);

  // BEV's pose-free sequence fallback is allowed only when earlier loop
  // factors have already made the graph prediction local.  It must not use
  // graph-only registration to jump into a distant repeated-road basin.
  config.plugin = "bevplace2";
  config.maximum_pose_free_graph_correction_m = 0.1;
  LI2Sup::PclLoopClosureVerifier guarded_graph_only_verifier(config);
  guarded_graph_only_verifier.prepare({{}, map, {}});
  EXPECT_FALSE(guarded_graph_only_verifier.verify(graph_only_input));
}

TEST(PclLoopClosureVerifier, RejectsPlanarSupportForSixDofFactor) {
  const auto timestamp = Timestamp::FromSeconds(100.0);
  ASSERT_TRUE(timestamp);
  std::vector<Eigen::Vector3f> target_points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -12; y <= 12; ++y) {
      target_points.emplace_back(static_cast<float>(x),
                                 static_cast<float>(y), 0.0F);
    }
  }
  const RigidTransform3d T_match_query{
      QuaternionFromRpy(0.08, -0.06, 0.35),
      Eigen::Vector3d(4.0, -3.0, 1.8)};
  const auto query_points =
      TransformPoints(target_points, T_match_query.inverse());

  auto map = std::make_shared<MapSnapshot>();
  map->version = 16;
  map->submaps.push_back(MakeLoopGeometrySubmap(
      10, 1, RigidTransform3d::Identity(), target_points));
  map->submaps.push_back(MakeLoopGeometrySubmap(
      100, 2, T_match_query, query_points));
  map->keyframes.push_back(MakeGeometryKeyFrame(
      1, 10, 1.0, RigidTransform3d::Identity(), target_points));
  map->keyframes.push_back(MakeGeometryKeyFrame(
      2, 100, 100.0, T_match_query, query_points));
  ASSERT_TRUE(map->valid());

  auto config = MakeSyntheticLoopConfig();
  LI2Sup::PclLoopClosureVerifier verifier(config);
  verifier.prepare({{}, map, {}});
  const hybrid_slam::backend::LoopCandidate candidate{
      NodeId(2), NodeId(1), 0.95};
  const hybrid_slam::backend::LoopVerificationInput input{
      candidate, *timestamp, map->keyframes[1].cloud_base,
      map->keyframes[0].cloud_base, T_match_query};

  EXPECT_FALSE(verifier.verify(input));
}

TEST(PclSubmapRelocalizer, SelectsBestFitnessAcrossVerifiedCandidates) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp);
  const auto query = MakeQuery(*timestamp);

  MapSnapshot map;
  map.version = 9;
  map.submaps.push_back(MakeSubmap(42, Eigen::Vector3d(5.0, 0.0, 0.0),
                                   std::vector<float>(8, 0.0F)));
  map.submaps.push_back(MakeSubmap(43, Eigen::Vector3d(50.0, 0.0, 0.0),
                                   std::vector<float>(8, 0.0F)));
  ASSERT_TRUE(map.valid());

  hybrid_slam::mapping::SubmapDescriptorIndex index;
  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [](const auto& candidate_query, const auto&,
                           std::stop_token) {
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}},
         10.0},
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d(50.0, 0.0, 0.0)}},
         5.0},
    };
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    const bool second_candidate =
        candidate.initial_T_map_base.T_target_source.translation.x() > 10.0;
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, second_candidate ? 0.02 : 0.08,
        second_candidate ? 0.8 : 0.9,
        hybrid_slam::localization::Confidence::kHigh,
        second_candidate ? 0.7 : 0.8, true};
  };

  LI2Sup::PclSubmapRelocalizer relocalizer({}, std::move(operations));
  const auto result = relocalizer.Relocalize(query, map, index);

  ASSERT_TRUE(result);
  EXPECT_EQ(result->submap_id, hybrid_slam::common::SubmapId(43));
  EXPECT_DOUBLE_EQ(result->fitness, 0.02);
}

TEST(PclSubmapRelocalizer, UsesCommittedActiveSubmapKeyframesForGlobalTarget) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp);
  const auto query = MakeQuery(*timestamp);

  MapSnapshot map;
  map.version = 8;
  SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(42);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = SubmapState::kActive;
  submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(1);
  submap.keyframe_ids = {submap.anchor_keyframe_id};
  map.submaps.push_back(submap);
  auto keyframe = MakeLoopKeyFrame(1, 1, TrajectoryId(1), 3.0);
  keyframe.T_map_base.T_target_source.translation =
      Eigen::Vector3d(5.0, 0.0, 0.0);
  map.keyframes.push_back(std::move(keyframe));
  ASSERT_TRUE(map.consistent());

  hybrid_slam::mapping::SubmapDescriptorIndex index;
  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [](const auto& candidate_query, const auto& target,
                           std::stop_token) {
    EXPECT_EQ(target.cloudMap().frame(), FrameId::kMap);
    EXPECT_EQ(target.cloudMap().size(), 3U);
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}},
         1.0}};
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, 0.01, 0.9,
        hybrid_slam::localization::Confidence::kHigh, 0.9, true};
  };

  LI2Sup::PclSubmapRelocalizationConfig config;
  LI2Sup::PclSubmapRelocalizer relocalizer(config, std::move(operations));
  const auto result = relocalizer.Relocalize(query, map, index);

  ASSERT_TRUE(result);
  EXPECT_EQ(result->submap_id, hybrid_slam::common::SubmapId(42));
  EXPECT_EQ(result->session_id, SessionId(1));
  EXPECT_EQ(result->trajectory_id, TrajectoryId(1));
}

TEST(PclSubmapRelocalizer, UsesMapOdomPriorWhenKissHasNoQualityCandidate) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp);
  auto query = MakeQuery(*timestamp);
  query.prior_T_map_base = {
      *timestamp, FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}};

  MapSnapshot map;
  map.version = 9;
  SubmapSnapshot submap;
  submap.id = hybrid_slam::common::SubmapId(42);
  submap.session_id = SessionId(1);
  submap.trajectory_id = TrajectoryId(1);
  submap.state = SubmapState::kActive;
  submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(1);
  submap.keyframe_ids = {submap.anchor_keyframe_id};
  map.submaps.push_back(submap);
  auto keyframe = MakeLoopKeyFrame(1, 1, TrajectoryId(1), 3.0);
  keyframe.T_map_base.T_target_source.translation =
      Eigen::Vector3d(5.0, 0.0, 0.0);
  map.keyframes.push_back(std::move(keyframe));
  ASSERT_TRUE(map.consistent());

  hybrid_slam::mapping::SubmapDescriptorIndex index;
  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [](const auto&, const auto&, std::stop_token) {
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{};
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    EXPECT_DOUBLE_EQ(candidate.retrieval_score, -1.0);
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, 0.01, 0.9,
        hybrid_slam::localization::Confidence::kHigh, 0.9, true};
  };

  LI2Sup::PclSubmapRelocalizationConfig config;
  LI2Sup::PclSubmapRelocalizer relocalizer(config, std::move(operations));
  const auto result = relocalizer.Relocalize(query, map, index);

  ASSERT_TRUE(result);
  EXPECT_EQ(result->submap_id, hybrid_slam::common::SubmapId(42));
  EXPECT_TRUE(result->T_map_base.T_target_source.translation.isApprox(
      Eigen::Vector3d(5.0, 0.0, 0.0)));
}

TEST(PclSubmapRelocalizer, RejectsGeometricallyUnqualifiedCandidate) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp);
  const auto query = MakeQuery(*timestamp);
  const auto descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);
  MapSnapshot map;
  map.version = 1;
  map.submaps.push_back(MakeSubmap(42, Eigen::Vector3d::Zero(), descriptor));
  hybrid_slam::mapping::SubmapDescriptorIndex index;
  index.rebuild(map);

  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [](const auto& candidate_query, const auto&,
                           std::stop_token) {
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}},
         1.0}};
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, 0.01, 0.95,
        hybrid_slam::localization::Confidence::kHigh, 0.95, false};
  };

  LI2Sup::PclSubmapRelocalizationConfig config;
  LI2Sup::PclSubmapRelocalizer relocalizer(config, std::move(operations));
  EXPECT_FALSE(relocalizer.Relocalize(query, map, index));
}

TEST(PclSubmapRelocalizationScheduler, ProcessesImmutableSnapshotOffCaller) {
  const auto timestamp = Timestamp::FromSeconds(3.0);
  ASSERT_TRUE(timestamp.has_value());
  const auto query = MakeQuery(*timestamp);
  const auto descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);
  auto map = std::make_shared<MapSnapshot>();
  map->version = 11;
  map->submaps.push_back(
      MakeSubmap(42, Eigen::Vector3d::Zero(), descriptor));

  LI2Sup::PclSubmapRelocalizerOperations operations;
  operations.retrieve = [](const auto& candidate_query, const auto&,
                           std::stop_token) {
    return std::vector<hybrid_slam::localization::RelocalizationCandidate>{
        {{candidate_query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}},
         1.0}};
  };
  operations.verify = [](const auto&, const auto&, const auto& candidate,
                         std::stop_token) {
    return hybrid_slam::localization::RegistrationVerification{
        candidate.initial_T_map_base, 0.01, 0.9,
        hybrid_slam::localization::Confidence::kHigh, 0.9, true};
  };
  LI2Sup::PclSubmapRelocalizationConfig config;
  auto relocalizer = std::make_unique<LI2Sup::PclSubmapRelocalizer>(
      config, std::move(operations));

  std::promise<std::optional<
      hybrid_slam::localization::SubmapRelocalizationResult>> promise;
  auto future = promise.get_future();
  LI2Sup::PclSubmapRelocalizationScheduler scheduler(
      std::move(relocalizer),
      [&promise](std::uint64_t,
                 const std::optional<
                     hybrid_slam::localization::SubmapRelocalizationResult>&
                     result,
                 const auto&, auto) { promise.set_value(result); });

  EXPECT_EQ(scheduler.submit(query, map),
            LI2Sup::PclSubmapRelocalizationSubmissionStatus::kAccepted);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)),
            std::future_status::ready);
  const auto result = future.get();
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->submap_id, hybrid_slam::common::SubmapId(42));
  EXPECT_GE(scheduler.stats().completed, 1U);
  scheduler.finish();
}

TEST(PclSubmapLoopDetector, UsesCompletedSubmapDescriptorAndAnchor) {
  const auto query = MakeQuery(*Timestamp::FromSeconds(100.0));
  const auto keyframe_descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);
  std::vector<float> submap_descriptor(8, 0.0F);
  submap_descriptor.front() = 1.0F;
  auto map = std::make_shared<MapSnapshot>();
  map->version = 12;
  auto target_submap = MakeSubmap(
      42, Eigen::Vector3d::Zero(), submap_descriptor);
  target_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(1)};
  target_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(1);
  map->submaps.push_back(std::move(target_submap));
  auto decoy_submap = MakeSubmap(
      43, Eigen::Vector3d::Zero(), keyframe_descriptor);
  decoy_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(2)};
  decoy_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(2);
  map->submaps.push_back(std::move(decoy_submap));
  auto query_submap = MakeSubmap(
      44, Eigen::Vector3d::Zero(), submap_descriptor);
  query_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(3)};
  query_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(3);
  map->submaps.push_back(std::move(query_submap));
  map->keyframes.push_back(MakeLoopKeyFrame(1, 1, TrajectoryId(1), 1.0));
  map->keyframes.back().submap_id = hybrid_slam::common::SubmapId(42);
  map->keyframes.push_back(MakeLoopKeyFrame(2, 2, TrajectoryId(1), 50.0));
  map->keyframes.back().submap_id = hybrid_slam::common::SubmapId(43);
  map->keyframes.push_back(MakeLoopKeyFrame(3, 3, TrajectoryId(1), 100.0));
  map->keyframes.back().submap_id = hybrid_slam::common::SubmapId(44);
  ASSERT_TRUE(map->valid());
  auto index = std::make_shared<hybrid_slam::mapping::SubmapDescriptorIndex>();
  index->rebuild(*map);

  LI2Sup::PclLoopClosureConfig config;
  config.maximum_candidates = 1;
  config.minimum_temporal_separation_s = 20.0;
  LI2Sup::PclSubmapLoopDetector detector(config);
  const hybrid_slam::backend::LoopDetectionSnapshot snapshot{
      {}, map, index};
  const auto candidates = detector.detect(snapshot, hybrid_slam::common::NodeId(3));

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().query_node, hybrid_slam::common::NodeId(3));
  EXPECT_EQ(candidates.front().match_node, hybrid_slam::common::NodeId(1));
}

TEST(PclSubmapLoopDetector, RejectsCrossTrajectoryCandidateInFirstPhase) {
  const auto query = MakeQuery(*Timestamp::FromSeconds(100.0));
  const auto descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);
  auto map = std::make_shared<MapSnapshot>();
  map->version = 13;
  auto submap = MakeSubmap(42, Eigen::Vector3d::Zero(), descriptor);
  submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(1)};
  map->submaps.push_back(std::move(submap));
  map->keyframes.push_back(MakeLoopKeyFrame(1, 1, TrajectoryId(1), 1.0));
  map->keyframes.push_back(MakeLoopKeyFrame(2, 2, TrajectoryId(2), 100.0));
  ASSERT_TRUE(map->valid());
  auto index = std::make_shared<hybrid_slam::mapping::SubmapDescriptorIndex>();
  index->rebuild(*map);

  LI2Sup::PclLoopClosureConfig config;
  config.maximum_candidates = 1;
  config.minimum_temporal_separation_s = 20.0;
  LI2Sup::PclSubmapLoopDetector detector(config);
  const hybrid_slam::backend::LoopDetectionSnapshot snapshot{
      {}, map, index};

  EXPECT_TRUE(detector.detect(snapshot, hybrid_slam::common::NodeId(2)).empty());
}

TEST(PclSubmapLoopDetector, BindsAnchorToCandidateSessionAndTrajectory) {
  const auto query = MakeQuery(*Timestamp::FromSeconds(100.0));
  const auto descriptor =
      hybrid_slam::mapping::SubmapDescriptorIndex::buildRadialDescriptor(
          query.current_scan_base, 8);
  auto map = std::make_shared<MapSnapshot>();
  map->version = 14;

  auto target_submap = MakeSubmap(42, Eigen::Vector3d::Zero(), descriptor);
  target_submap.session_id = SessionId(1);
  target_submap.trajectory_id = TrajectoryId(1);
  target_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(1)};
  target_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(1);
  map->submaps.push_back(std::move(target_submap));

  auto query_submap = MakeSubmap(44, Eigen::Vector3d::Zero(), descriptor);
  query_submap.session_id = SessionId(2);
  query_submap.trajectory_id = TrajectoryId(1);
  query_submap.keyframe_ids = {hybrid_slam::common::KeyFrameId(3)};
  query_submap.anchor_keyframe_id = hybrid_slam::common::KeyFrameId(3);
  map->submaps.push_back(std::move(query_submap));

  map->keyframes.push_back(MakeLoopKeyFrame(1, 1, TrajectoryId(1), 1.0));
  map->keyframes.back().session_id = SessionId(1);
  map->keyframes.back().submap_id = hybrid_slam::common::SubmapId(42);
  map->keyframes.push_back(MakeLoopKeyFrame(3, 3, TrajectoryId(1), 100.0));
  map->keyframes.back().session_id = SessionId(2);
  map->keyframes.back().submap_id = hybrid_slam::common::SubmapId(44);
  ASSERT_TRUE(map->valid());

  auto index = std::make_shared<
      hybrid_slam::mapping::SubmapDescriptorIndex>();
  index->rebuild(*map);

  LI2Sup::PclLoopClosureConfig config;
  config.maximum_candidates = 1;
  config.minimum_temporal_separation_s = 20.0;
  LI2Sup::PclSubmapLoopDetector detector(config);
  const hybrid_slam::backend::LoopDetectionSnapshot snapshot{{}, map, index};
  const auto candidates = detector.detect(snapshot, NodeId(3));

  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front().match_node, NodeId(1));
}

LI2Sup::BevPlace2SequenceObservation MakeSequenceObservation(
    const std::uint64_t query_submap, const std::uint64_t match_submap,
    const std::uint64_t query_session = 0,
    const std::uint64_t match_session = 0) {
  return {
      SessionId(query_session), TrajectoryId(0),
      hybrid_slam::common::SubmapId(query_submap),
      SessionId(match_session), TrajectoryId(0),
      hybrid_slam::common::SubmapId(match_submap),
  };
}

TEST(BevPlace2SequenceConsistency, RejectsAnIsolatedRepeatedRoadMatch) {
  const auto isolated = MakeSequenceObservation(34, 17);
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(isolated, {}, 2), 1U);

  const std::vector<LI2Sup::BevPlace2SequenceObservation> unrelated{
      MakeSequenceObservation(33, 7),
      MakeSequenceObservation(34, 17),  // same query must not self-confirm
  };
  EXPECT_EQ(
      LI2Sup::CountBevPlace2SequenceConsensus(isolated, unrelated, 2), 1U);
}

TEST(BevPlace2SequenceConsistency, AcceptsConsecutiveSameDirectionRevisits) {
  const std::vector<LI2Sup::BevPlace2SequenceObservation> history{
      MakeSequenceObservation(85, 1),
      MakeSequenceObservation(86, 2),
  };
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(
                MakeSequenceObservation(86, 2),
                {history.front()}, 2),
            2U);
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(
                MakeSequenceObservation(87, 3), history, 2),
            3U);
}

TEST(BevPlace2SequenceConsistency, SupportsReverseTraversalAndBindsSessions) {
  const auto current = MakeSequenceObservation(42, 8);
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(
                current, {MakeSequenceObservation(41, 9)}, 2),
            2U);
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(
                current, {MakeSequenceObservation(41, 9, 1, 0)}, 2),
            1U);
  EXPECT_EQ(LI2Sup::CountBevPlace2SequenceConsensus(
                current, {MakeSequenceObservation(39, 11)}, 2),
            1U);
}

TEST(BevPlace2SequenceConsistency, IdentifiesThePendingObservationToPromote) {
  const auto previous = MakeSequenceObservation(85, 1);
  const auto current = MakeSequenceObservation(86, 2);
  EXPECT_TRUE(LI2Sup::IsBevPlace2SequenceConsistent(current, previous, 2));
  EXPECT_FALSE(LI2Sup::IsBevPlace2SequenceConsistent(
      current, MakeSequenceObservation(84, 1), 2));
  EXPECT_FALSE(LI2Sup::IsBevPlace2SequenceConsistent(
      current, MakeSequenceObservation(85, 4), 2));
}

TEST(BevPlace2SequenceConsistency,
     AllowsPoseFreeThirdPairButDoesNotLowerTheNormalSequenceGate) {
  const std::vector<LI2Sup::BevPlace2SequenceObservation> pose_history{
      MakeSequenceObservation(85, 1),
      MakeSequenceObservation(86, 2),
  };
  EXPECT_TRUE(LI2Sup::ShouldVerifyBevPlace2WithoutCoarsePose(
      MakeSequenceObservation(87, 3), pose_history, 2, 3));
  EXPECT_FALSE(LI2Sup::ShouldVerifyBevPlace2WithoutCoarsePose(
      MakeSequenceObservation(86, 2), {pose_history.front()}, 2, 3));
  EXPECT_FALSE(LI2Sup::ShouldVerifyBevPlace2WithoutCoarsePose(
      MakeSequenceObservation(87, 4), pose_history, 2, 3));
}

}  // namespace
