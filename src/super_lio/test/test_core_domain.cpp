#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "adagraph/backend/backend_config.h"
#include "adagraph/backend/backend_scheduler.h"
#include "adagraph/backend/constraint_builder.h"
#include "adagraph/backend/keyframe_policy.h"
#include "adagraph/backend/loop_closure.h"
#include "adagraph/backend/odometry_covariance_estimator.h"
#include "adagraph/backend/pose_graph.h"
#include "adagraph/common/domain_ids.h"
#include "adagraph/common/timestamp.h"
#include "adagraph/frontend/frontend_config.h"
#include "adagraph/geometry/transform3.h"
#include "adagraph/io/map_io.h"
#include "adagraph/localization/relocalization_config.h"
#include "adagraph/localization/localization_map.h"
#include "adagraph/localization/composed_relocalizer.h"
#include "adagraph/localization/relocalizer.h"
#include "adagraph/localization/relocalization_scheduler.h"
#include "adagraph/localization/relocalization_update.h"
#include "adagraph/mapping/keyframe.h"
#include "adagraph/mapping/map_manager.h"
#include "adagraph/mapping/occupancy_grid.h"
#include "adagraph/mapping/snapshot_products.h"
#include "adagraph/sensor/imu_data.h"
#include "adagraph/sensor/immutable_point_cloud.h"
#include "adagraph/sensor/point_cloud_frame.h"
#include "adagraph/system/global_alignment_state.h"

namespace {

using adagraph::common::Timestamp;
using adagraph::geometry::FrameId;
using adagraph::geometry::RigidTransform3d;
using adagraph::geometry::StampedTransform3d;

adagraph::mapping::KeyFrame MakeKeyFrame(
    std::uint64_t id, std::int64_t timestamp_ns,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero()) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{
      {static_cast<float>(id), 0.0F, 0.0F, 1.0F, 0},
  });
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(adagraph::sensor::PointXYZIT),
      offsetof(adagraph::sensor::PointXYZIT, x),
      offsetof(adagraph::sensor::PointXYZIT, y),
      offsetof(adagraph::sensor::PointXYZIT, z), FrameId::kBase);
  return adagraph::mapping::KeyFrame(
      adagraph::common::KeyFrameId(id),
      adagraph::common::TrajectoryId(1),
      adagraph::common::SessionId(1),
      {Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
       FrameId::kBase,
       {Eigen::Quaterniond::Identity(), translation}},
      adagraph::sensor::ImmutablePointCloud::Share(storage, view), {});
}

adagraph::mapping::KeyFrame MakeKeyFrameWithCloud(
    std::uint64_t id, std::int64_t timestamp_ns,
    adagraph::sensor::ImmutablePointCloud cloud,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero()) {
  return adagraph::mapping::KeyFrame(
      adagraph::common::KeyFrameId(id),
      adagraph::common::TrajectoryId(1),
      adagraph::common::SessionId(1),
      {Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
       FrameId::kBase,
       {Eigen::Quaterniond::Identity(), translation}},
      std::move(cloud), {});
}

adagraph::sensor::ImmutablePointCloud MakeImmutableCloud(FrameId frame) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(
      Buffer{{1.0F, 0.0F, 0.0F, 0.0F, 0}});
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(), sizeof(adagraph::sensor::PointXYZIT),
      offsetof(adagraph::sensor::PointXYZIT, x),
      offsetof(adagraph::sensor::PointXYZIT, y),
      offsetof(adagraph::sensor::PointXYZIT, z), frame);
  return adagraph::sensor::ImmutablePointCloud::Share(storage, view);
}

adagraph::localization::RelocalizationQuery MakeRelocalizationQuery(
    std::int64_t timestamp_ns) {
  return {
      {Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
       FrameId::kBase, RigidTransform3d::Identity()},
      MakeImmutableCloud(FrameId::kBase),
      std::nullopt,
  };
}

std::shared_ptr<const adagraph::localization::LocalizationMap>
MakeLocalizationMap() {
  return std::make_shared<const adagraph::localization::LocalizationMap>(
      1, MakeImmutableCloud(FrameId::kMap));
}

static_assert(!std::is_convertible_v<adagraph::common::KeyFrameId,
                                     adagraph::common::NodeId>);
static_assert(!std::is_copy_constructible_v<
              adagraph::sensor::PointCloudFrame>);
static_assert(std::is_nothrow_move_constructible_v<
              adagraph::sensor::PointCloudFrame>);

TEST(Timestamp, ConvertsLegacySecondsAndRejectsInvalidValues) {
  const auto half_second = Timestamp::FromSeconds(123.5);
  ASSERT_TRUE(half_second.has_value());
  EXPECT_EQ(half_second->nanoseconds(), 123'500'000'000LL);
  EXPECT_DOUBLE_EQ(half_second->seconds(), 123.5);

  const auto dataset_epoch = Timestamp::FromSeconds(1'627'000'000.123456);
  ASSERT_TRUE(dataset_epoch.has_value());
  EXPECT_NEAR(dataset_epoch->seconds(), 1'627'000'000.123456, 3e-7);

  EXPECT_FALSE(Timestamp::FromSeconds(
      std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(Timestamp::FromSeconds(
      std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(Timestamp::FromSeconds(1.0e20));
}

TEST(Transform3d, ComposesAndInvertsWithExplicitFrames) {
  const auto timestamp = Timestamp::FromSeconds(10.0);
  ASSERT_TRUE(timestamp.has_value());

  const StampedTransform3d T_map_odom{
      *timestamp,
      FrameId::kMap,
      FrameId::kOdom,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(10.0, 0.0, 0.0)},
  };
  const StampedTransform3d T_odom_base{
      *timestamp,
      FrameId::kOdom,
      FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 2.0, 0.0)},
  };

  const auto T_map_base = T_map_odom.compose(T_odom_base);
  ASSERT_TRUE(T_map_base.has_value());
  EXPECT_EQ(T_map_base->target_frame, FrameId::kMap);
  EXPECT_EQ(T_map_base->source_frame, FrameId::kBase);
  EXPECT_TRUE(T_map_base->T_target_source.translation.isApprox(
      Eigen::Vector3d(11.0, 2.0, 0.0)));

  const auto identity = T_map_base->compose(T_map_base->inverse());
  ASSERT_TRUE(identity.has_value());
  EXPECT_EQ(identity->target_frame, FrameId::kMap);
  EXPECT_EQ(identity->source_frame, FrameId::kMap);
  EXPECT_TRUE(identity->T_target_source.translation.isZero(1e-12));
  EXPECT_TRUE(identity->T_target_source.rotation.isApprox(
      Eigen::Quaterniond::Identity(), 1e-12));
}

TEST(Transform3d, RejectsFrameAndTimestampMismatch) {
  const auto timestamp = Timestamp::FromSeconds(10.0);
  const auto later = Timestamp::FromSeconds(11.0);
  ASSERT_TRUE(timestamp && later);

  const StampedTransform3d T_map_odom{
      *timestamp, FrameId::kMap, FrameId::kOdom,
      RigidTransform3d::Identity()};
  const StampedTransform3d wrong_frame{
      *timestamp, FrameId::kLidar, FrameId::kBase,
      RigidTransform3d::Identity()};
  const StampedTransform3d wrong_time{
      *later, FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};

  EXPECT_FALSE(T_map_odom.compose(wrong_frame));
  EXPECT_FALSE(T_map_odom.compose(wrong_time));
}

TEST(Transform3d, PreservesKnownM2dgrLidarToImuExtrinsic) {
  // src/super_lio/config/M2DGR.yaml: lio.extrinsic.lidar_imu.
  adagraph::frontend::FrontendConfig config;
  config.T_imu_lidar = {
      FrameId::kImu,
      FrameId::kLidar,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.29, 0.0, 0.84)},
  };
  ASSERT_FALSE(config.validate());

  const Eigen::Vector3d imu_origin =
      config.T_imu_lidar.T_target_source.transformPoint(
          Eigen::Vector3d::Zero());
  EXPECT_TRUE(imu_origin.isApprox(Eigen::Vector3d(0.29, 0.0, 0.84)));
  const Eigen::Vector3d lidar_origin =
      config.T_imu_lidar.inverse().T_target_source.transformPoint(imu_origin);
  EXPECT_TRUE(lidar_origin.isZero(1e-12));
}

TEST(Transform3d, ConvertsLegacyRobotExtrinsicToOdomBaseExactly) {
  // Legacy g_odom_robo stores R_imu_base and p_base_imu. The typed static
  // transform is T_base_imu = {R_imu_base^T, p_base_imu}.
  const Eigen::Matrix3d R_imu_base =
      Eigen::AngleAxisd(3.141592653589793,
                        Eigen::Vector3d::UnitX()).toRotationMatrix();
  const adagraph::geometry::StaticTransform3d T_base_imu{
      FrameId::kBase, FrameId::kImu,
      {Eigen::Quaterniond(R_imu_base.transpose()),
       Eigen::Vector3d(-0.053, 0.0, 0.735)}};
  const StampedTransform3d T_odom_imu{
      Timestamp::FromNanoseconds(123), FrameId::kOdom, FrameId::kImu,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 2.0, 3.0)}};

  const auto T_odom_base = T_odom_imu.compose(T_base_imu.inverse());
  ASSERT_TRUE(T_odom_base);
  EXPECT_EQ(T_odom_base->source_frame, FrameId::kBase);
  EXPECT_TRUE(T_odom_base->T_target_source.rotation.toRotationMatrix().isApprox(
      R_imu_base, 1e-12));
  EXPECT_TRUE(T_odom_base->T_target_source.translation.isApprox(
      Eigen::Vector3d(1.053, 2.0, 3.735), 1e-12));
}

TEST(PointCloudFrame, MovesBufferWithoutCopyAndExposesConstView) {
  adagraph::sensor::PointCloudBuffer points{
      {1.0F, 2.0F, 3.0F, 4.0F, 100},
      {5.0F, 6.0F, 7.0F, 8.0F, 200},
  };
  const auto* const original_storage = points.data();
  const Timestamp start = Timestamp::FromNanoseconds(1'000);
  const Timestamp end = Timestamp::FromNanoseconds(2'000);

  adagraph::sensor::PointCloudFrame frame(
      start, end, FrameId::kLidar, std::move(points));
  ASSERT_TRUE(frame.valid());
  ASSERT_EQ(frame.points().size(), 2U);
  EXPECT_EQ(frame.points().data(), original_storage);
  EXPECT_FLOAT_EQ(frame.points()[1].z, 7.0F);

  adagraph::sensor::PointCloudFrame moved(std::move(frame));
  EXPECT_EQ(moved.points().data(), original_storage);
  auto released = std::move(moved).releasePoints();
  EXPECT_EQ(released.data(), original_storage);
}

TEST(ImmutablePointCloud, SharesConcreteStorageWithoutFormatCopy) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto mutable_storage = std::make_shared<Buffer>(Buffer{
      {1.0F, 2.0F, 3.0F, 4.0F, 0},
      {5.0F, 6.0F, 7.0F, 8.0F, 10},
  });
  const void* const original_data = mutable_storage->data();
  std::shared_ptr<const Buffer> frozen_storage = mutable_storage;
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      frozen_storage->data(), frozen_storage->size(),
      sizeof(adagraph::sensor::PointXYZIT),
      offsetof(adagraph::sensor::PointXYZIT, x),
      offsetof(adagraph::sensor::PointXYZIT, y),
      offsetof(adagraph::sensor::PointXYZIT, z), FrameId::kBase);
  const auto cloud = adagraph::sensor::ImmutablePointCloud::Share(
      frozen_storage, view);
  mutable_storage.reset();
  frozen_storage.reset();

  ASSERT_TRUE(cloud.valid());
  EXPECT_EQ(cloud.view().storageIdentity(), original_data);
  const auto recovered = cloud.ownerAs<Buffer>();
  ASSERT_TRUE(recovered);
  EXPECT_EQ(recovered->data(), original_data);
  EXPECT_FALSE(cloud.ownerAs<std::vector<float>>());
  EXPECT_EQ(cloud.size(), 2U);
  std::array<float, 3> xyz{};
  ASSERT_TRUE(cloud.view().xyz(1, xyz));
  EXPECT_FLOAT_EQ(xyz[2], 7.0F);
}

TEST(OccupancyGridBuilder, BuildsRaycastSnapshotFromBorrowedCloudView) {
  struct PointXYZ {
    float x;
    float y;
    float z;
  };
  const std::array<PointXYZ, 1> points{{{2.1F, 0.0F, 0.0F}}};
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      points.data(), points.size(), sizeof(PointXYZ), offsetof(PointXYZ, x),
      offsetof(PointXYZ, y), offsetof(PointXYZ, z), FrameId::kUnknown);

  adagraph::mapping::OccupancyGridConfig config;
  config.resolution = 1.0;
  config.z_min = -1.0;
  config.z_max = 1.0;
  config.map_padding = 0.0;
  config.max_range = 10.0;
  config.max_extent = 20.0;
  config.raycast_max_ray_length = 10.0;
  adagraph::mapping::OccupancyGridBuilder builder(config);
  ASSERT_TRUE(builder.valid());
  EXPECT_TRUE(builder.insertScan(view, Eigen::Vector3d::Zero()));

  const auto snapshot = builder.snapshot(7, 1234);
  ASSERT_TRUE(snapshot.valid());
  EXPECT_EQ(snapshot.version, 7U);
  EXPECT_EQ(snapshot.timestamp_nanoseconds, 1234);
  EXPECT_EQ(snapshot.origin_x, 0);
  EXPECT_EQ(snapshot.origin_y, 0);
  EXPECT_EQ(snapshot.width, 3U);
  EXPECT_EQ(snapshot.height, 1U);
  ASSERT_EQ(snapshot.data.size(), 3U);
  EXPECT_EQ(snapshot.data[0], 0);   // ray-carved free cell
  EXPECT_EQ(snapshot.data[1], 0);   // ray-carved free cell
  EXPECT_EQ(snapshot.data[2], 100); // occupied endpoint
  EXPECT_EQ(view.storageIdentity(), points.data());
}

TEST(OccupancyGridBuilder, ResetAndConfigValidationAreCoreOnly) {
  adagraph::mapping::OccupancyGridConfig invalid;
  invalid.resolution = 0.0;
  EXPECT_TRUE(invalid.validate().has_value());

  adagraph::mapping::OccupancyGridBuilder builder;
  EXPECT_TRUE(builder.valid());
  struct PointXYZ {
    float x;
    float y;
    float z;
  };
  const PointXYZ point{0.0F, 0.0F, 0.0F};
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      &point, 1, sizeof(PointXYZ), offsetof(PointXYZ, x),
      offsetof(PointXYZ, y), offsetof(PointXYZ, z), FrameId::kUnknown);
  ASSERT_TRUE(builder.insertCloud(view));
  EXPECT_FALSE(builder.empty());
  builder.reset();
  EXPECT_TRUE(builder.empty());
  const auto empty = builder.snapshot(2, 0);
  ASSERT_TRUE(empty.valid());
  EXPECT_EQ(empty.width, 1U);
  EXPECT_EQ(empty.height, 1U);
  ASSERT_EQ(empty.data.size(), 1U);
  EXPECT_EQ(empty.data.front(), -1);
}

TEST(OccupancyGridBuilder, RebuildsFromImmutableMapSnapshot) {
  const auto keyframe = MakeKeyFrame(3, 100,
                                     Eigen::Vector3d(2.0, 0.0, 0.0));
  const auto timestamp = Timestamp::FromNanoseconds(100);
  adagraph::mapping::KeyFrameSnapshot snapshot_keyframe{
      adagraph::common::NodeId(9),
      keyframe.id(),
      keyframe.trajectoryId(),
      keyframe.sessionId(),
      adagraph::common::SubmapId(1),
      keyframe.T_odom_base(),
      {timestamp, FrameId::kMap, FrameId::kBase,
       {Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0)}},
      keyframe.cloudBase(),
      keyframe.quality(),
  };
  adagraph::mapping::MapSnapshot snapshot;
  snapshot.version = 4;
  snapshot.keyframes.push_back(std::move(snapshot_keyframe));
  ASSERT_TRUE(snapshot.valid());

  adagraph::mapping::OccupancyGridConfig config;
  config.resolution = 1.0;
  config.z_min = -1.0;
  config.z_max = 1.0;
  config.map_padding = 0.0;
  config.max_extent = 20.0;
  adagraph::mapping::OccupancyGridBuilder builder(config);
  ASSERT_TRUE(builder.rebuildFromSnapshot(snapshot));

  const auto grid = builder.snapshot(snapshot.version, timestamp.nanoseconds());
  ASSERT_TRUE(grid.valid());
  EXPECT_EQ(grid.origin_x, 5);
  EXPECT_EQ(grid.origin_y, 0);
  EXPECT_EQ(grid.width, 1U);
  EXPECT_EQ(grid.height, 1U);
  ASSERT_EQ(grid.data.size(), 1U);
  EXPECT_EQ(grid.data.front(), 100);
}

TEST(OccupancyGridBuilder, RebuildHonorsCancellationAndScratchBudget) {
  const auto keyframe = MakeKeyFrame(4, 200,
                                     Eigen::Vector3d(1.0, 0.0, 0.0));
  const auto timestamp = Timestamp::FromNanoseconds(200);
  adagraph::mapping::KeyFrameSnapshot snapshot_keyframe{
      adagraph::common::NodeId(10),
      keyframe.id(),
      keyframe.trajectoryId(),
      keyframe.sessionId(),
      adagraph::common::SubmapId(1),
      keyframe.T_odom_base(),
      {timestamp, FrameId::kMap, FrameId::kBase,
       {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}},
      keyframe.cloudBase(),
      keyframe.quality(),
  };
  adagraph::mapping::MapSnapshot snapshot;
  snapshot.version = 5;
  snapshot.keyframes.push_back(std::move(snapshot_keyframe));
  ASSERT_TRUE(snapshot.valid());

  adagraph::mapping::OccupancyGridBuilder builder;
  adagraph::mapping::OccupancyGridRebuildOptions cancelled;
  cancelled.should_cancel = [] { return true; };
  const auto cancelled_stats =
      builder.rebuildFromSnapshot(snapshot, cancelled);
  EXPECT_EQ(cancelled_stats.status,
            adagraph::mapping::OccupancyGridRebuildStatus::kCancelled);
  EXPECT_EQ(cancelled_stats.processed_points, 0U);
  EXPECT_TRUE(builder.empty());

  adagraph::mapping::OccupancyGridRebuildOptions budget;
  budget.max_scratch_memory_bytes = 1;
  const auto budget_stats = builder.rebuildFromSnapshot(snapshot, budget);
  EXPECT_EQ(
      budget_stats.status,
      adagraph::mapping::OccupancyGridRebuildStatus::kScratchBudgetExceeded);
  EXPECT_FALSE(budget_stats.succeeded());
  EXPECT_TRUE(builder.empty());
}

TEST(OccupancyGridBuilder, RebuildHonorsPointBudgetWithoutPartialCommit) {
  const auto first = MakeKeyFrame(5, 300, Eigen::Vector3d(1.0, 0.0, 0.0));
  const auto second = MakeKeyFrame(6, 301, Eigen::Vector3d(2.0, 0.0, 0.0));
  const auto make_snapshot_keyframe =
      [](const adagraph::mapping::KeyFrame& keyframe,
         adagraph::common::NodeId node_id,
         std::int64_t timestamp_ns) {
        const auto timestamp = Timestamp::FromNanoseconds(timestamp_ns);
        return adagraph::mapping::KeyFrameSnapshot{
            node_id,
            keyframe.id(),
            keyframe.trajectoryId(),
            keyframe.sessionId(),
            adagraph::common::SubmapId(1),
            keyframe.T_odom_base(),
            {timestamp, FrameId::kMap, FrameId::kBase,
             {Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero()}},
            keyframe.cloudBase(),
            keyframe.quality(),
        };
      };

  adagraph::mapping::MapSnapshot snapshot;
  snapshot.version = 6;
  snapshot.keyframes.push_back(make_snapshot_keyframe(
      first, adagraph::common::NodeId(11), 300));
  snapshot.keyframes.push_back(make_snapshot_keyframe(
      second, adagraph::common::NodeId(12), 301));
  ASSERT_TRUE(snapshot.valid());

  adagraph::mapping::OccupancyGridBuilder builder;
  adagraph::mapping::OccupancyGridRebuildOptions options;
  options.max_points = 1;
  const auto stats = builder.rebuildFromSnapshot(snapshot, options);
  EXPECT_EQ(stats.status,
            adagraph::mapping::OccupancyGridRebuildStatus::kPointBudgetExceeded);
  EXPECT_EQ(stats.processed_points, 1U);
  EXPECT_FALSE(stats.succeeded());
  EXPECT_TRUE(builder.empty());
}

TEST(MapSaveScheduler, ReliablyWritesImmutablePointCloudProduct) {
  namespace fs = std::filesystem;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path directory =
      fs::temp_directory_path() / ("adagraph_map_io_" +
                                   std::to_string(suffix));
  const fs::path output = directory / "snapshot.pcd";
  std::error_code error;
  fs::remove_all(directory, error);

  std::vector<adagraph::io::MapSaveResult> results;
  auto product =
      std::make_shared<adagraph::mapping::SnapshotPointCloudMap>();
  product->snapshot_version = 17;
  product->points = {{0.01F, 0.01F, 0.0F},
                     {0.02F, 0.01F, 0.0F},
                     {1.01F, 0.01F, 0.0F}};

  adagraph::io::MapSaveScheduler scheduler(
      1, [&results](const adagraph::io::MapSaveResult& result) {
        results.push_back(result);
      });
  adagraph::io::MapSaveRequest request;
  request.request_id = 9;
  request.point_cloud = product;
  request.output_path = output;
  request.voxel_leaf_size = 1.0;
  EXPECT_EQ(scheduler.submit(std::move(request)),
            adagraph::io::MapSaveSubmissionStatus::kAccepted);
  scheduler.finish();

  ASSERT_EQ(results.size(), 1U);
  EXPECT_TRUE(results.front().success);
  EXPECT_EQ(results.front().request_id, 9U);
  EXPECT_EQ(results.front().snapshot_version, 17U);
  EXPECT_EQ(results.front().raw_point_count, 3U);
  EXPECT_EQ(results.front().saved_point_count, 2U);
  EXPECT_TRUE(fs::exists(output));
  EXPECT_GT(fs::file_size(output), 128U);
  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.submitted, 1U);
  EXPECT_EQ(stats.processed, 1U);
  EXPECT_EQ(stats.failed, 0U);

  fs::remove_all(directory, error);
}

TEST(MapSaveScheduler, MergesQueuedPcdFragmentsAfterFragmentWrites) {
  namespace fs = std::filesystem;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path directory =
      fs::temp_directory_path() / ("adagraph_fragment_io_" +
                                   std::to_string(suffix));
  const fs::path fragment_a = directory / "PCD" / "scans_0.pcd";
  const fs::path fragment_b = directory / "PCD" / "scans_1.pcd";
  const fs::path output = directory / "map.pcd";
  std::error_code error;
  fs::remove_all(directory, error);

  std::vector<adagraph::io::MapSaveResult> results;
  auto make_product = [](std::initializer_list<adagraph::mapping::MapPoint3f>
                             points) {
    auto product =
        std::make_shared<adagraph::mapping::SnapshotPointCloudMap>();
    product->points.assign(points.begin(), points.end());
    return product;
  };

  adagraph::io::MapSaveScheduler scheduler(
      4, [&results](const adagraph::io::MapSaveResult& result) {
        results.push_back(result);
      });
  adagraph::io::MapSaveRequest first;
  first.request_id = 10;
  first.point_cloud = make_product({{0.0F, 0.0F, 0.0F},
                                    {0.1F, 0.0F, 0.0F}});
  first.output_path = fragment_a;
  EXPECT_EQ(scheduler.submit(std::move(first)),
            adagraph::io::MapSaveSubmissionStatus::kAccepted);

  adagraph::io::MapSaveRequest second;
  second.request_id = 11;
  second.point_cloud = make_product({{0.2F, 0.0F, 0.0F},
                                     {1.0F, 0.0F, 0.0F}});
  second.output_path = fragment_b;
  EXPECT_EQ(scheduler.submit(std::move(second)),
            adagraph::io::MapSaveSubmissionStatus::kAccepted);

  auto fragments = std::make_shared<adagraph::io::PcdFragmentSet>();
  fragments->input_paths = {fragment_a, fragment_b};
  fragments->require_all_inputs = true;
  adagraph::io::MapSaveRequest merge;
  merge.request_id = 12;
  merge.fragment_set = fragments;
  merge.output_path = output;
  merge.voxel_leaf_size = 0.5;
  EXPECT_EQ(scheduler.submit(std::move(merge)),
            adagraph::io::MapSaveSubmissionStatus::kAccepted);
  scheduler.finish();

  ASSERT_EQ(results.size(), 3U);
  ASSERT_TRUE(results.back().success);
  EXPECT_EQ(results.back().request_id, 12U);
  EXPECT_EQ(results.back().input_file_count, 2U);
  EXPECT_EQ(results.back().raw_point_count, 4U);
  EXPECT_EQ(results.back().saved_point_count, 2U);
  EXPECT_TRUE(fs::exists(fragment_a));
  EXPECT_TRUE(fs::exists(fragment_b));
  ASSERT_TRUE(fs::exists(output));

  std::ifstream header(output, std::ios::binary);
  ASSERT_TRUE(header.good());
  std::string line;
  bool saw_points = false;
  while (std::getline(header, line)) {
    if (line.rfind("POINTS ", 0) == 0) {
      saw_points = true;
      EXPECT_EQ(line, "POINTS 2");
    }
    if (line.rfind("DATA ", 0) == 0) {
      break;
    }
  }
  EXPECT_TRUE(saw_points);

  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.submitted, 3U);
  EXPECT_EQ(stats.processed, 3U);
  EXPECT_EQ(stats.failed, 0U);
  fs::remove_all(directory, error);
}

TEST(Nav2GridArtifact, WritesMapServerCompatiblePgmAndMetadataAtomically) {
  namespace fs = std::filesystem;
  const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path directory =
      fs::temp_directory_path() / ("adagraph_nav2_grid_" +
                                   std::to_string(suffix));
  const fs::path yaml_path = directory / "online_map.yaml";
  std::error_code error;
  fs::remove_all(directory, error);

  auto grid = std::make_shared<adagraph::mapping::OccupancyGridSnapshot>();
  grid->version = 7;
  grid->timestamp_nanoseconds = 123456789;
  grid->resolution = 0.5;
  grid->origin_x = -2;
  grid->origin_y = 3;
  grid->width = 2;
  grid->height = 2;
  // Rows are lower-to-upper in OccupancyGridSnapshot.
  grid->data = {-1, 0, 100, 42};
  ASSERT_TRUE(grid->valid());

  adagraph::io::Nav2GridArtifactRequest request;
  request.request_id = 11;
  request.grid = grid;
  request.yaml_path = yaml_path;
  request.frame_id = "map";
  const auto written = adagraph::io::WriteNav2GridArtifact(request);

  ASSERT_TRUE(written.success) << written.message;
  EXPECT_EQ(written.request_id, 11U);
  EXPECT_EQ(written.grid_version, 7U);
  EXPECT_EQ(written.image_path, directory / "online_map.pgm");
  EXPECT_EQ(written.metadata_path, directory / "online_map.metadata.yaml");
  EXPECT_TRUE(fs::exists(written.yaml_path));
  EXPECT_TRUE(fs::exists(written.image_path));
  EXPECT_TRUE(fs::exists(written.metadata_path));

  std::ifstream pgm(written.image_path, std::ios::binary);
  ASSERT_TRUE(pgm.good());
  std::string line;
  ASSERT_TRUE(std::getline(pgm, line));
  EXPECT_EQ(line, "P5");
  ASSERT_TRUE(std::getline(pgm, line));
  EXPECT_EQ(line, "# AdaGraph-LIO online occupancy artifact");
  ASSERT_TRUE(std::getline(pgm, line));
  EXPECT_EQ(line, "2 2");
  ASSERT_TRUE(std::getline(pgm, line));
  EXPECT_EQ(line, "255");
  const std::vector<unsigned char> pixels{
      std::istreambuf_iterator<char>(pgm), std::istreambuf_iterator<char>()};
  // PGM is top-to-bottom, so the upper snapshot row appears first.
  EXPECT_EQ(pixels, (std::vector<unsigned char>{0, 205, 205, 254}));

  std::ifstream yaml(written.yaml_path);
  ASSERT_TRUE(yaml.good());
  const std::string yaml_text{std::istreambuf_iterator<char>(yaml), {}};
  EXPECT_NE(yaml_text.find("image: \"online_map.pgm\""), std::string::npos);
  EXPECT_NE(yaml_text.find("resolution: 0.5"), std::string::npos);
  EXPECT_NE(yaml_text.find("origin: [-1, 1.5, 0.0]"), std::string::npos);
  EXPECT_NE(yaml_text.find("mode: trinary"), std::string::npos);
  EXPECT_NE(yaml_text.find("occupied_thresh: 0.65"), std::string::npos);
  EXPECT_NE(yaml_text.find("free_thresh: 0.196"), std::string::npos);

  std::ifstream metadata(written.metadata_path);
  ASSERT_TRUE(metadata.good());
  const std::string metadata_text{
      std::istreambuf_iterator<char>(metadata), {}};
  EXPECT_NE(metadata_text.find("frame_id: \"map\""), std::string::npos);
  EXPECT_NE(metadata_text.find("source: \"online\""), std::string::npos);
  EXPECT_NE(metadata_text.find("grid_version: 7"), std::string::npos);
  EXPECT_NE(metadata_text.find("timestamp_nanoseconds: 123456789"),
            std::string::npos);

  grid->version = 8;
  grid->data = {-1, 0, 100, 42};
  request.request_id = 12;
  const auto overwritten = adagraph::io::WriteNav2GridArtifact(request);
  ASSERT_TRUE(overwritten.success) << overwritten.message;
  EXPECT_EQ(overwritten.grid_version, 8U);
  std::ifstream updated_metadata(overwritten.metadata_path);
  const std::string updated_metadata_text{
      std::istreambuf_iterator<char>(updated_metadata), {}};
  EXPECT_NE(updated_metadata_text.find("grid_version: 8"), std::string::npos);
  std::size_t files = 0;
  for (const auto& entry : fs::directory_iterator(directory)) {
    ++files;
    EXPECT_TRUE(entry.path().extension() == ".yaml" ||
                entry.path().extension() == ".pgm");
  }
  EXPECT_EQ(files, 3U);

  request.yaml_path = directory / "invalid.txt";
  EXPECT_FALSE(request.valid());
  request.yaml_path = yaml_path;
  request.frame_id = "map\ninvalid";
  EXPECT_FALSE(request.valid());
  if (std::getenv("ADAGRAPH_KEEP_TEST_ARTIFACTS") == nullptr) {
    fs::remove_all(directory, error);
  }
}

TEST(KeyFrame, EnforcesOdomBaseAndCloudBaseContract) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{
      {1.0F, 2.0F, 3.0F, 4.0F, 0},
  });
  const auto base_view =
      adagraph::sensor::PointCloudView::InterleavedFloatXyz(
          storage->data(), storage->size(),
          sizeof(adagraph::sensor::PointXYZIT),
          offsetof(adagraph::sensor::PointXYZIT, x),
          offsetof(adagraph::sensor::PointXYZIT, y),
          offsetof(adagraph::sensor::PointXYZIT, z), FrameId::kBase);
  auto cloud = adagraph::sensor::ImmutablePointCloud::Share(
      storage, base_view);
  const StampedTransform3d T_odom_base{
      Timestamp::FromNanoseconds(123), FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};
  const adagraph::mapping::KeyFrame keyframe(
      adagraph::common::KeyFrameId(7),
      adagraph::common::TrajectoryId(2),
      adagraph::common::SessionId(1), T_odom_base, cloud, {});

  EXPECT_TRUE(keyframe.valid());
  EXPECT_EQ(keyframe.id().value(), 7U);
  EXPECT_EQ(keyframe.timestamp().nanoseconds(), 123);
  EXPECT_EQ(keyframe.cloudBase().view().storageIdentity(), storage->data());

  const StampedTransform3d T_odom_imu{
      Timestamp::FromNanoseconds(123), FrameId::kOdom, FrameId::kImu,
      RigidTransform3d::Identity()};
  const adagraph::mapping::KeyFrame invalid_keyframe(
      adagraph::common::KeyFrameId(8),
      adagraph::common::TrajectoryId(2),
      adagraph::common::SessionId(1), T_odom_imu, cloud, {});
  EXPECT_FALSE(invalid_keyframe.valid());
}

TEST(PoseGraph, OwnsTypedNodesAndConstraintsWithoutOptimizerDependency) {
  using adagraph::backend::Constraint;
  using adagraph::backend::ConstraintType;
  using adagraph::backend::PoseGraph;
  using adagraph::backend::PoseGraphNode;
  using adagraph::backend::RelativePoseMeasurement;

  PoseGraph graph;
  PoseGraphNode first;
  first.keyframe_id = adagraph::common::KeyFrameId(10);
  first.trajectory_id = adagraph::common::TrajectoryId(2);
  first.session_id = adagraph::common::SessionId(1);
  first.T_odom_base = {
      Timestamp::FromNanoseconds(100), FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};
  const auto first_id = graph.addNode(first);
  ASSERT_TRUE(first_id);

  PoseGraphNode second = first;
  second.keyframe_id = adagraph::common::KeyFrameId(11);
  second.T_odom_base = {
      Timestamp::FromNanoseconds(200), FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)}};
  const auto second_id = graph.addNode(second);
  ASSERT_TRUE(second_id);

  Constraint odometry;
  odometry.type = ConstraintType::kOdometry;
  odometry.measurement = RelativePoseMeasurement{
      *first_id, *second_id, Timestamp::FromNanoseconds(200),
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(1.0, 0.0, 0.0)},
      adagraph::backend::Covariance6d::Identity() * 0.01};
  EXPECT_TRUE(graph.addConstraint(odometry));
  EXPECT_EQ(graph.nodeCount(), 2U);
  EXPECT_EQ(graph.constraintCount(), 1U);

  Constraint unknown = odometry;
  std::get<RelativePoseMeasurement>(unknown.measurement).to_node =
      adagraph::common::NodeId(999);
  EXPECT_FALSE(graph.addConstraint(unknown));

  const auto state = graph.buildState();
  EXPECT_EQ(state.nodes.size(), 2U);
  EXPECT_EQ(state.constraints.size(), 1U);
  EXPECT_EQ(state.version, 3U);
}

TEST(PoseGraph, AppliesOnlyConsistentVersionedOptimizationResults) {
  using adagraph::backend::OptimizationResult;
  using adagraph::backend::OptimizedNodePose;
  using adagraph::backend::PoseGraph;
  using adagraph::backend::PoseGraphNode;

  PoseGraph graph;
  PoseGraphNode node;
  node.keyframe_id = adagraph::common::KeyFrameId(1);
  node.trajectory_id = adagraph::common::TrajectoryId(1);
  node.session_id = adagraph::common::SessionId(1);
  node.T_odom_base = {
      Timestamp::FromNanoseconds(42), FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};
  const auto node_id = graph.addNode(node);
  ASSERT_TRUE(node_id);

  const StampedTransform3d T_map_base{
      Timestamp::FromNanoseconds(42), FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}};
  OptimizationResult result{
      graph.version(), {OptimizedNodePose{*node_id, T_map_base}}};
  EXPECT_TRUE(graph.apply(result));
  EXPECT_EQ(graph.version(), 2U);
  ASSERT_TRUE(graph.buildState().nodes.front().T_map_base);
  EXPECT_DOUBLE_EQ(
      graph.buildState().nodes.front().T_map_base->T_target_source.translation.x(),
      5.0);

  // A stale optimizer result cannot overwrite a newer committed graph.
  EXPECT_FALSE(graph.apply(result));

  OptimizationResult duplicate{
      graph.version(),
      {OptimizedNodePose{*node_id, T_map_base},
       OptimizedNodePose{*node_id, T_map_base}}};
  EXPECT_FALSE(graph.apply(duplicate));
  EXPECT_EQ(graph.version(), 2U);
}

TEST(ConstraintBuilder, ProducesTypedOdometryWithoutFrameAmbiguity) {
  using adagraph::backend::ConstraintBuilder;
  using adagraph::backend::PoseGraphNode;
  using adagraph::backend::RelativePoseMeasurement;

  PoseGraphNode from;
  from.id = adagraph::common::NodeId(5);
  from.keyframe_id = adagraph::common::KeyFrameId(10);
  from.trajectory_id = adagraph::common::TrajectoryId(2);
  from.session_id = adagraph::common::SessionId(3);
  from.T_odom_base = {
      Timestamp::FromNanoseconds(100), FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond(Eigen::AngleAxisd(
           1.5707963267948966, Eigen::Vector3d::UnitZ())),
       Eigen::Vector3d(1.0, 2.0, 0.0)}};

  PoseGraphNode to = from;
  to.id = adagraph::common::NodeId(6);
  to.keyframe_id = adagraph::common::KeyFrameId(11);
  to.T_odom_base = {
      Timestamp::FromNanoseconds(200), FrameId::kOdom, FrameId::kBase,
      {from.T_odom_base.T_target_source.rotation,
       Eigen::Vector3d(1.0, 3.0, 0.0)}};

  const ConstraintBuilder builder;
  const auto constraint = builder.makeOdometry(
      from, to, adagraph::backend::Covariance6d::Identity() * 0.02);
  ASSERT_TRUE(constraint);
  EXPECT_EQ(constraint->type,
            adagraph::backend::ConstraintType::kOdometry);
  const auto& measurement =
      std::get<RelativePoseMeasurement>(constraint->measurement);
  EXPECT_EQ(measurement.from_node, from.id);
  EXPECT_EQ(measurement.to_node, to.id);
  EXPECT_EQ(measurement.timestamp, to.T_odom_base.timestamp);
  EXPECT_TRUE(measurement.T_from_to.translation.isApprox(
      Eigen::Vector3d(1.0, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(measurement.T_from_to.rotation.isApprox(
      Eigen::Quaterniond::Identity(), 1e-12));
}

TEST(ConstraintBuilder, RejectsInvalidTrajectoryTimeAndGlobalFrames) {
  using adagraph::backend::ConstraintBuilder;
  using adagraph::backend::PoseGraphNode;

  PoseGraphNode from;
  from.id = adagraph::common::NodeId(1);
  from.keyframe_id = adagraph::common::KeyFrameId(1);
  from.trajectory_id = adagraph::common::TrajectoryId(7);
  from.session_id = adagraph::common::SessionId(8);
  from.T_odom_base = {
      Timestamp::FromNanoseconds(100), FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};
  PoseGraphNode to = from;
  to.id = adagraph::common::NodeId(2);
  to.keyframe_id = adagraph::common::KeyFrameId(2);
  to.T_odom_base.timestamp = Timestamp::FromNanoseconds(200);

  const ConstraintBuilder builder;
  const auto covariance = adagraph::backend::Covariance6d::Identity();
  PoseGraphNode other_trajectory = to;
  other_trajectory.trajectory_id = adagraph::common::TrajectoryId(9);
  EXPECT_FALSE(builder.makeOdometry(from, other_trajectory, covariance));

  PoseGraphNode older = to;
  older.T_odom_base.timestamp = Timestamp::FromNanoseconds(50);
  EXPECT_FALSE(builder.makeOdometry(from, older, covariance));

  const StampedTransform3d T_map_base{
      to.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
      RigidTransform3d::Identity()};
  EXPECT_TRUE(builder.makeRelocalization(to, T_map_base, covariance, 0.9));

  StampedTransform3d wrong_frame = T_map_base;
  wrong_frame.source_frame = FrameId::kImu;
  EXPECT_FALSE(
      builder.makeRelocalization(to, wrong_frame, covariance, 0.9));

  StampedTransform3d wrong_time = T_map_base;
  wrong_time.timestamp = Timestamp::FromNanoseconds(201);
  EXPECT_FALSE(builder.makePrior(to, wrong_time, covariance));
}

TEST(BackendScheduler, ReliablyDrainsAcceptedKeyFramesInGraphOrder) {
  adagraph::backend::BackendConfig config;
  config.submission_queue_capacity = 2;
  adagraph::backend::BackendScheduler scheduler(config);

  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(10, 100)),
            adagraph::backend::SubmissionStatus::kAccepted);
  EXPECT_EQ(scheduler.submitKeyFrame(
                MakeKeyFrame(11, 200, Eigen::Vector3d(1.0, 0.0, 0.0))),
            adagraph::backend::SubmissionStatus::kAccepted);
  EXPECT_EQ(scheduler.submitKeyFrame(
                MakeKeyFrame(12, 300, Eigen::Vector3d(3.0, 0.0, 0.0))),
            adagraph::backend::SubmissionStatus::kAccepted);

  EXPECT_FALSE(scheduler.finalState());
  scheduler.finish();

  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.enqueued, 3U);
  EXPECT_EQ(stats.processed, 3U);
  EXPECT_EQ(stats.failed, 0U);
  EXPECT_EQ(stats.node_count, 3U);
  EXPECT_EQ(stats.constraint_count, 2U);
  EXPECT_EQ(stats.optimizer_updates, 0U);
  EXPECT_EQ(stats.map_snapshot_version, 3U);
  EXPECT_LE(stats.maximum_queue_depth, 2U);

  const auto state = scheduler.finalState();
  ASSERT_TRUE(state);
  ASSERT_EQ(state->nodes.size(), 3U);
  ASSERT_EQ(state->constraints.size(), 2U);
  const auto& second_relative =
      std::get<adagraph::backend::RelativePoseMeasurement>(
          state->constraints[1].measurement);
  EXPECT_TRUE(second_relative.T_from_to.translation.isApprox(
      Eigen::Vector3d(2.0, 0.0, 0.0)));

  const auto map_snapshot = scheduler.mapSnapshot();
  ASSERT_TRUE(map_snapshot);
  EXPECT_TRUE(map_snapshot->valid());
  EXPECT_EQ(map_snapshot->version, 3U);
  EXPECT_EQ(map_snapshot->trajectories.size(), 1U);
  EXPECT_EQ(map_snapshot->submaps.size(), 1U);
  EXPECT_EQ(map_snapshot->keyframes.size(), 3U);
  EXPECT_EQ(map_snapshot->constraints.size(), 2U);
  EXPECT_TRUE(map_snapshot->consistent());

  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(13, 400)),
            adagraph::backend::SubmissionStatus::kClosed);
}

TEST(BackendScheduler, ObserverReceivesCommittedSnapshotsInVersionOrder) {
  adagraph::backend::BackendConfig config;
  std::vector<std::uint64_t> observed_versions;
  adagraph::backend::BackendScheduler scheduler(
      config, nullptr,
      [&observed_versions](
          std::shared_ptr<const adagraph::mapping::MapSnapshot> snapshot) {
        ASSERT_TRUE(snapshot);
        observed_versions.push_back(snapshot->version);
      });
  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(1, 100)),
            adagraph::backend::SubmissionStatus::kAccepted);
  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(2, 200)),
            adagraph::backend::SubmissionStatus::kAccepted);
  scheduler.finish();

  EXPECT_EQ(observed_versions, (std::vector<std::uint64_t>{1, 2}));
  EXPECT_EQ(scheduler.stats().snapshot_observer_failures, 0U);
}

TEST(BackendScheduler, ObserverFailureDoesNotCorruptCommittedGraph) {
  adagraph::backend::BackendConfig config;
  adagraph::backend::BackendScheduler scheduler(
      config, nullptr,
      [](std::shared_ptr<const adagraph::mapping::MapSnapshot>) {
        throw std::runtime_error("consumer failure");
      });
  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(1, 100)),
            adagraph::backend::SubmissionStatus::kAccepted);
  scheduler.finish();

  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.failed, 0U);
  EXPECT_EQ(stats.node_count, 1U);
  EXPECT_EQ(stats.map_snapshot_version, 1U);
  EXPECT_EQ(stats.snapshot_observer_failures, 1U);
}

TEST(BackendScheduler, RollsAndFinishesBoundedSingleTrajectorySubmaps) {
  adagraph::backend::BackendConfig config;
  config.submap_max_keyframes = 2;
  adagraph::backend::BackendScheduler scheduler(config);
  for (std::uint64_t id = 0; id < 5; ++id) {
    EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(id, id + 1)),
              adagraph::backend::SubmissionStatus::kAccepted);
  }
  scheduler.finish();

  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.submap_count, 3U);
  EXPECT_EQ(stats.finished_submap_count, 2U);
  const auto snapshot = scheduler.mapSnapshot();
  ASSERT_TRUE(snapshot->consistent());
  ASSERT_EQ(snapshot->submaps.size(), 3U);
  EXPECT_EQ(snapshot->submaps[0].state,
            adagraph::mapping::SubmapState::kFinished);
  EXPECT_EQ(snapshot->submaps[1].state,
            adagraph::mapping::SubmapState::kFinished);
  EXPECT_EQ(snapshot->submaps[2].state,
            adagraph::mapping::SubmapState::kActive);
  EXPECT_EQ(snapshot->submaps[0].keyframe_ids.size(), 2U);
  EXPECT_EQ(snapshot->submaps[1].keyframe_ids.size(), 2U);
  EXPECT_EQ(snapshot->submaps[2].keyframe_ids.size(), 1U);
}

TEST(BackendScheduler, AssociatesTimedGlobalPoseOnItsSingleWriter) {
  adagraph::backend::BackendConfig config;
  adagraph::backend::BackendScheduler scheduler(config);
  EXPECT_EQ(scheduler.submitKeyFrame(MakeKeyFrame(1, 100'000'000)),
            adagraph::backend::SubmissionStatus::kAccepted);

  adagraph::backend::TimedGlobalPoseConstraint request;
  request.type = adagraph::backend::ConstraintType::kRelocalization;
  request.session_id = adagraph::common::SessionId(1);
  request.trajectory_id = adagraph::common::TrajectoryId(1);
  request.T_map_base = {
      Timestamp::FromNanoseconds(110'000'000), FrameId::kMap,
      FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0)}};
  request.covariance = adagraph::backend::Covariance6d::Identity() * 0.01;
  request.score = 0.8;
  request.maximum_time_difference_s = 0.02;
  EXPECT_EQ(scheduler.submitGlobalPoseConstraint(request),
            adagraph::backend::SubmissionStatus::kAccepted);

  scheduler.finish();
  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.enqueued, 2U);
  EXPECT_EQ(stats.processed, 2U);
  EXPECT_EQ(stats.failed, 0U);
  EXPECT_EQ(stats.node_count, 1U);
  EXPECT_EQ(stats.constraint_count, 1U);
  EXPECT_EQ(stats.global_constraints_dropped, 0U);
  const auto state = scheduler.finalState();
  ASSERT_TRUE(state);
  ASSERT_EQ(state->constraints.size(), 1U);
  EXPECT_EQ(state->constraints.front().type,
            adagraph::backend::ConstraintType::kRelocalization);
  const auto& measurement =
      std::get<adagraph::backend::AbsolutePoseMeasurement>(
          state->constraints.front().measurement);
  EXPECT_EQ(measurement.T_map_base.timestamp,
            request.T_map_base.timestamp);
}

TEST(MapManager, PublishesConsistentImmutableSnapshotsToConcurrentReaders) {
  adagraph::mapping::MapManager manager;
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> invalid_reads{0};
  auto reader = [&] {
    std::uint64_t previous_version = 0;
    while (!stop.load(std::memory_order_acquire)) {
      const auto snapshot = manager.snapshot();
      if (!snapshot || !snapshot->valid() ||
          snapshot->version < previous_version ||
          snapshot->keyframes.size() != snapshot->version) {
        ++invalid_reads;
      }
      previous_version = snapshot ? snapshot->version : previous_version;
    }
  };
  std::thread first_reader(reader);
  std::thread second_reader(reader);

  bool writer_ok = true;
  for (std::uint64_t id = 0; id < 50; ++id) {
    const auto keyframe = MakeKeyFrame(
        id, static_cast<std::int64_t>((id + 1) * 100),
        Eigen::Vector3d(static_cast<double>(id), 0.0, 0.0));
    adagraph::backend::PoseGraphNode node;
    node.id = adagraph::common::NodeId(id);
    node.keyframe_id = keyframe.id();
    node.trajectory_id = keyframe.trajectoryId();
    node.session_id = keyframe.sessionId();
    node.T_odom_base = keyframe.T_odom_base();
    if (!manager.insertKeyFrame(
            keyframe, node, adagraph::common::SubmapId(0))) {
      writer_ok = false;
      break;
    }
    const auto snapshot = manager.publishSnapshot();
    if (!snapshot->valid() || snapshot->version != id + 1 ||
        snapshot->keyframes.size() != id + 1) {
      writer_ok = false;
      break;
    }
  }

  stop.store(true, std::memory_order_release);
  first_reader.join();
  second_reader.join();
  EXPECT_TRUE(writer_ok);
  EXPECT_EQ(invalid_reads.load(), 0U);
  const auto final_snapshot = manager.snapshot();
  ASSERT_EQ(final_snapshot->keyframes.size(), 50U);
  EXPECT_EQ(final_snapshot->submaps.front().keyframe_ids.size(), 50U);
  EXPECT_TRUE(final_snapshot->consistent());
}

TEST(MapManager, CountsSharedImmutableKeyframeCloudStorageOnce) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{
      {1.0F, 2.0F, 3.0F, 1.0F, 0},
      {4.0F, 5.0F, 6.0F, 1.0F, 0},
  });
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(adagraph::sensor::PointXYZIT),
      offsetof(adagraph::sensor::PointXYZIT, x),
      offsetof(adagraph::sensor::PointXYZIT, y),
      offsetof(adagraph::sensor::PointXYZIT, z), FrameId::kBase);
  constexpr std::size_t kBackingBytes = 4096;
  const auto cloud = adagraph::sensor::ImmutablePointCloud::Share(
      storage, view, kBackingBytes);

  adagraph::mapping::MapManager manager;
  const auto first = MakeKeyFrameWithCloud(1, 100, cloud);
  const auto second = MakeKeyFrameWithCloud(2, 200, cloud);
  adagraph::backend::PoseGraphNode first_node;
  first_node.id = adagraph::common::NodeId(1);
  first_node.keyframe_id = first.id();
  first_node.trajectory_id = first.trajectoryId();
  first_node.session_id = first.sessionId();
  first_node.T_odom_base = first.T_odom_base();
  adagraph::backend::PoseGraphNode second_node;
  second_node.id = adagraph::common::NodeId(2);
  second_node.keyframe_id = second.id();
  second_node.trajectory_id = second.trajectoryId();
  second_node.session_id = second.sessionId();
  second_node.T_odom_base = second.T_odom_base();

  ASSERT_TRUE(manager.insertKeyFrame(
      first, first_node, adagraph::common::SubmapId(0)));
  ASSERT_TRUE(manager.insertKeyFrame(
      second, second_node, adagraph::common::SubmapId(0)));

  const auto memory = manager.memoryStats();
  EXPECT_EQ(memory.keyframe_cloud_count, 2U);
  EXPECT_EQ(memory.unique_keyframe_cloud_count, 1U);
  EXPECT_EQ(memory.unique_keyframe_cloud_points, storage->size());
  EXPECT_EQ(memory.unique_keyframe_cloud_logical_bytes,
            storage->size() * sizeof(adagraph::sensor::PointXYZIT));
  EXPECT_EQ(memory.unique_keyframe_cloud_backing_bytes, kBackingBytes);
}

TEST(MapSnapshotProducts, BuildsVersionedTrajectoryAndMapInGlobalFrame) {
  adagraph::mapping::MapManager manager;
  for (std::uint64_t id = 0; id < 2; ++id) {
    const auto keyframe = MakeKeyFrame(
        id + 1, static_cast<std::int64_t>((id + 1) * 100),
        Eigen::Vector3d(static_cast<double>(id) * 10.0, 0.0, 0.0));
    adagraph::backend::PoseGraphNode node;
    node.id = adagraph::common::NodeId(id);
    node.keyframe_id = keyframe.id();
    node.trajectory_id = keyframe.trajectoryId();
    node.session_id = keyframe.sessionId();
    node.T_odom_base = keyframe.T_odom_base();
    node.T_map_base = StampedTransform3d{
        keyframe.timestamp(), FrameId::kMap, FrameId::kBase,
        {Eigen::Quaterniond::Identity(),
         Eigen::Vector3d(static_cast<double>(id) * 2.0, 1.0, 0.0)}};
    ASSERT_TRUE(manager.insertKeyFrame(
        keyframe, node, adagraph::common::SubmapId(0)));
  }
  const auto snapshot = manager.publishSnapshot();
  ASSERT_TRUE(snapshot->consistent());

  const auto trajectory =
      adagraph::mapping::SnapshotProductBuilder::BuildTrajectory(*snapshot);
  ASSERT_TRUE(trajectory);
  EXPECT_EQ(trajectory->snapshot_version, snapshot->version);
  ASSERT_EQ(trajectory->poses.size(), 2U);
  EXPECT_EQ(trajectory->poses[1].node_id, adagraph::common::NodeId(1));
  EXPECT_TRUE(trajectory->poses[1].T_map_base.T_target_source.translation
                  .isApprox(Eigen::Vector3d(2.0, 1.0, 0.0)));

  const auto point_map =
      adagraph::mapping::SnapshotProductBuilder::BuildPointCloudMap(*snapshot);
  ASSERT_TRUE(point_map);
  EXPECT_EQ(point_map->snapshot_version, snapshot->version);
  ASSERT_EQ(point_map->points.size(), 2U);
  EXPECT_FLOAT_EQ(point_map->points[0].x, 1.0F);
  EXPECT_FLOAT_EQ(point_map->points[0].y, 1.0F);
  EXPECT_FLOAT_EQ(point_map->points[1].x, 4.0F);
  EXPECT_FLOAT_EQ(point_map->points[1].y, 1.0F);
  EXPECT_TRUE(point_map->minimum.isApprox(Eigen::Vector3f(1.0F, 1.0F, 0.0F)));
  EXPECT_TRUE(point_map->maximum.isApprox(Eigen::Vector3f(4.0F, 1.0F, 0.0F)));

  const auto localization_map =
      adagraph::localization::BuildLocalizationMap(*snapshot);
  ASSERT_TRUE(localization_map);
  EXPECT_TRUE(localization_map->valid());
  EXPECT_EQ(localization_map->version(), snapshot->version);
  EXPECT_EQ(localization_map->cloudMap().frame(), FrameId::kMap);
  EXPECT_EQ(localization_map->cloudMap().size(), 2U);
}

TEST(RelocalizationContract, EnforcesTimestampAndMapBaseFrameSemantics) {
  using Buffer = adagraph::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(Buffer{{1.0F, 0.0F, 0.0F}});
  const auto view = adagraph::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(), sizeof(adagraph::sensor::PointXYZIT),
      offsetof(adagraph::sensor::PointXYZIT, x),
      offsetof(adagraph::sensor::PointXYZIT, y),
      offsetof(adagraph::sensor::PointXYZIT, z), FrameId::kBase);
  adagraph::localization::RelocalizationQuery query;
  query.T_odom_base = {
      Timestamp::FromNanoseconds(123), FrameId::kOdom, FrameId::kBase,
      RigidTransform3d::Identity()};
  query.current_scan_base =
      adagraph::sensor::ImmutablePointCloud::Share(storage, view);
  ASSERT_TRUE(query.valid());

  adagraph::localization::RelocalizationResult result;
  result.status = adagraph::localization::RelocalizationStatus::kSuccess;
  result.T_map_base = StampedTransform3d{
      query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
      RigidTransform3d::Identity()};
  EXPECT_TRUE(result.validFor(query));

  result.T_map_base->timestamp = Timestamp::FromNanoseconds(124);
  EXPECT_FALSE(result.validFor(query));
  result.status =
      adagraph::localization::RelocalizationStatus::kRegistrationRejected;
  result.T_map_base.reset();
  EXPECT_TRUE(result.validFor(query));

  result.status = adagraph::localization::RelocalizationStatus::kSuccess;
  EXPECT_FALSE(result.validFor(query));
  result.status =
      adagraph::localization::RelocalizationStatus::kRegistrationRejected;
  auto invalid_query = query;
  invalid_query.T_odom_base.target_frame = FrameId::kMap;
  EXPECT_FALSE(result.validFor(invalid_query));
}

TEST(GlobalAlignmentState, OwnsVersionedMapToOdomWithoutFrontendMutation) {
  adagraph::system::GlobalAlignmentState state;
  const StampedTransform3d T_map_base{
      Timestamp::FromNanoseconds(100), FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(12.0, 2.0, 0.0)}};
  const StampedTransform3d T_odom_base{
      Timestamp::FromNanoseconds(100), FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0, 2.0, 0.0)}};
  ASSERT_TRUE(state.commit(
      1, T_map_base, T_odom_base,
      adagraph::system::GlobalAlignmentSource::kRelocalization, 0.9));
  const auto alignment = state.snapshot();
  ASSERT_TRUE(alignment);
  EXPECT_EQ(alignment->version, 1U);
  EXPECT_TRUE(alignment->T_map_odom.T_target_source.translation.isApprox(
      Eigen::Vector3d(10.0, 0.0, 0.0)));
  EXPECT_FALSE(state.commit(
      1, T_map_base, T_odom_base,
      adagraph::system::GlobalAlignmentSource::kBackendOptimization, 1.0));

  const StampedTransform3d later_T_odom_base{
      Timestamp::FromNanoseconds(200), FrameId::kOdom, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(3.0, 4.0, 0.0)}};
  const auto later_T_map_base = state.mapBaseAt(later_T_odom_base);
  ASSERT_TRUE(later_T_map_base);
  EXPECT_EQ(later_T_map_base->timestamp, later_T_odom_base.timestamp);
  EXPECT_TRUE(later_T_map_base->T_target_source.translation.isApprox(
      Eigen::Vector3d(13.0, 4.0, 0.0)));
}

TEST(RelocalizationUpdate, PreservesMapToOdomAcrossBaseExtrinsic) {
  const auto timestamp = Timestamp::FromNanoseconds(500);
  const StampedTransform3d T_map_odom{
      timestamp, FrameId::kMap, FrameId::kOdom,
      {Eigen::Quaterniond(Eigen::AngleAxisd(
           0.2, Eigen::Vector3d::UnitZ())),
       Eigen::Vector3d(10.0, -2.0, 1.0)}};
  const StampedTransform3d T_odom_imu{
      timestamp, FrameId::kOdom, FrameId::kImu,
      {Eigen::Quaterniond(Eigen::AngleAxisd(
           -0.1, Eigen::Vector3d::UnitX())),
       Eigen::Vector3d(2.0, 3.0, 0.5)}};
  const adagraph::geometry::StaticTransform3d T_base_imu{
      FrameId::kBase, FrameId::kImu,
      {Eigen::Quaterniond(Eigen::AngleAxisd(
           3.141592653589793, Eigen::Vector3d::UnitX())),
       Eigen::Vector3d(-0.053, 0.0, 0.735)}};
  const auto update =
      adagraph::localization::BuildRelocalizationUpdateFromMapOdom(
          T_map_odom, T_odom_imu, T_base_imu, 0.04, 0.8,
          adagraph::localization::Confidence::kHigh);
  ASSERT_TRUE(update);
  ASSERT_TRUE(update->result.T_map_base);

  const auto reconstructed = update->result.T_map_base->compose(
      update->T_odom_base.inverse());
  ASSERT_TRUE(reconstructed);
  EXPECT_TRUE(reconstructed->T_target_source.translation.isApprox(
      T_map_odom.T_target_source.translation, 1e-12));
  EXPECT_TRUE(reconstructed->T_target_source.rotation.isApprox(
      T_map_odom.T_target_source.rotation, 1e-12));
  EXPECT_EQ(update->result.confidence,
            adagraph::localization::Confidence::kHigh);

  StampedTransform3d wrong_time = T_odom_imu;
  wrong_time.timestamp = Timestamp::FromNanoseconds(501);
  EXPECT_FALSE(
      adagraph::localization::BuildRelocalizationUpdateFromMapOdom(
          T_map_odom, wrong_time, T_base_imu, 0.04));
}

TEST(GlobalAlignmentUpdateMailbox,
     ReplacesPendingWithLatestFixedSizeTypedUpdate) {
  const auto make_update = [](std::int64_t timestamp_ns, double map_x) {
    adagraph::localization::RelocalizationUpdate update;
    update.result.status =
        adagraph::localization::RelocalizationStatus::kSuccess;
    update.result.T_map_base = StampedTransform3d{
        Timestamp::FromNanoseconds(timestamp_ns), FrameId::kMap,
        FrameId::kBase,
        {Eigen::Quaterniond::Identity(), Eigen::Vector3d(map_x, 0.0, 0.0)}};
    update.result.fitness = 0.1;
    update.T_odom_base = {
        Timestamp::FromNanoseconds(timestamp_ns), FrameId::kOdom,
        FrameId::kBase, RigidTransform3d::Identity()};
    return update;
  };

  adagraph::system::GlobalAlignmentUpdateMailbox mailbox;
  auto first = make_update(10, 1.0);
  auto latest = make_update(20, 2.0);
  ASSERT_TRUE(first.valid());
  ASSERT_TRUE(latest.valid());
  EXPECT_TRUE(mailbox.submit(first));
  EXPECT_TRUE(mailbox.submit(latest));

  const auto taken = mailbox.takeLatest();
  ASSERT_TRUE(taken);
  ASSERT_TRUE(taken->result.T_map_base);
  EXPECT_EQ(taken->result.T_map_base->timestamp,
            Timestamp::FromNanoseconds(20));
  EXPECT_DOUBLE_EQ(
      taken->result.T_map_base->T_target_source.translation.x(), 2.0);
  EXPECT_FALSE(mailbox.takeLatest());

  adagraph::localization::RelocalizationUpdate invalid;
  EXPECT_FALSE(mailbox.submit(invalid));
  const auto stats = mailbox.stats();
  EXPECT_EQ(stats.submitted, 2U);
  EXPECT_EQ(stats.replaced, 1U);
  EXPECT_EQ(stats.taken, 1U);
  EXPECT_EQ(stats.invalid, 1U);
}

TEST(ComposedRelocalizer, SeparatesRetrievalAndSelectsBestVerification) {
  class Retriever final : public adagraph::localization::CandidateRetriever {
   public:
    std::vector<adagraph::localization::RelocalizationCandidate> Retrieve(
        const adagraph::localization::RelocalizationQuery& query,
        const adagraph::localization::LocalizationMap&,
        std::stop_token) override {
      const auto candidate = [&query](double x, double score) {
        return adagraph::localization::RelocalizationCandidate{
            {query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
             {Eigen::Quaterniond::Identity(), Eigen::Vector3d(x, 0.0, 0.0)}},
            score};
      };
      return {candidate(1.0, 0.9), candidate(2.0, 0.8)};
    }
  };
  class Verifier final : public adagraph::localization::RegistrationVerifier {
   public:
    std::optional<adagraph::localization::RegistrationVerification> Verify(
        const adagraph::localization::RelocalizationQuery&,
        const adagraph::localization::LocalizationMap&,
        const adagraph::localization::RelocalizationCandidate& candidate,
        std::stop_token) override {
      const double x =
          candidate.initial_T_map_base.T_target_source.translation.x();
      return adagraph::localization::RegistrationVerification{
          candidate.initial_T_map_base,
          x == 2.0 ? 0.1 : 0.5,
          x == 2.0 ? 0.8 : 0.6,
          x == 2.0 ? adagraph::localization::Confidence::kHigh
                   : adagraph::localization::Confidence::kMedium};
    }
  };

  adagraph::localization::ComposedRelocalizer relocalizer(
      std::make_unique<Retriever>(), std::make_unique<Verifier>());
  const auto query = MakeRelocalizationQuery(100);
  const auto map = MakeLocalizationMap();
  const auto result = relocalizer.Relocalize(query, *map, {});
  ASSERT_TRUE(result.validFor(query));
  EXPECT_DOUBLE_EQ(result.fitness, 0.1);
  EXPECT_EQ(result.confidence, adagraph::localization::Confidence::kHigh);
  EXPECT_DOUBLE_EQ(result.T_map_base->T_target_source.translation.x(), 2.0);
}

TEST(RelocalizationScheduler, CancelsRunningQueryAndPublishesOnlyLatest) {
  struct Control {
    std::mutex mutex;
    std::condition_variable_any cv;
    bool first_started = false;
    int calls = 0;
  };
  class ControlledRelocalizer final : public adagraph::localization::Relocalizer {
   public:
    explicit ControlledRelocalizer(std::shared_ptr<Control> control)
        : control_(std::move(control)) {}
    adagraph::localization::RelocalizationResult Relocalize(
        const adagraph::localization::RelocalizationQuery& query,
        const adagraph::localization::LocalizationMap&,
        std::stop_token stop_token) override {
      int call = 0;
      {
        std::lock_guard<std::mutex> lock(control_->mutex);
        call = ++control_->calls;
        if (call == 1) {
          control_->first_started = true;
          control_->cv.notify_all();
        }
      }
      if (call == 1) {
        std::unique_lock<std::mutex> lock(control_->mutex);
        control_->cv.wait(lock, stop_token, [] { return false; });
        adagraph::localization::RelocalizationResult cancelled;
        cancelled.status =
            adagraph::localization::RelocalizationStatus::kCancelled;
        return cancelled;
      }
      adagraph::localization::RelocalizationResult success;
      success.status = adagraph::localization::RelocalizationStatus::kSuccess;
      success.T_map_base = StampedTransform3d{
          query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          RigidTransform3d::Identity()};
      return success;
    }

   private:
    std::shared_ptr<Control> control_;
  };

  auto control = std::make_shared<Control>();
  std::mutex result_mutex;
  std::condition_variable result_cv;
  std::vector<std::uint64_t> published;
  adagraph::localization::RelocalizationScheduler scheduler(
      std::make_unique<ControlledRelocalizer>(control),
      [&result_mutex, &result_cv, &published](
          std::uint64_t sequence,
          const adagraph::localization::RelocalizationResult&,
          const StampedTransform3d&) {
        std::lock_guard<std::mutex> lock(result_mutex);
        published.push_back(sequence);
        result_cv.notify_all();
      });
  const auto map = MakeLocalizationMap();
  EXPECT_EQ(scheduler.submit(MakeRelocalizationQuery(100), map),
            adagraph::localization::RelocalizationSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(control->mutex);
    ASSERT_TRUE(control->cv.wait_for(
        lock, std::chrono::seconds(2),
        [&control] { return control->first_started; }));
  }
  EXPECT_EQ(scheduler.submit(MakeRelocalizationQuery(200), map),
            adagraph::localization::RelocalizationSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(result_mutex);
    ASSERT_TRUE(result_cv.wait_for(
        lock, std::chrono::seconds(2),
        [&published] { return !published.empty(); }));
  }
  scheduler.finish();

  ASSERT_EQ(published.size(), 1U);
  EXPECT_EQ(published.front(), 2U);
  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.submitted, 2U);
  EXPECT_GE(stats.cancellation_requests, 1U);
  EXPECT_EQ(stats.completed, 2U);
  EXPECT_EQ(stats.published, 1U);
  EXPECT_EQ(stats.callback_failures, 0U);
  EXPECT_EQ(stats.timing_samples, 2U);
  EXPECT_GE(stats.queue_wait_total_ns, 0U);
  EXPECT_GE(stats.processing_total_ns, 0U);
  EXPECT_GE(stats.end_to_end_total_ns, 0U);
}

TEST(RelocalizationScheduler,
     CoalescesPendingWithoutStarvingNonInterruptibleVendorWork) {
  struct Control {
    std::mutex mutex;
    std::condition_variable cv;
    bool first_started = false;
    bool release_first = false;
  };
  class NonInterruptibleRelocalizer final
      : public adagraph::localization::Relocalizer {
   public:
    explicit NonInterruptibleRelocalizer(std::shared_ptr<Control> control)
        : control_(std::move(control)) {}
    adagraph::localization::RelocalizationResult Relocalize(
        const adagraph::localization::RelocalizationQuery& query,
        const adagraph::localization::LocalizationMap&,
        std::stop_token) override {
      {
        std::unique_lock<std::mutex> lock(control_->mutex);
        if (!control_->first_started) {
          control_->first_started = true;
          control_->cv.notify_all();
          control_->cv.wait(
              lock, [this] { return control_->release_first; });
        }
      }
      adagraph::localization::RelocalizationResult result;
      result.status =
          adagraph::localization::RelocalizationStatus::kSuccess;
      result.T_map_base = StampedTransform3d{
          query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
          RigidTransform3d::Identity()};
      return result;
    }

   private:
    std::shared_ptr<Control> control_;
  };

  auto control = std::make_shared<Control>();
  std::mutex result_mutex;
  std::condition_variable result_cv;
  std::vector<std::uint64_t> published;
  adagraph::localization::RelocalizationScheduler scheduler(
      std::make_unique<NonInterruptibleRelocalizer>(control),
      [&result_mutex, &result_cv, &published](
          std::uint64_t sequence,
          const adagraph::localization::RelocalizationResult&,
          const StampedTransform3d&) {
        std::lock_guard<std::mutex> lock(result_mutex);
        published.push_back(sequence);
        result_cv.notify_all();
      },
      adagraph::localization::ActiveRelocalizationPolicy::
          kFinishActiveAndPublish);
  const auto map = MakeLocalizationMap();
  ASSERT_EQ(scheduler.submit(MakeRelocalizationQuery(100), map),
            adagraph::localization::RelocalizationSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(control->mutex);
    ASSERT_TRUE(control->cv.wait_for(
        lock, std::chrono::seconds(2),
        [&control] { return control->first_started; }));
  }
  ASSERT_EQ(scheduler.submit(MakeRelocalizationQuery(200), map),
            adagraph::localization::RelocalizationSubmissionStatus::kAccepted);
  {
    std::lock_guard<std::mutex> lock(control->mutex);
    control->release_first = true;
  }
  control->cv.notify_all();
  {
    std::unique_lock<std::mutex> lock(result_mutex);
    ASSERT_TRUE(result_cv.wait_for(
        lock, std::chrono::seconds(2),
        [&published] { return published.size() == 2; }));
  }
  scheduler.finish();

  EXPECT_EQ(published, (std::vector<std::uint64_t>{1, 2}));
  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.submitted, 2U);
  EXPECT_EQ(stats.cancellation_requests, 0U);
  EXPECT_EQ(stats.completed, 2U);
  EXPECT_EQ(stats.published, 2U);
  EXPECT_EQ(stats.timing_samples, 2U);
  EXPECT_GE(stats.queue_wait_max_ns, 0U);
  EXPECT_GE(stats.processing_max_ns, 0U);
  EXPECT_GE(stats.end_to_end_max_ns, 0U);
}

TEST(MapManager, RejectsDuplicateOptimizationNodesWithoutPartialMutation) {
  adagraph::mapping::MapManager manager;
  const auto keyframe = MakeKeyFrame(1, 100);
  adagraph::backend::PoseGraphNode node;
  node.id = adagraph::common::NodeId(7);
  node.keyframe_id = keyframe.id();
  node.trajectory_id = keyframe.trajectoryId();
  node.session_id = keyframe.sessionId();
  node.T_odom_base = keyframe.T_odom_base();
  ASSERT_TRUE(manager.insertKeyFrame(
      keyframe, node, adagraph::common::SubmapId(0)));

  const StampedTransform3d optimized{
      keyframe.timestamp(), FrameId::kMap, FrameId::kBase,
      {Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0)}};
  adagraph::backend::OptimizationResult result;
  result.input_graph_version = 1;
  result.optimized_nodes = {{node.id, optimized}, {node.id, optimized}};
  EXPECT_FALSE(manager.applyOptimization(result));

  const auto snapshot = manager.publishSnapshot();
  ASSERT_EQ(snapshot->keyframes.size(), 1U);
  EXPECT_TRUE(snapshot->keyframes.front().T_map_base.T_target_source.translation
                  .isZero());
}

TEST(KeyFramePolicy, PreservesMotionAndMaximumFrameAdmissionSemantics) {
  adagraph::backend::BackendConfig config;
  config.keyframe_translation_m = 0.5;
  config.keyframe_rotation_rad = 0.1;
  config.keyframe_max_frames = 3;
  adagraph::backend::KeyFramePolicy policy(config);

  RigidTransform3d pose = RigidTransform3d::Identity();
  EXPECT_FALSE(policy.shouldAccept(pose));
  policy.reject();
  policy.reject();
  EXPECT_FALSE(policy.shouldAccept(pose));
  policy.reject();
  EXPECT_TRUE(policy.shouldAccept(pose));

  pose.translation.x() = 0.6;
  policy.accept(pose);
  EXPECT_EQ(policy.rejectedFrameCount(), 0U);
  EXPECT_FALSE(policy.shouldAccept(pose));

  RigidTransform3d rotated = pose;
  rotated.rotation = Eigen::Quaterniond(
      Eigen::AngleAxisd(0.11, Eigen::Vector3d::UnitZ()));
  EXPECT_TRUE(policy.shouldAccept(rotated));
}

TEST(OdometryCovarianceEstimator, PreservesRuleAndInformationBlend) {
  adagraph::backend::BackendConfig config;
  adagraph::backend::OdometryCovarianceEstimator estimator(config);
  adagraph::backend::OdometryUncertaintyInput input;
  input.inlier_ratio = 0.8;
  input.condition_number = 100.0;
  input.hessian_trace = 1e6;
  input.effective_point_count = 2000;
  input.angular_velocity_norm = 0.1;

  const auto rule_covariance = estimator.estimate(input);
  EXPECT_NEAR(rule_covariance(0, 0), 0.012 * 0.012, 1e-15);
  EXPECT_NEAR(rule_covariance(3, 3), 0.035 * 0.035, 1e-15);

  input.has_information = true;
  input.information.diagonal() << 2000.0, 3000.0, 4000.0,
      5000.0, 6000.0, 7000.0;
  const auto information_covariance = estimator.estimate(input);
  for (int index = 0; index < 6; ++index) {
    EXPECT_NEAR(information_covariance(index, index),
                1.0 / input.information(index, index), 1e-15);
  }
}

TEST(LoopClosure, DisabledSeamProducesNoCandidateOrMeasurement) {
  adagraph::backend::DisabledLoopDetector detector;
  adagraph::backend::DisabledLoopVerifier verifier;
  const adagraph::backend::PoseGraphState snapshot;
  EXPECT_TRUE(detector.detect(snapshot, adagraph::common::NodeId(1)).empty());

  adagraph::backend::LoopVerificationInput input;
  input.candidate = {
      adagraph::common::NodeId(1), adagraph::common::NodeId(2), 0.9};
  EXPECT_FALSE(verifier.verify(input));
}

TEST(SensorData, ImuValidationIsRosIndependent) {
  adagraph::sensor::ImuData imu;
  imu.timestamp = Timestamp::FromNanoseconds(42);
  imu.linear_acceleration_mps2 = Eigen::Vector3d(0.0, 0.0, -9.81);
  imu.angular_velocity_radps = Eigen::Vector3d(0.1, 0.2, 0.3);
  EXPECT_TRUE(imu.valid());

  imu.angular_velocity_radps.x() =
      std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(imu.valid());
}

TEST(Config, IndependentInstancesValidateWithoutRos) {
  adagraph::frontend::FrontendConfig indoor_frontend;
  adagraph::frontend::FrontendConfig outdoor_frontend;
  outdoor_frontend.max_range_m = 300.0;
  outdoor_frontend.local_map_capacity = 500'000;
  EXPECT_FALSE(indoor_frontend.validate());
  EXPECT_FALSE(outdoor_frontend.validate());
  EXPECT_NE(indoor_frontend.max_range_m, outdoor_frontend.max_range_m);

  adagraph::backend::BackendConfig backend;
  adagraph::localization::RelocalizationConfig relocalization;
  EXPECT_FALSE(backend.validate());
  EXPECT_FALSE(relocalization.validate());

  indoor_frontend.point_filter_stride = 0;
  backend.submission_queue_capacity = 0;
  relocalization.crop_radius_m = -1.0;
  EXPECT_TRUE(indoor_frontend.validate());
  EXPECT_TRUE(backend.validate());
  EXPECT_TRUE(relocalization.validate());
}

}  // namespace
