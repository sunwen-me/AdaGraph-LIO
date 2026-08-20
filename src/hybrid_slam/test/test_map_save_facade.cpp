#include "lio/system/map_save_facade.h"

#include <chrono>
#include <filesystem>
#include <future>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

namespace {

using LI2Sup::MapSaveFacade;

std::filesystem::path TempDir() {
  const auto dir = std::filesystem::temp_directory_path() /
                   "hybrid_slam_map_save_facade_test";
  std::filesystem::create_directories(dir);
  return dir;
}

// The lightest valid request: a point-cloud input needs no pose graph, and
// MapSaveRequest requires exactly one input kind.
hybrid_slam::io::MapSaveRequest MakeRequest(const std::string& name,
                                            std::uint64_t request_id = 0) {
  auto cloud = std::make_shared<hybrid_slam::mapping::SnapshotPointCloudMap>();
  cloud->snapshot_version = 7;
  cloud->points.push_back({1.0F, 2.0F, 3.0F});
  hybrid_slam::io::MapSaveRequest request;
  request.request_id = request_id;
  request.point_cloud = std::move(cloud);
  request.output_path = TempDir() / name;
  request.voxel_leaf_size = 0.0;
  return request;
}

hybrid_slam::system::SystemStatus Wait(
    std::future<hybrid_slam::system::SystemStatus>& future) {
  // Every path under test either completes on the worker or is answered
  // inline, so a bounded wait keeps a regression from hanging the suite.
  if (future.wait_for(std::chrono::seconds(10)) !=
      std::future_status::ready) {
    return hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kRejected, "future timed out");
  }
  return future.get();
}

TEST(MapSaveFacade, StartsClosedUntilFirstUse) {
  MapSaveFacade facade;
  // No scheduler thread exists until something is submitted, so a pipeline that
  // never saves a map does not pay for a worker.
  EXPECT_FALSE(facade.available());
  facade.ensureStarted();
  EXPECT_TRUE(facade.available());
}

TEST(MapSaveFacade, ResolvesTheFutureOfAnAcceptedRequest) {
  MapSaveFacade facade;
  auto future = facade.submitAsync(MakeRequest("accepted.pcd"));
  const auto status = Wait(future);
  EXPECT_TRUE(status.ok()) << status.message;
  EXPECT_TRUE(std::filesystem::exists(TempDir() / "accepted.pcd"));
}

TEST(MapSaveFacade, RejectsAnInvalidRequestWithoutStartingWork) {
  MapSaveFacade facade;
  hybrid_slam::io::MapSaveRequest request;  // no input kind, no output path
  auto future = facade.submitAsync(std::move(request));
  const auto status = Wait(future);
  EXPECT_EQ(status.code,
            hybrid_slam::system::SystemStatusCode::kInvalidArgument);
}

TEST(MapSaveFacade, RejectsADuplicateRequestIdRatherThanLosingAPromise) {
  MapSaveFacade facade;
  // Two futures may never share one id: the second would overwrite the first
  // entry in the promise table and leave that caller waiting forever.
  auto first = facade.submitAsync(MakeRequest("dup_a.pcd", 41));
  auto second = facade.submitAsync(MakeRequest("dup_b.pcd", 41));
  const auto second_status = Wait(second);
  EXPECT_EQ(second_status.code,
            hybrid_slam::system::SystemStatusCode::kRejected);
  const auto first_status = Wait(first);
  EXPECT_TRUE(first_status.ok()) << first_status.message;
}

TEST(MapSaveFacade, FailsOutstandingFuturesOnFinish) {
  MapSaveFacade facade;
  auto future = facade.submitAsync(MakeRequest("on_finish.pcd"));
  facade.finish();
  // Whoever was waiting must be answered exactly once, whether the worker
  // completed the request before the drain or finish() had to fail it.
  const auto status = Wait(future);
  EXPECT_TRUE(status.ok() ||
              status.code == hybrid_slam::system::SystemStatusCode::kClosed)
      << status.message;
  EXPECT_FALSE(facade.available());
}

TEST(MapSaveFacade, RefusesWorkAfterFinishAndStaysFinished) {
  MapSaveFacade facade;
  facade.ensureStarted();
  facade.finish();
  // ensureStarted() must not resurrect a stopped worker; a late submission has
  // to be reported as closed instead of silently starting a second scheduler.
  facade.ensureStarted();
  EXPECT_FALSE(facade.available());
  EXPECT_EQ(facade.submit(MakeRequest("after_finish.pcd")),
            hybrid_slam::io::MapSaveSubmissionStatus::kClosed);
  auto future = facade.submitAsync(MakeRequest("after_finish_async.pcd"));
  const auto status = Wait(future);
  EXPECT_EQ(status.code, hybrid_slam::system::SystemStatusCode::kClosed);
}

TEST(MapSaveFacade, FinishIsIdempotent) {
  MapSaveFacade facade;
  facade.ensureStarted();
  facade.finish();
  facade.finish();  // must not join twice or double-answer a promise
  EXPECT_FALSE(facade.available());
}

TEST(MapSaveFacade, DestructorDrainsWithoutAnExplicitFinish) {
  std::future<hybrid_slam::system::SystemStatus> future;
  {
    MapSaveFacade facade;
    future = facade.submitAsync(MakeRequest("destructor.pcd"));
  }
  // The pipeline destroys the facade on shutdown paths that never call
  // finish(); an abandoned future there would block a joining caller.
  const auto status = Wait(future);
  EXPECT_TRUE(status.ok() ||
              status.code == hybrid_slam::system::SystemStatusCode::kClosed)
      << status.message;
}

TEST(MapSaveFacade, AssignsDistinctIdsToUnnumberedRequests) {
  MapSaveFacade facade;
  std::vector<std::future<hybrid_slam::system::SystemStatus>> futures;
  futures.reserve(4);
  for (int index = 0; index < 4; ++index) {
    futures.push_back(
        facade.submitAsync(MakeRequest("auto_" + std::to_string(index) + ".pcd")));
  }
  // Auto-numbering collides with itself if the counter is not advanced, which
  // would surface as a duplicate-id rejection rather than a saved map.
  for (auto& future : futures) {
    const auto status = Wait(future);
    EXPECT_TRUE(status.ok()) << status.message;
  }
}

TEST(MapSaveFacade, KeepsExplicitAndAutomaticIdsFromColliding) {
  MapSaveFacade facade;
  auto explicit_future = facade.submitAsync(MakeRequest("explicit.pcd", 5));
  // The next automatic id must clear the explicit one; reusing 5 here would be
  // rejected as a duplicate and never written.
  auto automatic_future = facade.submitAsync(MakeRequest("automatic.pcd"));
  const auto explicit_status = Wait(explicit_future);
  const auto automatic_status = Wait(automatic_future);
  EXPECT_TRUE(explicit_status.ok()) << explicit_status.message;
  EXPECT_TRUE(automatic_status.ok()) << automatic_status.message;
}

}  // namespace
