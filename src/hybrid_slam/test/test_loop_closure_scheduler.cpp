#include <gtest/gtest.h>

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "hybrid_slam/backend/loop_closure_scheduler.h"
#include "hybrid_slam/common/timestamp.h"
#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/mapping/map_snapshot.h"
#include "hybrid_slam/sensor/immutable_point_cloud.h"

namespace {

using hybrid_slam::backend::Constraint;
using hybrid_slam::backend::LoopCandidate;
using hybrid_slam::backend::LoopClosureScheduler;
using hybrid_slam::backend::LoopDetector;
using hybrid_slam::backend::LoopDetectionSnapshot;
using hybrid_slam::backend::LoopVerifier;
using hybrid_slam::backend::LoopVerificationInput;
using hybrid_slam::backend::RelativePoseMeasurement;
using hybrid_slam::common::NodeId;
using hybrid_slam::common::SessionId;
using hybrid_slam::common::Timestamp;
using hybrid_slam::common::TrajectoryId;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::mapping::KeyFrameSnapshot;
using hybrid_slam::mapping::MapSnapshot;

hybrid_slam::sensor::ImmutablePointCloud MakeCloud() {
  using Buffer = hybrid_slam::sensor::PointCloudBuffer;
  auto storage = std::make_shared<const Buffer>(
      Buffer{{1.0F, 0.0F, 0.0F, 0.0F, 0}});
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      storage->data(), storage->size(),
      sizeof(hybrid_slam::sensor::PointXYZIT),
      offsetof(hybrid_slam::sensor::PointXYZIT, x),
      offsetof(hybrid_slam::sensor::PointXYZIT, y),
      offsetof(hybrid_slam::sensor::PointXYZIT, z), FrameId::kBase);
  return hybrid_slam::sensor::ImmutablePointCloud::Share(storage, view);
}

Eigen::Quaterniond QuaternionFromRpy(const double roll, const double pitch,
                                     const double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
      .normalized();
}

std::shared_ptr<MapSnapshot> MakeMap(const std::uint64_t version) {
  auto map = std::make_shared<MapSnapshot>();
  map->version = version;
  const auto cloud = MakeCloud();
  for (const auto node : {NodeId(1), NodeId(2)}) {
    const auto timestamp = Timestamp::FromNanoseconds(
        static_cast<std::int64_t>(node.value()) * 100'000'000'000LL);
    KeyFrameSnapshot keyframe;
    keyframe.node_id = node;
    keyframe.keyframe_id = hybrid_slam::common::KeyFrameId(node.value());
    keyframe.trajectory_id = TrajectoryId(1);
    keyframe.session_id = SessionId(1);
    keyframe.submap_id = hybrid_slam::common::SubmapId(1);
    keyframe.T_odom_base = {timestamp, FrameId::kOdom, FrameId::kBase,
                            RigidTransform3d::Identity()};
    keyframe.T_map_base = {timestamp, FrameId::kMap, FrameId::kBase,
                           RigidTransform3d::Identity()};
    keyframe.cloud_base = cloud;
    map->keyframes.push_back(std::move(keyframe));
  }
  EXPECT_TRUE(map->valid());
  return map;
}

class BlockingDetector final : public LoopDetector {
 public:
  [[nodiscard]] std::vector<LoopCandidate> detect(
      const LoopDetectionSnapshot&, const NodeId query_node) override {
    std::unique_lock<std::mutex> lock(mutex_);
    ++calls_;
    if (calls_ == 1) {
      first_entered_ = true;
      entered_cv_.notify_all();
      release_cv_.wait(lock, [this] { return release_first_; });
    }
    return {{query_node, NodeId(1), 0.1}};
  }

  bool waitForFirstCall() {
    std::unique_lock<std::mutex> lock(mutex_);
    return entered_cv_.wait_for(lock, std::chrono::seconds(2),
                                [this] { return first_entered_; });
  }

  void releaseFirstCall() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      release_first_ = true;
    }
    release_cv_.notify_all();
  }

  int calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable entered_cv_;
  std::condition_variable release_cv_;
  int calls_ = 0;
  bool first_entered_ = false;
  bool release_first_ = false;
};

class SecondQueryDetector final : public LoopDetector {
 public:
  [[nodiscard]] std::vector<LoopCandidate> detect(
      const LoopDetectionSnapshot&, const NodeId query_node) override {
    ++calls_;
    if (query_node != NodeId(2)) {
      return {};
    }
    return {{NodeId(2), NodeId(1), 0.1}};
  }

  int calls() const { return calls_; }

 private:
  int calls_ = 0;
};

class ConfirmedSequenceDetector final : public LoopDetector {
 public:
  [[nodiscard]] std::vector<LoopCandidate> detect(
      const LoopDetectionSnapshot&, const NodeId query_node) override {
    if (query_node != NodeId(2)) {
      return {};
    }
    // The first candidate was observed on the previous submap and is only
    // released after the current candidate confirms the sequence.
    return {{NodeId(1), NodeId(2), 0.2},
            {NodeId(2), NodeId(1), 0.3}};
  }
};

class AcceptingVerifier final : public LoopVerifier {
 public:
  [[nodiscard]] std::optional<RelativePoseMeasurement> verify(
      const LoopVerificationInput& input) override {
    std::lock_guard<std::mutex> lock(mutex_);
    callback_thread_ = std::this_thread::get_id();
    return RelativePoseMeasurement{
        input.candidate.match_node,
        input.candidate.query_node,
        input.timestamp,
        RigidTransform3d::Identity(),
        hybrid_slam::backend::Covariance6d::Identity(),
    };
  }

  std::thread::id callbackThread() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return callback_thread_;
  }

 private:
  mutable std::mutex mutex_;
  std::thread::id callback_thread_;
};

class EchoInitialVerifier final : public LoopVerifier {
 public:
  [[nodiscard]] std::optional<RelativePoseMeasurement> verify(
      const LoopVerificationInput& input) override {
    return RelativePoseMeasurement{
        input.candidate.match_node,
        input.candidate.query_node,
        input.timestamp,
        input.initial_T_match_query,
        hybrid_slam::backend::Covariance6d::Identity(),
    };
  }
};

TEST(LoopClosureScheduler, FinishLatestDrainsActiveAndLatestPendingSnapshot) {
  auto detector = std::make_unique<BlockingDetector>();
  auto* detector_ptr = detector.get();
  auto verifier = std::make_unique<AcceptingVerifier>();
  auto* verifier_ptr = verifier.get();

  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::vector<Constraint> received;
  std::thread::id callback_thread;
  LoopClosureScheduler scheduler(
      std::move(detector), std::move(verifier),
      [&callback_mutex, &callback_cv, &received, &callback_thread](
          Constraint constraint) {
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          callback_thread = std::this_thread::get_id();
          received.push_back(std::move(constraint));
        }
        callback_cv.notify_all();
      });

  const auto caller_thread = std::this_thread::get_id();
  ASSERT_EQ(scheduler.submit(MakeMap(1), NodeId(2)),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  ASSERT_TRUE(detector_ptr->waitForFirstCall());
  ASSERT_EQ(scheduler.submit(MakeMap(2), NodeId(2)),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  std::atomic<bool> finish_returned = false;
  std::thread finisher([&scheduler, &finish_returned] {
    scheduler.finishLatest();
    finish_returned.store(true, std::memory_order_release);
  });
  detector_ptr->releaseFirstCall();
  finisher.join();

  EXPECT_TRUE(finish_returned.load(std::memory_order_acquire));

  {
    std::unique_lock<std::mutex> lock(callback_mutex);
    ASSERT_TRUE(callback_cv.wait_for(lock, std::chrono::seconds(2),
                                     [&received] { return received.size() == 2U; }));
  }

  ASSERT_EQ(received.size(), 2U);
  EXPECT_EQ(received.front().type,
            hybrid_slam::backend::ConstraintType::kLoopClosure);
  const auto* measurement =
      std::get_if<RelativePoseMeasurement>(&received.front().measurement);
  ASSERT_NE(measurement, nullptr);
  EXPECT_EQ(measurement->from_node, NodeId(1));
  EXPECT_EQ(measurement->to_node, NodeId(2));
  EXPECT_EQ(detector_ptr->calls(), 2);
  const auto stats = scheduler.stats();
  EXPECT_EQ(stats.cancellation_requests, 0U);
  EXPECT_EQ(stats.constraints_published, 2U);
  EXPECT_EQ(callback_thread, verifier_ptr->callbackThread());
  EXPECT_NE(callback_thread, caller_thread);

  scheduler.finishLatest();
  scheduler.finishLatest();
}

TEST(LoopClosureScheduler, InvalidSnapshotIsDiscardedOnWorker) {
  auto detector = std::make_unique<BlockingDetector>();
  auto verifier = std::make_unique<AcceptingVerifier>();
  int callback_count = 0;
  LoopClosureScheduler scheduler(
      std::move(detector), std::move(verifier),
      [&callback_count](Constraint) { ++callback_count; });

  ASSERT_EQ(scheduler.submit(std::make_shared<MapSnapshot>(), NodeId(1)),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  scheduler.finish();
  EXPECT_EQ(callback_count, 0);
  EXPECT_EQ(scheduler.stats().constraints_published, 0U);
}

TEST(LoopClosureScheduler, VerifiesMultipleQueriesWithinOneBatch) {
  auto detector = std::make_unique<SecondQueryDetector>();
  auto* detector_ptr = detector.get();
  auto verifier = std::make_unique<AcceptingVerifier>();
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::vector<Constraint> received;
  LoopClosureScheduler scheduler(
      std::move(detector), std::move(verifier),
      [&callback_mutex, &callback_cv, &received](Constraint constraint) {
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          received.push_back(std::move(constraint));
        }
        callback_cv.notify_all();
      });

  ASSERT_EQ(scheduler.submit(MakeMap(3), {NodeId(1), NodeId(2)}),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(callback_mutex);
    ASSERT_TRUE(callback_cv.wait_for(
        lock, std::chrono::seconds(2),
        [&received] { return received.size() == 1U; }));
  }

  EXPECT_EQ(detector_ptr->calls(), 2);
  EXPECT_EQ(scheduler.stats().submitted, 1U);
  EXPECT_EQ(scheduler.stats().completed, 1U);
  EXPECT_EQ(scheduler.stats().constraints_published, 1U);
  ASSERT_EQ(received.size(), 1U);
  const auto* measurement =
      std::get_if<RelativePoseMeasurement>(&received.front().measurement);
  ASSERT_NE(measurement, nullptr);
  EXPECT_EQ(measurement->from_node, NodeId(1));
  EXPECT_EQ(measurement->to_node, NodeId(2));
  scheduler.finish();
}

TEST(LoopClosureScheduler,
     PublishesPendingAndCurrentCandidatesAfterSequenceConfirmation) {
  auto detector = std::make_unique<ConfirmedSequenceDetector>();
  auto verifier = std::make_unique<AcceptingVerifier>();
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::vector<Constraint> received;
  LoopClosureScheduler scheduler(
      std::move(detector), std::move(verifier),
      [&callback_mutex, &callback_cv, &received](Constraint constraint) {
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          received.push_back(std::move(constraint));
        }
        callback_cv.notify_all();
      });

  ASSERT_EQ(scheduler.submit(MakeMap(31), NodeId(2)),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(callback_mutex);
    ASSERT_TRUE(callback_cv.wait_for(
        lock, std::chrono::seconds(2),
        [&received] { return received.size() == 2U; }));
  }
  ASSERT_EQ(received.size(), 2U);
  const auto* first =
      std::get_if<RelativePoseMeasurement>(&received[0].measurement);
  const auto* second =
      std::get_if<RelativePoseMeasurement>(&received[1].measurement);
  ASSERT_NE(first, nullptr);
  ASSERT_NE(second, nullptr);
  EXPECT_EQ(first->from_node, NodeId(2));
  EXPECT_EQ(first->to_node, NodeId(1));
  EXPECT_EQ(first->timestamp, Timestamp::FromNanoseconds(100'000'000'000LL));
  EXPECT_EQ(second->from_node, NodeId(1));
  EXPECT_EQ(second->to_node, NodeId(2));
  EXPECT_EQ(second->timestamp, Timestamp::FromNanoseconds(200'000'000'000LL));
  scheduler.finish();
  EXPECT_EQ(scheduler.stats().constraints_published, 2U);
}

TEST(LoopClosureScheduler, BuildsMatchFromQueryInitialTransformInSixDof) {
  const RigidTransform3d T_map_match{
      QuaternionFromRpy(-0.11, 0.07, -0.28),
      Eigen::Vector3d(12.0, -4.0, 2.5)};
  const RigidTransform3d T_map_query{
      QuaternionFromRpy(0.09, -0.04, 0.43),
      Eigen::Vector3d(18.0, 3.0, 5.2)};
  const RigidTransform3d expected_T_match_query =
      T_map_match.inverse().compose(T_map_query);
  auto map = MakeMap(4);
  map->keyframes[0].T_map_base.T_target_source = T_map_match;
  map->keyframes[1].T_map_base.T_target_source = T_map_query;
  ASSERT_TRUE(map->valid());

  auto detector = std::make_unique<SecondQueryDetector>();
  auto verifier = std::make_unique<EchoInitialVerifier>();
  std::mutex callback_mutex;
  std::condition_variable callback_cv;
  std::optional<RelativePoseMeasurement> received;
  LoopClosureScheduler scheduler(
      std::move(detector), std::move(verifier),
      [&callback_mutex, &callback_cv, &received](Constraint constraint) {
        {
          std::lock_guard<std::mutex> lock(callback_mutex);
          const auto* measurement =
              std::get_if<RelativePoseMeasurement>(&constraint.measurement);
          if (measurement) {
            received = *measurement;
          }
        }
        callback_cv.notify_all();
      });

  ASSERT_EQ(scheduler.submit(map, NodeId(2)),
            hybrid_slam::backend::LoopClosureSubmissionStatus::kAccepted);
  {
    std::unique_lock<std::mutex> lock(callback_mutex);
    ASSERT_TRUE(callback_cv.wait_for(lock, std::chrono::seconds(2),
                                     [&received] { return received.has_value(); }));
  }
  ASSERT_TRUE(received);
  EXPECT_TRUE(received->T_from_to.translation.isApprox(
      expected_T_match_query.translation, 1e-12));
  const double rotation_error = Eigen::AngleAxisd(
      expected_T_match_query.rotation.inverse() *
      received->T_from_to.rotation).angle();
  EXPECT_NEAR(rotation_error, 0.0, 1e-12);
  scheduler.finish();
}

}  // namespace
