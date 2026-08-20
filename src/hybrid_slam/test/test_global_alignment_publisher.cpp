#include "lio/system/global_alignment_publisher.h"

#include <atomic>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

namespace {

using LI2Sup::GlobalAlignmentPublisher;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::geometry::RigidTransform3d;
using hybrid_slam::geometry::StampedTransform3d;
using hybrid_slam::system::GlobalAlignmentSource;

hybrid_slam::common::Timestamp Stamp(double seconds) {
  // FromNanoseconds is used rather than FromSeconds because the latter returns
  // an optional; these fixtures always supply representable values.
  return hybrid_slam::common::Timestamp::FromNanoseconds(
      static_cast<std::int64_t>(seconds * 1e9));
}

StampedTransform3d MapBase(double seconds, double x = 1.0) {
  StampedTransform3d transform;
  transform.timestamp = Stamp(seconds);
  transform.target_frame = FrameId::kMap;
  transform.source_frame = FrameId::kBase;
  transform.T_target_source.translation = Eigen::Vector3d(x, 0.0, 0.0);
  return transform;
}

StampedTransform3d OdomBase(double seconds, double x = 0.0) {
  StampedTransform3d transform;
  transform.timestamp = Stamp(seconds);
  transform.target_frame = FrameId::kOdom;
  transform.source_frame = FrameId::kBase;
  transform.T_target_source.translation = Eigen::Vector3d(x, 0.0, 0.0);
  return transform;
}

hybrid_slam::localization::RelocalizationUpdate Update(double seconds,
                                                      double x = 2.0) {
  hybrid_slam::localization::RelocalizationUpdate update;
  // status defaults to kInvalidInput, so success() - and therefore valid() -
  // is false until it is set explicitly.
  update.result.status = hybrid_slam::localization::RelocalizationStatus::kSuccess;
  update.result.T_map_base = MapBase(seconds, x);
  update.result.fitness = 0.05;
  update.result.inlier_ratio = 0.9;
  update.T_odom_base = OdomBase(seconds);
  return update;
}

TEST(GlobalAlignmentPublisher, StartsWithNoAlignment) {
  GlobalAlignmentPublisher publisher;
  // Before any registration there is no map->odom, and consumers must be able
  // to tell that apart from an identity transform.
  EXPECT_FALSE(publisher.currentMapToOdom().has_value());
  const auto stats = publisher.stats();
  EXPECT_EQ(stats.committed, 0U);
  EXPECT_EQ(stats.rejected, 0U);
}

TEST(GlobalAlignmentPublisher, CommitsAPosePairAndNotifiesTheSink) {
  std::vector<RigidTransform3d> published;
  GlobalAlignmentPublisher publisher(
      [&published](const RigidTransform3d& transform) {
        published.push_back(transform);
      });
  EXPECT_TRUE(publisher.commit(MapBase(100.0), OdomBase(100.0),
                               GlobalAlignmentSource::kRelocalization, 0.1));
  ASSERT_EQ(published.size(), 1U);
  // map->odom is derived from the pose pair, so a 1 m map-frame offset against
  // an origin odom pose has to surface as that same offset.
  EXPECT_NEAR(published.front().translation.x(), 1.0, 1e-9);
  const auto current = publisher.currentMapToOdom();
  ASSERT_TRUE(current.has_value());
  EXPECT_NEAR(current->translation.x(), 1.0, 1e-9);
  EXPECT_EQ(publisher.stats().committed, 1U);
}

TEST(GlobalAlignmentPublisher, RejectsMismatchedFramesWithoutNotifying) {
  int calls = 0;
  GlobalAlignmentPublisher publisher(
      [&calls](const RigidTransform3d&) { ++calls; });
  // Passing the odom pose where the map pose belongs must not publish anything:
  // a TF consumer cannot detect that the transform means something else.
  EXPECT_FALSE(publisher.commit(OdomBase(100.0), OdomBase(100.0),
                                GlobalAlignmentSource::kRelocalization, 0.1));
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(publisher.stats().rejected, 1U);
  EXPECT_FALSE(publisher.currentMapToOdom().has_value());
}

TEST(GlobalAlignmentPublisher, RejectsAPairWithDisagreeingTimestamps) {
  GlobalAlignmentPublisher publisher;
  EXPECT_FALSE(publisher.commit(MapBase(100.0), OdomBase(100.5),
                                GlobalAlignmentSource::kRelocalization, 0.1));
  EXPECT_EQ(publisher.stats().rejected, 1U);
}

TEST(GlobalAlignmentPublisher, AppliesAMailboxUpdate) {
  std::vector<RigidTransform3d> published;
  GlobalAlignmentPublisher publisher(
      [&published](const RigidTransform3d& transform) {
        published.push_back(transform);
      });
  publisher.submit(Update(100.0, 2.0));
  EXPECT_TRUE(publisher.applyPending());
  ASSERT_EQ(published.size(), 1U);
  EXPECT_NEAR(published.front().translation.x(), 2.0, 1e-9);
  const auto stats = publisher.stats();
  EXPECT_EQ(stats.mailbox_submitted, 1U);
  EXPECT_EQ(stats.committed, 1U);
}

TEST(GlobalAlignmentPublisher, ApplyingAnEmptyMailboxIsNotAFailure) {
  GlobalAlignmentPublisher publisher;
  // The frame loop drains unconditionally, so an empty mailbox is the common
  // case and must not be counted as a rejection.
  EXPECT_FALSE(publisher.applyPending());
  EXPECT_EQ(publisher.stats().rejected, 0U);
}

TEST(GlobalAlignmentPublisher, KeepsOnlyTheLatestSubmission) {
  std::vector<RigidTransform3d> published;
  GlobalAlignmentPublisher publisher(
      [&published](const RigidTransform3d& transform) {
        published.push_back(transform);
      });
  publisher.submit(Update(100.0, 1.0));
  publisher.submit(Update(101.0, 5.0));
  EXPECT_TRUE(publisher.applyPending());
  // Registration is latest-value work: an unconsumed older result must be
  // superseded rather than queued behind the newer one.
  ASSERT_EQ(published.size(), 1U);
  EXPECT_NEAR(published.front().translation.x(), 5.0, 1e-9);
  EXPECT_EQ(publisher.stats().mailbox_replaced, 1U);
  EXPECT_FALSE(publisher.applyPending());
}

TEST(GlobalAlignmentPublisher, DiscardPendingDoesNotCommit) {
  int calls = 0;
  GlobalAlignmentPublisher publisher(
      [&calls](const RigidTransform3d&) { ++calls; });
  publisher.submit(Update(100.0));
  // The recovery transaction discards rather than applies: a relocalization
  // computed against the old trajectory must never land on the new identity.
  // Committing here instead would be invisible to every other check.
  publisher.discardPending();
  EXPECT_FALSE(publisher.applyPending());
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(publisher.stats().committed, 0U);
  EXPECT_FALSE(publisher.currentMapToOdom().has_value());
}

TEST(GlobalAlignmentPublisher, CountsMalformedSubmissionsAsRejected) {
  GlobalAlignmentPublisher publisher;
  hybrid_slam::localization::RelocalizationUpdate update;  // no poses at all
  publisher.submit(update);
  // A refused submission is malformed, not merely superseded, so it must be
  // visible in the rejected total rather than looking like success.
  EXPECT_GE(publisher.stats().rejected, 1U);
}

TEST(GlobalAlignmentPublisher, SurvivesAThrowingSink) {
  GlobalAlignmentPublisher publisher(
      [](const RigidTransform3d&) { throw std::runtime_error("tf down"); });
  // Publication is an external side effect; a consumer that throws must not
  // lose the committed alignment, which the next publication can still repair.
  EXPECT_TRUE(publisher.commit(MapBase(100.0), OdomBase(100.0),
                               GlobalAlignmentSource::kRelocalization, 0.1));
  EXPECT_TRUE(publisher.currentMapToOdom().has_value());
  EXPECT_EQ(publisher.stats().committed, 1U);
}

TEST(GlobalAlignmentPublisher, ResetDropsTheAlignmentAndCounters) {
  GlobalAlignmentPublisher publisher;
  ASSERT_TRUE(publisher.commit(MapBase(100.0), OdomBase(100.0),
                               GlobalAlignmentSource::kRelocalization, 0.1));
  publisher.reset();
  EXPECT_FALSE(publisher.currentMapToOdom().has_value());
  EXPECT_EQ(publisher.stats().committed, 0U);
  // Version numbering restarts with the state, so the next session can commit
  // at a version the previous one already used.
  EXPECT_TRUE(publisher.commit(MapBase(50.0), OdomBase(50.0),
                               GlobalAlignmentSource::kRelocalization, 0.1));
}

TEST(GlobalAlignmentPublisher, ConcurrentCommitsNeverShareAVersion) {
  GlobalAlignmentPublisher publisher;
  constexpr int kThreads = 4;
  constexpr int kPerThread = 50;
  std::atomic<int> accepted{0};
  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int index = 0; index < kThreads; ++index) {
    threads.emplace_back([&publisher, &accepted, index] {
      for (int step = 0; step < kPerThread; ++step) {
        const double seconds = 100.0 + index * 1000.0 + step;
        if (publisher.commit(MapBase(seconds), OdomBase(seconds),
                             GlobalAlignmentSource::kRelocalization, 0.1)) {
          accepted.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  // GlobalAlignmentState refuses a version it has already accepted, so two
  // commits reading the same counter would silently lose one. Every commit here
  // is well formed, so all of them must be accepted.
  EXPECT_EQ(accepted.load(), kThreads * kPerThread);
  EXPECT_EQ(publisher.stats().committed,
            static_cast<std::uint64_t>(kThreads * kPerThread));
  EXPECT_EQ(publisher.stats().rejected, 0U);
}

}  // namespace
