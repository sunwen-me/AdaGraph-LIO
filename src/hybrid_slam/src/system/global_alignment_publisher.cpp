#include "lio/system/global_alignment_publisher.h"

#include <utility>

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {

GlobalAlignmentPublisher::GlobalAlignmentPublisher(
    MapToOdomSink map_to_odom_sink)
    : map_to_odom_sink_(std::move(map_to_odom_sink)),
      state_(std::make_unique<hybrid_slam::system::GlobalAlignmentState>()) {}

void GlobalAlignmentPublisher::submit(
    const hybrid_slam::localization::RelocalizationUpdate& update) {
  // The mailbox has its own lock and is latest-only, so this path does not take
  // mutex_: the relocalization worker must not queue behind a commit.
  if (mailbox_.submit(update)) {
    return;
  }
  const auto total =
      not_enqueued_.fetch_add(1, std::memory_order_relaxed) + 1;
  LOG(WARNING) << hybrid_slam::logging::kYellow
               << " ---> [GlobalAlignment]: relocalization update refused by"
                  " mailbox, not_enqueued_total="
               << total << hybrid_slam::logging::kReset;
}

bool GlobalAlignmentPublisher::applyPending() {
  auto update = mailbox_.takeLatest();
  if (!update) {
    return false;
  }
  if (!update->valid()) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++rejected_;
    return false;
  }
  return commit(*update->result.T_map_base, update->T_odom_base,
                hybrid_slam::system::GlobalAlignmentSource::kRelocalization,
                update->result.fitness);
}

bool GlobalAlignmentPublisher::commit(
    const hybrid_slam::geometry::StampedTransform3d& T_map_base,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base,
    const hybrid_slam::system::GlobalAlignmentSource source,
    const double score) {
  std::shared_ptr<const hybrid_slam::system::GlobalAlignmentSnapshot>
      committed_snapshot;
  {
    // The version number and the commit it identifies must be taken together.
    // GlobalAlignmentState refuses any version it has already accepted, so two
    // callers reading the same counter would lose one alignment silently.
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_ || !state_->commit(next_version_++, T_map_base, T_odom_base,
                                   source, score)) {
      ++rejected_;
      return false;
    }
    ++committed_;
    committed_snapshot = state_->snapshot();
  }

  // Deliberately outside the lock: this reaches a ROS/TF adapter, and holding
  // the lock across it would place a publication in front of the relocalization
  // worker's next submission.
  if (committed_snapshot && map_to_odom_sink_) {
    try {
      map_to_odom_sink_(committed_snapshot->T_map_odom.T_target_source);
    } catch (...) {
      // Publication is an external side effect. The typed alignment stays
      // committed; the next snapshot or TF publication repairs a consumer that
      // rejected this notification.
      LOG(ERROR) << hybrid_slam::logging::kRed
                 << " ---> [GlobalAlignmentState]: map->odom callback threw"
                 << hybrid_slam::logging::kReset;
    }
  }
  return true;
}

std::shared_ptr<const hybrid_slam::system::GlobalAlignmentSnapshot>
GlobalAlignmentPublisher::snapshot() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_ ? state_->snapshot() : nullptr;
}

std::optional<hybrid_slam::geometry::RigidTransform3d>
GlobalAlignmentPublisher::currentMapToOdom() const {
  const auto alignment = snapshot();
  if (!alignment || !alignment->valid()) {
    return std::nullopt;
  }
  return alignment->T_map_odom.T_target_source;
}

void GlobalAlignmentPublisher::discardPending() {
  (void)mailbox_.takeLatest();
}

void GlobalAlignmentPublisher::reset() {
  mailbox_.reset();
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = std::make_unique<hybrid_slam::system::GlobalAlignmentState>();
  next_version_ = 1;
  committed_ = 0;
  rejected_ = 0;
  not_enqueued_.store(0, std::memory_order_relaxed);
}

GlobalAlignmentPublisher::Stats GlobalAlignmentPublisher::stats() const {
  const auto mailbox_stats = mailbox_.stats();
  std::lock_guard<std::mutex> lock(mutex_);
  return Stats{
      committed_,
      // The mailbox counts malformed submissions it refused; commit() counts
      // pose pairs the state rejected. A caller reading one without the other
      // would see a partial picture of what was lost.
      rejected_ + mailbox_stats.invalid,
      not_enqueued_.load(std::memory_order_relaxed),
      mailbox_stats.submitted,
      mailbox_stats.replaced,
  };
}

}  // namespace LI2Sup
