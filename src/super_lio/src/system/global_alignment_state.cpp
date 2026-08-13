#include "adagraph/system/global_alignment_state.h"

#include <cmath>
#include <utility>

namespace adagraph::system {

bool GlobalAlignmentUpdateMailbox::submit(
    const adagraph::localization::RelocalizationUpdate& update) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!update.valid()) {
    ++stats_.invalid;
    return false;
  }
  if (pending_) {
    ++stats_.replaced;
  }
  pending_ = update;
  ++stats_.submitted;
  return true;
}

std::optional<adagraph::localization::RelocalizationUpdate>
GlobalAlignmentUpdateMailbox::takeLatest() {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = std::move(pending_);
  pending_.reset();
  if (result) {
    ++stats_.taken;
  }
  return result;
}

void GlobalAlignmentUpdateMailbox::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.reset();
  stats_ = GlobalAlignmentMailboxStats{};
}

GlobalAlignmentMailboxStats GlobalAlignmentUpdateMailbox::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

bool GlobalAlignmentSnapshot::valid() const noexcept {
  return version > 0 && T_map_odom.valid() &&
         T_map_odom.target_frame == adagraph::geometry::FrameId::kMap &&
         T_map_odom.source_frame == adagraph::geometry::FrameId::kOdom &&
         std::isfinite(score);
}

GlobalAlignmentState::GlobalAlignmentState() {
  published_.store(std::shared_ptr<const GlobalAlignmentSnapshot>{});
}

bool GlobalAlignmentState::commit(
    std::uint64_t version,
    const adagraph::geometry::StampedTransform3d& T_map_base,
    const adagraph::geometry::StampedTransform3d& T_odom_base,
    GlobalAlignmentSource source,
    double score) {
  if (version == 0 || version <= last_committed_version_ ||
      !std::isfinite(score) || !T_map_base.valid() ||
      !T_odom_base.valid() ||
      T_map_base.timestamp != T_odom_base.timestamp ||
      T_map_base.target_frame != adagraph::geometry::FrameId::kMap ||
      T_map_base.source_frame != adagraph::geometry::FrameId::kBase ||
      T_odom_base.target_frame != adagraph::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != adagraph::geometry::FrameId::kBase) {
    return false;
  }

  const auto T_map_odom = T_map_base.compose(T_odom_base.inverse());
  if (!T_map_odom) {
    return false;
  }
  auto next = std::make_shared<GlobalAlignmentSnapshot>(
      GlobalAlignmentSnapshot{version, *T_map_odom, source, score});
  if (!next->valid()) {
    return false;
  }
  last_committed_version_ = version;
  std::shared_ptr<const GlobalAlignmentSnapshot> immutable = std::move(next);
  published_.store(std::move(immutable), std::memory_order_release);
  return true;
}

std::shared_ptr<const GlobalAlignmentSnapshot>
GlobalAlignmentState::snapshot() const noexcept {
  return published_.load(std::memory_order_acquire);
}

std::optional<adagraph::geometry::StampedTransform3d>
GlobalAlignmentState::mapBaseAt(
    const adagraph::geometry::StampedTransform3d& T_odom_base) const {
  if (!T_odom_base.valid() ||
      T_odom_base.target_frame != adagraph::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != adagraph::geometry::FrameId::kBase) {
    return std::nullopt;
  }
  const auto alignment = snapshot();
  if (!alignment || !alignment->valid()) {
    return std::nullopt;
  }
  const adagraph::geometry::StampedTransform3d T_map_odom_at_query{
      T_odom_base.timestamp,
      adagraph::geometry::FrameId::kMap,
      adagraph::geometry::FrameId::kOdom,
      alignment->T_map_odom.T_target_source,
  };
  return T_map_odom_at_query.compose(T_odom_base);
}

}  // namespace adagraph::system
