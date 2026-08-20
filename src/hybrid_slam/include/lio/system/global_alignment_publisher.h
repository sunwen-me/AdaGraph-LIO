#ifndef LI2SUP_SYSTEM_GLOBAL_ALIGNMENT_PUBLISHER_H_
#define LI2SUP_SYSTEM_GLOBAL_ALIGNMENT_PUBLISHER_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

#include "hybrid_slam/geometry/transform3.h"
#include "hybrid_slam/localization/relocalization_update.h"
#include "hybrid_slam/system/global_alignment_state.h"

namespace LI2Sup {

// Owns global alignment: the cross-thread mailbox, the versioned state, and the
// map->odom notification.
//
// Extracted from PointLIOPipeline, where the version counter sat beside the
// recovery transaction and the commit/reject counters were three unrelated
// fields.
//
// Unlike FrameHealthGate this class does hold a lock, because it is genuinely
// cross-thread rather than merely called from a locked context. The prior-map
// localization worker submits results through submit(); the downstream worker
// drains them. The lock covers the version counter together with the commit it
// numbers: GlobalAlignmentState rejects any version it has already seen, so two
// commits that read the same counter would silently lose one alignment.
//
// The map->odom callback is invoked outside that lock. It reaches a ROS/TF
// adapter, so holding the lock across it would put a publication path in front
// of the relocalization worker.
class GlobalAlignmentPublisher {
 public:
  using MapToOdomSink =
      std::function<void(const hybrid_slam::geometry::RigidTransform3d&)>;

  struct Stats {
    std::uint64_t committed = 0;
    std::uint64_t rejected = 0;
    std::uint64_t not_enqueued = 0;
    std::uint64_t mailbox_submitted = 0;
    std::uint64_t mailbox_replaced = 0;
  };

  explicit GlobalAlignmentPublisher(MapToOdomSink map_to_odom_sink = {});

  GlobalAlignmentPublisher(const GlobalAlignmentPublisher&) = delete;
  GlobalAlignmentPublisher& operator=(const GlobalAlignmentPublisher&) = delete;

  // Called from the relocalization worker. A refused update is malformed rather
  // than superseded, so it is counted and logged instead of dropped quietly.
  void submit(const hybrid_slam::localization::RelocalizationUpdate& update);

  // Commits whatever the mailbox holds. Returns true when an update was both
  // present and accepted.
  [[nodiscard]] bool applyPending();

  // Commits a pose pair directly, for recovery and backend-driven alignment.
  [[nodiscard]] bool commit(
      const hybrid_slam::geometry::StampedTransform3d& T_map_base,
      const hybrid_slam::geometry::StampedTransform3d& T_odom_base,
      hybrid_slam::system::GlobalAlignmentSource source, double score);

  [[nodiscard]] std::shared_ptr<const hybrid_slam::system::GlobalAlignmentSnapshot>
  snapshot() const noexcept;

  // Latest map->odom, or nullopt while no alignment has been committed.
  [[nodiscard]] std::optional<hybrid_slam::geometry::RigidTransform3d>
  currentMapToOdom() const;

  // Drops any update the mailbox still holds without committing it. A recovery
  // transaction uses this: a relocalization computed against the old trajectory
  // must not be applied once the new identity is committed.
  void discardPending();

  // Discards pending work and resets the counters for a new session. The
  // committed alignment itself is dropped with the state.
  void reset();

  [[nodiscard]] Stats stats() const;

 private:
  const MapToOdomSink map_to_odom_sink_;

  mutable std::mutex mutex_;
  std::unique_ptr<hybrid_slam::system::GlobalAlignmentState> state_;
  hybrid_slam::system::GlobalAlignmentUpdateMailbox mailbox_;
  std::uint64_t next_version_ = 1;
  std::uint64_t committed_ = 0;
  std::uint64_t rejected_ = 0;
  std::atomic<std::uint64_t> not_enqueued_{0};
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_GLOBAL_ALIGNMENT_PUBLISHER_H_
