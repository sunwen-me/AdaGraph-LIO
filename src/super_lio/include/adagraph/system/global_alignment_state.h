#ifndef ADAGRAPH_SYSTEM_GLOBAL_ALIGNMENT_STATE_H_
#define ADAGRAPH_SYSTEM_GLOBAL_ALIGNMENT_STATE_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include "adagraph/geometry/transform3.h"
#include "adagraph/localization/relocalization_update.h"

namespace adagraph::system {

enum class GlobalAlignmentSource {
  kRelocalization = 0,
  kBackendOptimization,
  kLoadedInitialization,
};

struct GlobalAlignmentSnapshot {
  std::uint64_t version = 0;
  adagraph::geometry::StampedTransform3d T_map_odom;
  GlobalAlignmentSource source = GlobalAlignmentSource::kRelocalization;
  double score = 0.0;

  [[nodiscard]] bool valid() const noexcept;
};

struct GlobalAlignmentMailboxStats {
  std::uint64_t submitted = 0;
  std::uint64_t replaced = 0;
  std::uint64_t taken = 0;
  std::uint64_t invalid = 0;
};

// Fixed-size cross-thread handoff for relocalization results. Registration is
// latest-value work: a newer accepted global observation supersedes an older
// result that the system writer has not consumed yet. No cloud is copied or
// retained by this mailbox.
class GlobalAlignmentUpdateMailbox {
 public:
  [[nodiscard]] bool submit(
      const adagraph::localization::RelocalizationUpdate& update);
  [[nodiscard]] std::optional<adagraph::localization::RelocalizationUpdate>
  takeLatest();
  // Discard pending work and begin a fresh session. Statistics are session
  // scoped as well; no alignment event from the previous trajectory may leak
  // into the next one.
  void reset();
  [[nodiscard]] GlobalAlignmentMailboxStats stats() const;

 private:
  mutable std::mutex mutex_;
  std::optional<adagraph::localization::RelocalizationUpdate> pending_;
  GlobalAlignmentMailboxStats stats_;
};

// Single-writer, lock-free-reader semantic owner of global alignment. The
// system scheduler commits paired poses; ROS/TF and map consumers only read an
// immutable snapshot. No method writes frontend estimator/local-map state.
class GlobalAlignmentState {
 public:
  GlobalAlignmentState();

  [[nodiscard]] bool commit(
      std::uint64_t version,
      const adagraph::geometry::StampedTransform3d& T_map_base,
      const adagraph::geometry::StampedTransform3d& T_odom_base,
      GlobalAlignmentSource source,
      double score);

  [[nodiscard]] std::shared_ptr<const GlobalAlignmentSnapshot> snapshot()
      const noexcept;

  // Applies the latest rigid map<-odom alignment at the query timestamp.
  // Retimestamping is explicit because an alignment is valid beyond the
  // measurement instant while runtime poses remain stamped.
  [[nodiscard]] std::optional<adagraph::geometry::StampedTransform3d>
  mapBaseAt(
      const adagraph::geometry::StampedTransform3d& T_odom_base) const;

 private:
  std::uint64_t last_committed_version_ = 0;  // single-writer only
  std::atomic<std::shared_ptr<const GlobalAlignmentSnapshot>> published_;
};

}  // namespace adagraph::system

#endif  // ADAGRAPH_SYSTEM_GLOBAL_ALIGNMENT_STATE_H_
