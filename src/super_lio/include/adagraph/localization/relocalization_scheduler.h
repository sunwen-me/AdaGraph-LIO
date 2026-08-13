#ifndef ADAGRAPH_LOCALIZATION_RELOCALIZATION_SCHEDULER_H_
#define ADAGRAPH_LOCALIZATION_RELOCALIZATION_SCHEDULER_H_

#include <condition_variable>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

#include "adagraph/localization/relocalizer.h"

namespace adagraph::localization {

enum class RelocalizationSubmissionStatus {
  kAccepted = 0,
  kInvalid,
  kClosed,
};

enum class ActiveRelocalizationPolicy {
  // For implementations that inspect stop_token during expensive work.
  kCancelAndSuppressSuperseded = 0,
  // For vendor calls that cannot be interrupted. Finish/publish the active
  // timestamped result and retain only the newest pending query.
  kFinishActiveAndPublish,
};

struct RelocalizationSchedulerStats {
  std::uint64_t submitted = 0;
  std::uint64_t replaced_pending = 0;
  std::uint64_t cancellation_requests = 0;
  std::uint64_t completed = 0;
  std::uint64_t published = 0;
  std::uint64_t callback_failures = 0;
  // Timing values are kept as integer nanoseconds so callers can choose their
  // own presentation precision without accumulating floating-point error.
  std::uint64_t timing_samples = 0;
  std::uint64_t queue_wait_total_ns = 0;
  std::uint64_t queue_wait_max_ns = 0;
  std::uint64_t processing_total_ns = 0;
  std::uint64_t processing_max_ns = 0;
  std::uint64_t end_to_end_total_ns = 0;
  std::uint64_t end_to_end_max_ns = 0;
};

// Latest-only worker for expensive global registration. At most one query is
// running and one immutable query is pending. A newer query replaces pending
// work and requests cooperative cancellation of running retrieval/registration.
class RelocalizationScheduler {
 public:
  using ResultCallback = std::function<void(
      std::uint64_t sequence, const RelocalizationResult& result,
      const adagraph::geometry::StampedTransform3d& T_odom_base)>;

  RelocalizationScheduler(std::unique_ptr<Relocalizer> relocalizer,
                          ResultCallback callback,
                          ActiveRelocalizationPolicy active_policy =
                              ActiveRelocalizationPolicy::
                                  kCancelAndSuppressSuperseded);
  ~RelocalizationScheduler();

  RelocalizationScheduler(const RelocalizationScheduler&) = delete;
  RelocalizationScheduler& operator=(const RelocalizationScheduler&) = delete;

  [[nodiscard]] RelocalizationSubmissionStatus submit(
      RelocalizationQuery query,
      std::shared_ptr<const LocalizationMap> map);
  void finish();
  [[nodiscard]] RelocalizationSchedulerStats stats() const;

 private:
  struct WorkItem {
    std::uint64_t sequence = 0;
    RelocalizationQuery query;
    std::shared_ptr<const LocalizationMap> map;
    std::chrono::steady_clock::time_point submitted_at;
  };

  void workerLoop();

  std::unique_ptr<Relocalizer> relocalizer_;
  ResultCallback callback_;
  ActiveRelocalizationPolicy active_policy_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<WorkItem> pending_;
  std::stop_source active_stop_source_;
  std::thread worker_;
  bool closed_ = false;
  bool active_ = false;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t latest_sequence_ = 0;
  RelocalizationSchedulerStats stats_;
};

}  // namespace adagraph::localization

#endif  // ADAGRAPH_LOCALIZATION_RELOCALIZATION_SCHEDULER_H_
