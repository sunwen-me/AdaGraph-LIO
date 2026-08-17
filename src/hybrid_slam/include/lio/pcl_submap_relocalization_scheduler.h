#ifndef LI2SUP_PCL_SUBMAP_RELOCALIZATION_SCHEDULER_H_
#define LI2SUP_PCL_SUBMAP_RELOCALIZATION_SCHEDULER_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

#include "lio/pcl_submap_relocalizer.h"

namespace LI2Sup {

enum class PclSubmapRelocalizationSubmissionStatus {
  kAccepted = 0,
  kInvalid,
  kClosed,
};

struct PclSubmapRelocalizationSchedulerStats {
  std::uint64_t submitted = 0;
  std::uint64_t replaced_pending = 0;
  std::uint64_t cancellation_requests = 0;
  std::uint64_t completed = 0;
  std::uint64_t published = 0;
  std::uint64_t callback_failures = 0;
};

// Latest-only submap worker. It snapshots the map at submission time and
// rebuilds only the small descriptor metadata in the worker; PCL target
// materialization stays off the Point-LIO and backend threads.
class PclSubmapRelocalizationScheduler final {
 public:
  using ResultCallback = std::function<void(
      std::uint64_t sequence,
      const std::optional<hybrid_slam::localization::SubmapRelocalizationResult>&
          result,
      const hybrid_slam::localization::RelocalizationQuery& query,
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map)>;

  PclSubmapRelocalizationScheduler(
      std::unique_ptr<PclSubmapRelocalizer> relocalizer,
      ResultCallback callback);
  ~PclSubmapRelocalizationScheduler();

  PclSubmapRelocalizationScheduler(
      const PclSubmapRelocalizationScheduler&) = delete;
  PclSubmapRelocalizationScheduler& operator=(
      const PclSubmapRelocalizationScheduler&) = delete;

  [[nodiscard]] PclSubmapRelocalizationSubmissionStatus submit(
      hybrid_slam::localization::RelocalizationQuery query,
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map);
  void finish();
  [[nodiscard]] PclSubmapRelocalizationSchedulerStats stats() const;

 private:
  struct WorkItem {
    std::uint64_t sequence = 0;
    hybrid_slam::localization::RelocalizationQuery query;
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map;
  };

  void workerLoop();

  std::unique_ptr<PclSubmapRelocalizer> relocalizer_;
  ResultCallback callback_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<WorkItem> pending_;
  std::stop_source active_stop_source_;
  std::thread worker_;
  bool closed_ = false;
  bool active_ = false;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t latest_sequence_ = 0;
  PclSubmapRelocalizationSchedulerStats stats_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_PCL_SUBMAP_RELOCALIZATION_SCHEDULER_H_
