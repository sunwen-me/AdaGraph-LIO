#ifndef HYBRID_SLAM_BACKEND_LOOP_CLOSURE_SCHEDULER_H_
#define HYBRID_SLAM_BACKEND_LOOP_CLOSURE_SCHEDULER_H_

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>
#include <vector>

#include "hybrid_slam/backend/loop_closure.h"

namespace hybrid_slam::backend {

enum class LoopClosureSubmissionStatus { kAccepted = 0, kInvalid, kClosed };

struct LoopClosureSchedulerStats {
  std::uint64_t submitted = 0;
  std::uint64_t replaced_pending = 0;
  std::uint64_t cancellation_requests = 0;
  std::uint64_t completed = 0;
  std::uint64_t constraints_published = 0;
  std::uint64_t callback_failures = 0;
};

// Latest-only asynchronous loop pipeline. Detection, point-cloud materialize
// and registration never run on BackendScheduler's single-writer thread.
class LoopClosureScheduler final {
 public:
  using ConstraintCallback = std::function<void(Constraint)>;

  LoopClosureScheduler(std::unique_ptr<LoopDetector> detector,
                       std::unique_ptr<LoopVerifier> verifier,
                       ConstraintCallback callback);
  ~LoopClosureScheduler();

  LoopClosureScheduler(const LoopClosureScheduler&) = delete;
  LoopClosureScheduler& operator=(const LoopClosureScheduler&) = delete;

  [[nodiscard]] LoopClosureSubmissionStatus submit(
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map,
      hybrid_slam::common::NodeId query_node);
  [[nodiscard]] LoopClosureSubmissionStatus submit(
      std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map,
      std::vector<hybrid_slam::common::NodeId> query_nodes);
  // Rejects future submissions, then drains the active item and the one
  // coalesced latest pending item before joining the worker.
  void finishLatest();
  void finish();
  [[nodiscard]] LoopClosureSchedulerStats stats() const;

 private:
  struct WorkItem {
    std::uint64_t sequence = 0;
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map;
    std::vector<hybrid_slam::common::NodeId> query_nodes;
  };

  void workerLoop();

  std::unique_ptr<LoopDetector> detector_;
  std::unique_ptr<LoopVerifier> verifier_;
  ConstraintCallback callback_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<WorkItem> pending_;
  std::stop_source active_stop_source_;
  std::thread worker_;
  bool closed_ = false;
  bool active_ = false;
  std::uint64_t next_sequence_ = 1;
  std::uint64_t latest_sequence_ = 0;
  LoopClosureSchedulerStats stats_;
};

}  // namespace hybrid_slam::backend

#endif  // HYBRID_SLAM_BACKEND_LOOP_CLOSURE_SCHEDULER_H_
