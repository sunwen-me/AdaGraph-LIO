#include "hybrid_slam/localization/relocalization_scheduler.h"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace hybrid_slam::localization {

RelocalizationScheduler::RelocalizationScheduler(
    std::unique_ptr<Relocalizer> relocalizer,
    ResultCallback callback,
    ActiveRelocalizationPolicy active_policy)
    : relocalizer_(std::move(relocalizer)),
      callback_(std::move(callback)),
      active_policy_(active_policy) {
  if (!relocalizer_ || !callback_) {
    throw std::invalid_argument(
        "RelocalizationScheduler requires relocalizer and callback");
  }
  worker_ = std::thread(&RelocalizationScheduler::workerLoop, this);
}

RelocalizationScheduler::~RelocalizationScheduler() { finish(); }

RelocalizationSubmissionStatus RelocalizationScheduler::submit(
    RelocalizationQuery query,
    std::shared_ptr<const LocalizationMap> map) {
  if (!query.valid() || !map || !map->valid()) {
    return RelocalizationSubmissionStatus::kInvalid;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return RelocalizationSubmissionStatus::kClosed;
  }
  if (pending_) {
    ++stats_.replaced_pending;
  }
  if (active_ && active_policy_ ==
                     ActiveRelocalizationPolicy::
                         kCancelAndSuppressSuperseded) {
    active_stop_source_.request_stop();
    ++stats_.cancellation_requests;
  }
  const std::uint64_t sequence = next_sequence_++;
  latest_sequence_ = sequence;
  pending_ = WorkItem{sequence, std::move(query), std::move(map),
                      std::chrono::steady_clock::now()};
  ++stats_.submitted;
  cv_.notify_one();
  return RelocalizationSubmissionStatus::kAccepted;
}

void RelocalizationScheduler::finish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (closed_) {
      // A previous finish may still be joining outside this lock.
    } else {
      closed_ = true;
      pending_.reset();
      if (active_) {
        active_stop_source_.request_stop();
        ++stats_.cancellation_requests;
      }
    }
  }
  cv_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
}

RelocalizationSchedulerStats RelocalizationScheduler::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void RelocalizationScheduler::workerLoop() {
  using Clock = std::chrono::steady_clock;
  for (;;) {
    WorkItem item;
    std::stop_token stop_token;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] { return closed_ || pending_.has_value(); });
      if (closed_) {
        return;
      }
      item = std::move(*pending_);
      pending_.reset();
      active_stop_source_ = std::stop_source{};
      stop_token = active_stop_source_.get_token();
      active_ = true;
    }

    const auto started_at = Clock::now();
    RelocalizationResult result =
        relocalizer_->Relocalize(item.query, *item.map, stop_token);
    const auto completed_at = Clock::now();
    const auto elapsedNs = [](const auto start, const auto end) {
      const auto elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(end - start)
              .count();
      return static_cast<std::uint64_t>(std::max<std::int64_t>(0, elapsed));
    };
    const std::uint64_t queue_wait_ns =
        elapsedNs(item.submitted_at, started_at);
    const std::uint64_t processing_ns =
        elapsedNs(started_at, completed_at);
    const std::uint64_t end_to_end_ns =
        elapsedNs(item.submitted_at, completed_at);

    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = false;
      ++stats_.completed;
      ++stats_.timing_samples;
      stats_.queue_wait_total_ns += queue_wait_ns;
      stats_.queue_wait_max_ns =
          std::max(stats_.queue_wait_max_ns, queue_wait_ns);
      stats_.processing_total_ns += processing_ns;
      stats_.processing_max_ns =
          std::max(stats_.processing_max_ns, processing_ns);
      stats_.end_to_end_total_ns += end_to_end_ns;
      stats_.end_to_end_max_ns =
          std::max(stats_.end_to_end_max_ns, end_to_end_ns);
      publish = !closed_ && !stop_token.stop_requested() &&
                (active_policy_ ==
                     ActiveRelocalizationPolicy::kFinishActiveAndPublish ||
                 item.sequence == latest_sequence_);
    }
    if (!publish) {
      continue;
    }
    try {
      callback_(item.sequence, result, item.query.T_odom_base);
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.published;
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.callback_failures;
    }
  }
}

}  // namespace hybrid_slam::localization
