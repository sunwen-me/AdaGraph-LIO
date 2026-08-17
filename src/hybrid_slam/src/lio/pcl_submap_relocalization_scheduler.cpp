#include "lio/pcl_submap_relocalization_scheduler.h"

#include <stdexcept>
#include <utility>

namespace LI2Sup {

PclSubmapRelocalizationScheduler::PclSubmapRelocalizationScheduler(
    std::unique_ptr<PclSubmapRelocalizer> relocalizer,
    ResultCallback callback)
    : relocalizer_(std::move(relocalizer)), callback_(std::move(callback)) {
  if (!relocalizer_ || !callback_) {
    throw std::invalid_argument(
        "PclSubmapRelocalizationScheduler requires relocalizer and callback");
  }
  worker_ = std::thread(&PclSubmapRelocalizationScheduler::workerLoop, this);
}

PclSubmapRelocalizationScheduler::~PclSubmapRelocalizationScheduler() {
  finish();
}

PclSubmapRelocalizationSubmissionStatus
PclSubmapRelocalizationScheduler::submit(
    hybrid_slam::localization::RelocalizationQuery query,
    std::shared_ptr<const hybrid_slam::mapping::MapSnapshot> map) {
  if (!query.valid() || !map || !map->valid()) {
    return PclSubmapRelocalizationSubmissionStatus::kInvalid;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_) {
    return PclSubmapRelocalizationSubmissionStatus::kClosed;
  }
  if (pending_) {
    ++stats_.replaced_pending;
  }
  if (active_) {
    active_stop_source_.request_stop();
    ++stats_.cancellation_requests;
  }
  const auto sequence = next_sequence_++;
  latest_sequence_ = sequence;
  pending_ = WorkItem{sequence, std::move(query), std::move(map)};
  ++stats_.submitted;
  cv_.notify_one();
  return PclSubmapRelocalizationSubmissionStatus::kAccepted;
}

void PclSubmapRelocalizationScheduler::finish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!closed_) {
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

PclSubmapRelocalizationSchedulerStats
PclSubmapRelocalizationScheduler::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

void PclSubmapRelocalizationScheduler::workerLoop() {
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

    hybrid_slam::mapping::SubmapDescriptorIndex index;
    index.rebuild(*item.map);
    const auto result = relocalizer_->Relocalize(
        item.query, *item.map, index, stop_token);

    bool publish = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_ = false;
      ++stats_.completed;
      publish = !closed_ && !stop_token.stop_requested() &&
                item.sequence == latest_sequence_;
    }
    if (!publish) {
      continue;
    }
    try {
      callback_(item.sequence, result, item.query, std::move(item.map));
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.published;
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.callback_failures;
    }
  }
}

}  // namespace LI2Sup
