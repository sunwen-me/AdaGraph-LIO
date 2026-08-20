#include "lio/system/map_save_facade.h"

#include <utility>

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {

MapSaveFacade::MapSaveFacade(const std::size_t queue_capacity)
    : queue_capacity_(queue_capacity == 0 ? 1 : queue_capacity) {}

MapSaveFacade::~MapSaveFacade() { finish(); }

void MapSaveFacade::ensureStarted() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  ensureStartedLocked();
}

void MapSaveFacade::ensureStartedLocked() {
  if (scheduler_ || finished_) {
    return;
  }
  scheduler_ = std::make_unique<hybrid_slam::io::MapSaveScheduler>(
      queue_capacity_, [this](const hybrid_slam::io::MapSaveResult& result) {
        complete(result);
      });
}

bool MapSaveFacade::available() const noexcept {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return scheduler_ != nullptr && !finished_;
}

std::uint64_t MapSaveFacade::allocateRequestIdLocked(
    const std::uint64_t requested_id) {
  if (requested_id == 0) {
    return next_request_id_++;
  }
  if (requested_id >= next_request_id_) {
    next_request_id_ = requested_id + 1;
  }
  return requested_id;
}

hybrid_slam::io::MapSaveSubmissionStatus MapSaveFacade::submit(
    hybrid_slam::io::MapSaveRequest request) {
  std::lock_guard<std::mutex> submit_lock(submit_mutex_);
  std::lock_guard<std::mutex> lock(state_mutex_);
  ensureStartedLocked();
  if (!scheduler_ || finished_) {
    return hybrid_slam::io::MapSaveSubmissionStatus::kClosed;
  }
  request.request_id = allocateRequestIdLocked(request.request_id);
  return scheduler_->submit(std::move(request));
}

std::future<hybrid_slam::system::SystemStatus> MapSaveFacade::submitAsync(
    hybrid_slam::io::MapSaveRequest request) {
  std::promise<hybrid_slam::system::SystemStatus> promise;
  auto future = promise.get_future();
  if (!request.valid()) {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kInvalidArgument,
        "invalid map-save request"));
    return future;
  }
  std::lock_guard<std::mutex> submit_lock(submit_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  ensureStartedLocked();
  if (!scheduler_ || finished_) {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler is closed"));
    return future;
  }
  request.request_id = allocateRequestIdLocked(request.request_id);
  auto* const scheduler = scheduler_.get();
  // The state lock is dropped before the pending table is touched: complete()
  // runs on the scheduler thread and takes only pending_mutex_, so holding
  // both here would invert the order against it.
  state_lock.unlock();

  const auto request_id = request.request_id;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.contains(request_id)) {
      promise.set_value(hybrid_slam::system::SystemStatus::Error(
          hybrid_slam::system::SystemStatusCode::kRejected,
          "duplicate map-save request id"));
      return future;
    }
    pending_.emplace(request_id, std::move(promise));
  }

  const auto submission = scheduler->submit(std::move(request));
  if (submission == hybrid_slam::io::MapSaveSubmissionStatus::kAccepted) {
    return future;
  }

  // Rejected after the promise was published, so reclaim it and answer here.
  std::promise<hybrid_slam::system::SystemStatus> failed_promise;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto found = pending_.find(request_id);
    if (found != pending_.end()) {
      failed_promise = std::move(found->second);
      pending_.erase(found);
    }
  }
  if (submission == hybrid_slam::io::MapSaveSubmissionStatus::kInvalid) {
    failed_promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kInvalidArgument,
        "map IO rejected an invalid request"));
  } else {
    failed_promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler is closed"));
  }
  return future;
}

void MapSaveFacade::complete(
    const hybrid_slam::io::MapSaveResult& result) noexcept {
  std::promise<hybrid_slam::system::SystemStatus> promise;
  bool has_waiter = false;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto found = pending_.find(result.request_id);
    if (found != pending_.end()) {
      promise = std::move(found->second);
      pending_.erase(found);
      has_waiter = true;
    }
  }

  if (result.success) {
    LOG(INFO) << hybrid_slam::logging::kGreen
              << " ---> [MapSaveScheduler]: request=" << result.request_id
              << ", snapshot=" << result.snapshot_version
              << ", raw/saved points=" << result.raw_point_count << "/"
              << result.saved_point_count << ", input files="
              << result.input_file_count << ", " << result.message
              << hybrid_slam::logging::kReset;
  } else {
    LOG(ERROR) << hybrid_slam::logging::kRed
               << " ---> [MapSaveScheduler]: request=" << result.request_id
               << " failed: " << result.message
               << hybrid_slam::logging::kReset;
  }

  if (!has_waiter) {
    return;
  }
  if (result.success) {
    promise.set_value(hybrid_slam::system::SystemStatus::Ok());
  } else {
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kRejected, result.message));
  }
}

void MapSaveFacade::finish() {
  std::lock_guard<std::mutex> submit_lock(submit_mutex_);
  std::unique_lock<std::mutex> state_lock(state_mutex_);
  if (!scheduler_ || finished_) {
    return;
  }
  scheduler_->finish();
  finished_ = true;
  const auto stats = scheduler_->stats();
  state_lock.unlock();

  // Named distinctly from pending_: a local that differs from its member by a
  // single underscore is exactly how a shadowed declaration silently defeated
  // an earlier fix in this pipeline.
  std::unordered_map<std::uint64_t,
                     std::promise<hybrid_slam::system::SystemStatus>>
      drained;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    drained.swap(pending_);
  }
  for (auto& [request_id, promise] : drained) {
    (void)request_id;
    promise.set_value(hybrid_slam::system::SystemStatus::Error(
        hybrid_slam::system::SystemStatusCode::kClosed,
        "map IO scheduler stopped before completing request"));
  }

  LOG(INFO) << hybrid_slam::logging::kGreen
            << " ---> [MapSaveScheduler]: submitted/processed/failed="
            << stats.submitted << "/" << stats.processed << "/"
            << stats.failed << ", max_queue=" << stats.maximum_queue_depth
            << ", capacity_waits=" << stats.capacity_waits
            << hybrid_slam::logging::kReset;
}

}  // namespace LI2Sup

