#ifndef LI2SUP_SYSTEM_MAP_SAVE_FACADE_H_
#define LI2SUP_SYSTEM_MAP_SAVE_FACADE_H_

#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "hybrid_slam/io/map_io.h"
#include "hybrid_slam/system/slam_system.h"

namespace LI2Sup {

// Owns the map-save scheduler and the futures handed to asynchronous callers.
//
// Extracted from PointLIOPipeline, where this plumbing contributed three of
// that class's four mutexes and left the promise table adjacent to unrelated
// recovery and alignment state.
//
// The lock order is fixed: submit_mutex_ -> state_mutex_ -> pending_mutex_,
// and state_mutex_ is always released before pending_mutex_ is taken. The
// scheduler calls complete() from its own worker thread and that path takes
// pending_mutex_ only, so a completing request never waits behind a
// submission. submit_mutex_ exists so finish() cannot block on the state lock
// while a submission is waiting for queue capacity.
class MapSaveFacade {
 public:
  explicit MapSaveFacade(std::size_t queue_capacity = 2);
  ~MapSaveFacade();

  MapSaveFacade(const MapSaveFacade&) = delete;
  MapSaveFacade& operator=(const MapSaveFacade&) = delete;

  // Creates the scheduler on first use. Does nothing once finished, so a late
  // submission cannot resurrect a stopped worker.
  void ensureStarted();

  [[nodiscard]] bool available() const noexcept;

  // Submission whose result is only logged.
  [[nodiscard]] hybrid_slam::io::MapSaveSubmissionStatus submit(
      hybrid_slam::io::MapSaveRequest request);

  // Submission whose completion is reported through the returned future.
  [[nodiscard]] std::future<hybrid_slam::system::SystemStatus> submitAsync(
      hybrid_slam::io::MapSaveRequest request);

  // Drains the scheduler, fails every future still outstanding and logs the
  // scheduler statistics. Safe to call more than once.
  void finish();

 private:
  void complete(const hybrid_slam::io::MapSaveResult& result) noexcept;
  void ensureStartedLocked();
  [[nodiscard]] std::uint64_t allocateRequestIdLocked(
      std::uint64_t requested_id);

  const std::size_t queue_capacity_;
  mutable std::mutex submit_mutex_;
  mutable std::mutex state_mutex_;
  std::unique_ptr<hybrid_slam::io::MapSaveScheduler> scheduler_;
  std::uint64_t next_request_id_ = 1;
  bool finished_ = false;
  mutable std::mutex pending_mutex_;
  std::unordered_map<std::uint64_t,
                     std::promise<hybrid_slam::system::SystemStatus>>
      pending_;
};

}  // namespace LI2Sup

#endif  // LI2SUP_SYSTEM_MAP_SAVE_FACADE_H_
