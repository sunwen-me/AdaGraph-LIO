
#ifndef BASIC_BUFFER_RINGBUFFER_HPP
#define BASIC_BUFFER_RINGBUFFER_HPP

#include <vector>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <optional>

// NOTE: currently unused anywhere in this repository (kept as a utility).

template <typename T, size_t BUFFER_SIZE = 10>
class RingBuffer {
public:
RingBuffer() : buffer_(BUFFER_SIZE), head_(0), tail_(0), full_(false), closed_(false) {}

  // When full, the single oldest element is dropped. Pushes after close() are ignored.
  void push(const T& data) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) return;

    buffer_[head_] = data;
    head_ = (head_ + 1) % BUFFER_SIZE;

    if (full_) {
      tail_ = (tail_ + 1) % BUFFER_SIZE;
    }

    full_ = head_ == tail_;
    lock.unlock();
    cv_.notify_one();
  }

  void push(T&& data) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (closed_) return;

    buffer_[head_] = std::move(data);
    head_ = (head_ + 1) % BUFFER_SIZE;

    if (full_) {
      tail_ = (tail_ + 1) % BUFFER_SIZE;
    }

    full_ = head_ == tail_;
    lock.unlock();
    cv_.notify_one();
  }

  // Wake every blocked consumer; remaining elements can still be drained,
  // after which the pop calls return false.
  void close() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      closed_ = true;
    }
    cv_.notify_all();
  }

  // Block until data is available (returns false once closed and drained).
  bool waitAndPop(T& out) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait(lock, [&]() { return closed_ || !empty_locked(); });
    if (empty_locked()) return false;
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
    return true;
  }

  // Block with timeout (returns true if data is successfully retrieved).
  bool waitAndPopFor(T& out, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, timeout, [&]() { return closed_ || !empty_locked(); });
    if (empty_locked()) return false;
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
    return true;
  }

  // Non-blocking pop (returns false if the buffer is empty).
  bool tryPop(T& out) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (empty_locked()) return false;
    out = std::move(buffer_[tail_]);
    tail_ = (tail_ + 1) % BUFFER_SIZE;
    full_ = false;
    return true;
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (full_) return BUFFER_SIZE;
    if (head_ >= tail_) return head_ - tail_;
    return BUFFER_SIZE + head_ - tail_;
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return empty_locked();
  }

private:
  bool empty_locked() const {
    return (!full_ && (head_ == tail_));
  }

  std::vector<T> buffer_;
  size_t head_;
  size_t tail_;
  bool full_;
  bool closed_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
};


#endif // BASIC_BUFFER_RINGBUFFER_HPP
