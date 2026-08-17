#ifndef TIMER_H_
#define TIMER_H_

#include <glog/logging.h>
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <map>
#include <string>

class Timer {
 public:
  // Rolling statistics only: per-call samples are not stored, so memory stays
  // bounded no matter how long the run is.
  struct TimerRecord {
    TimerRecord() = default;
    explicit TimerRecord(const std::string& name) : func_name_(name) {}
    std::string func_name_;
    std::size_t count_ = 0;
    double mean_ms_ = 0.0;
    double max_ms_ = 0.0;
  };

  template <class F>
  void Evaluate(F&& func, const std::string& func_name) {
    auto t1 = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)();
    auto t2 = std::chrono::high_resolution_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count() *
        1000;
    auto& record = records_[func_name];
    if (record.count_ == 0) {
      record.func_name_ = func_name;
    }
    record.count_++;
    record.mean_ms_ += (time_used - record.mean_ms_) / double(record.count_);
    record.max_ms_ = std::max(record.max_ms_, time_used);
  }

  void PrintAll() {
    LOG(INFO) << ">>> ===== Printing run time =====";
    for (const auto& r : records_) {
      LOG(INFO) << "> [ " << r.first << " ] average time usage: " << r.second.mean_ms_
                << " ms , called times: " << r.second.count_
                << " , max: " << r.second.max_ms_ << " ms";
    }
  }

  /// clean the records
  void Clear() { records_.clear(); }

 private:
  std::map<std::string, TimerRecord> records_;
};

#endif
