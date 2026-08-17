#ifndef LI2SUP_ODOMETRY_RATE_LIMITER_H_
#define LI2SUP_ODOMETRY_RATE_LIMITER_H_

#include <algorithm>
#include <cmath>
#include <limits>

namespace LI2Sup {

// Limits externally published estimator states without affecting Point-LIO's
// point-level propagation and measurement updates.
class OdometryRateLimiter final {
 public:
  explicit OdometryRateLimiter(const double max_rate_hz = 0.0) noexcept
      : max_rate_hz_(normalizeRate(max_rate_hz)) {}

  void reset() noexcept {
    last_published_timestamp_s_ = std::numeric_limits<double>::quiet_NaN();
  }

  [[nodiscard]] bool shouldPublish(const double timestamp_s) noexcept {
    if (max_rate_hz_ <= 0.0 || !std::isfinite(timestamp_s)) {
      return true;
    }
    if (!std::isfinite(last_published_timestamp_s_) ||
        timestamp_s < last_published_timestamp_s_) {
      last_published_timestamp_s_ = timestamp_s;
      return true;
    }

    const double period_s = 1.0 / max_rate_hz_;
    const double comparison_epsilon_s =
        std::max(1e-12, period_s * 1e-9);
    if (timestamp_s - last_published_timestamp_s_ + comparison_epsilon_s <
        period_s) {
      return false;
    }

    last_published_timestamp_s_ = timestamp_s;
    return true;
  }

 private:
  [[nodiscard]] static double normalizeRate(const double max_rate_hz) noexcept {
    return std::isfinite(max_rate_hz) && max_rate_hz > 0.0 ? max_rate_hz
                                                           : 0.0;
  }

  double max_rate_hz_ = 0.0;
  double last_published_timestamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
};

}  // namespace LI2Sup

#endif  // LI2SUP_ODOMETRY_RATE_LIMITER_H_
