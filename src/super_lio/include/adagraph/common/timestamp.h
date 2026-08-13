#ifndef ADAGRAPH_COMMON_TIMESTAMP_H_
#define ADAGRAPH_COMMON_TIMESTAMP_H_

#include <cmath>
#include <compare>
#include <cstdint>
#include <limits>
#include <optional>

namespace adagraph::common {

// Signed nanoseconds on the system's sensor time line. The epoch is selected
// by the input adapter; core algorithms only compare timestamps from the same
// trajectory. This value type performs no allocation.
class Timestamp {
 public:
  constexpr Timestamp() noexcept = default;

  [[nodiscard]] static constexpr Timestamp FromNanoseconds(
      std::int64_t nanoseconds) noexcept {
    return Timestamp(nanoseconds);
  }

  // Converts legacy floating-point seconds once at an adapter boundary.
  // Invalid/non-representable values are rejected rather than saturated.
  [[nodiscard]] static std::optional<Timestamp> FromSeconds(
      double seconds) noexcept {
    if (!std::isfinite(seconds)) {
      return std::nullopt;
    }

    constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
    const long double nanoseconds =
        static_cast<long double>(seconds) * kNanosecondsPerSecond;
    constexpr long double kMin =
        static_cast<long double>(std::numeric_limits<std::int64_t>::min());
    constexpr long double kMax =
        static_cast<long double>(std::numeric_limits<std::int64_t>::max());
    if (nanoseconds < kMin || nanoseconds > kMax) {
      return std::nullopt;
    }

    const long double rounded = std::round(nanoseconds);
    if (rounded < kMin || rounded > kMax) {
      return std::nullopt;
    }
    return Timestamp(static_cast<std::int64_t>(rounded));
  }

  [[nodiscard]] constexpr std::int64_t nanoseconds() const noexcept {
    return nanoseconds_;
  }

  [[nodiscard]] double seconds() const noexcept {
    return static_cast<double>(nanoseconds_) * 1e-9;
  }

  auto operator<=>(const Timestamp&) const = default;

 private:
  explicit constexpr Timestamp(std::int64_t nanoseconds) noexcept
      : nanoseconds_(nanoseconds) {}

  std::int64_t nanoseconds_ = 0;
};

static_assert(sizeof(Timestamp) == sizeof(std::int64_t));

}  // namespace adagraph::common

#endif  // ADAGRAPH_COMMON_TIMESTAMP_H_
