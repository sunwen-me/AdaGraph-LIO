#ifndef ADAGRAPH_COMMON_DOMAIN_IDS_H_
#define ADAGRAPH_COMMON_DOMAIN_IDS_H_

#include <compare>
#include <cstdint>

namespace adagraph::common {

template <typename Tag>
class StrongId {
 public:
  constexpr StrongId() noexcept = default;
  explicit constexpr StrongId(std::uint64_t value) noexcept : value_(value) {}

  [[nodiscard]] constexpr std::uint64_t value() const noexcept {
    return value_;
  }

  auto operator<=>(const StrongId&) const = default;

 private:
  std::uint64_t value_ = 0;
};

struct SessionIdTag;
struct TrajectoryIdTag;
struct SubmapIdTag;
struct KeyFrameIdTag;
struct NodeIdTag;
struct ConstraintIdTag;

using SessionId = StrongId<SessionIdTag>;
using TrajectoryId = StrongId<TrajectoryIdTag>;
using SubmapId = StrongId<SubmapIdTag>;
using KeyFrameId = StrongId<KeyFrameIdTag>;
using NodeId = StrongId<NodeIdTag>;
using ConstraintId = StrongId<ConstraintIdTag>;

static_assert(sizeof(KeyFrameId) == sizeof(std::uint64_t));

}  // namespace adagraph::common

#endif  // ADAGRAPH_COMMON_DOMAIN_IDS_H_
