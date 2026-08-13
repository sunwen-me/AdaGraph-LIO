#include "adagraph/mapping/map_snapshot.h"

#include <set>
#include <tuple>

namespace adagraph::mapping {

bool MapSnapshot::consistent() const {
  if (!valid()) {
    return false;
  }

  std::set<std::uint64_t> node_ids;
  std::set<std::uint64_t> keyframe_ids;
  std::set<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>
      keyframe_membership;
  for (const auto& keyframe : keyframes) {
    if (!node_ids.insert(keyframe.node_id.value()).second ||
        !keyframe_ids.insert(keyframe.keyframe_id.value()).second) {
      return false;
    }
    keyframe_membership.emplace(
        keyframe.session_id.value(), keyframe.trajectory_id.value(),
        keyframe.submap_id.value());
  }

  std::set<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>
      submap_ids;
  for (const auto& submap : submaps) {
    const auto identity = std::make_tuple(
        submap.session_id.value(), submap.trajectory_id.value(),
        submap.id.value());
    if (!submap_ids.insert(identity).second) {
      return false;
    }
    std::set<std::uint64_t> local_keyframes;
    for (const auto id : submap.keyframe_ids) {
      if (!keyframe_ids.contains(id.value()) ||
          !local_keyframes.insert(id.value()).second) {
        return false;
      }
      bool matching_record = false;
      for (const auto& keyframe : keyframes) {
        if (keyframe.keyframe_id == id &&
            keyframe.session_id == submap.session_id &&
            keyframe.trajectory_id == submap.trajectory_id &&
            keyframe.submap_id == submap.id) {
          matching_record = true;
          break;
        }
      }
      if (!matching_record) {
        return false;
      }
    }
  }
  if (keyframe_membership != submap_ids && !keyframes.empty()) {
    return false;
  }

  std::set<std::pair<std::uint64_t, std::uint64_t>> trajectory_ids;
  for (const auto& trajectory : trajectories) {
    if (!trajectory_ids
             .emplace(trajectory.session_id.value(), trajectory.id.value())
             .second) {
      return false;
    }
    std::set<std::uint64_t> local_submaps;
    for (const auto id : trajectory.submap_ids) {
      if (!local_submaps.insert(id.value()).second ||
          !submap_ids.contains({trajectory.session_id.value(),
                               trajectory.id.value(), id.value()})) {
        return false;
      }
    }
  }

  for (const auto& constraint : constraints) {
    if (!constraint.valid()) {
      return false;
    }
    if (const auto* relative =
            std::get_if<adagraph::backend::RelativePoseMeasurement>(
                &constraint.measurement)) {
      if (!node_ids.contains(relative->from_node.value()) ||
          !node_ids.contains(relative->to_node.value())) {
        return false;
      }
    } else if (!node_ids.contains(
                   std::get<adagraph::backend::AbsolutePoseMeasurement>(
                       constraint.measurement)
                       .node_id.value())) {
      return false;
    }
  }
  return true;
}

}  // namespace adagraph::mapping
