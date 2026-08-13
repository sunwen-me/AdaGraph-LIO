#ifndef ADAGRAPH_MAPPING_MAP_SNAPSHOT_H_
#define ADAGRAPH_MAPPING_MAP_SNAPSHOT_H_

#include <cstdint>
#include <vector>

#include "adagraph/backend/constraint.h"
#include "adagraph/common/domain_ids.h"
#include "adagraph/geometry/transform3.h"
#include "adagraph/mapping/keyframe.h"

namespace adagraph::mapping {

struct KeyFrameSnapshot {
  adagraph::common::NodeId node_id;
  adagraph::common::KeyFrameId keyframe_id;
  adagraph::common::TrajectoryId trajectory_id;
  adagraph::common::SessionId session_id;
  adagraph::common::SubmapId submap_id;
  adagraph::geometry::StampedTransform3d T_odom_base;
  adagraph::geometry::StampedTransform3d T_map_base;
  adagraph::sensor::ImmutablePointCloud cloud_base;
  KeyFrameQuality quality;

  [[nodiscard]] bool valid() const noexcept {
    return T_odom_base.valid() && T_map_base.valid() &&
           T_odom_base.timestamp == T_map_base.timestamp &&
           T_odom_base.target_frame == adagraph::geometry::FrameId::kOdom &&
           T_odom_base.source_frame == adagraph::geometry::FrameId::kBase &&
           T_map_base.target_frame == adagraph::geometry::FrameId::kMap &&
           T_map_base.source_frame == adagraph::geometry::FrameId::kBase &&
           cloud_base.valid() && !cloud_base.empty() &&
           cloud_base.frame() == adagraph::geometry::FrameId::kBase;
  }
};

enum class SubmapState {
  kActive = 0,
  kFinished,
};

struct SubmapSnapshot {
  adagraph::common::SubmapId id;
  adagraph::common::TrajectoryId trajectory_id;
  adagraph::common::SessionId session_id;
  SubmapState state = SubmapState::kActive;
  std::vector<adagraph::common::KeyFrameId> keyframe_ids;
};

struct TrajectorySnapshot {
  adagraph::common::TrajectoryId id;
  adagraph::common::SessionId session_id;
  std::vector<adagraph::common::SubmapId> submap_ids;
};

// Immutable, internally consistent committed map view. Cloud payload copies
// increment one shared owner count; point storage is never deep-copied.
struct MapSnapshot {
  std::uint64_t version = 0;
  std::vector<TrajectorySnapshot> trajectories;
  std::vector<SubmapSnapshot> submaps;
  std::vector<KeyFrameSnapshot> keyframes;
  std::vector<adagraph::backend::Constraint> constraints;

  [[nodiscard]] bool valid() const noexcept {
    for (const auto& keyframe : keyframes) {
      if (!keyframe.valid()) {
        return false;
      }
    }
    return true;
  }

  // O(metadata) structural audit intended for tests, IO and diagnostics, not
  // for the per-keyframe publish path.
  [[nodiscard]] bool consistent() const;
};

}  // namespace adagraph::mapping

#endif  // ADAGRAPH_MAPPING_MAP_SNAPSHOT_H_
