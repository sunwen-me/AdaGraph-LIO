#include "lio/scan_context_loop_detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "hybrid_slam/logging/logging.h"
namespace LI2Sup {
namespace {

using hybrid_slam::backend::LoopCandidate;
using hybrid_slam::backend::LoopDetectionSnapshot;
using hybrid_slam::common::NodeId;
using hybrid_slam::mapping::KeyFrameSnapshot;
using hybrid_slam::mapping::SubmapSnapshot;

constexpr double kTwoPi = 6.28318530717958647692;

const KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map, const NodeId node_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [node_id](const auto& keyframe) { return keyframe.node_id == node_id; });
  return found == map.keyframes.end() ? nullptr : &*found;
}

const SubmapSnapshot* FindFinishedSubmap(
    const hybrid_slam::mapping::MapSnapshot& map,
    const KeyFrameSnapshot& keyframe) {
  const auto found = std::find_if(
      map.submaps.begin(), map.submaps.end(), [&keyframe](const auto& submap) {
        return submap.id == keyframe.submap_id &&
               submap.session_id == keyframe.session_id &&
               submap.trajectory_id == keyframe.trajectory_id &&
               submap.state == hybrid_slam::mapping::SubmapState::kFinished &&
               submap.geometry_ready && submap.valid();
      });
  return found == map.submaps.end() ? nullptr : &*found;
}

std::vector<float> BuildDescriptor(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const ScanContextConfig& config) {
  std::vector<float> descriptor(config.ring_count * config.sector_count, 0.0F);
  if (!cloud.valid() || cloud.empty()) {
    return descriptor;
  }
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::array<float, 3> point{};
    if (!cloud.view().xyz(index, point)) {
      continue;
    }
    const double x = point[0];
    const double y = point[1];
    const double radius = std::hypot(x, y);
    if (!std::isfinite(radius) || radius >= config.max_radius_m) {
      continue;
    }
    const auto ring = std::min(
        config.ring_count - 1,
        static_cast<std::size_t>(radius / config.max_radius_m *
                                 config.ring_count));
    double angle = std::atan2(y, x);
    if (angle < 0.0) {
      angle += kTwoPi;
    }
    const auto sector = std::min(
        config.sector_count - 1,
        static_cast<std::size_t>(angle / kTwoPi * config.sector_count));
    // Positive offset keeps flat ground informative without depending on an
    // external implementation's point pre-processing convention.
    const float height = static_cast<float>(point[2] + 2.0F);
    if (std::isfinite(height)) {
      descriptor[ring * config.sector_count + sector] = std::max(
          descriptor[ring * config.sector_count + sector], height);
    }
  }
  return descriptor;
}

std::vector<float> BuildRingKey(const std::vector<float>& descriptor,
                                const ScanContextConfig& config) {
  std::vector<float> key(config.ring_count, 0.0F);
  for (std::size_t ring = 0; ring < config.ring_count; ++ring) {
    double sum = 0.0;
    for (std::size_t sector = 0; sector < config.sector_count; ++sector) {
      sum += descriptor[ring * config.sector_count + sector];
    }
    key[ring] = static_cast<float>(sum / config.sector_count);
  }
  return key;
}

double SquaredDistance(const std::vector<float>& lhs,
                       const std::vector<float>& rhs) {
  if (lhs.size() != rhs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double result = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    const double delta = static_cast<double>(lhs[index]) - rhs[index];
    result += delta * delta;
  }
  return result;
}

std::pair<double, std::size_t> ScanContextDistance(
    const std::vector<float>& query, const std::vector<float>& candidate,
    const ScanContextConfig& config) {
  if (query.size() != candidate.size()) {
    return {std::numeric_limits<double>::infinity(), 0};
  }
  double best_distance = std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0;
  for (std::size_t shift = 0; shift < config.sector_count; ++shift) {
    double cosine_sum = 0.0;
    std::size_t comparable_columns = 0;
    for (std::size_t sector = 0; sector < config.sector_count; ++sector) {
      const auto candidate_sector = (sector + shift) % config.sector_count;
      double dot = 0.0;
      double query_norm = 0.0;
      double candidate_norm = 0.0;
      for (std::size_t ring = 0; ring < config.ring_count; ++ring) {
        const double query_value = query[ring * config.sector_count + sector];
        const double candidate_value =
            candidate[ring * config.sector_count + candidate_sector];
        dot += query_value * candidate_value;
        query_norm += query_value * query_value;
        candidate_norm += candidate_value * candidate_value;
      }
      if (query_norm > 1e-12 && candidate_norm > 1e-12) {
        cosine_sum += dot / std::sqrt(query_norm * candidate_norm);
        ++comparable_columns;
      }
    }
    if (comparable_columns == 0) {
      continue;
    }
    const double distance = 1.0 - cosine_sum / comparable_columns;
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }
  return {best_distance, best_shift};
}

}  // namespace

std::optional<std::string> ScanContextConfig::validate() const {
  if (ring_count == 0 || sector_count == 0 || ring_key_candidates == 0 ||
      !std::isfinite(max_radius_m) || max_radius_m <= 0.0 ||
      !std::isfinite(maximum_registration_initial_translation_m) ||
      maximum_registration_initial_translation_m <= 0.0) {
    return "invalid Scan Context configuration";
  }
  return std::nullopt;
}

std::optional<double> EstimateScanContextYaw(
    const hybrid_slam::sensor::ImmutablePointCloud& query,
    const hybrid_slam::sensor::ImmutablePointCloud& candidate,
    const ScanContextConfig& config) {
  if (const auto error = config.validate()) {
    return std::nullopt;
  }
  if (!query.valid() || query.empty() || !candidate.valid() ||
      candidate.empty()) {
    return std::nullopt;
  }
  const auto query_descriptor = BuildDescriptor(query, config);
  const auto candidate_descriptor = BuildDescriptor(candidate, config);
  const auto [distance, shift] =
      ScanContextDistance(query_descriptor, candidate_descriptor, config);
  if (!std::isfinite(distance)) {
    return std::nullopt;
  }
  return std::remainder(
      kTwoPi * static_cast<double>(shift) /
          static_cast<double>(config.sector_count),
      kTwoPi);
}

ScanContextLoopDetector::ScanContextLoopDetector(Config config)
    : config_(std::move(config)) {
  if (const auto error = config_.scan_context.validate()) {
    throw std::invalid_argument(*error);
  }
  if (config_.maximum_candidates == 0 ||
      !std::isfinite(config_.minimum_temporal_separation_s) ||
      config_.minimum_temporal_separation_s < 0.0) {
    throw std::invalid_argument("invalid Scan Context loop detector limits");
  }
}

void ScanContextLoopDetector::rebuildIfNeeded(
    const LoopDetectionSnapshot& snapshot) {
  if (!snapshot.map || cached_map_version_ == snapshot.map->version) {
    return;
  }
  entries_.clear();
  for (const auto& submap : snapshot.map->submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid()) {
      continue;
    }
    const auto anchor = std::find_if(
        snapshot.map->keyframes.begin(), snapshot.map->keyframes.end(),
        [&submap](const auto& keyframe) {
          return keyframe.keyframe_id == submap.anchor_keyframe_id &&
                 keyframe.session_id == submap.session_id &&
                 keyframe.trajectory_id == submap.trajectory_id;
        });
    if (anchor == snapshot.map->keyframes.end()) {
      continue;
    }
    auto descriptor = BuildDescriptor(submap.cloud_submap, config_.scan_context);
    entries_.push_back({submap.session_id, submap.trajectory_id, submap.id,
                        anchor->node_id,
                        anchor->T_odom_base.timestamp.seconds(),
                        std::move(descriptor), {}});
    entries_.back().ring_key =
        BuildRingKey(entries_.back().descriptor, config_.scan_context);
  }
  cached_map_version_ = snapshot.map->version;
}

std::vector<LoopCandidate> ScanContextLoopDetector::detect(
    const LoopDetectionSnapshot& snapshot, const NodeId query_node) {
  std::vector<LoopCandidate> result;
  if (!snapshot.map || !snapshot.map->consistent()) {
    return result;
  }
  rebuildIfNeeded(snapshot);
  const auto* query = FindKeyFrame(*snapshot.map, query_node);
  if (!query || !query->valid()) {
    return result;
  }
  const auto* query_submap = FindFinishedSubmap(*snapshot.map, *query);
  if (!query_submap) {
    return result;
  }
  const auto query_entry = std::find_if(
      entries_.begin(), entries_.end(), [query_submap](const auto& entry) {
        return entry.session_id == query_submap->session_id &&
               entry.trajectory_id == query_submap->trajectory_id &&
               entry.submap_id == query_submap->id;
      });
  if (query_entry == entries_.end()) {
    return result;
  }

  std::vector<std::pair<double, const DescriptorEntry*>> shortlist;
  shortlist.reserve(entries_.size());
  for (const auto& entry : entries_) {
    if (entry.submap_id == query_submap->id &&
        entry.session_id == query_submap->session_id &&
        entry.trajectory_id == query_submap->trajectory_id) {
      continue;
    }
    if (std::abs(entry.timestamp_s - query->T_odom_base.timestamp.seconds()) <
        config_.minimum_temporal_separation_s) {
      continue;
    }
    shortlist.emplace_back(SquaredDistance(query_entry->ring_key, entry.ring_key),
                           &entry);
  }
  const auto shortlist_count = std::min(config_.scan_context.ring_key_candidates,
                                        shortlist.size());
  std::partial_sort(shortlist.begin(), shortlist.begin() + shortlist_count,
                    shortlist.end(),
                    [](const auto& lhs, const auto& rhs) {
                      return lhs.first < rhs.first;
                    });

  struct RankedCandidate { double distance; std::size_t shift; const DescriptorEntry* entry; };
  std::vector<RankedCandidate> ranked;
  ranked.reserve(shortlist_count);
  for (std::size_t index = 0; index < shortlist_count; ++index) {
    const auto [distance, shift] = ScanContextDistance(
        query_entry->descriptor, shortlist[index].second->descriptor,
        config_.scan_context);
    ranked.push_back({distance, shift, shortlist[index].second});
  }
  std::sort(ranked.begin(), ranked.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.distance < rhs.distance;
  });
  std::ostringstream shortlist_nodes;
  for (std::size_t index = 0; index < shortlist_count; ++index) {
    if (index != 0) {
      shortlist_nodes << ',';
    }
    shortlist_nodes << shortlist[index].second->anchor_node.value();
  }
  LOG(INFO) << "[ScanContextLoopDetector]: query_node=" << query_node.value()
            << " eligible_submaps=" << shortlist.size()
            << " ring_key_shortlist_nodes=" << shortlist_nodes.str()
            << " ranked_candidates=" << ranked.size()
            << " best_distance="
            << (ranked.empty() ? -1.0 : ranked.front().distance);
  for (const auto& candidate : ranked) {
    if (result.size() == config_.maximum_candidates) {
      break;
    }
    // ScanContextDistance compares query sector s to candidate sector
    // s + shift.  That shift is already the rotation from query coordinates
    // into candidate coordinates, which is the transform VGICP consumes.
    const double yaw = std::remainder(
        kTwoPi * static_cast<double>(candidate.shift) /
            static_cast<double>(config_.scan_context.sector_count),
        kTwoPi);
    result.push_back({query_node, candidate.entry->anchor_node,
                      1.0 - candidate.distance, yaw});
  }
  return result;
}

}  // namespace LI2Sup
