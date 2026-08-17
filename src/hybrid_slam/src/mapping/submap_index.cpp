#include "hybrid_slam/mapping/submap_index.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace hybrid_slam::mapping {
namespace {

double Norm(const std::vector<float>& descriptor) {
  double squared = 0.0;
  for (const float value : descriptor) {
    if (!std::isfinite(value)) {
      return 0.0;
    }
    squared += static_cast<double>(value) * value;
  }
  return std::sqrt(squared);
}

}  // namespace

void SubmapDescriptorIndex::rebuild(const MapSnapshot& snapshot) {
  map_version_ = snapshot.version;
  entries_.clear();
  if (!snapshot.valid()) {
    return;
  }
  entries_.reserve(snapshot.submaps.size());
  for (const auto& submap : snapshot.submaps) {
    if (submap.state != SubmapState::kFinished || !submap.geometry_ready ||
        submap.descriptor.empty()) {
      continue;
    }
    const double norm = Norm(submap.descriptor);
    if (!(norm > 0.0)) {
      continue;
    }
    entries_.push_back(
        {submap.session_id, submap.trajectory_id, submap.id,
         submap.descriptor, norm});
  }
}

std::vector<SubmapIndexCandidate> SubmapDescriptorIndex::query(
    const std::vector<float>& descriptor,
    const std::size_t maximum_candidates) const {
  std::vector<SubmapIndexCandidate> candidates;
  const double query_norm = Norm(descriptor);
  if (maximum_candidates == 0 || !(query_norm > 0.0)) {
    return candidates;
  }
  candidates.reserve(entries_.size());
  for (const auto& entry : entries_) {
    if (entry.descriptor.size() != descriptor.size()) {
      continue;
    }
    double dot = 0.0;
    for (std::size_t index = 0; index < descriptor.size(); ++index) {
      dot += static_cast<double>(descriptor[index]) * entry.descriptor[index];
    }
    candidates.push_back(
        {entry.session_id, entry.trajectory_id, entry.submap_id,
         dot / (query_norm * entry.norm)});
  }
  const auto order = [](const SubmapIndexCandidate& lhs,
                        const SubmapIndexCandidate& rhs) {
    if (lhs.score != rhs.score) {
      return lhs.score > rhs.score;
    }
    if (lhs.session_id != rhs.session_id) {
      return lhs.session_id < rhs.session_id;
    }
    if (lhs.trajectory_id != rhs.trajectory_id) {
      return lhs.trajectory_id < rhs.trajectory_id;
    }
    return lhs.submap_id < rhs.submap_id;
  };
  if (candidates.size() > maximum_candidates) {
    std::partial_sort(candidates.begin(),
                      candidates.begin() + maximum_candidates,
                      candidates.end(), order);
    candidates.resize(maximum_candidates);
  } else {
    std::sort(candidates.begin(), candidates.end(), order);
  }
  return candidates;
}

std::vector<float> SubmapDescriptorIndex::buildRadialDescriptor(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const std::size_t radial_bins) {
  std::vector<float> descriptor(radial_bins, 0.0F);
  if (!cloud.valid() || cloud.empty() || radial_bins == 0) {
    return descriptor;
  }
  double max_radius = 0.0;
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::array<float, 3> point{};
    if (!cloud.view().xyz(index, point)) {
      return {};
    }
    max_radius = std::max(max_radius,
                          std::hypot(static_cast<double>(point[0]),
                                     static_cast<double>(point[1])));
  }
  if (!(max_radius > 0.0) || !std::isfinite(max_radius)) {
    descriptor.front() = 1.0F;
    return descriptor;
  }
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::array<float, 3> point{};
    if (!cloud.view().xyz(index, point)) {
      return {};
    }
    const double radius = std::hypot(static_cast<double>(point[0]),
                                     static_cast<double>(point[1]));
    const auto bin = std::min(
        radial_bins - 1,
        static_cast<std::size_t>(radius / max_radius * radial_bins));
    descriptor[bin] += 1.0F;
  }
  const float scale = 1.0F / static_cast<float>(cloud.size());
  for (float& value : descriptor) {
    value *= scale;
  }
  return descriptor;
}

}  // namespace hybrid_slam::mapping
