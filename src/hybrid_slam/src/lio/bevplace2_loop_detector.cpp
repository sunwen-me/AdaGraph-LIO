#include "lio/bevplace2_loop_detector.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_set>
#include <utility>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {
namespace {

using hybrid_slam::backend::LoopCandidate;
using hybrid_slam::backend::LoopDetectionSnapshot;
using hybrid_slam::common::NodeId;
using hybrid_slam::mapping::KeyFrameSnapshot;
using hybrid_slam::mapping::SubmapSnapshot;

struct WireHeader {
  char magic[4];
  std::uint32_t size;
};
static_assert(sizeof(WireHeader) == 8);

struct BevPoseEstimate {
  hybrid_slam::geometry::RigidTransform3d T_match_query;
  std::size_t inliers = 0;
  double residual_m = std::numeric_limits<double>::infinity();
};

bool WriteAll(const int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  while (size > 0) {
    const auto written = send(fd, bytes, size, MSG_NOSIGNAL);
    if (written <= 0) {
      return false;
    }
    bytes += written;
    size -= static_cast<std::size_t>(written);
  }
  return true;
}

bool ReadAll(const int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<std::uint8_t*>(data);
  while (size > 0) {
    const auto read_size = recv(fd, bytes, size, 0);
    if (read_size <= 0) {
      return false;
    }
    bytes += read_size;
    size -= static_cast<std::size_t>(read_size);
  }
  return true;
}

std::optional<std::vector<float>> RequestDescriptor(
    const BevPlace2Config& config, const std::vector<std::uint8_t>& image) {
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return std::nullopt;
  }
  timeval timeout{};
  timeout.tv_sec = config.request_timeout_ms / 1000;
  timeout.tv_usec = (config.request_timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (config.socket_path.size() >= sizeof(address.sun_path)) {
    close(fd);
    return std::nullopt;
  }
  std::strncpy(address.sun_path, config.socket_path.c_str(),
               sizeof(address.sun_path) - 1);
  if (connect(fd, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) !=
      0) {
    close(fd);
    return std::nullopt;
  }
  const WireHeader request{{'B', 'V', 'P', '2'},
                           static_cast<std::uint32_t>(image.size())};
  WireHeader response{};
  std::vector<float> descriptor(config.descriptor_dimension);
  const bool valid = WriteAll(fd, &request, sizeof(request)) &&
                     WriteAll(fd, image.data(), image.size()) &&
                     ReadAll(fd, &response, sizeof(response)) &&
                     std::memcmp(response.magic, "BVR2", 4) == 0 &&
                     response.size == config.descriptor_dimension &&
                     ReadAll(fd, descriptor.data(),
                             descriptor.size() * sizeof(float));
  close(fd);
  if (!valid || !std::all_of(descriptor.begin(), descriptor.end(),
                             [](const float value) {
                               return std::isfinite(value);
                             })) {
    return std::nullopt;
  }
  return descriptor;
}

std::optional<BevPoseEstimate> RequestPoseEstimate(
    const BevPlace2Config& config, const std::vector<std::uint8_t>& query,
    const std::vector<std::uint8_t>& match) {
  if (query.size() != config.image_size * config.image_size ||
      match.size() != query.size()) {
    return std::nullopt;
  }
  const int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd < 0) {
    return std::nullopt;
  }
  timeval timeout{};
  timeout.tv_sec = config.request_timeout_ms / 1000;
  timeout.tv_usec = (config.request_timeout_ms % 1000) * 1000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  sockaddr_un address{};
  address.sun_family = AF_UNIX;
  if (config.socket_path.size() >= sizeof(address.sun_path)) {
    close(fd);
    return std::nullopt;
  }
  std::strncpy(address.sun_path, config.socket_path.c_str(),
               sizeof(address.sun_path) - 1);
  if (connect(fd, reinterpret_cast<const sockaddr*>(&address),
              sizeof(address)) != 0) {
    close(fd);
    return std::nullopt;
  }
  const WireHeader request{{'B', 'V', 'P', '3'},
                           static_cast<std::uint32_t>(query.size() +
                                                      match.size())};
  WireHeader response{};
  std::array<float, 8> values{};
  const bool valid = WriteAll(fd, &request, sizeof(request)) &&
                     WriteAll(fd, query.data(), query.size()) &&
                     WriteAll(fd, match.data(), match.size()) &&
                     ReadAll(fd, &response, sizeof(response)) &&
                     std::memcmp(response.magic, "BVR3", 4) == 0 &&
                     response.size == values.size() &&
                     ReadAll(fd, values.data(), values.size() * sizeof(float));
  close(fd);
  if (!valid || !std::all_of(values.begin(), values.end(),
                             [](const float value) {
                               return std::isfinite(value);
                             })) {
    return std::nullopt;
  }
  const auto inliers = static_cast<std::size_t>(std::lround(values[6]));
  const double yaw = std::atan2(values[2], values[0]);
  return BevPoseEstimate{{Eigen::Quaterniond(
                              Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
                          Eigen::Vector3d(values[4], values[5], 0.0)},
                         inliers, values[7]};
}

const KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map, const NodeId node_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [node_id](const auto& keyframe) { return keyframe.node_id == node_id; });
  return found == map.keyframes.end() ? nullptr : &*found;
}

const KeyFrameSnapshot* FindKeyFrame(
    const hybrid_slam::mapping::MapSnapshot& map,
    const hybrid_slam::common::KeyFrameId keyframe_id,
    const hybrid_slam::common::SessionId session_id,
    const hybrid_slam::common::TrajectoryId trajectory_id) {
  const auto found = std::find_if(
      map.keyframes.begin(), map.keyframes.end(),
      [keyframe_id, session_id, trajectory_id](const auto& keyframe) {
        return keyframe.keyframe_id == keyframe_id &&
               keyframe.session_id == session_id &&
               keyframe.trajectory_id == trajectory_id;
      });
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

std::vector<std::uint8_t> BuildBevImage(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud,
    const BevPlace2Config& config,
    const hybrid_slam::geometry::RigidTransform3d& T_bev_cloud =
        hybrid_slam::geometry::RigidTransform3d::Identity()) {
  const auto pixel_count = config.image_size * config.image_size;
  std::vector<std::uint8_t> image(pixel_count, 0);
  if (!cloud.valid() || cloud.empty()) {
    return image;
  }
  const auto half_cells = static_cast<int>(std::llround(
      config.half_extent_m / config.voxel_size_m));
  std::unordered_set<std::uint32_t> occupied_voxels;
  occupied_voxels.reserve(cloud.size());
  for (std::size_t index = 0; index < cloud.size(); ++index) {
    std::array<float, 3> point{};
    if (!cloud.view().xyz(index, point)) {
      continue;
    }
    const Eigen::Vector3d transformed =
        T_bev_cloud.transformPoint(Eigen::Vector3d(point[0], point[1], point[2]));
    const double x = transformed.x();
    const double y = transformed.y();
    const double z = transformed.z();
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        std::abs(x) >= config.half_extent_m ||
        std::abs(y) >= config.half_extent_m ||
        std::abs(z) >= config.half_extent_m) {
      continue;
    }
    const auto vx = static_cast<int>(std::floor(x / config.voxel_size_m));
    const auto vy = static_cast<int>(std::floor(y / config.voxel_size_m));
    const auto vz = static_cast<int>(std::floor(z / config.voxel_size_m));
    const auto voxel_key = static_cast<std::uint32_t>(vx + 128) << 16U |
                           static_cast<std::uint32_t>(vy + 128) << 8U |
                           static_cast<std::uint32_t>(vz + 128);
    if (!occupied_voxels.insert(voxel_key).second) {
      continue;
    }
    const auto column = half_cells - static_cast<int>(
        std::floor(y / config.voxel_size_m));
    const auto row = half_cells - static_cast<int>(
        std::floor(x / config.voxel_size_m));
    if (row < 0 || column < 0 ||
        row >= static_cast<int>(config.image_size) ||
        column >= static_cast<int>(config.image_size)) {
      continue;
    }
    auto& cell = image[static_cast<std::size_t>(row) * config.image_size +
                       static_cast<std::size_t>(column)];
    cell = static_cast<std::uint8_t>(std::min(10, static_cast<int>(cell) + 1));
  }
  const auto maximum = *std::max_element(image.begin(), image.end());
  if (maximum == 0) {
    return image;
  }
  for (auto& cell : image) {
    cell = static_cast<std::uint8_t>(std::lround(
        static_cast<double>(cell) / maximum * 255.0));
  }
  return image;
}

double Similarity(const std::vector<float>& lhs, const std::vector<float>& rhs) {
  if (lhs.size() != rhs.size()) {
    return -std::numeric_limits<double>::infinity();
  }
  double score = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    score += static_cast<double>(lhs[index]) * rhs[index];
  }
  return score;
}

}  // namespace

std::optional<std::string> BevPlace2Config::validate() const {
  if (socket_path.empty() || image_size != 201 || descriptor_dimension != 8192 ||
      !std::isfinite(voxel_size_m) || voxel_size_m <= 0.0 ||
      !std::isfinite(half_extent_m) || half_extent_m <= 0.0 ||
      request_timeout_ms <= 0 || minimum_pose_inliers < 3 ||
      !std::isfinite(maximum_pose_residual_m) ||
      maximum_pose_residual_m <= 0.0 ||
      !std::isfinite(maximum_registration_initial_translation_m) ||
      maximum_registration_initial_translation_m <= 0.0 ||
      minimum_sequence_consensus == 0 ||
      maximum_sequence_gap_submaps == 0 ||
      minimum_pose_free_sequence_consensus <= minimum_sequence_consensus ||
      representative_keyframes_per_submap == 0 ||
      representative_keyframes_per_submap > 9) {
    return "invalid BEVPlace2 configuration";
  }
  return std::nullopt;
}

std::size_t CountBevPlace2SequenceConsensus(
    const BevPlace2SequenceObservation& current,
    const std::vector<BevPlace2SequenceObservation>& history,
    const std::size_t maximum_gap_submaps) {
  if (maximum_gap_submaps == 0) {
    return 1;
  }
  std::size_t count = 1;
  std::unordered_set<std::uint64_t> counted_query_submaps;
  for (const auto& previous : history) {
    if (!IsBevPlace2SequenceConsistent(current, previous,
                                       maximum_gap_submaps) ||
        !counted_query_submaps.insert(
            previous.query_submap_id.value()).second) {
      continue;
    }
    ++count;
  }
  return count;
}

bool ShouldVerifyBevPlace2WithoutCoarsePose(
    const BevPlace2SequenceObservation& current,
    const std::vector<BevPlace2SequenceObservation>& pose_history,
    const std::size_t maximum_gap_submaps,
    const std::size_t minimum_pose_free_sequence_consensus) {
  return minimum_pose_free_sequence_consensus > 1 &&
         CountBevPlace2SequenceConsensus(current, pose_history,
                                         maximum_gap_submaps) >=
             minimum_pose_free_sequence_consensus;
}

bool IsBevPlace2SequenceConsistent(
    const BevPlace2SequenceObservation& current,
    const BevPlace2SequenceObservation& previous,
    const std::size_t maximum_gap_submaps) {
  if (maximum_gap_submaps == 0 ||
      previous.query_session_id != current.query_session_id ||
      previous.query_trajectory_id != current.query_trajectory_id ||
      previous.match_session_id != current.match_session_id ||
      previous.match_trajectory_id != current.match_trajectory_id) {
    return false;
  }
  const auto current_query = current.query_submap_id.value();
  const auto previous_query = previous.query_submap_id.value();
  if (previous_query >= current_query) {
    return false;
  }
  const auto query_gap = current_query - previous_query;
  if (query_gap > maximum_gap_submaps) {
    return false;
  }
  const auto current_match = current.match_submap_id.value();
  const auto previous_match = previous.match_submap_id.value();
  const auto match_gap = current_match >= previous_match
                             ? current_match - previous_match
                             : previous_match - current_match;
  return match_gap == query_gap;
}

BevPlace2LoopDetector::BevPlace2LoopDetector(Config config)
    : config_(std::move(config)) {
  if (const auto error = config_.bevplace2.validate()) {
    throw std::invalid_argument(*error);
  }
  if (const auto error = config_.yaw_initializer.validate()) {
    throw std::invalid_argument(*error);
  }
  if (config_.maximum_candidates == 0 ||
      !std::isfinite(config_.minimum_temporal_separation_s) ||
      config_.minimum_temporal_separation_s < 0.0) {
    throw std::invalid_argument("invalid BEVPlace2 loop detector limits");
  }
}

bool BevPlace2LoopDetector::rebuildIfNeeded(
    const LoopDetectionSnapshot& snapshot) {
  if (!snapshot.map || cached_map_version_ == snapshot.map->version) {
    return true;
  }
  std::vector<DescriptorEntry> next_entries;
  next_entries.reserve(
      snapshot.map->submaps.size() *
      (config_.bevplace2.representative_keyframes_per_submap +
       (config_.bevplace2.use_submap_geometry_descriptor ? 1U : 0U)));
  for (const auto& submap : snapshot.map->submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid()) {
      continue;
    }
    std::vector<const KeyFrameSnapshot*> keyframes;
    for (const auto& keyframe : snapshot.map->keyframes) {
      if (keyframe.submap_id == submap.id &&
          keyframe.session_id == submap.session_id &&
          keyframe.trajectory_id == submap.trajectory_id && keyframe.valid()) {
        keyframes.push_back(&keyframe);
      }
    }
    std::sort(keyframes.begin(), keyframes.end(), [](const auto* lhs,
                                                      const auto* rhs) {
      return lhs->T_odom_base.timestamp < rhs->T_odom_base.timestamp;
    });

    const auto append_entry =
        [&](const KeyFrameSnapshot& representative,
            const bool submap_geometry_view,
            const hybrid_slam::sensor::ImmutablePointCloud& cloud,
            const hybrid_slam::geometry::RigidTransform3d& T_bev_cloud) {
          const auto cached = std::find_if(
              entries_.begin(), entries_.end(), [&](const auto& entry) {
                return entry.session_id == submap.session_id &&
                       entry.trajectory_id == submap.trajectory_id &&
                       entry.submap_id == submap.id &&
                       entry.representative_node == representative.node_id &&
                       entry.submap_geometry_view == submap_geometry_view;
              });
          if (cached != entries_.end()) {
            next_entries.push_back(*cached);
            return true;
          }
          auto image = BuildBevImage(cloud, config_.bevplace2, T_bev_cloud);
          const auto descriptor = RequestDescriptor(config_.bevplace2, image);
          if (!descriptor) {
            LOG(WARNING)
                << "[BEVPlace2LoopDetector]: descriptor request failed for "
                << "submap=" << submap.id.value() << " node="
                << representative.node_id.value() << " view="
                << (submap_geometry_view ? "geometry" : "sweep");
            return false;
          }
          next_entries.push_back(
              {submap.session_id, submap.trajectory_id, submap.id,
               representative.node_id, submap_geometry_view,
               representative.T_odom_base.timestamp.seconds(), *descriptor,
               std::move(image)});
          return true;
        };

    if (!keyframes.empty()) {
      const auto sample_count = std::min(
          config_.bevplace2.representative_keyframes_per_submap,
          keyframes.size());
      std::unordered_set<std::uint64_t> sampled_nodes;
      for (std::size_t sample = 0; sample < sample_count; ++sample) {
        const auto index = sample_count == 1
                               ? keyframes.size() - 1
                               : sample * (keyframes.size() - 1) /
                                     (sample_count - 1);
        const auto* representative = keyframes[index];
        if (!sampled_nodes.insert(representative->node_id.value()).second) {
          continue;
        }
        if (!append_entry(
                *representative, false, representative->cloud_base,
                hybrid_slam::geometry::RigidTransform3d::Identity())) {
          return false;
        }
      }
    }

    if (config_.bevplace2.use_submap_geometry_descriptor) {
      const auto* anchor = FindKeyFrame(
          *snapshot.map, submap.anchor_keyframe_id, submap.session_id,
          submap.trajectory_id);
      const auto geometry_anchor_id = submap.geometry_anchor_keyframe_id.value_or(
          submap.anchor_keyframe_id);
      const auto* geometry_anchor = FindKeyFrame(
          *snapshot.map, geometry_anchor_id, submap.session_id,
          submap.trajectory_id);
      if (anchor && anchor->valid() && geometry_anchor &&
          geometry_anchor->valid()) {
        // cloud_submap is expressed in geometry_anchor. Recenter it at the
        // primary submap anchor so the descriptor pose and graph endpoint use
        // exactly the same coordinate frame even for rolling geometry.
        const auto T_anchor_geometry =
            anchor->T_odom_base.T_target_source.inverse().compose(
                geometry_anchor->T_odom_base.T_target_source);
        if (!append_entry(*anchor, true, submap.cloud_submap,
                          T_anchor_geometry)) {
          return false;
        }
      }
    }
  }
  entries_ = std::move(next_entries);
  cached_map_version_ = snapshot.map->version;
  return true;
}

std::vector<LoopCandidate> BevPlace2LoopDetector::detect(
    const LoopDetectionSnapshot& snapshot, const NodeId query_node) {
  std::vector<LoopCandidate> result;
  if (!snapshot.map || !snapshot.map->consistent() || !rebuildIfNeeded(snapshot)) {
    return result;
  }
  const auto* query = FindKeyFrame(*snapshot.map, query_node);
  if (!query || !query->valid()) {
    return result;
  }
  const auto* query_submap = FindFinishedSubmap(*snapshot.map, *query);
  if (!query_submap) {
    return result;
  }
  std::vector<const DescriptorEntry*> query_entries;
  for (const auto& entry : entries_) {
    if (entry.session_id == query_submap->session_id &&
        entry.trajectory_id == query_submap->trajectory_id &&
        entry.submap_id == query_submap->id) {
      query_entries.push_back(&entry);
    }
  }
  if (query_entries.empty()) {
    return result;
  }

  struct RankedDescriptorPair {
    double score = -std::numeric_limits<double>::infinity();
    const DescriptorEntry* query = nullptr;
    const DescriptorEntry* match = nullptr;
  };
  std::vector<RankedDescriptorPair> descriptor_pairs;
  descriptor_pairs.reserve(query_entries.size() * entries_.size());
  for (const auto* query_entry : query_entries) {
    for (const auto& entry : entries_) {
      if (entry.session_id == query_entry->session_id &&
          entry.trajectory_id == query_entry->trajectory_id &&
          entry.submap_id == query_entry->submap_id) {
        continue;
      }
      if (std::abs(entry.timestamp_s - query_entry->timestamp_s) <
          config_.minimum_temporal_separation_s) {
        continue;
      }
      descriptor_pairs.push_back(
          {Similarity(query_entry->descriptor, entry.descriptor), query_entry,
           &entry});
    }
  }
  std::sort(descriptor_pairs.begin(), descriptor_pairs.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.score > rhs.score;
            });
  std::vector<RankedDescriptorPair> ranked;
  std::set<std::tuple<std::uint64_t, std::uint64_t, std::uint64_t>>
      ranked_submaps;
  for (const auto& pair : descriptor_pairs) {
    const auto identity = std::tuple{
        pair.match->session_id.value(), pair.match->trajectory_id.value(),
        pair.match->submap_id.value()};
    if (ranked_submaps.insert(identity).second) {
      ranked.push_back(pair);
    }
  }
  std::ostringstream candidates;
  std::vector<BevPlace2SequenceObservation> observations;
  std::vector<PendingSequenceCandidate> pending_for_current_query;
  std::unordered_set<std::uint64_t> promoted_query_nodes;
  for (std::size_t index = 0;
       index < std::min(config_.maximum_candidates, ranked.size()); ++index) {
    const auto& ranked_pair = ranked[index];
    const auto* query_entry = ranked_pair.query;
    const auto* entry = ranked_pair.match;
    if (index != 0) {
      candidates << ',';
    }
    candidates << query_entry->representative_node.value() << "->"
               << entry->representative_node.value() << '['
               << (query_entry->submap_geometry_view ? 'g' : 's')
               << (entry->submap_geometry_view ? 'g' : 's') << ']';
    const BevPlace2SequenceObservation observation{
        query_entry->session_id,
        query_entry->trajectory_id,
        query_entry->submap_id,
        entry->session_id,
        entry->trajectory_id,
        entry->submap_id,
    };
    const auto sequence_consensus = CountBevPlace2SequenceConsensus(
        observation, sequence_history_,
        config_.bevplace2.maximum_sequence_gap_submaps);
    std::optional<BevPoseEstimate> pose;
    bool pose_quality_valid = false;
    std::size_t pose_view_attempts = 0;
    constexpr std::size_t kMaximumPoseViewAttempts = 4;
    constexpr std::size_t kMultiViewPoseRankLimit = 3;
    const auto maximum_pose_view_attempts =
        index < kMultiViewPoseRankLimit || sequence_consensus > 1
            ? kMaximumPoseViewAttempts
            : 1U;
    // Retrieval ranks a submap by its best view pair. That pair is not always
    // the pair with the largest local-feature overlap, especially under a
    // 180-degree outbound/return viewpoint change. Try a few of the same
    // submap's next-best view pairs before declaring its geometric pose
    // unavailable.
    for (const auto& view_pair : descriptor_pairs) {
      if (view_pair.match->session_id != entry->session_id ||
          view_pair.match->trajectory_id != entry->trajectory_id ||
          view_pair.match->submap_id != entry->submap_id) {
        continue;
      }
      const auto attempt = RequestPoseEstimate(
          config_.bevplace2, view_pair.query->bev_image,
          view_pair.match->bev_image);
      ++pose_view_attempts;
      if (attempt &&
          (!pose || attempt->inliers > pose->inliers ||
           (attempt->inliers == pose->inliers &&
            attempt->residual_m < pose->residual_m))) {
        pose = attempt;
      }
      if (attempt &&
          attempt->inliers >= config_.bevplace2.minimum_pose_inliers &&
          attempt->residual_m <= config_.bevplace2.maximum_pose_residual_m) {
        pose = attempt;
        query_entry = view_pair.query;
        entry = view_pair.match;
        pose_quality_valid = true;
        break;
      }
      if (pose_view_attempts >= maximum_pose_view_attempts) {
        break;
      }
    }
    if (!pose_quality_valid) {
      const bool verify_without_coarse_pose =
          ShouldVerifyBevPlace2WithoutCoarsePose(
              observation, sequence_history_,
              config_.bevplace2.maximum_sequence_gap_submaps,
              config_.bevplace2.minimum_pose_free_sequence_consensus);
      if (verify_without_coarse_pose) {
        result.push_back({query_entry->representative_node,
                          entry->representative_node, ranked_pair.score,
                          std::nullopt, std::nullopt});
        LOG(INFO) << "[BEVPlace2LoopDetector]: pose_free_sequence_candidate"
                  << " query_node="
                  << query_entry->representative_node.value()
                  << " match_node="
                  << entry->representative_node.value()
                  << " descriptor_score=" << ranked_pair.score
                  << " views="
                  << (query_entry->submap_geometry_view ? 'g' : 's')
                  << (entry->submap_geometry_view ? 'g' : 's')
                  << " view_attempts=" << pose_view_attempts
                  << " sequence_consensus=" << sequence_consensus
                  << " required="
                  << config_.bevplace2.minimum_pose_free_sequence_consensus
                  << " bev_pose=" << (pose ? "subthreshold" : "unavailable")
                  << " inliers=" << (pose ? pose->inliers : 0)
                  << " residual="
                  << (pose ? pose->residual_m
                           : std::numeric_limits<double>::infinity());
        continue;
      }
      LOG(INFO) << "[BEVPlace2LoopDetector]: rejected query_node="
                << query_entry->representative_node.value() << " match_node="
                << entry->representative_node.value()
                << " views="
                << (query_entry->submap_geometry_view ? 'g' : 's')
                << (entry->submap_geometry_view ? 'g' : 's')
                << " view_attempts=" << pose_view_attempts
                << " reason="
                << (pose ? "pose_quality_failed" : "pose_estimation_failed")
                << " sequence_consensus=" << sequence_consensus
                << " pose_free_required="
                << config_.bevplace2.minimum_pose_free_sequence_consensus
                << " inliers=" << (pose ? pose->inliers : 0)
                << " residual="
                << (pose ? pose->residual_m
                         : std::numeric_limits<double>::infinity());
      continue;
    }
    LOG(INFO) << "[BEVPlace2LoopDetector]: coarse_pose query_node="
              << query_entry->representative_node.value() << " match_node="
              << entry->representative_node.value() << " inliers=" << pose->inliers
              << " residual=" << pose->residual_m << " m translation="
              << pose->T_match_query.translation.norm() << " m views="
              << (query_entry->submap_geometry_view ? 'g' : 's')
              << (entry->submap_geometry_view ? 'g' : 's')
              << " view_attempts=" << pose_view_attempts;
    observations.push_back(observation);
    const LoopCandidate candidate{
        query_entry->representative_node, entry->representative_node,
        ranked_pair.score, std::nullopt, pose->T_match_query};
    if (sequence_consensus <
        config_.bevplace2.minimum_sequence_consensus) {
      pending_for_current_query.push_back({observation, candidate});
      LOG(INFO) << "[BEVPlace2LoopDetector]: rejected query_node="
                << query_entry->representative_node.value() << " match_node="
                << entry->representative_node.value()
                << " reason=sequence_consistency_pending"
                << " sequence_consensus=" << sequence_consensus
                << " required="
                << config_.bevplace2.minimum_sequence_consensus
                << " maximum_gap_submaps="
                << config_.bevplace2.maximum_sequence_gap_submaps;
      continue;
    }
    // Once a second adjacent submap confirms the sequence, the first
    // observation is no longer an isolated appearance match.  Forward both
    // endpoints to the same strict 3D verifier.  Permanently discarding the
    // first observation left only one constraint when the next BEV RANSAC
    // happened to fail, unlike Cartographer's multiple node-to-submap
    // constraints over a revisit interval.
    for (const auto& pending : pending_sequence_candidates_) {
      if (!IsBevPlace2SequenceConsistent(
              observation, pending.observation,
              config_.bevplace2.maximum_sequence_gap_submaps)) {
        continue;
      }
      if (promoted_query_nodes.insert(
              pending.candidate.query_node.value()).second) {
        result.push_back(pending.candidate);
        LOG(INFO) << "[BEVPlace2LoopDetector]: promoted_pending query_node="
                  << pending.candidate.query_node.value() << " match_node="
                  << pending.candidate.match_node.value()
                  << " confirmed_by_query_node="
                  << query_entry->representative_node.value()
                  << " sequence_consensus=" << sequence_consensus;
      }
    }
    result.push_back(candidate);
  }
  sequence_history_.erase(
      std::remove_if(
          sequence_history_.begin(), sequence_history_.end(),
          [&](const auto& observation) {
            return observation.query_session_id == query_submap->session_id &&
                   observation.query_trajectory_id ==
                       query_submap->trajectory_id &&
                   observation.query_submap_id == query_submap->id;
          }),
      sequence_history_.end());
  sequence_history_.insert(sequence_history_.end(), observations.begin(),
                           observations.end());
  pending_sequence_candidates_.erase(
      std::remove_if(
          pending_sequence_candidates_.begin(),
          pending_sequence_candidates_.end(),
          [&](const auto& pending) {
            return promoted_query_nodes.contains(
                       pending.candidate.query_node.value()) ||
                   (pending.observation.query_session_id ==
                        query_submap->session_id &&
                    pending.observation.query_trajectory_id ==
                        query_submap->trajectory_id &&
                    pending.observation.query_submap_id ==
                        query_submap->id);
          }),
      pending_sequence_candidates_.end());
  pending_sequence_candidates_.insert(
      pending_sequence_candidates_.end(), pending_for_current_query.begin(),
      pending_for_current_query.end());
  constexpr std::size_t kMaximumSequenceHistory = 512;
  if (sequence_history_.size() > kMaximumSequenceHistory) {
    sequence_history_.erase(
        sequence_history_.begin(),
        sequence_history_.end() - kMaximumSequenceHistory);
  }
  if (pending_sequence_candidates_.size() > kMaximumSequenceHistory) {
    pending_sequence_candidates_.erase(
        pending_sequence_candidates_.begin(),
        pending_sequence_candidates_.end() - kMaximumSequenceHistory);
  }
  LOG(INFO) << "[BEVPlace2LoopDetector]: query_node=" << query_node.value()
            << " eligible_submaps=" << ranked.size()
            << " ranked_candidates=" << candidates.str();
  return result;
}

}  // namespace LI2Sup
