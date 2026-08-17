#include "lio/pcl_submap_relocalizer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

#include <pcl/common/transforms.h>

#include "hybrid_slam/logging/logging.h"
#include "hybrid_slam/localization/localization_map.h"

namespace LI2Sup {
namespace {

using hybrid_slam::adapter::PclPoint;
using hybrid_slam::adapter::PclPointCloud;
using hybrid_slam::geometry::FrameId;
using hybrid_slam::localization::LocalizationMap;
using hybrid_slam::localization::RelocalizationCandidate;
using hybrid_slam::localization::RelocalizationQuery;
using hybrid_slam::localization::RegistrationVerification;
using hybrid_slam::localization::SubmapRelocalizationResult;
using hybrid_slam::mapping::MapSnapshot;
using hybrid_slam::mapping::SubmapSnapshot;

std::vector<const SubmapSnapshot*> SeedableSubmaps(const MapSnapshot& map) {
  std::vector<const SubmapSnapshot*> result;
  result.reserve(map.submaps.size());
  for (const auto& submap : map.submaps) {
    if (submap.valid() &&
        (submap.geometry_ready || !submap.keyframe_ids.empty())) {
      result.push_back(&submap);
    }
  }
  return result;
}

std::optional<LocalizationMap> BuildFinishedSubmapTarget(
    const MapSnapshot& map) {
  auto cloud = std::make_shared<PclPointCloud>();
  for (const auto& submap : map.submaps) {
    if (submap.state != hybrid_slam::mapping::SubmapState::kFinished ||
        !submap.geometry_ready || !submap.valid() ||
        !submap.T_map_submap.isFinite()) {
      continue;
    }
    const auto& view = submap.cloud_submap.view();
    cloud->points.reserve(cloud->size() + view.size());
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> point{};
      if (!view.xyz(index, point)) {
        return std::nullopt;
      }
      const Eigen::Vector3d mapped = submap.T_map_submap.transformPoint(
          Eigen::Vector3d(point[0], point[1], point[2]));
      if (mapped.allFinite()) {
        cloud->points.push_back({static_cast<float>(mapped.x()),
                                 static_cast<float>(mapped.y()),
                                 static_cast<float>(mapped.z()), 0.0F});
      }
    }
  }
  if (cloud->empty()) {
    return std::nullopt;
  }
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = false;
  std::shared_ptr<const PclPointCloud> frozen = cloud;
  const auto view = hybrid_slam::sensor::PointCloudView::InterleavedFloatXyz(
      frozen->data(), frozen->size(), sizeof(PclPoint), offsetof(PclPoint, x),
      offsetof(PclPoint, y), offsetof(PclPoint, z), FrameId::kMap);
  LocalizationMap target(
      map.version, hybrid_slam::sensor::ImmutablePointCloud::Share(
                       std::move(frozen), view,
                       cloud->points.capacity() * sizeof(PclPoint)));
  return target.valid() ? std::optional<LocalizationMap>(std::move(target))
                        : std::nullopt;
}

hybrid_slam::adapter::PclPointCloud::ConstPtr CropTargetAroundCandidate(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& target,
    const BASIC::SE3& initial_T_map_base, const double radius_m) {
  if (!target || target->empty() || !std::isfinite(radius_m) ||
      radius_m <= 0.0) {
    return {};
  }
  const Eigen::Vector3d center = initial_T_map_base.t_.cast<double>();
  if (!center.allFinite()) {
    return {};
  }
  const double squared_radius = radius_m * radius_m;
  auto cropped = std::make_shared<PclPointCloud>();
  for (const auto& point : target->points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    const Eigen::Vector3d delta(
        static_cast<double>(point.x) - center.x(),
        static_cast<double>(point.y) - center.y(),
        static_cast<double>(point.z) - center.z());
    if (delta.squaredNorm() <= squared_radius) {
      cropped->points.push_back(point);
    }
  }
  if (cropped->empty()) {
    return {};
  }
  cropped->width = static_cast<std::uint32_t>(cropped->size());
  cropped->height = 1;
  cropped->is_dense = false;
  return cropped;
}

double DistanceSquaredToBounds(
    const Eigen::Vector3d& point,
    const hybrid_slam::mapping::SubmapBounds& bounds) {
  if (!bounds.has_points) {
    return point.squaredNorm();
  }
  Eigen::Vector3d delta = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis) {
    if (point[axis] < bounds.minimum[axis]) {
      delta[axis] = bounds.minimum[axis] - point[axis];
    } else if (point[axis] > bounds.maximum[axis]) {
      delta[axis] = point[axis] - bounds.maximum[axis];
    }
  }
  return delta.squaredNorm();
}

const SubmapSnapshot* FindSeedSubmap(
    const MapSnapshot& map,
    const hybrid_slam::geometry::RigidTransform3d& T_map_base) {
  if (!T_map_base.isFinite()) {
    return nullptr;
  }
  const auto submaps = SeedableSubmaps(map);
  const SubmapSnapshot* best = nullptr;
  double best_distance_squared = std::numeric_limits<double>::infinity();
  for (const auto* submap : submaps) {
    double distance_squared = std::numeric_limits<double>::infinity();
    if (submap->geometry_ready && submap->T_map_submap.isFinite()) {
      const Eigen::Vector3d point_submap =
          submap->T_map_submap.inverse().transformPoint(
              T_map_base.translation);
      distance_squared = DistanceSquaredToBounds(point_submap, submap->bounds);
    } else {
      for (const auto keyframe_id : submap->keyframe_ids) {
        for (const auto& keyframe : map.keyframes) {
          if (keyframe.keyframe_id == keyframe_id &&
              keyframe.session_id == submap->session_id &&
              keyframe.trajectory_id == submap->trajectory_id &&
              keyframe.submap_id == submap->id && keyframe.valid()) {
            distance_squared = std::min(
                distance_squared,
                (T_map_base.translation -
                 keyframe.T_map_base.T_target_source.translation)
                    .squaredNorm());
          }
        }
      }
    }
    if (distance_squared < best_distance_squared) {
      best_distance_squared = distance_squared;
      best = submap;
    }
  }
  return best;
}

const SubmapSnapshot* FindSeedSubmap(
    const MapSnapshot& map,
    const hybrid_slam::geometry::StampedTransform3d& T_map_base) {
  if (!T_map_base.valid() || T_map_base.target_frame != FrameId::kMap ||
      T_map_base.source_frame != FrameId::kBase) {
    return nullptr;
  }
  return FindSeedSubmap(map, T_map_base.T_target_source);
}

PclPointCloud::ConstPtr BuildNeighborSubmapTarget(
    const MapSnapshot& map,
    const hybrid_slam::geometry::RigidTransform3d& initial_T_map_base,
    const std::size_t neighbor_submaps) {
  const auto* seed = FindSeedSubmap(map, initial_T_map_base);
  if (seed == nullptr) {
    return {};
  }
  std::unordered_set<std::uint64_t> selected_submaps;
  for (const auto& submap : map.submaps) {
    if (submap.session_id != seed->session_id ||
        submap.trajectory_id != seed->trajectory_id) {
      continue;
    }
    const auto id = submap.id.value();
    const auto seed_id = seed->id.value();
    const auto distance = id >= seed_id ? id - seed_id : seed_id - id;
    if (distance <= neighbor_submaps) {
      selected_submaps.insert(id);
    }
  }
  if (selected_submaps.empty()) {
    return {};
  }

  auto target = std::make_shared<PclPointCloud>();
  for (const auto& keyframe : map.keyframes) {
    if (!keyframe.valid() || keyframe.session_id != seed->session_id ||
        keyframe.trajectory_id != seed->trajectory_id ||
        !selected_submaps.contains(keyframe.submap_id.value())) {
      continue;
    }
    const auto& view = keyframe.cloud_base.view();
    target->points.reserve(target->size() + view.size());
    for (std::size_t index = 0; index < view.size(); ++index) {
      std::array<float, 3> point{};
      if (!view.xyz(index, point)) {
        return {};
      }
      const Eigen::Vector3d mapped =
          keyframe.T_map_base.T_target_source.transformPoint(
              Eigen::Vector3d(point[0], point[1], point[2]));
      if (mapped.allFinite()) {
        target->points.push_back({static_cast<float>(mapped.x()),
                                  static_cast<float>(mapped.y()),
                                  static_cast<float>(mapped.z()), 0.0F});
      }
    }
  }
  if (target->empty()) {
    return {};
  }
  target->width = static_cast<std::uint32_t>(target->size());
  target->height = 1;
  target->is_dense = false;
  return target;
}

bool ValidCandidate(const RelocalizationCandidate& candidate,
                    const RelocalizationQuery& query) {
  return candidate.initial_T_map_base.valid() &&
         candidate.initial_T_map_base.timestamp == query.T_odom_base.timestamp &&
         candidate.initial_T_map_base.target_frame == FrameId::kMap &&
         candidate.initial_T_map_base.source_frame == FrameId::kBase &&
         std::isfinite(candidate.retrieval_score);
}

std::optional<hybrid_slam::geometry::StampedTransform3d> SnapshotMapPrior(
    const RelocalizationQuery& query, const MapSnapshot& map,
    const double max_age_seconds) {
  if (!query.valid()) {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot prior diagnostic"
              << " reason=invalid_query";
    return std::nullopt;
  }
  if (!std::isfinite(max_age_seconds) || max_age_seconds < 0.0) {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot prior diagnostic"
              << " reason=invalid_max_age"
              << " max_age_s=" << max_age_seconds;
    return std::nullopt;
  }
  const auto query_ns = query.T_odom_base.timestamp.nanoseconds();
  const hybrid_slam::mapping::KeyFrameSnapshot* latest = nullptr;
  std::size_t valid_keyframes = 0;
  std::size_t future_keyframes = 0;
  for (const auto& keyframe : map.keyframes) {
    if (!keyframe.valid()) {
      continue;
    }
    ++valid_keyframes;
    if (keyframe.T_odom_base.timestamp.nanoseconds() > query_ns) {
      ++future_keyframes;
      continue;
    }
    if (latest == nullptr ||
        keyframe.T_odom_base.timestamp > latest->T_odom_base.timestamp) {
      latest = &keyframe;
    }
  }
  if (latest == nullptr) {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot prior diagnostic"
              << " reason=no_past_keyframe"
              << " query_timestamp_ns=" << query_ns
              << " valid_keyframes=" << valid_keyframes
              << " future_keyframes=" << future_keyframes;
    return std::nullopt;
  }
  const double age_seconds =
      static_cast<double>(query_ns -
                          latest->T_odom_base.timestamp.nanoseconds()) *
      1e-9;
  if (!std::isfinite(age_seconds) || age_seconds > max_age_seconds) {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot prior diagnostic"
              << " reason=stale_keyframe"
              << " query_timestamp_ns=" << query_ns
              << " latest_timestamp_ns="
              << latest->T_odom_base.timestamp.nanoseconds()
              << " age_s=" << age_seconds
              << " max_age_s=" << max_age_seconds
              << " valid_keyframes=" << valid_keyframes
              << " future_keyframes=" << future_keyframes;
    return std::nullopt;
  }
  const auto T_map_odom = latest->T_map_base.T_target_source.compose(
      latest->T_odom_base.T_target_source.inverse());
  const auto T_map_base = T_map_odom.compose(query.T_odom_base.T_target_source);
  return hybrid_slam::geometry::StampedTransform3d{
      query.T_odom_base.timestamp, FrameId::kMap, FrameId::kBase,
      T_map_base};
}

bool MeetsQuality(const RegistrationVerification& verification,
                  const PclSubmapRelocalizationConfig& config) {
  return verification.T_map_base.valid() &&
         verification.T_map_base.target_frame == FrameId::kMap &&
         verification.T_map_base.source_frame == FrameId::kBase &&
         std::isfinite(verification.fitness) && verification.fitness >= 0.0 &&
         verification.fitness <= config.registration.max_fitness &&
         std::isfinite(verification.inlier_ratio) &&
         verification.inlier_ratio >= config.minimum_inlier_ratio &&
         verification.inlier_ratio <= 1.0 && std::isfinite(verification.overlap) &&
         verification.overlap >= config.minimum_overlap &&
         verification.overlap <= 1.0 && verification.observable;
}

std::string QualityRejectionReason(
    const RegistrationVerification& verification,
    const PclSubmapRelocalizationConfig& config) {
  if (!verification.T_map_base.valid()) {
    return "invalid_pose";
  }
  if (verification.T_map_base.target_frame != FrameId::kMap ||
      verification.T_map_base.source_frame != FrameId::kBase) {
    return "wrong_pose_frame";
  }
  if (!std::isfinite(verification.fitness) || verification.fitness < 0.0) {
    return "invalid_fitness";
  }
  if (verification.fitness > config.registration.max_fitness) {
    return "fitness";
  }
  if (!std::isfinite(verification.inlier_ratio) ||
      verification.inlier_ratio < config.minimum_inlier_ratio) {
    return "inlier_ratio";
  }
  if (verification.inlier_ratio > 1.0) {
    return "inlier_ratio_range";
  }
  if (!std::isfinite(verification.overlap) ||
      verification.overlap < config.minimum_overlap) {
    return "overlap";
  }
  if (verification.overlap > 1.0) {
    return "overlap_range";
  }
  if (!verification.observable) {
    return "observability";
  }
  return "unknown";
}

}  // namespace

std::optional<std::string> PclSubmapRelocalizationConfig::validate() const {
  if (const auto error = registration.validate()) {
    return error;
  }
  if (query_window_frames == 0) {
    return "submap recovery query window must be positive";
  }
  if (!std::isfinite(map_odom_prior_max_age_seconds) ||
      map_odom_prior_max_age_seconds < 0.0) {
    return "map to odom prior age must be non-negative";
  }
  if (!std::isfinite(minimum_inlier_ratio) || minimum_inlier_ratio <= 0.0 ||
      minimum_inlier_ratio > 1.0 || !std::isfinite(minimum_overlap) ||
      minimum_overlap <= 0.0 || minimum_overlap > 1.0) {
    return "submap quality ratios must be within (0, 1]";
  }
  return std::nullopt;
}

PclSubmapRelocalizer::PclSubmapRelocalizer(
    PclSubmapRelocalizationConfig config,
    PclSubmapRelocalizerOperations operations)
    : config_(std::move(config)), operations_(std::move(operations)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid PCL submap relocalizer config: " +
                                *error);
  }
}

std::optional<SubmapRelocalizationResult> PclSubmapRelocalizer::Relocalize(
    const RelocalizationQuery& query, const MapSnapshot& map,
    const hybrid_slam::mapping::SubmapDescriptorIndex&,
    std::stop_token stop_token) const {
  if (stop_token.stop_requested() || !query.valid() || !map.consistent()) {
    return std::nullopt;
  }
  auto target = hybrid_slam::localization::BuildLocalizationMap(map);
  if (!target && map.keyframes.empty()) {
    // Compatibility for persisted snapshots created before keyframe clouds
    // were retained. Current runtime snapshots always take the keyframe path.
    target = BuildFinishedSubmapTarget(map);
  }
  if (!target) {
    LOG(WARNING) << "[PclSubmapRelocalizer] global target unavailable"
                 << " keyframes=" << map.keyframes.size();
    return std::nullopt;
  }
  std::vector<RelocalizationCandidate> coarse_candidates;
  if (operations_.retrieve) {
    coarse_candidates = operations_.retrieve(query, *target, stop_token);
  } else {
    PclKissCandidateRetriever retriever(config_.kiss);
    coarse_candidates = retriever.Retrieve(query, *target, stop_token);
  }
  if (query.prior_T_map_base) {
    // Recovery has a short-horizon map->odom prior from the last healthy
    // alignment. KISS remains the first global proposal; this prior is only a
    // second proposal when KISS produces no quality-gated candidate.
    coarse_candidates.push_back({*query.prior_T_map_base, -1.0});
  } else if (const auto snapshot_prior = SnapshotMapPrior(
                 query, map, config_.map_odom_prior_max_age_seconds)) {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot map-odom prior available"
              << " max_age_s=" << config_.map_odom_prior_max_age_seconds;
    coarse_candidates.push_back({*snapshot_prior, -1.0});
  } else {
    LOG(INFO) << "[PclSubmapRelocalizer] snapshot map-odom prior unavailable"
              << " keyframes=" << map.keyframes.size()
              << " max_age_s=" << config_.map_odom_prior_max_age_seconds;
  }
  std::shared_ptr<PclPointCloud> target_cloud;
  if (!operations_.verify && !coarse_candidates.empty()) {
    target_cloud = std::make_shared<PclPointCloud>();
    const auto& target_view = target->cloudMap().view();
    target_cloud->points.reserve(target_view.size());
    for (std::size_t index = 0; index < target_view.size(); ++index) {
      std::array<float, 3> point{};
      if (!target_view.xyz(index, point)) {
        return std::nullopt;
      }
      target_cloud->points.push_back({point[0], point[1], point[2], 0.0F});
    }
    if (target_cloud->empty()) {
      return std::nullopt;
    }
    target_cloud->width = static_cast<std::uint32_t>(target_cloud->size());
    target_cloud->height = 1;
    target_cloud->is_dense = false;
  }
  std::optional<SubmapRelocalizationResult> best_result;
  for (const auto& coarse : coarse_candidates) {
    if (stop_token.stop_requested()) {
      return std::nullopt;
    }
    if (!ValidCandidate(coarse, query)) {
      LOG(WARNING) << "[PclSubmapRelocalizer] global coarse candidate rejected"
                   << " reason=invalid_coarse_pose";
      continue;
    }
    const auto& kiss_pose = coarse.initial_T_map_base.T_target_source;
    LOG(INFO) << "[PclSubmapRelocalizer] "
              << (coarse.retrieval_score < 0.0 ? "prior fallback pose"
                                               : "global KISS coarse pose")
              << " target_points=" << target->cloudMap().size()
              << " timestamp_ns="
              << coarse.initial_T_map_base.timestamp.nanoseconds()
              << " retrieval_score=" << coarse.retrieval_score
              << " tx=" << kiss_pose.translation.x()
              << " ty=" << kiss_pose.translation.y()
              << " tz=" << kiss_pose.translation.z()
              << " qx=" << kiss_pose.rotation.x()
              << " qy=" << kiss_pose.rotation.y()
              << " qz=" << kiss_pose.rotation.z()
              << " qw=" << kiss_pose.rotation.w();

    std::optional<RegistrationVerification> verification;
    if (operations_.verify) {
      verification = operations_.verify(query, *target, coarse, stop_token);
    } else {
      PclRegistrationVerifier verifier(
          config_.registration,
          [&map, target_cloud, radius = config_.registration.crop_radius_m,
           neighbor_submaps = config_.neighbor_submaps](
              const BASIC::SE3& initial_T_map_base) {
            const hybrid_slam::geometry::RigidTransform3d initial_pose{
                Eigen::Quaterniond(initial_T_map_base.R_.cast<double>()),
                initial_T_map_base.t_.cast<double>()};
            auto neighbor_target = BuildNeighborSubmapTarget(
                map, initial_pose, neighbor_submaps);
            if (neighbor_target && !neighbor_target->empty()) {
              LOG(INFO) << "[PclSubmapRelocalizer] verifying against "
                        << "neighbor submaps=" << neighbor_submaps
                        << " target_points=" << neighbor_target->size();
              return CropTargetAroundCandidate(
                  neighbor_target, initial_T_map_base, radius);
            }
            LOG(WARNING) << "[PclSubmapRelocalizer] neighbor target unavailable; "
                         << "falling back to global target";
            return CropTargetAroundCandidate(target_cloud, initial_T_map_base,
                                             radius);
          });
      verification = verifier.Verify(query, *target, coarse, stop_token);
    }
    if (!verification) {
      LOG(WARNING) << "[PclSubmapRelocalizer] global candidate rejected"
                   << " retrieval_score=" << coarse.retrieval_score
                   << " reason=registration_failed";
      continue;
    }
    if (stop_token.stop_requested()) {
      return std::nullopt;
    }
    if (verification->T_map_base.timestamp != query.T_odom_base.timestamp) {
      LOG(WARNING) << "[PclSubmapRelocalizer] global candidate rejected"
                   << " reason=timestamp_mismatch";
      continue;
    }
    if (!MeetsQuality(*verification, config_)) {
      LOG(WARNING) << "[PclSubmapRelocalizer] global candidate rejected"
                   << " retrieval_score=" << coarse.retrieval_score
                   << " fitness=" << verification->fitness
                   << " inlier_ratio=" << verification->inlier_ratio
                   << " overlap=" << verification->overlap
                   << " observable=" << verification->observable
                   << " reason="
                   << QualityRejectionReason(*verification, config_);
      continue;
    }
    const auto* seed_submap = FindSeedSubmap(map, verification->T_map_base);
    if (seed_submap == nullptr) {
      LOG(WARNING) << "[PclSubmapRelocalizer] global candidate rejected"
                   << " reason=no_seed_submap";
      continue;
    }

    SubmapRelocalizationResult result;
    result.T_map_base = verification->T_map_base;
    result.session_id = seed_submap->session_id;
    result.trajectory_id = seed_submap->trajectory_id;
    result.submap_id = seed_submap->id;
    result.fitness = verification->fitness;
    result.inlier_ratio = verification->inlier_ratio;
    result.overlap = verification->overlap;
    result.observable = verification->observable;
    if (!result.valid()) {
      continue;
    }
    if (!best_result || result.fitness < best_result->fitness ||
        (result.fitness == best_result->fitness &&
         result.inlier_ratio > best_result->inlier_ratio) ||
        (result.fitness == best_result->fitness &&
         result.inlier_ratio == best_result->inlier_ratio &&
         result.overlap > best_result->overlap)) {
      best_result = std::move(result);
    }
  }
  if (best_result) {
    LOG(INFO) << "[PclSubmapRelocalizer] selected best verified candidate"
              << " fitness=" << best_result->fitness
              << " inlier_ratio=" << best_result->inlier_ratio
              << " overlap=" << best_result->overlap;
  }
  return best_result;
}

}  // namespace LI2Sup
