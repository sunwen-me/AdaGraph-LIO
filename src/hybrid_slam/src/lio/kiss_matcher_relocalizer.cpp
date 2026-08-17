#include "lio/kiss_matcher_relocalizer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include "hybrid_slam/logging/logging.h"
#ifdef HAS_KISS_MATCHER
#include <kiss_matcher/KISSMatcher.hpp>
#endif

namespace LI2Sup {

namespace {

std::vector<Eigen::Vector3f> toEigenVector(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& cloud) {
  std::vector<Eigen::Vector3f> points;
  if (!cloud || cloud->empty()) {
    return points;
  }

  points.reserve(cloud->size());
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    const auto& point = cloud->points[i];
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    points.emplace_back(point.x, point.y, point.z);
  }
  return points;
}

std::vector<Eigen::Vector3f> LimitPoints(
    std::vector<Eigen::Vector3f> points, const int max_points) {
  if (max_points <= 0 || points.size() <= static_cast<std::size_t>(max_points)) {
    return points;
  }
  const std::size_t limit = static_cast<std::size_t>(max_points);
  const std::size_t stride = static_cast<std::size_t>(std::ceil(
      static_cast<double>(points.size()) / static_cast<double>(limit)));
  std::vector<Eigen::Vector3f> limited;
  limited.reserve(limit);
  for (std::size_t index = 0; index < points.size(); index += stride) {
    limited.push_back(points[index]);
  }
  return limited;
}

}  // namespace

bool KISSMatcherRelocalizer::estimate(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& source,
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& target,
    BASIC::SE3& target_source) const {
  auto candidates = estimateCandidates(source, target);
  if (candidates.empty()) {
    return false;
  }

  const KISSMatcherCandidate* best = nullptr;
  for (const auto& candidate : candidates) {
    if (!candidate.passes_quality_gates ||
        (best != nullptr && best->final_inliers >= candidate.final_inliers)) {
      continue;
    }
    best = &candidate;
  }
  if (best == nullptr) {
    return false;
  }
  target_source = best->target_source;
  return true;
}

std::vector<KISSMatcherCandidate> KISSMatcherRelocalizer::estimateCandidates(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& source,
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& target,
    MemoryCallback memory_callback) const {
  std::vector<KISSMatcherCandidate> candidates;
  if (!config_.enabled) {
    return candidates;
  }

#ifndef HAS_KISS_MATCHER
  static bool warned = false;
  if (!warned) {
    warned = true;
    LOG(WARNING) << hybrid_slam::logging::kYellow
                 << "KISS-Matcher is enabled but kiss_matcher was not found at build time."
                 << hybrid_slam::logging::kReset;
  }
  return candidates;
#else
  const auto source_raw = toEigenVector(source);
  const auto target_raw = toEigenVector(target);
  std::vector<bool> ratio_modes{config_.use_ratio_test};
  if (config_.adaptive_ratio_test) {
    ratio_modes.push_back(!config_.use_ratio_test);
  }

  std::vector<float> proposal_voxel_sizes{config_.voxel_size};
  if (config_.multi_scale && std::isfinite(config_.coarse_voxel_scale) &&
      config_.coarse_voxel_scale > 1.0F) {
    proposal_voxel_sizes.push_back(config_.voxel_size *
                                   config_.coarse_voxel_scale);
  }
  std::sort(proposal_voxel_sizes.begin(), proposal_voxel_sizes.end());
  proposal_voxel_sizes.erase(
      std::unique(proposal_voxel_sizes.begin(), proposal_voxel_sizes.end()),
      proposal_voxel_sizes.end());

  for (const float proposal_voxel_size : proposal_voxel_sizes) {
    auto src_vec = source_raw;
    auto tgt_vec = target_raw;
    if (config_.use_voxel_sampling) {
      // Apply the same metric voxel grid to the query and target before
      // enforcing the bounded KISS work budget.
      src_vec = kiss_matcher::VoxelgridSamplingDeterministic(
          source_raw, proposal_voxel_size);
      tgt_vec = kiss_matcher::VoxelgridSamplingDeterministic(
          target_raw, proposal_voxel_size);
    }
    src_vec = LimitPoints(std::move(src_vec), config_.max_source_points);
    tgt_vec = LimitPoints(std::move(tgt_vec), config_.max_target_points);
    if (src_vec.size() < 30 || tgt_vec.size() < 30) {
      LOG(WARNING) << hybrid_slam::logging::kYellow
                   << "KISS-Matcher skipped: too few points. source="
                   << src_vec.size() << " target=" << tgt_vec.size()
                   << hybrid_slam::logging::kReset;
      continue;
    }

    for (const bool use_ratio_test : ratio_modes) {
      try {
        kiss_matcher::KISSMatcherConfig config(
            proposal_voxel_size,
            false,
            config_.use_quatro);
        config.use_ratio_test_ = use_ratio_test;
        config.deterministic_voxel_sampling_ =
            config_.deterministic_voxel_sampling;
        kiss_matcher::KISSMatcher matcher(config);

      const auto solution = matcher.estimate(src_vec, tgt_vec);
      if (memory_callback) {
        // Capture while matcher/FasterPFH/ROBIN state is still alive.
        memory_callback(CaptureRuntimeMemory());
      }
      const auto processed_sizes = matcher.getProcessedInputSizes();
      const auto keypoint_sizes = matcher.getFasterPFHKeypointSizes();
      const auto initial_matches = matcher.getNumInitialMatches();
      const auto rotation_inliers = matcher.getNumRotationInliers();
      const auto final_inliers = matcher.getNumFinalInliers();
        LOG(INFO) << "KISS-Matcher diagnostics [ratio="
                << (use_ratio_test ? "on" : "off")
                << " voxel_size=" << proposal_voxel_size
                << "]: raw=" << source_raw.size()
                << "/" << target_raw.size()
                << " prevoxel="
                << (config_.use_voxel_sampling ? "on" : "off")
                << " input=" << src_vec.size()
                << "/" << tgt_vec.size()
                << " deterministic_voxel="
                << (config_.deterministic_voxel_sampling ? "on" : "off")
                << " processed=" << processed_sizes.first
                << "/" << processed_sizes.second
                << " keypoints=" << keypoint_sizes.first
                << "/" << keypoint_sizes.second
                << " matches=" << initial_matches
                << " rotation_inliers=" << rotation_inliers
                << " final_inliers=" << final_inliers;
        const bool passes_quality_gates =
          solution.valid &&
          initial_matches >=
              static_cast<std::size_t>(config_.min_initial_matches) &&
          rotation_inliers >=
              static_cast<std::size_t>(config_.min_rotation_inliers) &&
          final_inliers >=
              static_cast<std::size_t>(config_.min_final_inliers);
        if (!passes_quality_gates) {
        LOG(WARNING) << hybrid_slam::logging::kYellow
                     << "KISS-Matcher candidate rejected [ratio="
                     << (use_ratio_test ? "on" : "off")
                     << "]. valid=" << solution.valid
                     << " final_inliers=" << final_inliers
                     << hybrid_slam::logging::kReset;
          if (!solution.valid) {
            continue;
          }
        }

        KISSMatcherCandidate candidate;
        const BASIC::M3 target_source_R =
            solution.rotation.cast<BASIC::scalar>();
        const BASIC::V3 target_source_t =
            solution.translation.cast<BASIC::scalar>();
        candidate.target_source = BASIC::SE3(target_source_R, target_source_t);
        candidate.used_ratio_test = use_ratio_test;
        candidate.passes_quality_gates = passes_quality_gates;
        candidate.initial_matches = initial_matches;
        candidate.rotation_inliers = rotation_inliers;
        candidate.final_inliers = final_inliers;
        candidates.push_back(std::move(candidate));
      } catch (const std::exception& e) {
        LOG(WARNING) << hybrid_slam::logging::kYellow
                     << "KISS-Matcher exception [ratio="
                     << (use_ratio_test ? "on" : "off")
                     << "]: " << e.what()
                     << hybrid_slam::logging::kReset;
      }
    }
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const KISSMatcherCandidate& lhs,
               const KISSMatcherCandidate& rhs) {
              if (lhs.passes_quality_gates != rhs.passes_quality_gates) {
                return lhs.passes_quality_gates > rhs.passes_quality_gates;
              }
              if (lhs.final_inliers != rhs.final_inliers) {
                return lhs.final_inliers > rhs.final_inliers;
              }
              if (lhs.rotation_inliers != rhs.rotation_inliers) {
                return lhs.rotation_inliers > rhs.rotation_inliers;
              }
              return lhs.initial_matches > rhs.initial_matches;
            });
  if (config_.max_candidates > 0 &&
      candidates.size() > static_cast<std::size_t>(config_.max_candidates)) {
    candidates.resize(static_cast<std::size_t>(config_.max_candidates));
  }

  return candidates;
#endif
}

}  // namespace LI2Sup
