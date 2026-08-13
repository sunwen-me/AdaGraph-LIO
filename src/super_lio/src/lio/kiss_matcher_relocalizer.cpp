#include "lio/kiss_matcher_relocalizer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>
#include <vector>

#include "basic/logs.h"
#ifdef HAS_KISS_MATCHER
#include <kiss_matcher/KISSMatcher.hpp>
#endif

namespace LI2Sup {

namespace {

std::vector<Eigen::Vector3f> toEigenVector(
    const BASIC::PointCloudType::ConstPtr& cloud,
    const int max_points) {
  std::vector<Eigen::Vector3f> points;
  if (!cloud || cloud->empty()) {
    return points;
  }

  const std::size_t limit =
      max_points > 0 ? static_cast<std::size_t>(max_points) : cloud->size();
  const std::size_t stride =
      cloud->size() > limit ? static_cast<std::size_t>(
          std::ceil(static_cast<double>(cloud->size()) / limit)) : 1;

  points.reserve(std::min(cloud->size(), limit));
  for (std::size_t i = 0; i < cloud->size(); i += stride) {
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

}  // namespace

bool KISSMatcherRelocalizer::estimate(
    const BASIC::PointCloudType::ConstPtr& source,
    const BASIC::PointCloudType::ConstPtr& target,
    BASIC::SE3& target_source) const {
  auto candidates = estimateCandidates(source, target);
  if (candidates.empty()) {
    return false;
  }

  const auto best = std::max_element(
      candidates.begin(), candidates.end(),
      [](const KISSMatcherCandidate& lhs, const KISSMatcherCandidate& rhs) {
        return lhs.final_inliers < rhs.final_inliers;
      });
  target_source = best->target_source;
  return true;
}

std::vector<KISSMatcherCandidate> KISSMatcherRelocalizer::estimateCandidates(
    const BASIC::PointCloudType::ConstPtr& source,
    const BASIC::PointCloudType::ConstPtr& target) const {
  std::vector<KISSMatcherCandidate> candidates;
  if (!config_.enabled) {
    return candidates;
  }

#ifndef HAS_KISS_MATCHER
  static bool warned = false;
  if (!warned) {
    warned = true;
    LOG(WARNING) << YELLOW
                 << "KISS-Matcher is enabled but kiss_matcher was not found at build time."
                 << RESET;
  }
  return candidates;
#else
  const auto src_vec = toEigenVector(source, config_.max_source_points);
  const auto tgt_vec = toEigenVector(target, config_.max_target_points);

  if (src_vec.size() < 30 || tgt_vec.size() < 30) {
    LOG(WARNING) << YELLOW
                 << "KISS-Matcher skipped: too few points. source="
                 << src_vec.size() << " target=" << tgt_vec.size()
                 << RESET;
    return candidates;
  }

  std::vector<bool> ratio_modes{config_.use_ratio_test};
  if (config_.adaptive_ratio_test) {
    ratio_modes.push_back(!config_.use_ratio_test);
  }

  for (const bool use_ratio_test : ratio_modes) {
    try {
      kiss_matcher::KISSMatcherConfig config(
          config_.voxel_size,
          config_.use_voxel_sampling,
          config_.use_quatro);
      config.use_ratio_test_ = use_ratio_test;
      config.deterministic_voxel_sampling_ =
          config_.deterministic_voxel_sampling;
      kiss_matcher::KISSMatcher matcher(config);

      const auto solution = matcher.estimate(src_vec, tgt_vec);
      const auto processed_sizes = matcher.getProcessedInputSizes();
      const auto keypoint_sizes = matcher.getFasterPFHKeypointSizes();
      const auto initial_matches = matcher.getNumInitialMatches();
      const auto rotation_inliers = matcher.getNumRotationInliers();
      const auto final_inliers = matcher.getNumFinalInliers();
      LOG(INFO) << "KISS-Matcher diagnostics [ratio="
                << (use_ratio_test ? "on" : "off")
                << "]: input=" << src_vec.size()
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
      if (!solution.valid ||
          final_inliers <
              static_cast<std::size_t>(config_.min_final_inliers)) {
        LOG(WARNING) << YELLOW
                     << "KISS-Matcher candidate rejected [ratio="
                     << (use_ratio_test ? "on" : "off")
                     << "]. valid=" << solution.valid
                     << " final_inliers=" << final_inliers
                     << RESET;
        continue;
      }

      KISSMatcherCandidate candidate;
      const BASIC::M3 target_source_R =
          solution.rotation.cast<BASIC::scalar>();
      const BASIC::V3 target_source_t =
          solution.translation.cast<BASIC::scalar>();
      candidate.target_source = BASIC::SE3(target_source_R, target_source_t);
      candidate.used_ratio_test = use_ratio_test;
      candidate.initial_matches = initial_matches;
      candidate.final_inliers = final_inliers;
      candidates.push_back(std::move(candidate));
    } catch (const std::exception& e) {
      LOG(WARNING) << YELLOW
                   << "KISS-Matcher exception [ratio="
                   << (use_ratio_test ? "on" : "off")
                   << "]: " << e.what()
                   << RESET;
    }
  }

  return candidates;
#endif
}

}  // namespace LI2Sup
