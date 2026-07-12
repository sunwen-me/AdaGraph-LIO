#include "lio/kiss_matcher_relocalizer.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <vector>

#include "basic/logs.h"
#include "lio/params.h"

#ifdef HAS_KISS_MATCHER
#include <kiss_matcher/KISSMatcher.hpp>
#endif

namespace LI2Sup {

namespace {

std::vector<Eigen::Vector3f> toEigenVector(
    const BASIC::CloudPtr& cloud,
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
    const BASIC::CloudPtr& source,
    const BASIC::CloudPtr& target,
    BASIC::SE3& target_source) const {
  if (!g_kiss_matcher_enable) {
    return false;
  }

#ifndef HAS_KISS_MATCHER
  static bool warned = false;
  if (!warned) {
    warned = true;
    LOG(WARNING) << YELLOW
                 << "KISS-Matcher is enabled but kiss_matcher was not found at build time."
                 << RESET;
  }
  return false;
#else
  const auto src_vec = toEigenVector(source, g_kiss_matcher_max_source_points);
  const auto tgt_vec = toEigenVector(target, g_kiss_matcher_max_target_points);

  if (src_vec.size() < 30 || tgt_vec.size() < 30) {
    LOG(WARNING) << YELLOW
                 << "KISS-Matcher skipped: too few points. source="
                 << src_vec.size() << " target=" << tgt_vec.size()
                 << RESET;
    return false;
  }

  try {
    kiss_matcher::KISSMatcherConfig config(
        g_kiss_matcher_voxel_size,
        g_kiss_matcher_use_voxel_sampling,
        g_kiss_matcher_use_quatro);
    kiss_matcher::KISSMatcher matcher(config);

    const auto solution = matcher.estimate(src_vec, tgt_vec);
    const auto final_inliers = matcher.getNumFinalInliers();
    if (!solution.valid ||
        final_inliers < static_cast<std::size_t>(g_kiss_matcher_min_final_inliers)) {
      LOG(WARNING) << YELLOW
                   << "KISS-Matcher failed. valid=" << solution.valid
                   << " final_inliers=" << final_inliers
                   << RESET;
      return false;
    }

    const BASIC::M3 target_source_R = solution.rotation.cast<BASIC::scalar>();
    const BASIC::V3 target_source_t = solution.translation.cast<BASIC::scalar>();
    target_source = BASIC::SE3(target_source_R, target_source_t);

    LOG(INFO) << GREEN
              << "KISS-Matcher succeeded. final_inliers=" << final_inliers
              << RESET;
    return true;
  } catch (const std::exception& e) {
    LOG(WARNING) << YELLOW
                 << "KISS-Matcher exception: " << e.what()
                 << RESET;
    return false;
  }
#endif
}

}  // namespace LI2Sup
