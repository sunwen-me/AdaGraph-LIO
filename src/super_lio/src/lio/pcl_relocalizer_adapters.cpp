#include "lio/pcl_relocalizer_adapters.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/registration/gicp.h>

#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

#include "basic/logs.h"

namespace LI2Sup {
namespace {

adagraph::geometry::RigidTransform3d ToDomainPose(const BASIC::SE3& pose);

Eigen::Matrix4f ToMatrix4f(
    const adagraph::geometry::RigidTransform3d& pose) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) =
      pose.rotation.toRotationMatrix().cast<float>();
  matrix.block<3, 1>(0, 3) = pose.translation.cast<float>();
  return matrix;
}

BASIC::PointCloudType::ConstPtr ToOdomCloud(
    const BASIC::PointCloudType::ConstPtr& source_base,
    const adagraph::geometry::StampedTransform3d& T_odom_base) {
  if (!source_base || source_base->empty() || !T_odom_base.valid() ||
      T_odom_base.target_frame != adagraph::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != adagraph::geometry::FrameId::kBase) {
    return {};
  }

  auto source_odom = std::make_shared<BASIC::PointCloudType>();
  pcl::transformPointCloud(
      *source_base, *source_odom,
      ToMatrix4f(T_odom_base.T_target_source));
  source_odom->width = static_cast<std::uint32_t>(source_odom->size());
  source_odom->height = 1;
  source_odom->is_dense = false;
  return source_odom;
}

std::size_t CloudBackingBytes(
    const BASIC::PointCloudType::ConstPtr& cloud) noexcept {
  return cloud ? cloud->points.capacity() * sizeof(BASIC::PointType) : 0;
}

std::size_t CloudBackingBytes(const BASIC::PointCloudType& cloud) noexcept {
  return cloud.points.capacity() * sizeof(BASIC::PointType);
}

std::optional<adagraph::geometry::StampedTransform3d> ToMapBase(
    const adagraph::geometry::RigidTransform3d& T_map_odom,
    const adagraph::localization::RelocalizationQuery& query) {
  const adagraph::geometry::StampedTransform3d T_map_odom_domain{
      query.T_odom_base.timestamp,
      adagraph::geometry::FrameId::kMap,
      adagraph::geometry::FrameId::kOdom,
      T_map_odom,
  };
  return T_map_odom_domain.compose(query.T_odom_base);
}

std::optional<adagraph::geometry::StampedTransform3d> ToMapOdom(
    const adagraph::geometry::StampedTransform3d& T_map_base,
    const adagraph::localization::RelocalizationQuery& query) {
  return T_map_base.compose(query.T_odom_base.inverse());
}

adagraph::geometry::RigidTransform3d FromMatrix4f(
    const Eigen::Matrix4f& matrix) {
  return {
      Eigen::Quaterniond(matrix.block<3, 3>(0, 0).cast<double>()).normalized(),
      matrix.block<3, 1>(0, 3).cast<double>(),
  };
}

BASIC::SE3 ToLegacyPose(
    const adagraph::geometry::RigidTransform3d& pose) {
  return BASIC::SE3(
      BASIC::SO3(
          pose.rotation.toRotationMatrix().cast<BASIC::scalar>()),
      pose.translation.cast<BASIC::scalar>());
}

adagraph::geometry::RigidTransform3d ToDomainPose(const BASIC::SE3& pose) {
  return {
      Eigen::Quaterniond(pose.R_.cast<double>()).normalized(),
      pose.t_.cast<double>(),
  };
}

}  // namespace

std::vector<adagraph::localization::RelocalizationCandidate>
PclKissCandidateRetriever::Retrieve(
    const adagraph::localization::RelocalizationQuery& query,
    const adagraph::localization::LocalizationMap& map,
    std::stop_token stop_token) {
  std::vector<adagraph::localization::RelocalizationCandidate> result;
  if (stop_token.stop_requested() || !query.valid() || !map.valid()) {
    return result;
  }
  const auto source =
      query.current_scan_base.ownerAs<BASIC::PointCloudType>();
  const auto target = map.cloudMap().ownerAs<BASIC::PointCloudType>();
  if (!source || !target) {
    return result;
  }
  // The legacy KISS/VGICP implementation consumes the accumulated local
  // submap in odom coordinates.  The typed query intentionally exposes that
  // same immutable payload in base coordinates, so convert only at this
  // concrete adapter boundary.  Keeping this conversion here preserves the
  // old voxel sampling/registration frame semantics without leaking PCL or
  // an odom-frame cloud into the ROS-independent contract.
  const auto source_odom = ToOdomCloud(source, query.T_odom_base);
  if (!source_odom) {
    return result;
  }
  if (memory_callback_) {
    memory_callback_({PclRelocalizationTemporaryMemory::Stage::kKiss,
                      CloudBackingBytes(source_odom), 0,
                      CaptureRuntimeMemory()});
  }
  const auto candidates = matcher_.estimateCandidates(
      source_odom, target,
      [this, source_bytes = CloudBackingBytes(source_odom)](
          const RuntimeMemorySample runtime) {
        if (memory_callback_) {
          memory_callback_({PclRelocalizationTemporaryMemory::Stage::kKiss,
                            source_bytes, 0, runtime});
        }
      });
  if (candidates.empty()) {
    LOG(WARNING) << "[PclKissCandidateRetriever]: no KISS candidates"
                 << " source_points=" << source_odom->size()
                 << " target_points=" << target->size();
  }
  result.reserve(candidates.size());
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    if (stop_token.stop_requested()) {
      result.clear();
      return result;
    }
    const auto T_map_base =
        ToMapBase(ToDomainPose(candidate.target_source), query);
    if (!T_map_base) {
      continue;
    }
    LOG(INFO) << "[PclKissCandidateRetriever]: candidate=" << index
              << " ratio=" << (candidate.used_ratio_test ? "on" : "off")
              << " initial_matches=" << candidate.initial_matches
              << " final_inliers=" << candidate.final_inliers;
    result.push_back({*T_map_base, static_cast<double>(candidate.final_inliers)});
  }
  return result;
}

PclRegistrationVerifier::PclRegistrationVerifier(
    adagraph::localization::RelocalizationConfig config,
    TargetSelector target_selector,
    PclRelocalizationMemoryCallback memory_callback)
    : config_(std::move(config)),
      target_selector_(std::move(target_selector)),
      memory_callback_(std::move(memory_callback)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid PCL verifier config: " + *error);
  }
  if (!target_selector_) {
    throw std::invalid_argument("PCL verifier requires target selector");
  }
}

std::optional<adagraph::localization::RegistrationVerification>
PclRegistrationVerifier::Verify(
    const adagraph::localization::RelocalizationQuery& query,
    const adagraph::localization::LocalizationMap& map,
    const adagraph::localization::RelocalizationCandidate& candidate,
    std::stop_token stop_token) {
  if (stop_token.stop_requested() || !query.valid() || !map.valid() ||
      !candidate.initial_T_map_base.valid() ||
      candidate.initial_T_map_base.timestamp != query.T_odom_base.timestamp ||
      candidate.initial_T_map_base.target_frame !=
          adagraph::geometry::FrameId::kMap ||
      candidate.initial_T_map_base.source_frame !=
          adagraph::geometry::FrameId::kBase) {
    return std::nullopt;
  }
  const auto source_base =
      query.current_scan_base.ownerAs<BASIC::PointCloudType>();
  const auto target = target_selector_(
      ToLegacyPose(candidate.initial_T_map_base.T_target_source));
  if (!source_base || source_base->empty() || !target || target->empty() ||
      stop_token.stop_requested()) {
    LOG(WARNING) << "[PclRegistrationVerifier]: skipped candidate"
                 << " retrieval_score=" << candidate.retrieval_score
                 << " source_points=" << (source_base ? source_base->size() : 0)
                 << " target_points=" << (target ? target->size() : 0)
                 << " cancelled=" << stop_token.stop_requested();
    return std::nullopt;
  }

  const auto source_odom = ToOdomCloud(source_base, query.T_odom_base);
  const auto T_map_odom = ToMapOdom(candidate.initial_T_map_base, query);
  if (!source_odom || !T_map_odom) {
    return std::nullopt;
  }

  BASIC::PointCloudType aligned;
  Eigen::Matrix4f final_transformation = Eigen::Matrix4f::Identity();
  bool converged = false;
  double fitness = std::numeric_limits<double>::infinity();

#ifdef HAS_FAST_GICP
  fast_gicp::FastVGICP<BASIC::PointType, BASIC::PointType> vgicp;
  vgicp.setInputSource(source_odom);
  vgicp.setInputTarget(target);
  vgicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  vgicp.setTransformationEpsilon(1e-4);
  vgicp.setEuclideanFitnessEpsilon(1e-4);
  vgicp.setResolution(static_cast<float>(config_.source_voxel_size_m));
  vgicp.setNumThreads(static_cast<int>(config_.registration_threads));
  vgicp.align(aligned, ToMatrix4f(
      T_map_odom->T_target_source));
  converged = vgicp.hasConverged();
  fitness = vgicp.getFitnessScore();
  final_transformation = vgicp.getFinalTransformation();
#else
  pcl::GeneralizedIterativeClosestPoint<BASIC::PointType, BASIC::PointType>
      gicp;
  gicp.setInputSource(source_odom);
  gicp.setInputTarget(target);
  gicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  gicp.setMaxCorrespondenceDistance(
      std::max(1.0, config_.source_voxel_size_m * 8.0));
  gicp.setTransformationEpsilon(1e-4);
  gicp.setEuclideanFitnessEpsilon(1e-4);
  gicp.align(aligned, ToMatrix4f(
      T_map_odom->T_target_source));
  converged = gicp.hasConverged();
  fitness = gicp.getFitnessScore();
  final_transformation = gicp.getFinalTransformation();
#endif

  if (memory_callback_) {
    memory_callback_(
        {PclRelocalizationTemporaryMemory::Stage::kRegistration,
         CloudBackingBytes(source_odom), CloudBackingBytes(aligned),
         CaptureRuntimeMemory()});
  }

  if (stop_token.stop_requested() || !converged || !std::isfinite(fitness) ||
      fitness < 0.0) {
    LOG(WARNING) << "[PclRegistrationVerifier]: rejected candidate"
                 << " retrieval_score=" << candidate.retrieval_score
                 << " converged=" << converged
                 << " fitness=" << fitness
                 << " source_points=" << source_odom->size()
                 << " target_points=" << target->size()
                 << " cancelled=" << stop_token.stop_requested();
    return std::nullopt;
  }
  const auto T_map_base = ToMapBase(FromMatrix4f(final_transformation), query);
  if (!T_map_base) {
    return std::nullopt;
  }
  adagraph::localization::RegistrationVerification verification{
      *T_map_base,
      fitness,
      0.0,
      adagraph::localization::Confidence::kUnknown,
  };
  LOG(INFO) << "[PclRegistrationVerifier]: accepted candidate"
            << " retrieval_score=" << candidate.retrieval_score
            << " fitness=" << fitness
            << " source_points=" << source_odom->size()
            << " target_points=" << target->size();
  return verification;
}

}  // namespace LI2Sup
