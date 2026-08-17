#include "lio/pcl_relocalizer_adapters.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

#include "hybrid_slam/logging/logging.h"

namespace LI2Sup {
namespace {

hybrid_slam::geometry::RigidTransform3d ToDomainPose(const BASIC::SE3& pose);

Eigen::Matrix4f ToMatrix4f(
    const hybrid_slam::geometry::RigidTransform3d& pose) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) =
      pose.rotation.toRotationMatrix().cast<float>();
  matrix.block<3, 1>(0, 3) = pose.translation.cast<float>();
  return matrix;
}

hybrid_slam::adapter::PclPointCloud::ConstPtr ToOdomCloud(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& source_base,
    const hybrid_slam::geometry::StampedTransform3d& T_odom_base) {
  if (!source_base || source_base->empty() || !T_odom_base.valid() ||
      T_odom_base.target_frame != hybrid_slam::geometry::FrameId::kOdom ||
      T_odom_base.source_frame != hybrid_slam::geometry::FrameId::kBase) {
    return {};
  }

  auto source_odom = std::make_shared<hybrid_slam::adapter::PclPointCloud>();
  pcl::transformPointCloud(
      *source_base, *source_odom,
      ToMatrix4f(T_odom_base.T_target_source));
  source_odom->width = static_cast<std::uint32_t>(source_odom->size());
  source_odom->height = 1;
  source_odom->is_dense = false;
  return source_odom;
}

hybrid_slam::adapter::PclPointCloud::ConstPtr ToPclCloud(
    const hybrid_slam::sensor::ImmutablePointCloud& cloud) {
  if (!cloud.valid() || cloud.empty()) {
    return {};
  }
  auto pcl_cloud = std::make_shared<hybrid_slam::adapter::PclPointCloud>();
  const auto& view = cloud.view();
  pcl_cloud->points.reserve(view.size());
  for (std::size_t index = 0; index < view.size(); ++index) {
    std::array<float, 3> point{};
    if (!view.xyz(index, point)) {
      return {};
    }
    pcl_cloud->points.push_back({point[0], point[1], point[2], 0.0F});
  }
  if (pcl_cloud->empty()) {
    return {};
  }
  pcl_cloud->width = static_cast<std::uint32_t>(pcl_cloud->size());
  pcl_cloud->height = 1;
  pcl_cloud->is_dense = false;
  return pcl_cloud;
}

std::size_t CloudBackingBytes(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& cloud) noexcept {
  return cloud ? cloud->points.capacity() * sizeof(hybrid_slam::adapter::PclPoint) : 0;
}

std::size_t CloudBackingBytes(const hybrid_slam::adapter::PclPointCloud& cloud) noexcept {
  return cloud.points.capacity() * sizeof(hybrid_slam::adapter::PclPoint);
}

hybrid_slam::adapter::PclPointCloud::ConstPtr PrepareRegistrationCloud(
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& cloud,
    const double voxel_size, const std::size_t max_points) {
  if (!cloud || cloud->empty() || !std::isfinite(voxel_size) ||
      voxel_size <= 0.0 || max_points == 0) {
    return {};
  }

  auto voxelized = std::make_shared<hybrid_slam::adapter::PclPointCloud>();
  pcl::VoxelGrid<hybrid_slam::adapter::PclPoint> voxel_filter;
  voxel_filter.setLeafSize(static_cast<float>(voxel_size),
                           static_cast<float>(voxel_size),
                           static_cast<float>(voxel_size));
  voxel_filter.setInputCloud(cloud);
  voxel_filter.filter(*voxelized);
  if (voxelized->empty()) {
    return {};
  }
  voxelized->width = static_cast<std::uint32_t>(voxelized->size());
  voxelized->height = 1;
  voxelized->is_dense = false;

  if (voxelized->size() <= max_points) {
    return voxelized;
  }

  auto limited = std::make_shared<hybrid_slam::adapter::PclPointCloud>();
  limited->points.reserve(max_points);
  const double step = static_cast<double>(voxelized->size()) /
                      static_cast<double>(max_points);
  for (std::size_t index = 0; index < max_points; ++index) {
    const auto source_index = std::min<std::size_t>(
        static_cast<std::size_t>(std::floor(index * step)),
        voxelized->size() - 1);
    limited->points.push_back(voxelized->points[source_index]);
  }
  limited->width = static_cast<std::uint32_t>(limited->size());
  limited->height = 1;
  limited->is_dense = false;
  return limited;
}

std::optional<hybrid_slam::geometry::StampedTransform3d> ToMapBase(
    const hybrid_slam::geometry::RigidTransform3d& T_map_odom,
    const hybrid_slam::localization::RelocalizationQuery& query) {
  const hybrid_slam::geometry::StampedTransform3d T_map_odom_domain{
      query.T_odom_base.timestamp,
      hybrid_slam::geometry::FrameId::kMap,
      hybrid_slam::geometry::FrameId::kOdom,
      T_map_odom,
  };
  return T_map_odom_domain.compose(query.T_odom_base);
}

std::optional<hybrid_slam::geometry::StampedTransform3d> ToMapOdom(
    const hybrid_slam::geometry::StampedTransform3d& T_map_base,
    const hybrid_slam::localization::RelocalizationQuery& query) {
  return T_map_base.compose(query.T_odom_base.inverse());
}

hybrid_slam::geometry::RigidTransform3d FromMatrix4f(
    const Eigen::Matrix4f& matrix) {
  return {
      Eigen::Quaterniond(matrix.block<3, 3>(0, 0).cast<double>()).normalized(),
      matrix.block<3, 1>(0, 3).cast<double>(),
  };
}

BASIC::SE3 ToBasicPose(
    const hybrid_slam::geometry::RigidTransform3d& pose) {
  return BASIC::SE3(
      BASIC::SO3(
          pose.rotation.toRotationMatrix().cast<BASIC::scalar>()),
      pose.translation.cast<BASIC::scalar>());
}

hybrid_slam::geometry::RigidTransform3d ToDomainPose(const BASIC::SE3& pose) {
  return {
      Eigen::Quaterniond(pose.R_.cast<double>()).normalized(),
      pose.t_.cast<double>(),
  };
}

struct RegistrationGeometryEvidence {
  double inlier_ratio = 0.0;
  double overlap = 0.0;
  bool observable = false;
};

RegistrationGeometryEvidence EvaluateGeometryEvidence(
    const hybrid_slam::adapter::PclPointCloud& aligned_source_map,
    const hybrid_slam::adapter::PclPointCloud::ConstPtr& target_map,
    const double correspondence_distance) {
  RegistrationGeometryEvidence evidence;
  if (aligned_source_map.empty() || !target_map || target_map->empty() ||
      !std::isfinite(correspondence_distance) ||
      correspondence_distance <= 0.0) {
    return evidence;
  }

  pcl::KdTreeFLANN<hybrid_slam::adapter::PclPoint> target_tree;
  target_tree.setInputCloud(target_map);
  const float squared_gate = static_cast<float>(
      correspondence_distance * correspondence_distance);
  std::vector<Eigen::Vector3d> inliers;
  inliers.reserve(aligned_source_map.size());
  std::unordered_set<int> covered_target_indices;
  covered_target_indices.reserve(aligned_source_map.size());

  for (const auto& point : aligned_source_map.points) {
    if (!pcl::isFinite(point)) {
      continue;
    }
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    if (target_tree.nearestKSearch(
            point, 1, nearest_index, nearest_squared_distance) != 1 ||
        nearest_squared_distance.front() > squared_gate) {
      continue;
    }
    inliers.emplace_back(point.x, point.y, point.z);
    covered_target_indices.insert(nearest_index.front());
  }

  evidence.inlier_ratio = static_cast<double>(inliers.size()) /
                           static_cast<double>(aligned_source_map.size());
  evidence.overlap = static_cast<double>(covered_target_indices.size()) /
                     static_cast<double>(std::min(
                         aligned_source_map.size(), target_map->size()));
  if (inliers.size() < 3) {
    return evidence;
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& point : inliers) {
    mean += point;
  }
  mean /= static_cast<double>(inliers.size());
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (const auto& point : inliers) {
    const Eigen::Vector3d centered = point - mean;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(inliers.size());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return evidence;
  }
  const Eigen::Vector3d eigenvalues = solver.eigenvalues();
  // A planar scene can still constrain a lidar registration. Reject only
  // point- or line-like matches, where the second spatial direction vanishes.
  evidence.observable = eigenvalues.allFinite() && eigenvalues[2] > 1e-8 &&
                        eigenvalues[1] >
                            std::max(1e-8, eigenvalues[2] * 1e-4);
  return evidence;
}

}  // namespace

std::vector<hybrid_slam::localization::RelocalizationCandidate>
PclKissCandidateRetriever::Retrieve(
    const hybrid_slam::localization::RelocalizationQuery& query,
    const hybrid_slam::localization::LocalizationMap& map,
    std::stop_token stop_token) {
  std::vector<hybrid_slam::localization::RelocalizationCandidate> result;
  if (stop_token.stop_requested() || !query.valid() || !map.valid()) {
    return result;
  }
  const auto source =
      query.current_scan_base.ownerAs<hybrid_slam::adapter::PclPointCloud>();
  auto target =
      map.cloudMap().ownerAs<hybrid_slam::adapter::PclPointCloud>();
  if (!target) {
    target = ToPclCloud(map.cloudMap());
  }
  if (!source || !target) {
    return result;
  }
  // KISS/VGICP consumes the accumulated local
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
    if (!candidate.passes_quality_gates) {
      const auto& pose = T_map_base->T_target_source;
      LOG(INFO) << "[PclKissCandidateRetriever] KISS diagnostic coarse pose"
                << " quality_gated=0"
                << " timestamp_ns=" << T_map_base->timestamp.nanoseconds()
                << " retrieval_score=" << candidate.final_inliers
                << " initial_matches=" << candidate.initial_matches
                << " rotation_inliers=" << candidate.rotation_inliers
                << " final_inliers=" << candidate.final_inliers
                << " tx=" << pose.translation.x()
                << " ty=" << pose.translation.y()
                << " tz=" << pose.translation.z()
                << " qx=" << pose.rotation.x()
                << " qy=" << pose.rotation.y()
                << " qz=" << pose.rotation.z()
                << " qw=" << pose.rotation.w();
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
    hybrid_slam::localization::RelocalizationConfig config,
    TargetSelector target_selector,
    PclRelocalizationMemoryCallback memory_callback,
    PclRegistrationDiagnosticsCallback diagnostics_callback)
    : config_(std::move(config)),
      target_selector_(std::move(target_selector)),
      memory_callback_(std::move(memory_callback)),
      diagnostics_callback_(std::move(diagnostics_callback)) {
  if (const auto error = config_.validate()) {
    throw std::invalid_argument("invalid PCL verifier config: " + *error);
  }
  if (!target_selector_) {
    throw std::invalid_argument("PCL verifier requires target selector");
  }
}

std::optional<hybrid_slam::localization::RegistrationVerification>
PclRegistrationVerifier::Verify(
    const hybrid_slam::localization::RelocalizationQuery& query,
    const hybrid_slam::localization::LocalizationMap& map,
    const hybrid_slam::localization::RelocalizationCandidate& candidate,
    std::stop_token stop_token) {
  if (stop_token.stop_requested() || !query.valid() || !map.valid() ||
      !candidate.initial_T_map_base.valid() ||
      candidate.initial_T_map_base.timestamp != query.T_odom_base.timestamp ||
      candidate.initial_T_map_base.target_frame !=
          hybrid_slam::geometry::FrameId::kMap ||
      candidate.initial_T_map_base.source_frame !=
          hybrid_slam::geometry::FrameId::kBase) {
    return std::nullopt;
  }
  const auto source_base =
      query.current_scan_base.ownerAs<hybrid_slam::adapter::PclPointCloud>();
  const auto target = target_selector_(
      ToBasicPose(candidate.initial_T_map_base.T_target_source));
  if (!source_base || source_base->empty() || !target || target->empty() ||
      stop_token.stop_requested()) {
    LOG(WARNING) << "[PclRegistrationVerifier]: skipped candidate"
                 << " retrieval_score=" << candidate.retrieval_score
                 << " source_points=" << (source_base ? source_base->size() : 0)
                 << " target_points=" << (target ? target->size() : 0)
                 << " cancelled=" << stop_token.stop_requested();
    return std::nullopt;
  }

  const auto source_odom_raw = ToOdomCloud(source_base, query.T_odom_base);
  const auto target_registration = PrepareRegistrationCloud(
      target, config_.target_voxel_size_m, config_.max_target_points);
  const auto source_odom = PrepareRegistrationCloud(
      source_odom_raw, config_.source_voxel_size_m,
      config_.max_source_points);
  const auto T_map_odom = ToMapOdom(candidate.initial_T_map_base, query);
  if (!source_odom || !target_registration || !T_map_odom) {
    return std::nullopt;
  }

  hybrid_slam::adapter::PclPointCloud aligned;
  Eigen::Matrix4f registration_initial = ToMatrix4f(
      T_map_odom->T_target_source);
  Eigen::Matrix4f final_transformation = registration_initial;
  bool converged = false;
  double fitness = std::numeric_limits<double>::infinity();
  bool coarse_attempted = false;
  bool coarse_converged = false;
  double coarse_fitness = std::numeric_limits<double>::infinity();
  std::size_t coarse_source_points = 0;
  std::size_t coarse_target_points = 0;

#ifdef HAS_FAST_GICP
  if (config_.coarse_registration_voxel_size_m >
      config_.source_voxel_size_m) {
    const auto coarse_source = PrepareRegistrationCloud(
        source_odom_raw, config_.coarse_registration_voxel_size_m,
        config_.max_source_points);
    const auto coarse_target = PrepareRegistrationCloud(
        target, config_.coarse_registration_voxel_size_m,
        config_.max_target_points);
    if (coarse_source && coarse_target) {
      coarse_attempted = true;
      coarse_source_points = coarse_source->size();
      coarse_target_points = coarse_target->size();
      fast_gicp::FastVGICP<hybrid_slam::adapter::PclPoint,
                           hybrid_slam::adapter::PclPoint> coarse_vgicp;
      coarse_vgicp.setInputSource(coarse_source);
      coarse_vgicp.setInputTarget(coarse_target);
      coarse_vgicp.setMaximumIterations(
          static_cast<int>(config_.max_registration_iterations));
      coarse_vgicp.setTransformationEpsilon(1e-4);
      coarse_vgicp.setEuclideanFitnessEpsilon(1e-4);
      coarse_vgicp.setResolution(config_.coarse_registration_voxel_size_m);
      // DIRECT27 lets the coarse layer bridge a small graph-pose error.  The
      // subsequent fine pass and geometry gates remain the acceptance test.
      coarse_vgicp.setNeighborSearchMethod(
          fast_gicp::NeighborSearchMethod::DIRECT27);
      coarse_vgicp.setNumThreads(static_cast<int>(config_.registration_threads));
      hybrid_slam::adapter::PclPointCloud coarse_aligned;
      coarse_vgicp.align(coarse_aligned, registration_initial);
      coarse_converged = coarse_vgicp.hasConverged();
      coarse_fitness = coarse_vgicp.getFitnessScore();
      if (coarse_converged) {
        registration_initial = coarse_vgicp.getFinalTransformation();
      }
      LOG(INFO) << "[PclRegistrationVerifier]: coarse_vgicp"
                << " converged=" << coarse_converged
                << " fitness=" << coarse_fitness
                << " source_points=" << coarse_source->size()
                << " target_points=" << coarse_target->size();
    }
  }
  fast_gicp::FastVGICP<hybrid_slam::adapter::PclPoint, hybrid_slam::adapter::PclPoint> vgicp;
  vgicp.setInputSource(source_odom);
  vgicp.setInputTarget(target_registration);
  vgicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  vgicp.setTransformationEpsilon(1e-4);
  vgicp.setEuclideanFitnessEpsilon(1e-4);
  vgicp.setResolution(static_cast<float>(config_.source_voxel_size_m));
  vgicp.setNumThreads(static_cast<int>(config_.registration_threads));
  vgicp.align(aligned, registration_initial);
  converged = vgicp.hasConverged();
  fitness = vgicp.getFitnessScore();
  final_transformation = vgicp.getFinalTransformation();
#else
  pcl::GeneralizedIterativeClosestPoint<hybrid_slam::adapter::PclPoint, hybrid_slam::adapter::PclPoint>
      gicp;
  gicp.setInputSource(source_odom);
  gicp.setInputTarget(target_registration);
  gicp.setMaximumIterations(
      static_cast<int>(config_.max_registration_iterations));
  gicp.setMaxCorrespondenceDistance(
      std::max(1.0, config_.source_voxel_size_m * 8.0));
  gicp.setTransformationEpsilon(1e-4);
  gicp.setEuclideanFitnessEpsilon(1e-4);
  gicp.align(aligned, registration_initial);
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

  const auto evidence = EvaluateGeometryEvidence(
      aligned, target_registration,
      std::max(0.1, config_.source_voxel_size_m * 2.0));
  if (diagnostics_callback_) {
    diagnostics_callback_(
        {T_map_odom->T_target_source,
         FromMatrix4f(final_transformation),
         source_odom->size(),
         target_registration->size(),
         coarse_source_points,
         coarse_target_points,
         coarse_attempted,
         coarse_converged,
         converged,
         coarse_fitness,
         fitness,
         evidence.inlier_ratio,
         evidence.overlap,
         evidence.observable},
        *source_odom, *target_registration, aligned);
  }

  if (stop_token.stop_requested() || !converged || !std::isfinite(fitness) ||
      fitness < 0.0) {
    LOG(WARNING) << "[PclRegistrationVerifier]: rejected candidate"
                 << " retrieval_score=" << candidate.retrieval_score
                 << " converged=" << converged
                 << " fitness=" << fitness
                 << " source_points=" << source_odom->size()
                 << " target_points=" << target_registration->size()
                 << " cancelled=" << stop_token.stop_requested();
    return std::nullopt;
  }
  const auto T_map_base = ToMapBase(FromMatrix4f(final_transformation), query);
  if (!T_map_base) {
    return std::nullopt;
  }
  if (!evidence.observable ||
      evidence.inlier_ratio < config_.minimum_inlier_ratio ||
      evidence.overlap < config_.minimum_overlap) {
    LOG(WARNING) << "[PclRegistrationVerifier]: rejected candidate"
                 << " retrieval_score=" << candidate.retrieval_score
                 << " reason=geometry_evidence"
                 << " inlier_ratio=" << evidence.inlier_ratio
                 << " overlap=" << evidence.overlap
                 << " observable=" << evidence.observable;
    return std::nullopt;
  }
  hybrid_slam::localization::RegistrationVerification verification{
      *T_map_base,
      fitness,
      evidence.inlier_ratio,
      hybrid_slam::localization::Confidence::kUnknown,
      evidence.overlap,
      evidence.observable,
  };
  LOG(INFO) << "[PclRegistrationVerifier]: accepted candidate"
            << " retrieval_score=" << candidate.retrieval_score
            << " fitness=" << fitness
            << " inlier_ratio=" << evidence.inlier_ratio
            << " overlap=" << evidence.overlap
            << " observable=" << evidence.observable
            << " source_points=" << source_odom->size()
            << " target_points=" << target_registration->size();
  return verification;
}

}  // namespace LI2Sup
