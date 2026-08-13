#pragma once

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <kiss_matcher/points/fast_floor.hpp>
#include <kiss_matcher/points/point_cloud.hpp>
#include <kiss_matcher/points/traits.hpp>
#include <kiss_matcher/points/vector3i_hash.hpp>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

namespace kiss_matcher {

/**
 * @brief Voxel grid downsampling with TBB backend.
 * @note  This function has minor run-by-run non-deterministic behavior due to parallel data
 * collection.
 * @param points     Input points
 * @param leaf_size  Downsampling resolution
 * @return           Downsampled points
 */
template <typename InputPointCloud, typename OutputPointCloud = InputPointCloud>
std::shared_ptr<OutputPointCloud> VoxelgridSampling(const InputPointCloud& points,
                                                    const double leaf_size) {
  if (traits::size(points) == 0) {
    return std::make_shared<OutputPointCloud>();
  }

  const double inv_leaf_size = 1.0 / leaf_size;

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size          = 21;
  constexpr size_t coord_bit_mask       = (1 << 21) - 1;
  constexpr int coord_offset            = 1 << (coord_bit_size - 1);

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt(traits::size(points));
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, traits::size(points), 64),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); i++) {
          const Eigen::Array4i coord =
              fast_floor(traits::point(points, i) * inv_leaf_size) + coord_offset;
          if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
            std::cerr << "warning: voxel coord is out of range!!" << std::endl;
            coord_pt[i] = {invalid_coord, i};
            continue;
          }

          const std::uint64_t bits =
              (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |
              (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |
              (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
          coord_pt[i] = {bits, i};
        }
      });

  tbb::parallel_sort(coord_pt,
                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  auto downsampled = std::make_shared<OutputPointCloud>();
  traits::resize(*downsampled, traits::size(points));

  const int block_size            = 2048;
  std::atomic_uint64_t num_points = 0;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, traits::size(points), block_size),
                    [&](const tbb::blocked_range<size_t>& range) {
                      std::vector<Eigen::Vector4d> sub_points;
                      sub_points.reserve(block_size);

                      Eigen::Vector4d sum_pt =
                          traits::point(points, coord_pt[range.begin()].second);
                      for (size_t i = range.begin() + 1; i != range.end(); i++) {
                        if (coord_pt[i].first == invalid_coord) {
                          continue;
                        }

                        if (coord_pt[i - 1].first != coord_pt[i].first) {
                          sub_points.emplace_back(sum_pt / sum_pt.w());
                          sum_pt.setZero();
                        }
                        sum_pt += traits::point(points, coord_pt[i].second);
                      }
                      sub_points.emplace_back(sum_pt / sum_pt.w());

                      const size_t point_index_begin = num_points.fetch_add(sub_points.size());
                      for (size_t i = 0; i < sub_points.size(); i++) {
                        traits::set_point(*downsampled, point_index_begin + i, sub_points[i]);
                      }
                    });

  traits::resize(*downsampled, num_points);

  return downsampled;
}

inline std::vector<Eigen::Vector3f> VoxelgridSampling(const std::vector<Eigen::Vector3f>& points,
                                                      const double leaf_size) {
  if (points.empty()) {
    return {};
  }

  size_t num_raw_points      = points.size();
  const double inv_leaf_size = 1.0 / leaf_size;

  constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size          = 21;
  constexpr size_t coord_bit_mask       = (1 << 21) - 1;
  constexpr int coord_offset            = 1 << (coord_bit_size - 1);

  std::vector<std::pair<std::uint64_t, size_t>> coord_pt(num_raw_points);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, num_raw_points, 64),
      [&](const tbb::blocked_range<size_t>& range) {
        for (size_t i = range.begin(); i != range.end(); i++) {
          const Eigen::Array3i coord =
              fast_floor_vector3f(points[i] * inv_leaf_size) + coord_offset;
          if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
            std::cerr << "warning: voxel coord is out of range!!" << std::endl;
            coord_pt[i] = {invalid_coord, i};
            continue;
          }

          const std::uint64_t bits =
              (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (coord_bit_size * 0)) |
              (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (coord_bit_size * 1)) |
              (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (coord_bit_size * 2));
          coord_pt[i] = {bits, i};
        }
      });

  tbb::parallel_sort(coord_pt,
                     [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

  std::vector<Eigen::Vector3f> downsampled;
  downsampled.resize(num_raw_points);

  const int block_size            = 2048;
  std::atomic_uint64_t num_points = 0;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, num_raw_points, block_size),
                    [&](const tbb::blocked_range<size_t>& range) {
                      std::vector<Eigen::Vector3f> sub_points;
                      sub_points.reserve(block_size);

                      Eigen::Vector3f sum_pt = points[coord_pt[range.begin()].second];
                      float count            = 1.0;
                      for (size_t i = range.begin() + 1; i != range.end(); i++) {
                        if (coord_pt[i].first == invalid_coord) {
                          continue;
                        }

                        if (coord_pt[i - 1].first != coord_pt[i].first) {
                          sub_points.emplace_back(sum_pt / count);
                          sum_pt.setZero();
                          count = 0.0;
                        }
                        sum_pt += points[coord_pt[i].second];
                        count += 1.0;
                      }
                      sub_points.emplace_back(sum_pt / count);

                      const size_t point_index_begin = num_points.fetch_add(sub_points.size());
                      for (size_t i = 0; i < sub_points.size(); i++) {
                        downsampled[point_index_begin + i] = sub_points[i];
                      }
                    });

  downsampled.resize(num_points);

  return downsampled;
}

// The parallel implementation above intentionally favors throughput, and its
// atomic output reservation makes both the order of output voxels and the
// floating-point reduction order depend on task scheduling.  That is useful
// for scan-to-scan matching, but a global relocalization candidate must be
// reproducible: a one-point difference in the sampled cloud can otherwise
// change descriptor correspondences and delay a valid recovery by a retry
// period.  Keep this opt-in helper close to the fast path so callers can make
// the policy explicit without changing the default third-party behaviour.
inline std::vector<Eigen::Vector3f> VoxelgridSamplingDeterministic(
    const std::vector<Eigen::Vector3f>& points, const double leaf_size) {
  if (points.empty()) {
    return {};
  }

  const double inv_leaf_size = 1.0 / leaf_size;
  constexpr std::uint64_t invalid_coord =
      std::numeric_limits<std::uint64_t>::max();
  constexpr int coord_bit_size = 21;
  constexpr std::size_t coord_bit_mask = (1 << 21) - 1;
  constexpr int coord_offset = 1 << (coord_bit_size - 1);

  std::vector<std::pair<std::uint64_t, std::size_t>> coord_pt;
  coord_pt.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Array3i coord =
        fast_floor_vector3f(points[i] * inv_leaf_size) + coord_offset;
    if ((coord < 0).any() || (coord > coord_bit_mask).any()) {
      std::cerr << "warning: voxel coord is out of range!!" << std::endl;
      coord_pt.emplace_back(invalid_coord, i);
      continue;
    }
    const std::uint64_t bits =
        (static_cast<std::uint64_t>(coord[0] & coord_bit_mask)
         << (coord_bit_size * 0)) |
        (static_cast<std::uint64_t>(coord[1] & coord_bit_mask)
         << (coord_bit_size * 1)) |
        (static_cast<std::uint64_t>(coord[2] & coord_bit_mask)
         << (coord_bit_size * 2));
    coord_pt.emplace_back(bits, i);
  }

  // The secondary source-index ordering fixes the reduction sequence for
  // points in a common voxel.  std::sort makes the output voxel sequence
  // stable too, unlike parallel_sort plus atomic block publication.
  std::sort(coord_pt.begin(), coord_pt.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first != rhs.first ? lhs.first < rhs.first
                                             : lhs.second < rhs.second;
            });

  std::vector<Eigen::Vector3f> downsampled;
  downsampled.reserve(coord_pt.size());
  for (std::size_t begin = 0; begin < coord_pt.size();) {
    if (coord_pt[begin].first == invalid_coord) {
      ++begin;
      continue;
    }
    const std::uint64_t key = coord_pt[begin].first;
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    std::size_t count = 0;
    std::size_t end = begin;
    while (end < coord_pt.size() && coord_pt[end].first == key) {
      sum += points[coord_pt[end].second];
      ++count;
      ++end;
    }
    downsampled.emplace_back(sum / static_cast<float>(count));
    begin = end;
  }
  return downsampled;
}

}  // namespace kiss_matcher
