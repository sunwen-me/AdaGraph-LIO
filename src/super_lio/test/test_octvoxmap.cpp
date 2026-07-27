// Unit tests for OctVoxMap::getTopK: compare against brute-force 5-NN over
// the stored map points, at resolution 0.5 (the table's native resolution)
// and 0.3 (exercises the SetOptions rescaling of orders_min_dis2).
//
// The map merges points that fall into the same sub-voxel (averaging within
// 0.1 m), so the test cloud is generated with at most one point per sub-voxel:
// stored representatives are then exactly the inserted points and brute force
// over the input cloud is an exact reference.
//
// getTopK enumerates a bounded neighborhood; the search is only guaranteed
// exhaustive while the 5th-neighbor distance stays below the last group shell
// bound orders_min_dis2[4] * (resolution/0.5)^2. Queries whose brute-force
// 5th distance exceeds 90% of that bound are skipped.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_set>
#include <vector>

#include "OctVoxMap/OctVoxMap.hpp"
#include "basic/alias.h"

namespace {

using MapType = LI2Sup::OctVoxMap<BASIC::V3, BASIC::scalar>;
using Points = MapType::Points;
using KNNHeapType = MapType::KNNHeapType;
using BASIC::V3;

// Same sub-voxel key computation as OctVoxMap::insert (float arithmetic
// mirrored on purpose so deduplication matches the map's binning exactly).
long long subVoxelKey(const V3& p, float sub_inv_resolution) {
  const Eigen::Vector3i fine_key =
      (p * sub_inv_resolution).array().floor().cast<int>();
  const long long off = 1 << 20;
  const long long x = fine_key[0] + off;
  const long long y = fine_key[1] + off;
  const long long z = fine_key[2] + off;
  return ((x << 21) + y) * (1ll << 21) + z;
}

Points makeUniqueSubVoxelCloud(float resolution, float half_box, int attempts,
                               unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> coord(-half_box, half_box);
  const float sub_inv = static_cast<float>(1.0 / (resolution / 2.0));
  std::unordered_set<long long> used;
  Points pts;
  pts.reserve(attempts);
  for (int i = 0; i < attempts; ++i) {
    const V3 p(coord(rng), coord(rng), coord(rng));
    if (used.insert(subVoxelKey(p, sub_inv)).second) {
      pts.push_back(p);
    }
  }
  return pts;
}

std::vector<float> bruteForceKnnDist2(const Points& pts, const V3& query,
                                      std::size_t k) {
  std::vector<float> d2;
  d2.reserve(pts.size());
  for (const auto& p : pts) {
    d2.push_back((p - query).squaredNorm());
  }
  const std::size_t kk = std::min(k, d2.size());
  std::partial_sort(d2.begin(), d2.begin() + kk, d2.end());
  d2.resize(kk);
  return d2;
}

std::vector<float> heapDist2Sorted(const KNNHeapType& heap) {
  std::vector<float> d2(heap.dist2_, heap.dist2_ + heap.count);
  std::sort(d2.begin(), d2.end());
  return d2;
}

void compareAgainstBruteForce(float resolution, float half_box, int attempts,
                              unsigned seed) {
  const Points cloud = makeUniqueSubVoxelCloud(resolution, half_box, attempts, seed);
  ASSERT_GT(cloud.size(), 1000u);

  MapType map(MapType::Options(resolution, 1000000));
  map.insert(cloud);

  // Exhaustive-search guarantee radius^2 (see header comment).
  const float shell_scale = resolution * resolution * 4.0f;  // (res/0.5)^2
  const float guard2 = 0.9f * 0.375f * shell_scale;

  std::mt19937 rng(seed ^ 0x9e3779b9u);
  std::uniform_real_distribution<float> jitter(-0.4f * resolution,
                                               0.4f * resolution);
  std::uniform_int_distribution<std::size_t> pick(0, cloud.size() - 1);
  std::uniform_real_distribution<float> interior(-half_box + 1.0f,
                                                 half_box - 1.0f);

  int checked = 0;
  for (int q = 0; q < 400; ++q) {
    V3 query;
    if (q % 2 == 0) {
      const V3& base = cloud[pick(rng)];
      query = base + V3(jitter(rng), jitter(rng), jitter(rng));
    } else {
      query = V3(interior(rng), interior(rng), interior(rng));
    }

    const std::vector<float> brute = bruteForceKnnDist2(cloud, query, 5);
    ASSERT_EQ(brute.size(), 5u);
    if (brute[4] >= guard2) continue;  // outside the exhaustive guarantee

    KNNHeapType heap;
    heap.reset();
    map.getTopK(query, heap);
    ASSERT_EQ(static_cast<int>(heap.count), 5)
        << "resolution=" << resolution << " query=" << query.transpose();

    const std::vector<float> got = heapDist2Sorted(heap);
    for (int i = 0; i < 5; ++i) {
      EXPECT_NEAR(got[i], brute[i], 1e-4f)
          << "resolution=" << resolution << " neighbor " << i
          << " query=" << query.transpose();
    }
    ++checked;
  }
  // The clouds are dense enough that most queries fall inside the guarantee.
  EXPECT_GT(checked, 100) << "resolution=" << resolution;
}

TEST(OctVoxMapTest, GetTopKMatchesBruteForceAtNativeResolution05) {
  compareAgainstBruteForce(0.5f, 4.0f, 60000, 20240001u);
}

TEST(OctVoxMapTest, GetTopKMatchesBruteForceAtResolution03) {
  compareAgainstBruteForce(0.3f, 3.0f, 100000, 20240002u);
}

TEST(OctVoxMapTest, ExactHitReturnsZeroDistance) {
  const float resolution = 0.5f;
  const Points cloud =
      makeUniqueSubVoxelCloud(resolution, 3.0f, 20000, 777u);
  MapType map(MapType::Options(resolution, 1000000));
  map.insert(cloud);

  std::mt19937 rng(778u);
  std::uniform_int_distribution<std::size_t> pick(0, cloud.size() - 1);
  for (int i = 0; i < 20; ++i) {
    const V3& query = cloud[pick(rng)];
    KNNHeapType heap;
    heap.reset();
    map.getTopK(query, heap);
    ASSERT_GE(static_cast<int>(heap.count), 1);
    const std::vector<float> got = heapDist2Sorted(heap);
    EXPECT_NEAR(got[0], 0.0f, 1e-9f);
  }
}

TEST(OctVoxMapTest, SparseMapReturnsFewerThanFiveNeighbors) {
  const float resolution = 0.5f;
  MapType map(MapType::Options(resolution, 1000000));
  // Three points in distinct sub-voxels, all within the first search shells.
  Points pts;
  pts.push_back(V3(0.05f, 0.05f, 0.05f));
  pts.push_back(V3(0.30f, 0.05f, 0.05f));
  pts.push_back(V3(0.05f, 0.30f, 0.05f));
  map.insert(pts);

  const V3 query(0.10f, 0.10f, 0.05f);
  KNNHeapType heap;
  heap.reset();
  map.getTopK(query, heap);
  ASSERT_EQ(static_cast<int>(heap.count), 3);

  const std::vector<float> got = heapDist2Sorted(heap);
  const std::vector<float> brute = bruteForceKnnDist2(pts, query, 3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], brute[i], 1e-6f);
  }
}

}  // namespace
