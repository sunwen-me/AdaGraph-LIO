
#ifndef VOXEL_GRID_CLOSEST_H
#define VOXEL_GRID_CLOSEST_H


#include <cstdint>

#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <glog/logging.h>

#include "tsl/robin_hood.h"


namespace LI2Sup {


template<typename PointType>
class VoxelGridClosest {
private:
  using Point = PointType;
  using PointCloud = pcl::PointCloud<Point>;
  using CloudPtr = typename PointCloud::Ptr;

  CloudPtr cloud_;
  float voxel_size_ = 0.5f;
  float inv_voxel_size_ = 2.0f;
  robin_hood::unordered_flat_map<std::size_t, std::size_t> voxel_map_;

  std::vector<Point, Eigen::aligned_allocator<Point>> points_;
  std::vector<float> dist2_;
  const Eigen::Vector3i offset_ = Eigen::Vector3i(1000, 1000, 1000);
  bool range_warned_ = false;

public:
  VoxelGridClosest() {
    dist2_.reserve(10000);
    points_.reserve(10000);
    voxel_map_.reserve(10000);
  }

  void setLeafSize(float lx) {
    voxel_size_ = lx;
    inv_voxel_size_ = 1.0f / lx;
  }

  void setInputCloud(const CloudPtr& cloud) {
    cloud_ = cloud;
  }

  void filter(CloudPtr& output) {
    voxel_map_.clear();
    dist2_.clear();
    points_.clear();

    for (const auto& pt : cloud_->points) {
      Eigen::Vector3f pf = pt.getVector3fMap();
      Eigen::Vector3i idx = (pf * inv_voxel_size_).array().round().cast<int>();
      Eigen::Vector3f center = voxel_size_ * idx.cast<float>();
      float d2 = (pf - center).squaredNorm();

      idx += offset_; // Avoid negative indices
      // The key packs each axis into 15 bits (x: bits 0-14, y: 15-29,
      // z: 30+). Valid offset cells are [0, 32767] per axis, i.e. raw cells
      // in [-1000, +31767] ([-1000, +31767] * leaf in meters); anything
      // outside would bleed into the neighboring bit field and silently
      // alias distant voxels, so such points are dropped instead. z is held
      // to the same bound for a uniform check even though its field is wider.
      if ((std::uint32_t(idx[0]) | std::uint32_t(idx[1]) | std::uint32_t(idx[2])) > 0x7FFFu) {
        if (!range_warned_) {
          LOG(WARNING) << "VoxelGridClosest: dropping point(s) outside packable cell range,"
                       << " first at [" << pf.transpose() << "] m (|p| = " << pf.norm()
                       << " m); valid range per axis is [" << -1000.0f * voxel_size_
                       << ", " << 31767.0f * voxel_size_ << "] m at leaf " << voxel_size_;
          range_warned_ = true;
        }
        continue;
      }
      const std::size_t key = ((std::size_t(idx[2])) << 30) |
                              ((std::size_t(idx[1])) << 15) |
                              ( std::size_t(idx[0]));

      auto it = voxel_map_.find(key);
      if (it == voxel_map_.end()) {
        voxel_map_.emplace(key, points_.size());
        points_.push_back(pt);
        dist2_.push_back(d2);
      } else if (d2 < dist2_[it->second]) {
        points_[it->second] = pt;
        dist2_[it->second] = d2;
      }
    }

    output->points.swap(points_);
    output->width = output->points.size();
    output->height = 1;
    output->is_dense = true;
    output->header = cloud_->header;
  }
};

}
#endif // VOXEL_GRID_CLOSEST_H