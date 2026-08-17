#ifndef HYBRID_SLAM_ADAPTER_PCL_TYPES_H_
#define HYBRID_SLAM_ADAPTER_PCL_TYPES_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace hybrid_slam::adapter {

using PclPoint = pcl::PointXYZI;
using PclPointCloud = pcl::PointCloud<PclPoint>;
using PclCloudPtr = PclPointCloud::Ptr;
using PclCloudConstPtr = PclPointCloud::ConstPtr;

}  // namespace hybrid_slam::adapter

#endif  // HYBRID_SLAM_ADAPTER_PCL_TYPES_H_
