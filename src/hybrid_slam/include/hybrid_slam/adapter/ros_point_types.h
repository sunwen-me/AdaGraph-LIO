#ifndef HYBRID_SLAM_ADAPTER_ROS_POINT_TYPES_H_
#define HYBRID_SLAM_ADAPTER_ROS_POINT_TYPES_H_

#include <cstdint>

#include <pcl/point_types.h>

// PCL point layouts used only while decoding ROS sensor messages. Keep these
// registrations at the ROS adapter boundary; typed core clouds never expose
// sensor-vendor point structs.
namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    velodyne_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (float, time, time))

namespace NCLT {
struct EIGEN_ALIGN16 Point {
  float x;
  float y;
  float z;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace NCLT

POINT_CLOUD_REGISTER_POINT_STRUCT(
    NCLT::Point, (float, x, x)(float, y, y)(float, z, z)(float, time, time))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    ouster_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (std::uint32_t, t, t)(std::uint32_t, range, range))

namespace hesai_ros {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace hesai_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(
    hesai_ros::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
        (double, timestamp, timestamp)(std::uint16_t, ring, ring))

namespace LivoxSIM {
struct EIGEN_ALIGN16 Point {
  PCL_ADD_POINT4D;
  float intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace LivoxSIM

POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxSIM::Point,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))

#endif  // HYBRID_SLAM_ADAPTER_ROS_POINT_TYPES_H_
