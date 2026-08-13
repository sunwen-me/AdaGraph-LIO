/**
 * @file alias.h
 * @author Liansheng Wang (lswang@mail.ecust.edu.cn)
 * @date 2023-10-06
 * @copyright Copyright (c) 2023
 */

 #ifndef ALIAS_TYPES_H_H_
 #define ALIAS_TYPES_H_H_
 
 #include "core_alias.h"
 #include <pcl/point_types.h>
 #include <pcl/point_cloud.h>
 
 namespace velodyne_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D;
     float intensity;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace velodyne_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, time, time))
 
 
 namespace NCLT {
 struct EIGEN_ALIGN16 Point {
     float x;
     float y;
     float z;
     float time;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace velodyne_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(NCLT::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, time, time))
 
 
 namespace ouster_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D;
     float intensity;
     uint32_t t;
     uint32_t range;
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }  // namespace ouster_ros
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (std::uint32_t, t, t)
                                   (std::uint32_t, range, range)
                                   )
 
 
 namespace hesai_ros {
 struct EIGEN_ALIGN16 Point {
     PCL_ADD_POINT4D
     float intensity;
     double timestamp;
     uint16_t ring; 
     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 };
 }
 
 POINT_CLOUD_REGISTER_POINT_STRUCT(hesai_ros::Point,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (double, timestamp, timestamp)
                                   (std::uint16_t, ring, ring)
                                   )
 
 
namespace LI2Sup {
struct EIGEN_ALIGN16 PointXTZIT {
  PCL_ADD_POINT4D
  float intensity;
  double offset_time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXTZIT() = default;

  PointXTZIT(float x_, float y_, float z_, float intensity_, double time_)
  {
    x = x_;
    y = y_;
    z = z_;
    intensity = intensity_;
    offset_time = time_;
  }
};
}
 
POINT_CLOUD_REGISTER_POINT_STRUCT(LI2Sup::PointXTZIT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (double, offset_time, offset_time)
                                  )
 
 
 
namespace LivoxSIM {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    float intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
 
 
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxSIM::Point,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  )

// x, y, z, intensity, tag, line, timestamp
 
namespace BASIC {

/* PCL compatibility aliases. Typed core code must include core_alias.h. */
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

}
#endif
