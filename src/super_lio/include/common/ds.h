
#ifndef LI2Sup_DS_COMMON_H
#define LI2Sup_DS_COMMON_H

#include <deque>
#include <string>

#include "basic/alias.h"
#include "common/legacy_state.h"


namespace LI2Sup{

enum LID_TYPE
{
  LIVOX = 1,
  HESAI16 = 2,
  VELO16 = 3,
  VELO32 = 4,
  VEL_NCLT = 5,
  LS16 = 6,
  OUSTER = 7,
  SIM_XYZI = 8,
  LIVOX_POINTCLOUD2 = 9,
  CUSTOM_MID360 = 10,
  UNILIDAR = 11
};

static const std::array<std::string, 12> LID_TYPE_NAMES = {
  "INVALID",
  "LIVOX",
  "HESAI16",
  "VELO16",
  "VELO32",
  "VEL_NCLT",
  "LS16",
  "OUSTER",
  "SIM_XYZI",
  "LIVOX_POINTCLOUD2",
  "CUSTOM_MID360",
  "UNILIDAR"
};

struct LidarData
{
  double start_time = 0.0;
  double end_time = 0.0;
  pcl::PointCloud<LI2Sup::PointXTZIT>::Ptr pc{nullptr};
};


struct MeasureGroup {
  LidarData lidar;
  std::deque<IMUData> imu;
};


}

#endif
