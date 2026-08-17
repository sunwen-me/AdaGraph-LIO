
#ifndef LI2Sup_DS_COMMON_H
#define LI2Sup_DS_COMMON_H

#include <array>
#include <string>

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

}

#endif
