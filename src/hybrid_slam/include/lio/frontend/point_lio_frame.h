#ifndef HYBRID_SLAM_LIO_FRONTEND_POINT_LIO_FRAME_H_
#define HYBRID_SLAM_LIO_FRONTEND_POINT_LIO_FRAME_H_

#include "small_point_lio/small_point_lio.h"

namespace LI2Sup {

// Vendor-frame alias shared by the Point-LIO adapter and its pipeline. It
// avoids coupling the pipeline to the PointLIOFrontend class itself.
using PointLioFrame = small_point_lio::SmallPointLio::FrontendFrame;

}  // namespace LI2Sup

#endif  // HYBRID_SLAM_LIO_FRONTEND_POINT_LIO_FRAME_H_
