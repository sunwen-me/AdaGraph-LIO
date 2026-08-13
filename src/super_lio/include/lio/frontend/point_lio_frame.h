#ifndef ADAGRAPH_LIO_FRONTEND_POINT_LIO_FRAME_H_
#define ADAGRAPH_LIO_FRONTEND_POINT_LIO_FRAME_H_

#include "small_point_lio/small_point_lio.h"

namespace LI2Sup {

// Transitional vendor-frame alias shared by the Point-LIO adapter and the
// system-level legacy consumer. It keeps the dependency pointing toward the
// vendor adapter rather than making the system pipeline depend on the
// PointLIOFrontend class itself.
using PointLioFrame = small_point_lio::SmallPointLio::FrontendFrame;

}  // namespace LI2Sup

#endif  // ADAGRAPH_LIO_FRONTEND_POINT_LIO_FRAME_H_
