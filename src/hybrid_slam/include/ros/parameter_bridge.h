#ifndef LI2SUP_PARAMETER_BRIDGE_H_
#define LI2SUP_PARAMETER_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>

#include "lio/system/point_lio_runtime_config.h"

namespace LI2Sup {

// Read Point-LIO pipeline values into an instance-owned snapshot.
[[nodiscard]] PointLIORuntimeConfig ReadPointLIORuntimeConfigFromRos(
    rclcpp::Node& node);

}  // namespace LI2Sup

#endif  // LI2SUP_PARAMETER_BRIDGE_H_
