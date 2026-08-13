#ifndef LI2SUP_LEGACY_GLOBAL_PARAMETER_BRIDGE_H_
#define LI2SUP_LEGACY_GLOBAL_PARAMETER_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>

namespace LI2Sup {

// Compatibility-only entry points that populate the process-wide legacy
// parameter symbols declared by lio/params.h. Standard applications must use
// the instance-owned readers in legacy_parameter_bridge.h instead.
void LoadLegacyParametersFromRos(rclcpp::Node& node);

// Historical spelling retained for external/legacy callers.
void LoadParamFromRos(rclcpp::Node& node);

}  // namespace LI2Sup

#endif  // LI2SUP_LEGACY_GLOBAL_PARAMETER_BRIDGE_H_
