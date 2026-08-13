#ifndef LI2SUP_LEGACY_PARAMETER_BRIDGE_H_
#define LI2SUP_LEGACY_PARAMETER_BRIDGE_H_

#include <rclcpp/rclcpp.hpp>

#include "lio/system/legacy_runtime_config.h"

namespace LI2Sup {

// Read the transitional downstream values into an instance-owned snapshot.
// This function does not mutate any process-wide compatibility globals and is
// the standard-node entry point.
[[nodiscard]] LegacyRuntimeConfig ReadLegacyRuntimeConfigFromRos(
    rclcpp::Node& node);

}  // namespace LI2Sup

#endif  // LI2SUP_LEGACY_PARAMETER_BRIDGE_H_
