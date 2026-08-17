#include "lio/loop_closure_plugin.h"

#include <stdexcept>
#include <utility>

namespace LI2Sup {

LoopClosurePlugin CreateLoopClosurePlugin(const PclLoopClosureConfig& config) {
  if (const auto error = config.validate()) {
    throw std::invalid_argument("invalid loop closure plugin config: " + *error);
  }
  if (config.plugin == "scan_context") {
    return {
        std::make_unique<ScanContextLoopDetector>(ScanContextLoopDetector::Config{
            config.scan_context, config.maximum_candidates,
            config.minimum_temporal_separation_s}),
        std::make_unique<PclLoopClosureVerifier>(config),
    };
  }
  if (config.plugin == "bevplace2") {
    return {
        std::make_unique<BevPlace2LoopDetector>(BevPlace2LoopDetector::Config{
            config.bevplace2, config.scan_context, config.maximum_candidates,
            config.minimum_temporal_separation_s}),
        std::make_unique<PclLoopClosureVerifier>(config),
    };
  }
  throw std::invalid_argument("unsupported loop closure plugin: " +
                              config.plugin);
}

}  // namespace LI2Sup
