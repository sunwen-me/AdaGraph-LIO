#ifndef LI2SUP_LOOP_CLOSURE_PLUGIN_H_
#define LI2SUP_LOOP_CLOSURE_PLUGIN_H_

#include <memory>

#include "hybrid_slam/backend/loop_closure.h"
#include "lio/pcl_loop_closure.h"

namespace LI2Sup {

struct LoopClosurePlugin {
  std::unique_ptr<hybrid_slam::backend::LoopDetector> detector;
  std::unique_ptr<hybrid_slam::backend::LoopVerifier> verifier;
};

// Application-layer composition point for replaceable loop-recognition
// modules. Core scheduling and graph mutation remain plugin-agnostic.
[[nodiscard]] LoopClosurePlugin CreateLoopClosurePlugin(
    const PclLoopClosureConfig& config);

}  // namespace LI2Sup

#endif  // LI2SUP_LOOP_CLOSURE_PLUGIN_H_
