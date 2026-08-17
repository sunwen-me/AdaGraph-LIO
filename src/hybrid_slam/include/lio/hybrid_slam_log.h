#pragma once

// HybridSLAM logging macros
// Can be disabled by defining HYBRID_SLAM_LOG_DISABLED

#include <glog/logging.h>

#ifndef HYBRID_SLAM_LOG_DISABLED
  #define HYBRID_SLAM_LOG_INFO(msg) LOG(INFO) << " ---> [HybridSLAM]: " << msg
  #define HYBRID_SLAM_LOG_WARN(msg) LOG(WARNING) << " ---> [HybridSLAM]: " << msg
  #define HYBRID_SLAM_LOG_DEBUG(msg) LOG(INFO) << " ---> [HybridSLAM DEBUG]: " << msg
#else
  #define HYBRID_SLAM_LOG_INFO(msg)
  #define HYBRID_SLAM_LOG_WARN(msg)
  #define HYBRID_SLAM_LOG_DEBUG(msg)
#endif

// Keyframe logging (can be disabled separately)
#ifndef HYBRID_SLAM_KEYFRAME_LOG_DISABLED
  #define HYBRID_SLAM_LOG_KEYFRAME(id, s_t, s_r) \
    LOG(INFO) << " ---> [HybridSLAM]: Keyframe " << id \
              << ", scale: [" << s_t << ", " << s_r << "]"
#else
  #define HYBRID_SLAM_LOG_KEYFRAME(id, s_t, s_r)
#endif
