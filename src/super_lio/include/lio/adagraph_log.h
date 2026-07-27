#pragma once

// AdaGraph-LIO logging macros
// Can be disabled by defining ADAGRAPH_LOG_DISABLED

#include <glog/logging.h>

#ifndef ADAGRAPH_LOG_DISABLED
  #define ADAGRAPH_LOG_INFO(msg) LOG(INFO) << " ---> [AdaGraph]: " << msg
  #define ADAGRAPH_LOG_WARN(msg) LOG(WARNING) << " ---> [AdaGraph]: " << msg
  #define ADAGRAPH_LOG_DEBUG(msg) LOG(INFO) << " ---> [AdaGraph DEBUG]: " << msg
#else
  #define ADAGRAPH_LOG_INFO(msg)
  #define ADAGRAPH_LOG_WARN(msg)
  #define ADAGRAPH_LOG_DEBUG(msg)
#endif

// Keyframe logging (can be disabled separately)
#ifndef ADAGRAPH_KEYFRAME_LOG_DISABLED
  #define ADAGRAPH_LOG_KEYFRAME(id, s_t, s_r) \
    LOG(INFO) << " ---> [AdaGraph]: Keyframe " << id \
              << ", scale: [" << s_t << ", " << s_r << "]"
#else
  #define ADAGRAPH_LOG_KEYFRAME(id, s_t, s_r)
#endif
