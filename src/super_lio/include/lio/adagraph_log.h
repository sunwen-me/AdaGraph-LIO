#pragma once

// AdaGraph-LIO logging macros
// Can be disabled by defining ADAGRAPH_LOG_DISABLED

#ifndef ADAGRAPH_LOG_DISABLED
  #define ADAGRAPH_LOG_INFO(msg) std::cout << " ---> [AdaGraph]: " << msg << std::endl
  #define ADAGRAPH_LOG_WARN(msg) std::cerr << " ---> [AdaGraph WARNING]: " << msg << std::endl
  #define ADAGRAPH_LOG_DEBUG(msg) std::cout << " ---> [AdaGraph DEBUG]: " << msg << std::endl
#else
  #define ADAGRAPH_LOG_INFO(msg)
  #define ADAGRAPH_LOG_WARN(msg)
  #define ADAGRAPH_LOG_DEBUG(msg)
#endif

// Keyframe logging (can be disabled separately)
#ifndef ADAGRAPH_KEYFRAME_LOG_DISABLED
  #define ADAGRAPH_LOG_KEYFRAME(id, s_t, s_r) \
    std::cout << " ---> [AdaGraph]: Keyframe " << id \
              << ", scale: [" << s_t << ", " << s_r << "]" << std::endl
#else
  #define ADAGRAPH_LOG_KEYFRAME(id, s_t, s_r)
#endif
