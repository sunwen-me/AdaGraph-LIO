#ifndef HYBRID_SLAM_LOGGING_LOGGING_H_
#define HYBRID_SLAM_LOGGING_LOGGING_H_

#include <string>

#include <glog/logging.h>

namespace hybrid_slam::logging {

inline constexpr char kReset[] = "\033[0m";
inline constexpr char kRed[] = "\033[31m";
inline constexpr char kGreen[] = "\033[32m";
inline constexpr char kYellow[] = "\033[33m";

void InitializeConsole();
void InitializeDirectory(std::string log_directory);

}  // namespace hybrid_slam::logging

#endif  // HYBRID_SLAM_LOGGING_LOGGING_H_
