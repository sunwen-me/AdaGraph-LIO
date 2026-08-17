#include "hybrid_slam/logging/logging.h"

#include <filesystem>
#include <system_error>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace hybrid_slam::logging {
namespace {

void EnsureGlogInitialized() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  if (!google::IsGoogleLoggingInitialized()) {
    google::InitGoogleLogging("hybrid_slam");
  }
  initialized = true;
}

}  // namespace

void InitializeConsole() {
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = true;
  EnsureGlogInitialized();
}

void InitializeDirectory(std::string log_directory) {
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr = true;
  std::error_code error;
  std::filesystem::create_directories(log_directory, error);
  FLAGS_log_dir = std::move(log_directory);
  EnsureGlogInitialized();
}

}  // namespace hybrid_slam::logging
