#include "basic/logs.h"

namespace BASIC {

namespace {

// glog aborts on a second InitGoogleLogging call, so guard against repeated
// init_log/init_log_dir calls and against glog already initialized elsewhere.
void ensure_glog_initialized(){
  static bool initialized = false;
  if (initialized) return;
  if (!google::IsGoogleLoggingInitialized()) {
    // string literal: glog keeps the argv0 pointer, so it must stay valid
    google::InitGoogleLogging("adagraph_lio");
  }
  initialized = true;
}

}  // namespace

void init_log(){
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = true;
  ensure_glog_initialized();
}


void init_log_dir(std::string log_dir){
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_logtostderr = false;
  FLAGS_alsologtostderr = true;
  std::error_code ec;
  std::filesystem::create_directories(log_dir, ec);  // glog does not create it
  FLAGS_log_dir = log_dir;
  ensure_glog_initialized();
}

}  // namespace BASIC
