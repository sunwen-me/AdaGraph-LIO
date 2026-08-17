#include "lio/runtime_memory.h"

#include <fstream>
#include <string>

#if defined(__GLIBC__)
#include <malloc.h>
#endif

namespace LI2Sup {

RuntimeMemorySample CaptureRuntimeMemory() noexcept {
  RuntimeMemorySample sample;

  std::ifstream status("/proc/self/status");
  std::string key;
  while (status >> key) {
    if (key != "VmRSS:") {
      std::string ignored;
      std::getline(status, ignored);
      continue;
    }
    std::size_t resident_kib = 0;
    status >> resident_kib;
    sample.resident_kib = resident_kib;
    break;
  }

#if defined(__GLIBC__)
  const struct mallinfo2 heap = mallinfo2();
  sample.glibc_allocated_bytes = heap.uordblks + heap.hblkhd;
  sample.glibc_retained_bytes = heap.arena + heap.hblkhd;
#endif

  return sample;
}

}  // namespace LI2Sup
