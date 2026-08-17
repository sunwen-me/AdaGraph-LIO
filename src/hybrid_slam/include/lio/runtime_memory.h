#ifndef LI2SUP_RUNTIME_MEMORY_H_
#define LI2SUP_RUNTIME_MEMORY_H_

#include <cstddef>

namespace LI2Sup {

// Best-effort process allocator snapshot for low-frequency diagnostics. RSS
// remains the cross-platform regression metric; glibc fields identify whether
// a retained process high-water mark is visible to the standard allocator.
struct RuntimeMemorySample {
  std::size_t resident_kib = 0;
  std::size_t glibc_allocated_bytes = 0;
  std::size_t glibc_retained_bytes = 0;
};

[[nodiscard]] RuntimeMemorySample CaptureRuntimeMemory() noexcept;

}  // namespace LI2Sup

#endif  // LI2SUP_RUNTIME_MEMORY_H_
