#ifndef HYBRID_SLAM_SENSOR_POINT_CLOUD_VIEW_H_
#define HYBRID_SLAM_SENSOR_POINT_CLOUD_VIEW_H_

#include <array>
#include <cstddef>
#include <cstring>

#include "hybrid_slam/geometry/transform3.h"

namespace hybrid_slam::sensor {

// Zero-allocation, non-owning view over interleaved float XYZ storage. The
// producer defines its lifetime; use ImmutablePointCloud when the storage must
// cross a callback/thread boundary.
class PointCloudView {
 public:
  PointCloudView() = default;

  static PointCloudView InterleavedFloatXyz(
      const void* data,
      std::size_t size,
      std::size_t stride_bytes,
      std::size_t x_offset_bytes,
      std::size_t y_offset_bytes,
      std::size_t z_offset_bytes,
      hybrid_slam::geometry::FrameId frame) noexcept {
    PointCloudView view;
    view.data_ = static_cast<const std::byte*>(data);
    view.size_ = size;
    view.stride_bytes_ = stride_bytes;
    view.offsets_ = {x_offset_bytes, y_offset_bytes, z_offset_bytes};
    view.frame_ = frame;
    return view;
  }

  [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
  [[nodiscard]] std::size_t size() const noexcept { return size_; }
  [[nodiscard]] std::size_t strideBytes() const noexcept {
    return stride_bytes_;
  }
  [[nodiscard]] hybrid_slam::geometry::FrameId frame() const noexcept {
    return frame_;
  }
  [[nodiscard]] const void* storageIdentity() const noexcept { return data_; }

  [[nodiscard]] bool valid() const noexcept {
    if (size_ == 0) {
      return true;
    }
    if (data_ == nullptr || stride_bytes_ < sizeof(float)) {
      return false;
    }
    for (const std::size_t offset : offsets_) {
      if (offset + sizeof(float) > stride_bytes_) {
        return false;
      }
    }
    return true;
  }

  [[nodiscard]] bool xyz(
      std::size_t index, std::array<float, 3>& point) const noexcept {
    if (!valid() || index >= size_) {
      return false;
    }
    const std::byte* element = data_ + index * stride_bytes_;
    for (std::size_t axis = 0; axis < point.size(); ++axis) {
      std::memcpy(&point[axis], element + offsets_[axis], sizeof(float));
    }
    return true;
  }

 private:
  const std::byte* data_ = nullptr;
  std::size_t size_ = 0;
  std::size_t stride_bytes_ = 0;
  std::array<std::size_t, 3> offsets_{0, 0, 0};
  hybrid_slam::geometry::FrameId frame_ =
      hybrid_slam::geometry::FrameId::kUnknown;
};

}  // namespace hybrid_slam::sensor

#endif  // HYBRID_SLAM_SENSOR_POINT_CLOUD_VIEW_H_
