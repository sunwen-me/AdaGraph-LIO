#ifndef ADAGRAPH_SENSOR_IMMUTABLE_POINT_CLOUD_H_
#define ADAGRAPH_SENSOR_IMMUTABLE_POINT_CLOUD_H_

#include <cstddef>
#include <memory>
#include <typeinfo>
#include <utility>

#include "adagraph/sensor/point_cloud_view.h"

namespace adagraph::sensor {

// Owns arbitrary concrete point storage while exposing only the common XYZ
// view. This avoids a mandatory format conversion/copy at frontend boundaries.
// The one shared control block is justified by cross-thread/fan-out lifetime;
// point access itself has no virtual dispatch.
class ImmutablePointCloud {
 public:
  ImmutablePointCloud() = default;

  template <typename Storage>
  [[nodiscard]] static ImmutablePointCloud Share(
      std::shared_ptr<const Storage> storage,
      PointCloudView view,
      std::size_t backing_bytes_for_diagnostics = 0) noexcept {
    if (!storage || !view.valid()) {
      return {};
    }
    ImmutablePointCloud cloud;
    cloud.storage_ = std::move(storage);
    cloud.storage_type_ = &typeid(Storage);
    cloud.view_ = view;
    cloud.backing_bytes_for_diagnostics_ =
        backing_bytes_for_diagnostics == 0
            ? view.size() * view.strideBytes()
            : backing_bytes_for_diagnostics;
    return cloud;
  }

  [[nodiscard]] bool valid() const noexcept {
    return storage_ != nullptr && view_.valid();
  }
  [[nodiscard]] bool empty() const noexcept { return view_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return view_.size(); }
  [[nodiscard]] adagraph::geometry::FrameId frame() const noexcept {
    return view_.frame();
  }
  [[nodiscard]] const PointCloudView& view() const noexcept { return view_; }
  [[nodiscard]] long ownerCountForDiagnostics() const noexcept {
    return storage_.use_count();
  }
  // Producer-supplied allocation estimate for diagnostics. It is never an
  // admission or allocation control, and generic owners use the visible span.
  [[nodiscard]] std::size_t backingBytesForDiagnostics() const noexcept {
    return backing_bytes_for_diagnostics_;
  }

  // Coarse adapter escape hatch for a concrete algorithm that can consume the
  // original immutable storage directly (for example PCL registration). Type
  // identity is checked before constructing an aliasing shared_ptr; no point
  // data is copied and point access still has no virtual dispatch.
  template <typename Storage>
  [[nodiscard]] std::shared_ptr<const Storage> ownerAs() const noexcept {
    if (!storage_ || storage_type_ == nullptr ||
        *storage_type_ != typeid(Storage)) {
      return {};
    }
    return std::shared_ptr<const Storage>(
        storage_, static_cast<const Storage*>(storage_.get()));
  }

 private:
  std::shared_ptr<const void> storage_;
  const std::type_info* storage_type_ = nullptr;
  PointCloudView view_;
  std::size_t backing_bytes_for_diagnostics_ = 0;
};

}  // namespace adagraph::sensor

#endif  // ADAGRAPH_SENSOR_IMMUTABLE_POINT_CLOUD_H_
