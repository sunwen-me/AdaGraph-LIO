# Refactoring Roadmap

## Complete

- Point-LIO is the only local-odometry frontend and ROS node.
- The retired frontend, node, launch/configuration assets, tests and adapters
  are removed.
- Prior-map localization remains on the Point-LIO downstream path through
  `PriorMapLocalization` and `RelocalizationScheduler`.
- Regression manifests and offline tools are Point-LIO-only.
- Cross-module Point-LIO publication callbacks now use
  `ImmutablePointCloud` and `RigidTransform3d`; ROS publication owns PCL
  recovery/conversion. The PCL-backed relocalization implementation remains
  an algorithm adapter, not a core data contract.
- `FrontendConfig` owns shared range, filtering, gravity, map-resolution and
  extrinsic semantics. `PointLIOConfig` now holds only Small Point-LIO tuning,
  and `ToVendorParameters()` is the single vendor conversion point.
- `basic` is reduced to `basic_core`: Eigen/Lie-group and buffer utilities.
  PCL aliases and ROS point registrations are project adapter types, and glog
  initialization is owned by `hybrid_slam_logging`.
- Online occupancy has bounded, in-process timing telemetry for update and
  publish lock wait/hold/end-to-end paths. The synthetic long-run test uses
  20,000 PCL-backed scans of 384 points with `source=online`, `lazy` updates
  and ray casting: update lock-hold p99 was `<=386 us`, update end-to-end p99
  was `<=390 us`; publish materialization lock-hold p99 was `<=51 us`, while
  mutex contention produced a maximum observed publish wait of `4.034 ms`.
  These are local synthetic measurements, not a robot or deployment SLA.

## Remaining

1. Only if a strict header-dependency boundary becomes necessary, hide the
   Point-LIO pipeline's private PCL relocalization bridge behind a PImpl. It
   is no longer exposed through publication callbacks or ROS-facing APIs; its
   remaining `basic_core` dependency is only the legacy Lie-group bridge.
2. Profile large-map KeyFrame/Submap retention only when a concrete product
   memory constraint is supplied; resource diagnostics remain available.
