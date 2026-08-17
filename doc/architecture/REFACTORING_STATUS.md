# Refactoring Status

Status: Point-LIO-only cleanup in progress, 2026-08-14.

## Completed

| Item | Result |
| --- | --- |
| Production frontend | `point_lio_node` is the only ROS local-odometry node. |
| Offline path | `hybrid_slam_dataset_runner` uses the same Point-LIO frontend and downstream pipeline. |
| Prior-map localization | Retained through `PriorMapLocalization` and `RelocalizationScheduler`. |
| Frontend cleanup | Removed the retired frontend, node, launch/config assets, tests and compatibility targets. |
| Global configuration rollback | Removed `g_*` parameter implementation and global ROS bridge. |
| Regression surface | Manifests and tools are Point-LIO-only. |
| Boundary protection | `verify_core_boundaries.py` rejects reintroduction of retired legacy paths and CMake targets. |
| Core-to-adapter boundary | Publication callbacks carry `ImmutablePointCloud` and `RigidTransform3d`; PCL recovery/conversion is inside ROS and relocalization adapters. |
| `basic` boundary | `basic_core` now contains only Eigen/Lie-group and buffer utilities. Project-owned PCL point types, ROS sensor point registrations and glog initialization live under `hybrid_slam::adapter` and `hybrid_slam::logging`. |
| Frontend configuration | `FrontendConfig` owns shared physical/map semantics, `PointLIOConfig` owns only vendor tuning, and `ToVendorParameters()` is the one conversion path. |
| Online occupancy measurement | Timing histograms cover update/publish lock wait, lock hold and end-to-end latency. A 20,000-scan local synthetic run measured update hold p99 `<=386 us`, update end-to-end p99 `<=390 us`, publish hold p99 `<=51 us`, and max publish lock wait `4.034 ms`. This is not an on-robot SLA. |

## Remaining

- If a strict transitive-header boundary is required later, place the private
  Point-LIO PCL relocalization bridge behind a PImpl. Its remaining
  `basic_core` use is limited to the legacy Point-LIO/relocalization Lie-group
  bridge; no PCL or logging compatibility aliases remain.
- Profile large-map KeyFrame/Submap retention only after a concrete product
  memory constraint is provided. No memory target is assumed by this project.
