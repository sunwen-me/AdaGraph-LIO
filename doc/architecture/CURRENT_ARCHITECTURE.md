# Current Architecture

Status: Point-LIO-only architecture, 2026-08-14.

`src/hybrid_slam` is the ROS package and installation namespace. Its
production implementation is HybridSLAM with the Point-LIO frontend.

## Runtime Path

```text
SensorBridge
  -> SlamSystem typed IMU / cloud queues
  -> PointLIOFrontend (Small Point-LIO)
  -> Point-LIO downstream pipeline
       -> BackendScheduler / MapSnapshot
       -> PriorMapLocalization / RelocalizationScheduler
       -> PublicationBridge / occupancy / map save
```

`point_lio_node` is the ROS application. `hybrid_slam_dataset_runner` drives the
same frontend and downstream path from typed text packets without ROS
transport.

`PointLIOFrontend` owns the local odometry estimate and vendor voxel map. It
emits bounded asynchronous downstream frames containing the accepted
point-to-plane update information. The downstream pipeline creates map
keyframes, estimates covariance, schedules backend optimization and publishes
snapshot products.

## Prior-Map Localization

`PriorMapLocalization` owns the loaded prior map and uses
`RelocalizationScheduler` to keep only the newest pending query. KISS-Matcher
may propose a global initialization; PCL GICP or FastVGICP verifies and tracks
the local registration. A valid result becomes a `map -> odom` alignment:

```text
T_map_odom = T_map_base_registration * inverse(T_odom_base_same_timestamp)
```

The alignment does not rewrite Point-LIO's odom-frame state or vendor map.
After a verified loop, the backend may derive `map -> odom` from the newest
optimized keyframe in the active trajectory. This is the Cartographer-style
soft feedback path: `odom -> base` remains continuous while `map -> odom`
carries the global correction when `lio.backend.drive_map_to_odom` is enabled.

## Configuration

ROS/YAML values are read at the application boundary into Point-LIO frontend
and runtime configuration objects. The deleted process-wide `g_*` parameter
surface is not present in the standard path. `FrontendConfig` owns shared
range, filtering, gravity, map-resolution and extrinsic semantics;
`PointLIOConfig` holds only vendor tuning and is converted once by
`ToVendorParameters()`. Publication callbacks use `ImmutablePointCloud` and
`RigidTransform3d`; PCL recovery/conversion stays in ROS and relocalization
adapters. `basic_core` contains only the legacy Eigen/Lie-group and buffer
utilities; PCL types, ROS sensor point registrations and glog initialization
are owned by project adapter/logging code.

## Outputs

- `/lio/odom`, `/lio/path`, `/lio/cloud_world`
- `/lio/global_map` when a prior map is active
- `/lio/backend/optimized_path`
- `/map` occupancy grid when enabled
- `map -> odom -> base` TF when navigation TF is enabled

Map saving is snapshot-based and asynchronous. Occupancy supports snapshot and
online sources. Online timing telemetry measures update/publish mutex wait,
hold and end-to-end latency; the 20,000-scan local synthetic run is code-path
evidence, not a robot latency guarantee. Large-map KeyFrame/Submap retention
is deliberately deferred until a product memory constraint exists.
