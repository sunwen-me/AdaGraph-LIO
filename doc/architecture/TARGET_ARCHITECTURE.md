# Target Architecture

HybridSLAM has one production local-odometry frontend: Small Point-LIO.

```text
ROS/YAML
  -> typed frontend and runtime config
  -> PointLIOFrontend
  -> backend / localization / map instances
  -> ROS publication adapters
```

The core owns typed geometry, sensor frames, map snapshots, backend scheduling
and relocalization policy. ROS, PCL message conversion and service wiring stay
at the adapter boundary.

Current boundary rules are:

- publication callbacks use `ImmutablePointCloud` and `RigidTransform3d`;
  PCL storage is recovered only by ROS/relocalization adapters;
- `basic_core` is limited to legacy Eigen/Lie-group and buffer utilities;
  PCL types and logging are owned by HybridSLAM adapter/logging targets;
- `FrontendConfig` owns shared input and map semantics, while `PointLIOConfig`
  contains only Small Point-LIO tuning and is converted once at the vendor
  boundary;
- online occupancy exposes lock wait/hold/end-to-end telemetry. Its local
  synthetic long-run test is evidence for code-path behavior only, not a
  deployment latency guarantee.

The only deferred analysis is large-map KeyFrame/Submap retention, and it is
gated on an explicit product memory constraint. No numerical memory target is
assumed.
