# Current Architecture

Status: architecture audit baseline 2026-08-11, updated through Phase 8 full recorded-data validation on 2026-08-13
Branch audited: `feature/point-lio-frontend`
Scope: the Point-LIO integration and prior-map localization architecture on
`feature/point-lio-frontend`.

This document records what the repository currently does, not what the target design should look like. The code remains the final truth. If this document and the code disagree, inspect the code first and then update this document.

## System Overview

The workspace contains three ROS 2 packages:

| Package | Current responsibility | Important build characteristics |
| --- | --- | --- |
| `src/basic` | Math aliases, manifolds, legacy PCL point/cloud aliases and buffers | `basic_core` is Eigen/Threads-only; the `basic` compatibility facade retains PCL/glog and legacy point-cloud headers. |
| `src/livox_ros_driver2` | Livox transport and ROS messages | Vendor/transport package. |
| `src/super_lio` | Super-LIO, Point-LIO vendor integration, backend, relocalization, occupancy grid, ROS adapters and applications | `src/super_lio/CMakeLists.txt` now declares explicit core/frontend/vendor/adapter/composition targets; `lio` remains an interface-only compatibility alias. |

The main executables are declared in `src/super_lio/CMakeLists.txt`:

- `super_lio_node`: Super-LIO mapping/localization path.
- `point_lio_node`: Point-LIO integration path.

The current implementation has two isolated standard local-odometry/downstream compositions. Their algorithm adapters no longer own backend/relocalization/map orchestration. The standard executables convert ROS messages in `SensorBridge`, enqueue typed values through `SlamSystem`, and advance the frontend from an application-owned processing timer. Prior-map relocalization is part of both standard downstream compositions through `PriorMapLocalization` and `RelocalizationScheduler`; there is no dedicated relocation executable. `ROSWrapper` and `LegacyInputBridge` remain external compatibility facades and are not instantiated by a standard application. The standard core/frontend/vendor/downstream targets are now independently buildable without ROS. `basic_core` supplies the lightweight Eigen/Threads math boundary, while PCL/glog remain in the `basic` compatibility facade. The instance-owned ROS reader is `legacy_parameter_bridge.h/.cpp`; process-wide `g_*` loading is exposed only through the separate compatibility `legacy_global_parameter_bridge.h/.cpp` target.

Both paths emit the same ROS-independent `adagraph::frontend::FrontendResult` defined in `src/super_lio/include/lio/frontend/frontend_result.h`. Its local pose is a reusable `StampedTransform3d` carrying a nanosecond `Timestamp`, target/source frames and `T_target_source`; the observation scan remains a callback-lifetime borrowed XYZ view. Phase 3 added separate owning move payloads for asynchronous legacy consumers while retaining this no-copy observation seam.

### Super-LIO composition

```text
super_lio_node
  ├── SensorBridge -> SlamSystem typed IMU/lidar queues
  ├── PublicationBridge / ServiceBridge
  └── application-owned lifecycle/process timer
        │
        ▼
SuperLIO
  ├── ESKF propagation and update
  ├── deskew and downsampling
  ├── OctVox scan matching/local map
  └── move-only pooled output transfer

LegacyDownstreamPipeline (owned by super_lio_node)
  ├── reliable bounded worker queue
  ├── keyframe selection
  ├── BackendScheduler -> PoseGraph / Isam2Optimizer / MapManager
  ├── PriorMapLocalization
  ├── OccupancyGridBuilder -> immutable grid snapshot -> ROS output
  ├── MapSaveScheduler for Snapshot/final point-map products
  └── bounded fragment/cache compatibility output (when explicitly enabled)

```

`SuperLIO` is defined in `src/super_lio/include/lio/super_lio.h`. It owns its ESKF, OctVox local map and reusable frontend working buffers, but no longer includes or owns AdaGraph/prior localization or the legacy global-configuration header. The standard mapping application transfers clouds and observation terms by move into an application-owned pipeline and receives cleared capacity-retaining buffers from a small pool. Its map-output behavior is composed downstream rather than inherited by a separate relocation frontend.

### Point-LIO composition

```text
point_lio_node
  ├── SensorBridge -> SlamSystem typed IMU/lidar queues
  ├── PublicationBridge / ServiceBridge
  └── application-owned lifecycle/process timer
        │
        ▼
PointLIOFrontend
  ├── SmallPointLio vendor core
  ├── typed input queues (ROS-independent values)
  ├── injected odometry output callback
  ├── bounded downstream frame queue
  └── move-only downstream callback

LegacyDownstreamPipeline (owned by point_lio_node)
  ├── BackendScheduler -> PoseGraph / Isam2Optimizer / MapManager
  ├── PriorMapLocalization
  ├── OccupancyGridBuilder -> immutable grid snapshot -> OccupancyGridPublisher
  ├── MapSaveScheduler for final point-map products
  └── Snapshot-derived final map save (no per-scan accumulated owner)
```

`PointLIOFrontend` is defined in `src/super_lio/include/lio/point_lio_frontend.h`. The Phase 3 extraction removed backend, relocalization, occupancy, publication and map-save ownership from this class. `point_lio_node` parses ROS parameters into project-owned `PointLIOConfig`, injects the odometry publisher, and composes the same transitional `LegacyDownstreamPipeline`; the one conversion to `small_point_lio::Parameters` is private to `point_lio_adapter`. `SensorBridge` owns ROS subscriptions; its Point-LIO configuration is lossless (`blind_squared=0`, a large maximum range and `filter_rate=1`) so vendor filtering is applied exactly once inside Small Point-LIO. `ROSWrapper` is no longer part of this standard composition.

`ROSWrapper`, `LegacyInputBridge` and `PublicationBridge` retain a small
external compatibility facade for callers that still use their public surface.
They are not an application composition and do not provide a second
relocalization algorithm or schedule.

## Runtime Data Flow

### Standard Super-LIO runtime path

```text
sensor_msgs / livox_ros_driver2 messages
        │
        ▼
SensorBridge
        ├── convert ROS messages to ImuData / PointCloudFrame
        └── enqueue through SlamSystem
        │
        ▼
SlamSystem::ProcessPending()
        │
        ▼
SuperLIO::process
        ├── Propagation_Undistort
        ├── DownSample
        ├── Observe: OctVox point-to-plane matching
        ├── ESKF measurement update
        ├── UpdateMap
        ├── emit FrontendResult borrowed observation
        └── move clouds/Hessian into LegacySuperFrame
                │ reliable bounded queue (capacity 16)
                ▼
        LegacyDownstreamPipeline worker
          ├── keyframe selection / typed BackendScheduler
          ├── PriorMapLocalization submission
          ├── occupancy evidence update and immutable grid snapshot handoff
          ├── PublicationBridge occupancy publication from the latest grid snapshot
          ├── global point-map accumulation
          ├── MapSaveScheduler submission after shutdown
          └── recycle cleared buffers to frontend pool
```

The real-time algorithm sequence remains in `SuperLIO::stateProcess`. In the standard node, `SensorBridge` only converts/enqueues and the application timer calls `SlamSystem::ProcessPending()`, so ROS callbacks do not call the estimator or mutate its state. `SuperLIO::Output` packages fixed-size state plus moved cloud/vector ownership and performs a short reliable queue submission. `LegacyDownstreamPipeline::superWorkerLoop` drains all submitted frames before join. If capacity is exhausted it applies explicit condition-variable backpressure rather than dropping a frame that might be accepted as a keyframe.

### Standard Point-LIO runtime path

```text
SensorBridge ROS LiDAR/IMU callback
        │
        ▼
SlamSystem typed queues
        │
        ▼
SlamSystem::ProcessPending() -> PointLIOFrontend::processInput
        │
        ▼
SmallPointLio::handle_once
        ├── Preprocess
        ├── IMU propagation
        ├── point-wise state update
        ├── SmallIVox insertion
        └── FrontendFrame
        │ move
        ▼
PointLIOFrontend bounded queue (capacity 8)
        │ overflow: drop oldest and count
        ▼
PointLIOFrontend::workerLoop
        │ move project-owned LegacyPointFrame through callback
        ▼
        LegacyDownstreamPipeline::processPointFrame
        ├── typed BackendScheduler / MapSnapshot
        ├── PriorMapLocalization
        ├── occupancy evidence update / immutable grid snapshot
        ├── OccupancyGridPublisher timer publication
        ├── ROS output
        └── accumulated map
```

The ROS sensor callback no longer runs the Point-LIO algorithm: it converts and enqueues typed values only. `SlamSystem::ProcessPending()` drains the typed ingress on the application timer and invokes `PointLIOFrontend::processInput()`. Before downstream enqueue, the adapter moves both vendor point vectors into a project-owned `LegacyPointFrame`; Super-LIO and the system target therefore do not depend on Point-LIO vendor types. Its capacity-8 queue and worker remain in the adapter. Map saving is invoked explicitly after `PointLIOFrontend::finish()` drains and joins the queue.

The moved `FrontendFrame` and bounded downstream queue are useful behavior to preserve during refactoring: slow backend/map work does not directly block the Point-LIO filter.

### Transitional frontend observation seam

```text
SuperLIO::stateProcess after UpdateMap --------┐
                                               ├── FrontendResultCallback
PointLIOFrontend before downstream queue move -┘
```

`SuperLIO::emitFrontendResult` and `PointLIOFrontend::emitFrontendResult` now expose typed `T_odom_base`. The conversion uses the configured static `T_base_imu` and normalizes float legacy rotations once at the adapter boundary. Super's borrowed full scan remains in IMU coordinates and Point's vendor registered scan remains in odom coordinates; `PointCloudView::frame` makes that distinction explicit without adding a point-cloud copy to the synchronous estimator callback.

### Phase 2 core boundary now present

ROS-independent headers now live under `src/super_lio/include/adagraph/` as a transitional location before build-target extraction:

- `common/timestamp.h` stores signed nanoseconds and rejects non-finite/out-of-range legacy seconds;
- `common/domain_ids.h` supplies type-distinct session/trajectory/submap/keyframe/node/constraint IDs;
- `geometry/transform3.h` defines `FrameId`, `RigidTransform3d`, time-invariant `StaticTransform3d` extrinsics and strict same-time/frame-checked `StampedTransform3d` composition;
- `sensor/imu_data.h` and `sensor/point_cloud_frame.h` define ROS-independent input values; `PointCloudFrame` is move-only and transfers its vector allocation without copying;
- frontend/backend/relocalization config headers provide construction-time validation without ROS.

`SuperLIO::init` still translates legacy global parameters into `FrontendConfig`. `PointLIOFrontend::init` instead receives a validated project-owned `PointLIOConfig` and converts it privately to vendor `Parameters`. `AdaGraphBackend::init` snapshots legacy globals into `BackendConfig`, while an explicit config constructor supports independent instances. `PriorMapLocalization::init` snapshots all current `g_prior_map_loc_*` values into `RelocalizationConfig`; its runtime reads that member rather than process-wide globals. Standard applications read instance-owned values through `legacy_parameter_bridge.cpp`; the process-wide loader is isolated in `legacy_global_parameter_bridge.cpp` and its dedicated compatibility header. Compatibility adapter parameters are loaded by `LegacyInputBridge::Config::FromNode` and `PublicationBridge::Config::FromNode`; Super-LIO/frontend legacy globals remain transitional.

The Phase 2 types add no per-point work, virtual dispatch or queue/mutex. Frontend observation performs one legacy-seconds conversion and one pose conversion only when the optional observer is installed; the borrowed cloud identity contract is unchanged.

### Phase 3 frontend/downstream boundary now present

`adagraph::mapping::KeyFrame` and `adagraph::sensor::ImmutablePointCloud` define the immutable cross-thread ownership contract. Point-LIO moves vendor vectors into `LegacyPointFrame`, while Super-LIO transfers its PCL clouds and Hessian vector into `LegacySuperFrame` and reuses them through a bounded buffer pool. Neither transfer deep-copies the point data.

Neither standard frontend includes or owns `AdaGraphBackend`, `PriorMapLocalization`, occupancy/global-map orchestration or standard map IO. Point shutdown is `vendor flush -> frontend queue drain/join -> map save`; Super shutdown is `system queue drain/join -> map save`. The retired relocation subclass no longer inherits protected frontend or map state.

### Phase 4 backend boundary now present

The ROS-independent `adagraph_backend_core` target now owns typed `PoseGraphNode`, `Constraint`, `OptimizationResult`, `ConstraintBuilder`, `OdometryCovarianceEstimator` and the reliable bounded `BackendScheduler`. `PoseGraph` is mutex-free and mutated only by the scheduler worker. Accepted immutable KeyFrames and typed global-pose requests cross the queue by moved value; accepted work drains before shutdown. The scheduler creates odometry constraints in deterministic order and associates asynchronous global observations by timestamp on its single writer.

`adagraph_isam2_optimizer` is a separate GTSAM-only target behind the coarse `Optimizer::apply(GraphUpdate)` interface. A `GraphUpdate` contains only newly committed nodes/constraints, so the keyframe path does not copy the complete graph. Optimization results return typed `T_map_base` values and are applied by `PoseGraph` only when the input graph version matches. Minimal disabled `LoopDetector`/`LoopVerifier` seams exist, but no loop algorithm is claimed.

The typed scheduler/MapManager path is now the only graph owner in the standard
downstream and application targets. `AdaGraphBackend` is compiled into the
explicit `adagraph_legacy_backend` compatibility archive only; it remains
available for external legacy callers and its standalone tests. The final Phase
4 regression remains historical parity evidence from the
period when both graph implementations were exercised.

### Phase 5 map/snapshot boundary now present

`adagraph_mapping_core` now provides `MapManager`, `MapSnapshot` and `SnapshotProductBuilder`. The backend worker is the only writer of PoseGraph topology and MapManager payload/optimized poses. After each committed work item it publishes a complete immutable Snapshot with an atomic `shared_ptr` exchange; KeyFrame clouds retain shared immutable storage and are not deep-copied per version. Concurrent-reader tests verify monotonic, internally consistent versions without locking PoseGraph.

Each current session/trajectory has bounded Submap lifecycle metadata. `BackendConfig::submap_max_keyframes` defaults to 100; a full Submap becomes `kFinished` and the next KeyFrame opens the next monotonic `SubmapId`. This is lifecycle grouping only and does not change frontend local-map or graph optimization mathematics.

`/lio/backend/optimized_path` now consumes Snapshot pose metadata. The scheduler observer performs only an atomic ownership handoff; a ROS timer coalesces versions and creates the Path outside the graph writer. The unused legacy full-trajectory back-substitution was removed. Shutdown map save queues both the final Point-LIO Snapshot-derived PCD and a versioned optimized-KeyFrame comparison PCD on `MapSaveScheduler`; both are materialized/voxelized/written off the pipeline thread. Super-LIO retains a bounded accumulated fragment buffer only when the explicit legacy fragment-save mode is enabled; its fragment writes and merge remain reliable asynchronous IO requests.

### Phase 6 localization/system boundary now present

`adagraph_localization_core` now provides ROS-independent `LocalizationMap`, `RelocalizationQuery`, `RelocalizationResult`, `CandidateRetriever`, `RegistrationVerifier`, `ComposedRelocalizer` and `RelocalizationScheduler`. The scheduler has one active query and one replaceable pending query; newer work requests cooperative cancellation of obsolete active work. `BuildLocalizationMap(MapSnapshot)` materializes a map-frame immutable localization cloud as an explicit background operation.

`adagraph_system_core` owns `GlobalAlignmentState`. A commit requires same-time, frame-checked `T_map_base` and `T_odom_base`, then derives `T_map_odom = T_map_base * inverse(T_odom_base)`.

The standard Super-LIO and Point-LIO compositions no longer let the localization worker or backend write ROS correction state directly. `PriorMapLocalization` emits a fixed-size typed `RelocalizationUpdate` into `GlobalAlignmentUpdateMailbox`; the application-owned downstream/system context commits the latest event to `GlobalAlignmentState`. Optional backend correction is gated/smoothed by the legacy localization policy, returned to that same system writer and committed with source `kBackendOptimization`. `ROSWrapper::set_map_to_odom` is downstream cache/publication only, and backend initialization reads `GlobalAlignmentState`, not `ROSWrapper`.

Production prior-map queries now pass through `RelocalizationScheduler` and the concrete `ComposedRelocalizer` adapters: `PclKissCandidateRetriever` recovers the immutable PCL source/map owners, while `PclRegistrationVerifier` crops the target and runs FastVGICP or PCL GICP. `PriorMapLocalization` keeps no mutable persistent prior-map owner; `LocalizationMap` owns the loaded/downsampled target and the crop grid stores only derived indices accessed through a const PCL alias. The adapters do not rematerialize the current query cloud at the typed boundary. For deployments that enable it, the coarse KISS input uses stable serial voxel reduction and evaluates the configured ratio-test mode plus its alternate; candidate/verifier diagnostics identify which strict VGICP/GICP check accepted or rejected each path. This makes startup/recovery latency a reproducible gate without relaxing the legacy stateful fitness/jump gate, which remains above the generic verifier in `PriorMapLocalization`. Non-interruptible vendor calls use finish-active/publish with one newest pending query. Scheduler shutdown reports queue-wait, processing and end-to-end latency, while the prior-map adapter reports startup/LOST KISS attempts, outcomes and first-success delay; both standard frontend regressions gate these diagnostics. This is the sole supported prior-map relocalization implementation.

`RelocalizationConfig::max_local_history_points` bounds retained local input
before it becomes the KISS/registration frame history. Each frame is voxelized
at the existing source leaf size and receives an even share of the total
budget; the final registration source remains independently bounded by
`max_source_points`. The standard ROS/YAML and DatasetRunner paths parse the
field, while the retained global-compat adapter derives it from its existing
source-point budget rather than adding another global parameter. In the final
recorded Point-LIO prior-map run, this reduced retained history from 406,776
points / 13,016,832 B to 9,002 points / 288,064 B. This changes only the
bounded history representation, not Point-LIO local odometry, KeyFrames or the
final registration point cap.

### Phase 7 occupancy/map-IO boundary now present

`adagraph_mapping_core` now includes the ROS-independent `OccupancyGridBuilder`,
which owns mutable evidence only in its single-writer execution context and
returns an immutable `OccupancyGridSnapshot`. It accepts borrowed XYZ views for
online/static clouds, supports height/range filtering, ray casting, inflation
and bounded extents, and can rebuild static evidence directly from an immutable
`MapSnapshot` by transforming shared KeyFrame cloud views into the map frame.
The builder does not include ROS, PCL or publisher/timer code.

`OccupancyGridPublisher` is now an adapter: it loads ROS parameters, performs
ground segmentation and the required PCL/body transform, serializes an
immutable grid snapshot into `nav_msgs::msg::OccupancyGrid`, and publishes it.
The mutable builder remains behind a short adapter mutex; dense snapshot
materialization is handed off through an atomic latest-only `shared_ptr`, and
ROS message conversion/publication happens after releasing that mutex. The
explicit `incremental`, `rebuild` and `lazy` policy is configured through
`lio.occupancy_grid.update_policy`; the default is `lazy` to coalesce dense
publication work. `lio.occupancy_grid.source` selects `snapshot` or the
explicit `online` compatibility path. Enabled standard configurations use
`snapshot`: `PublicationBridge` hands the latest immutable `MapSnapshot` to an
adapter-owned rebuild worker, which rebuilds the core grid without holding a
PoseGraph/frontend lock; the ROS timer only publishes the latest materialized
grid. Snapshot and online are distinct products: Snapshot uses optimized
KeyFrame payloads and has no per-scan sensor-origin history for free-ray
evidence, while online consumes downstream scans and can raycast free cells.
Both preserve the configured topic/frame/resolution/value encoding, but their
origin, extents and occupied/free/unknown counts are not expected to be byte
equal. `scripts/record_occupancy_summary.py` and
`scripts/compare_occupancy_summaries.py` record this contract on both
frontends. Prior-map localization retains its static PCL reset path, and the
configured map/odom frame is selected consistently with the source mode.

The recorded-data Snapshot rebuild budget run
`/tmp/adagraph_phase7_occupancy_budget_regression_final_20260813` exercised real
high-frequency snapshot handoffs with `lio.backend.keyframe_max_frames=1`.
Point-LIO produced 226 handoffs/requests, 216 completed rebuilds and 10
superseded cancellations; Super-LIO produced 224 handoffs/requests and 224
completed rebuilds. Both runs had zero shutdown cancellations, obsolete
discards, budget overruns and failures. Peak scratch was 689,928 B for
Point-LIO and 725,896 B for Super-LIO against the configured 64 MiB budget,
and the machine-readable terminal-outcome accounting passed.

`adagraph_map_io` provides the ROS/PCL-independent reliable bounded
`MapSaveScheduler`. A request retains either an immutable `MapSnapshot` or an
immutable point-cloud product or an explicit PCD fragment set; the worker
performs Snapshot materialization, deterministic voxel-centroid reduction,
binary PCD fragment reads and atomic temporary-file rename. Snapshot
comparison output, standard final point-map save and standard Super fragment
writes/merge all use this worker. The prior-map loader remains a synchronous
PCL startup adapter. Production save regression `phase7_20260812_map_save_regression`
validated Super fragment FIFO merge and final/Snapshot outputs, Point-LIO
final/Snapshot outputs, and independent `pcl::io::loadPCDFile` readback.

### Phase 8 ROS adapter, lifecycle and build boundary now validated

`SlamSystem` is the lifecycle/input authority for the standard compositions:
`SensorBridge` converts ROS messages to typed `ImuData`/`PointCloudFrame`
values, the application-owned 2 ms timer calls `ProcessPending()`, and
`FinishTrajectory()` drains frontend and downstream work before the application
requests final map products. `DatasetRunner` uses the same `SlamSystem`,
frontend, downstream and map-snapshot composition without importing ROS.

`scripts/rosbag_to_typed_text.py` is a read-only ROS bag boundary tool. It
reuses the SensorBridge filtering/time semantics and externally sorts packets by
sensor time; `scripts/compare_tum_trajectories.py` compares the resulting
offline trajectory to the recorded ROS trajectory at reference timestamps.
The full `room_01` bag produced 10,057 IMU packets, 728 clouds and 10,785 typed
packets. The ROS-free runner completed both full frontend replays with status
zero and no rejected packets. The retained ROS baselines and parity JSON are
under `log/refactoring_baselines/phase8_20260813_ros_free_dataset/`:

| frontend | offline poses | ROS reference poses | matched | translation RMSE / max | rotation max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Super-LIO | 719 | 494 | 494/494 | 0.012104 / 0.029173 m | 0.008382 rad |
| Point-LIO | 713 | 1007 | 1007/1007 | 0.003662 / 0.012516 m | 0.003103 rad |

The offline runner's `--save-map` path explicitly calls the frontend-specific
final save operation after lifecycle drain. Both frontends produced finite
binary final/Snapshot PCD products; Super-LIO additionally produced its FIFO
fragment set and merged final map. The empty-tree build/install was verified in
`/tmp/adagraph_phase8_clean_build3` and `/tmp/adagraph_phase8_clean_install3`;
clean CTest passed 13/13 with `ROS_LOG_DIR` redirected to `/tmp`, and
`verify_core_boundaries.py` passed for 49 files and 13 core/frontend targets.

## Module Dependency Graph

### Current source-level dependencies

```text
SuperLIO
  ├── ESKF
  ├── OctVoxMap
  └── LegacySuperFrame output callback

PointLIOFrontend
  ├── SmallPointLio
  └── LegacyPointFrame output callback

ROSWrapper (fallback facade)
  ├── LegacyInputBridge
  └── PublicationBridge

point_lio_node
  ├── PointLIOFrontend
  └── LegacyDownstreamPipeline
       ├── typed BackendScheduler / MapSnapshot
       ├── PriorMapLocalization
       ├── OccupancyGridBuilder / immutable grid snapshot
       ├── OccupancyGridPublisher
  └── MapSaveScheduler

super_lio_node
  ├── SuperLIO
  └── LegacyDownstreamPipeline
       ├── reliable bounded worker/buffer pool
       ├── typed BackendScheduler / MapSnapshot
       ├── PriorMapLocalization
       ├── OccupancyGridBuilder / immutable grid snapshot
       ├── OccupancyGridPublisher
       └── MapSaveScheduler

PriorMapLocalization
  ├── KissMatcherRelocalizer
  ├── FastVGICP / PCL GICP
  ├── prior-map clouds and crop index
  ├── private predicted alignment for its tracking policy
  └── typed RelocalizationUpdate -> GlobalAlignmentUpdateMailbox

OccupancyGridBuilder
  └── borrowed clouds or immutable MapSnapshot -> OccupancyGridSnapshot

MapSaveScheduler
  └── immutable MapSnapshot/point-cloud product -> background PCD IO

GlobalAlignmentState
  ├── semantic T_map_odom authority for standard compositions
  ├── fixed-size latest-only mailbox input from localization worker
  └── immutable state -> ROS TF/publication cache

adagraph_legacy_backend (compatibility only)
  ├── legacy GTSAM ISAM2 facade
  ├── raw frontend pose history
  └── optional prior-map observations
```

No literal include strongly connected component is present in the active
composition. The remaining runtime/ownership debt is deliberately isolated:

1. The former `SuperLIO -> ROSWrapper -> ESKF` scheduling cycle is removed.
   Standard nodes own estimator scheduling through `SlamSystem`; an external
   facade may inject a predictor but has no standard application schedule.
2. The former `PriorMapLocalization -> MapObservation -> AdaGraphBackend -> ROS
   map-to-odom -> PriorMapLocalization` authority cycle is removed on standard
   compositions. Snapshot-based backend refinement reads committed typed
   `MapSnapshot` poses, may update the localizer's private tracking prediction,
   and returns a value committed only by the system-owned `GlobalAlignmentState`.
3. The former relocation subclass and its protected-state dependency were
   retired. Arbitrary-start/recovery is measured only through the standard
   prior-map path.
4. The old `ROSWrapper::set_initial_data` reference-capture hazard is removed.
   `LegacyInputBridge` stores an optional value-owned initial pose and the
   application consumes it explicitly.

## Core Classes

| Class | Location | Actual responsibilities |
| --- | --- | --- |
| `SuperLIO` | `src/super_lio/include/lio/super_lio.h` | Frontend ESKF, deskew, matching and OctVox local map. Standard output is a moved owning frame; map IO is composed downstream. |
| `ESKF` | `src/super_lio/include/lio/ESKF.h:12` | IMU prediction and measurement-state update. It reads process-wide parameters and exposes many internal state getters; `init_` is public at `ESKF.h:88`. |
| `OctVoxMap` | `src/super_lio/include/lio/OctVoxMap/OctVoxMap.hpp:133` | Super-LIO voxel local map and nearest-neighbor lookup. Storage uses a list plus hash map of list iterators at `OctVoxMap.hpp:243-247`. |
| `PointLIOFrontend` | `src/super_lio/include/lio/point_lio_frontend.h` | Point-LIO vendor algorithm and bounded frame scheduling over ROS-independent typed input. It receives `PointLIOConfig`; its public API exposes neither ROS nor vendor parameter types. |
| `LegacyDownstreamPipeline` | `src/super_lio/include/lio/system/legacy_downstream_pipeline.h` | Transitional application-owned consumer for both frontends; owns typed backend/snapshot submission, prior localization, occupancy, visualization/global-map/save behavior and the reliable Super worker. |
| `SmallPointLio` | `src/super_lio/3rdparty/small_point_lio/src/small_point_lio/small_point_lio.h` | Point-LIO vendor orchestration. It is constructed from `Parameters`; only `point_lio_adapter` materializes that vendor type from `PointLIOConfig`. |
| `Estimator` | `src/super_lio/3rdparty/small_point_lio/include/small_point_lio/estimator.h:16` | Point-LIO estimator and SmallIVox map. It stores a raw `Parameters*` and shared `SmallIVox`. |
| `BackendScheduler` / `PoseGraph` | `src/super_lio/include/adagraph/backend/` | Reliable bounded accepted-work queue, single-writer typed graph topology, constraint construction and version-checked optimizer-result commit. No ROS or point registration. |
| `MapManager` / `MapSnapshot` | `src/super_lio/include/adagraph/mapping/` | Single-writer durable KeyFrame/optimized-pose/Submap owner and immutable versioned multi-reader boundary. |
| `SnapshotProductBuilder` | `src/super_lio/include/adagraph/mapping/snapshot_products.h` | Builds lightweight optimized trajectories or explicitly materialized map-frame point maps without ROS/PCL dependencies. |
| `LocalizationMap` / `ComposedRelocalizer` / `RelocalizationScheduler` | `src/super_lio/include/adagraph/localization/` | ROS-independent immutable map/query/result contracts, production PCL/KISS candidate + GICP/VGICP verification composition and one-active/one-pending scheduling. Vendor calls use finish-active/publish; cancellable implementations use cooperative cancellation. |
| `GlobalAlignmentState` / `GlobalAlignmentUpdateMailbox` | `src/super_lio/include/adagraph/system/global_alignment_state.h` | Versioned system authority for `T_map_odom`, lock-free immutable reads and fixed-size latest-only worker-to-system handoff. |
| `Isam2Optimizer` | `src/super_lio/include/adagraph/backend/isam2_optimizer.h` | GTSAM-only incremental optimizer adapter consuming small immutable `GraphUpdate` values. |
| `AdaGraphBackend` | `src/super_lio/include/lio/adagraph_backend.h` | Transitional rollback/reference ISAM2 facade and optional legacy localization-factor association. Keyframe policy and covariance estimation no longer belong to it. |
| `AdaGraphFeatureExtractor` | `src/super_lio/include/lio/adagraph_feature_extractor.h:13` | Scene scatter and Hessian-derived statistics from a mutable cloud and copied point-Hessian terms. |
| `PriorMapLocalization` | `src/super_lio/include/lio/prior_map_localization.h:24` | Transitional PCL adapter: prior map loading, local odom-submap construction, KISS retrieval, VGICP/GICP verification, tracking prediction and typed result emission. It no longer publishes or owns external `T_map_odom`. |
| `OccupancyGridBuilder` | `src/super_lio/include/adagraph/mapping/occupancy_grid.h` | ROS-independent single-writer evidence storage/building, ray casting, inflation, bounded grid snapshot and immutable MapSnapshot rebuild. |
| `MapSaveScheduler` | `src/super_lio/include/adagraph/io/map_io.h` | Reliable bounded background materialization, optional voxel reduction and atomic binary PCD installation for immutable map products. |
| `OccupancyGridPublisher` | `src/super_lio/include/ros/occupancy_grid_publisher.h:19` | ROS parameter loading, ground/PCL adapter work, immutable grid snapshot to `nav_msgs` conversion, timer and publication. It no longer owns the grid representation. |
| `LegacyInputBridge` | `src/super_lio/include/ros/legacy_input_bridge.h` | External compatibility ROS subscriptions, typed-to-legacy sensor conversion, bounded buffers, initial-pose/service events and explicit IMU prediction hook. |
| `PublicationBridge` | `src/super_lio/include/ros/publication_bridge.h` | Compatibility and standard ROS publishers, TF, occupancy adapter, map snapshot handoff and publication cache. |
| `ROSWrapper` | `src/super_lio/include/ros/ROSWrapper.h:30` | Fallback ROS facade delegating compatibility input to `LegacyInputBridge` and output/derived-map behavior to `PublicationBridge`; no active application instantiates it. |

### Current backend semantics

The standard path contains one graph/map implementation:

- `BackendScheduler -> PoseGraph -> ConstraintBuilder -> Isam2Optimizer -> MapManager` owns topology, optimized poses and versioned `MapSnapshot` publication;
- optional backend refinement reads the committed Snapshot and submits a candidate to `GlobalAlignmentState` through the downstream writer;
- `adagraph_legacy_backend` is a separate compatibility archive and is not linked by `adagraph_legacy_downstream`, either standard node or `adagraph_dataset_runner`.

The legacy facade still has standalone tests and rollback callers, but it is not
part of standard graph execution. There is still no complete loop-closure
implementation or spatial submap optimizer.

## Ownership Model

| State/data | Current owner(s) | Audit finding |
| --- | --- | --- |
| Super-LIO ESKF | `SuperLIO` | Estimator is owned by the standard frontend composition. |
| Super local voxel map | `SuperLIO::voxel_map_` | Appropriate conceptual owner, but not hidden behind a frontend boundary. |
| Point local voxel map | `Estimator::ivox_` | Shared ownership is not required by the observed use. |
| Typed backend graph/map | `BackendScheduler` exclusively owns `PoseGraph` and calls the single-writer `MapManager`; readers atomically load `shared_ptr<const MapSnapshot>` | Graph topology, durable KeyFrame payload, optimized poses and Submap metadata share one committed version boundary. The legacy facade is outside this owner set. |
| Prior point map | `LocalizationMap` shared through `PriorMapLocalization`; the adapter keeps only a const PCL alias and derived crop index | The loaded/downsampled registration target has one immutable persistent owner; crop/registration working clouds remain per-query products. |
| Occupancy evidence/cells | `OccupancyGridBuilder` in the downstream single-writer context; immutable `OccupancyGridSnapshot` is atomically handed to the ROS adapter | The ROS publisher no longer owns the representation. Its mutex protects builder mutation/snapshot materialization only; dense message conversion is outside the lock. |
| Accumulated point map | `LegacyDownstreamPipeline` owns only the bounded Super fragment buffer in explicit compatibility mode; Point-LIO derives final output from `MapSnapshot` | Standard Point-LIO no longer retains a per-scan accumulated owner. |
| `T_map_odom` | `GlobalAlignmentState` is semantic owner; `PublicationBridge` is a publication cache | Standard backend/localization consumers read the typed system state. Cache parity is measured at shutdown. |
| Trajectory poses | `MapManager` owns authoritative optimized `T_map_base`; frontend/ROS retain derived values, while compatibility callers may retain rollback values | Snapshot version is authoritative on all standard paths. |
| Configuration | instance-owned readers for standard paths; process-wide `g_*` loader only in `legacy_global_parameter_bridge` and explicit legacy algorithm adapters | Point-LIO frontend/node/downstream consume construction-time config snapshots; Super-LIO legacy math remains compatibility debt. |

Large point clouds are generally represented as mutable PCL shared pointers. This makes copies cheap in some paths but does not communicate immutability or a single writer. The intended replacement is unique mutable ownership while building a cloud, `std::span<const Point>` for same-thread views, and `shared_ptr<const CloudBuffer>` only for large immutable data that genuinely crosses threads or fans out.

## Thread Model

### Super-LIO

The standard nodes use `rclcpp::spin` with one mutually-exclusive adapter group. Sensor callbacks only convert/enqueue; a 2 ms application timer advances the typed frontend/system queue.

```text
ROS executor
  └── mutually-exclusive adapter callback group
       ├── SensorBridge conversion and typed enqueue
       ├── PublicationBridge/ServiceBridge callbacks
       └── SlamSystem processing timer -> SuperLIO typed input drain

LegacyDownstreamPipeline Super worker
  ├── keyframe policy / AdaGraph
  ├── immutable KeyFrame adaptation / reliable BackendScheduler submission
  ├── prior-localization submission
  ├── occupancy builder mutation and latest immutable grid handoff
  ├── visualization publication
  ├── global-map accumulation
  └── cloud/Hessian buffer recycling

PriorMapLocalization
  ├── one condition-variable KISS/VGICP worker
  └── fixed-size latest-only GlobalAlignmentUpdateMailbox handoff

LegacyDownstreamPipeline system execution context
  └── sole GlobalAlignmentState commit point for relocalization/backend updates

BackendScheduler
  └── one condition-variable single-writer graph/ISAM2/MapManager worker

ROS executor default callback group
  ├── 250 ms latest-only Snapshot optimized-path timer
  └── occupancy timer: immutable grid snapshot -> ROS message

MapSaveScheduler
  └── reliable bounded worker: Snapshot/product/fragment materialization and
      binary PCD read/write -> atomic install

TBB
  └── parallel work inside deskew/matching paths
```

The Super queue is capacity 16, condition-variable driven, reliable and draining. Queue algorithms run outside its mutex. The final Phase 3 run reached depth one with zero capacity waits. The prior-localization worker also uses a condition variable and joins during shutdown; neither uses sleep polling.

### Point-LIO standard composition

```text
ROS executor callback
  └── SensorBridge conversion and typed enqueue

SlamSystem processing timer
  └── PointLIOFrontend::processInput -> SmallPointLio processing

PointLIOFrontend worker
  └── backend, grid, relocalization submission, output and map accumulation

PriorMapLocalization worker
  ├── KISS and VGICP/GICP
  └── fixed-size latest-only GlobalAlignmentUpdateMailbox handoff

PointLIOFrontend/system worker
  └── sole GlobalAlignmentState commit point for relocalization/backend updates
```

Both frontends submit accepted immutable KeyFrames to the common reliable `BackendScheduler`; Point retains its capacity-8 pre-acceptance latest-frame policy and Super retains reliable raw-frame backpressure because its acceptance still occurs on the consumer. The scheduler has its own capacity from `BackendConfig`, runs graph/optimizer/map work outside its queue mutex and drains before join. Its Snapshot observer is constrained to an O(1) atomic handoff; ROS message allocation runs on a coalescing timer. Production prior-map queries use the one-active/one-pending `RelocalizationScheduler`; non-interruptible PCL vendor calls finish and publish the active result, while newer pending work replaces stale pending work. A general priority/cancellation/coalescing policy for loop, map representation and IO tasks is still absent.

### Important locks

- `LegacyInputBridge` sensor/prediction/initial-pose mutexes on the compatibility path.
- `PublicationBridge` map-to-odom mutex and `LegacyInputBridge` manual-relocalization atomic.
- `PointLIOFrontend` downstream queue mutex and condition variable.
- `LegacyDownstreamPipeline` Super queue not-empty/not-full condition variables plus a short buffer-pool mutex.
- `BackendScheduler` reliable queue mutex/CVs; graph and ISAM2 work execute after releasing the queue mutex.
- `PriorMapLocalization` work mutex/CV, observation mutex and correction-state mutex.
- `GlobalAlignmentUpdateMailbox` has one short mutex protecting only an optional fixed-size typed update and four counters; no cloud or registration work runs under it.
- `OccupancyGridPublisher` mutex protects only builder mutation and dense
  snapshot materialization; immutable Snapshot rebuild runs on a dedicated
  adapter worker and ROS message conversion/publication happen after releasing
  the mutex. Online ray casting still runs in the downstream/adapter context.
  The recorded Snapshot budget run bounded rebuild scratch below 1 MiB, but
  has not measured online-path lock hold time, per-update P50/P95/P99,
  downstream/ROS interference, queue depth or long-run representation growth.
  Those measurements are required before an `online` source is enabled for a
  production deployment; they are not a blocker for the current Snapshot
  default.
- `MapSaveScheduler` queue mutex protects moved request bookkeeping only;
  Snapshot materialization, voxel reduction and filesystem IO run outside it.

No single project-wide global mutex was found, but the process-wide mutable `g_*` parameter set is global mutable state.

## Pose / Coordinate Frame Semantics

The intended current navigation frame chain is:

```text
map -> odom -> base_footprint -> base_link -> lidar / imu
```

The Super-LIO and Point-LIO filters operate in a continuous local `odom` frame. Prior-map localization estimates global alignment and must not overwrite the ESKF/IEKF or its odom-frame local voxel map.

The desired composition already implicit in the current localization design is:

```text
T_map_base = T_map_odom * T_odom_base
```

However, current types do not enforce this:

- `BASIC::SE3` exposes duplicated public `T_`, `R_` and `t_` state at `src/basic/include/Manifold/Manifold.h:259-334`.
- `src/basic/include/basic/ds.h:32-64` and `src/super_lio/include/common/ds.h:42-116` define overlapping state/pose types.
- Variables use `odom_lidar`, `odom_body`, robot, IMU and base terminology inconsistently.
- Legacy `g_odom_robo` remains ambiguously named, but frontend initialization converts it once into typed static `T_base_imu = {R_imu_base^T, p_base_imu}`; a known upside-down Mid-360 transform test proves the existing ROS formula and typed composition agree.
- The standard path enforces typed `T_map_odom` authority in `GlobalAlignmentState`; the localization adapter's private prediction and ROS publisher cache are not read as semantic authorities.

The numeric pipeline may be internally consistent for current configurations, but the type system cannot detect a frame-direction error when changing frontend or sensor mounting.

## Map / KeyFrame State Ownership

There is an immutable `adagraph::mapping::KeyFrame` contract with strong IDs, `T_odom_base`, quality and immutable cloud lifetime. `BackendScheduler` consumes it and commits graph topology plus payload/optimized poses into `MapManager`. Runtime `MapSnapshot` values contain session/trajectory/Submap identities, `T_odom_base`, `T_map_base`, constraints and shared immutable cloud owners. `BuildLocalizationMap` and `OccupancyGridBuilder::rebuildFromSnapshot` derive immutable map-frame products from a Snapshot. The standard path no longer has a duplicate AdaGraph graph; only explicit compatibility callers and the Super fragment-save mode retain rollback state.

Current map-like states are:

1. Super-LIO `OctVoxMap` local map.
2. Point-LIO `SmallIVox` local map.
3. Super-LIO's bounded accumulated fragment buffer (only when legacy
   fragment saving is enabled); Point-LIO final output is Snapshot-derived.
4. `PriorMapLocalization`'s immutable `LocalizationMap` owner and its
   cropped/local-frame working clouds; the crop index stores only derived
   point indices.
5. `OccupancyGridBuilder` hash map of observed evidence cells plus immutable `OccupancyGridSnapshot` products.
6. Versioned `MapSnapshot` plus derived optimized path/comparison PCD through `MapSaveScheduler`.
7. Serialized ROS global-map/path messages.
8. Legacy AdaGraph/GTSAM node pose arrays retained only inside explicit compatibility callers.

Keyframe admission is system-owned `KeyFramePolicy`. Accepted frames are adapted to base-frame immutable KeyFrames and reliably submitted to the typed graph; compatibility-only legacy pose/time arrays are not part of standard execution.

## ROS Dependencies

ROS is an adapter boundary for the standard compositions:

- Standard `super_lio_node` and `point_lio_node` use `SensorBridge`, `PublicationBridge`, `ServiceBridge` and `SlamSystem`; they do not include or instantiate `ROSWrapper`.
- `LegacyInputBridge` and `ROSWrapper` remain external compatibility facades; neither is instantiated by a standard node or provides a relocation execution path.
- `point_lio_node` parses ROS parameters into `PointLIOConfig`; `point_lio_adapter` alone converts it to `small_point_lio::Parameters`. The vendor core is not given an `rclcpp::Node`; its `parameters.cpp` only validates the fully constructed value object.
- `OccupancyGridPublisher` still performs ROS-side ground/PCL adaptation, but
  core occupancy evidence/building is now in `OccupancyGridBuilder`; its timer
  consumes an immutable snapshot rather than owning the grid representation.
- `MapSaveScheduler` and `OccupancyGridBuilder` are ROS-independent targets;
  standard applications wire them through explicit callbacks and
  `PublicationBridge`. `ROSWrapper` only delegates to that bridge pair.
- `basic_core` links only Eigen and Threads. PCL/glog and legacy point/cloud aliases remain in the installed `basic` compatibility facade, so legacy PCL consumers are intentionally not yet lightweight.
- The dataset runner can call the typed system without ROS for smoke coverage,
  while the full regression script still launches ROS nodes and replays rosbag
  data. See `scripts/run_m2dgr_eval.sh:60-90`.

`ROSWrapper` remains a compatibility facade because its old public surface is
still useful to external rollback callers, but it is no longer an adapter God
Object: the implementation is a small delegation layer and parameter loading
is performed by `LegacyInputBridge::Config::FromNode` and
`PublicationBridge::Config::FromNode`.

## Performance-sensitive Data Flow

### Super-LIO critical path

The remaining real-time/front-system boundary steps are:

1. Standard ROS message conversion in `SensorBridge`.
2. Deskewed output cloud.
3. Downsampled cloud.
4. Move the two cloud pointers and Hessian vector through one bounded queue operation; acquire a cleared capacity-retaining replacement buffer pair.

Prior-localization copies, occupancy allocation/raycast, visualization transforms and map accumulation/save now occur on `LegacyDownstreamPipeline`'s Super worker. The output path added no point copy, but double buffering raised the final measured peak RSS by 4.61% versus Phase 2. Normal load had queue depth one and zero backpressure waits. `saveSuperMap()` runs only after the worker drains and joins.

At accepted-keyframe frequency, the Phase 4 adapter performs one downsampled IMU-cloud to base-cloud transform so immutable KeyFrame storage has truthful coordinates. It then shares that frozen allocation across the backend queue; there is no per-consumer deep copy. The final Phase 4 Super run retained queue depth one/zero waits, 0.153 ms mean legacy downstream time and 110576 KiB peak RSS.

The most significant algorithmic issue found is `SuperLIO::Propagation_Undistort` at `src/super_lio/src/lio/super_lio.cpp:505-576`: for every point, lines 549-556 linearly search propagated IMU states. The resulting complexity approaches `O(points * IMU states)` and should later be replaced by a monotonic cursor or indexed lookup.

### Point-LIO critical path

The vendor adapter creates a vector, `Preprocess` copies points into dense/filtered buffers and deques (`small_point_lio/src/preprocess.cpp:21-63`), and per-point state/map updates run synchronously on the `SlamSystem` processing-timer thread. The ROS callback only enqueues typed input. The resulting `FrontendFrame` moves its vectors into a bounded queue, avoiding a downstream deep copy. The remaining memory work is PCL conversion, prior-localization scratch, occupancy allocations and retained `MapManager` KeyFrame payloads; Point-LIO no longer keeps a second per-scan accumulated global map.

### Occupancy and localization background work

- Occupancy evidence/raycast remains hash-set work in the single-writer
  builder context. Snapshot-source rebuilds run in the adapter's dedicated
  latest-only worker; the ROS adapter mutex protects builder mutation and
  snapshot materialization only, while dense ROS message conversion
  (`O(width * height)`) occurs after the lock is released. The explicit online
  source still performs scan insertion in the downstream/adapter context.
- `PriorMapLocalization` rebuilds a local submap by concatenating stored frames at `prior_map_localization.cpp:581-600`, then allocates crop/filter/alignment clouds at `prior_map_localization.cpp:602-753`.
- KISS conversion builds new `std::vector<Eigen::Vector3f>` source/target arrays for attempts at `src/super_lio/src/lio/kiss_matcher_relocalizer.cpp:20-45`.
- KISS global recovery may deliberately trade one low-frequency background
  parallel voxel pass for stable descriptor input when
  `lio.kiss_matcher.deterministic_voxel_sampling` is enabled. FPFH/ROBIN/GNC
  can still use internal parallel work, so acceptance and latency—not
  bitwise-identical intermediate inlier counts—are the production contract.
- Snapshot publication copies light KeyFrame/constraint/Submap metadata at commit cadence but shares cloud storage. This is currently `O(keyframes + constraints)` per committed version.
- ROS optimized-path publication is latest-version/coalesced at 250 ms and copies only Pose metadata/message fields outside the backend worker.
- `MapSaveScheduler` performs Snapshot/product materialization, deterministic
  voxel reduction, binary PCD fragment reads and atomic PCD serialization on
  its IO worker. Standard Super fragment writers/mergers hand off immutable
  products or explicit fragment paths and wait only for reliable drain at
  shutdown; output parity remains a validation gate.

These findings are static code-path risks, not measured latency claims. Phase 0 of the roadmap must establish P50/P95/P99 latency, memory and queue baselines before performance-sensitive changes.

## Architecture Problems

### P0 — boundaries and correctness risks

#### P0-1: Standard frontend/downstream boundary — resolved in Phase 3

- `SuperLIO` and `PointLIOFrontend` own their local estimator/map and output scheduling only.
- Application composition owns `LegacyDownstreamPipeline`, which owns backend, localization, grid, global-map/output and standard map save.
- Both frontends still require ROS sensor adapters; this is tracked under P0-2/P0-5, not hidden as completed core/ROS separation.

The two executable compositions now select the frontend adapter without duplicating downstream owners. Phase 4 replaces the transitional frontend-specific owning payloads with accepted immutable KeyFrames.

#### P0-2: ROS directly schedules and mutates frontend state

The standard nodes no longer have this violation: typed sensor values are queued into `SlamSystem` and processed by the application timer. The retired compatibility path did not need to be preserved; arbitrary-start/recovery now uses the standard prior-map pipeline.

#### P0-3: Pose/frame migration remains incomplete; standard global correction authority resolved in Phase 6

`GlobalAlignmentState` now provides one typed, versioned `T_map_odom` authority for both standard frontend compositions, and same-time `T_map_base`/`T_odom_base` pairing is tested. Multiple legacy pose/state models, mutable duplicated `SE3` fields and ambiguous external-transform names still make the broader pose migration incomplete.

#### P0-4: Backend decomposition — resolved in Phase 4

The target path now has independently tested `PoseGraph`, `ConstraintBuilder`, dynamic covariance estimator, `Optimizer`, GTSAM adapter, `MapManager`, disabled loop seams and a reliable single-writer scheduler. Typed odometry and relocalization constraints no longer require frontend or ROS types. The legacy facade's global-correction role has been removed from standard execution; Snapshot/Submap ownership and Snapshot-based backend refinement are established.

#### P0-5: Core algorithms depend on ROS and process-wide configuration

The replacement standard core/frontend/vendor/transitional-downstream targets
can be independently built and tested without ROS, multiple construction-time
configs can coexist at the new boundaries, and the complete recorded standard
pipeline now runs offline through `DatasetRunner`. The remaining limitation is
the explicitly retained Super/legacy surface: process-wide `g_*` algorithm
configuration and the PCL/glog `basic` compatibility facade still prevent a
complete multi-instance migration of every legacy path. Point-LIO itself now
uses typed construction configuration.

The standard Point-LIO path now uses project-owned
`adagraph::frontend::PointLIOConfig`: ROS/YAML is parsed at the application
boundary and `small_point_lio::Parameters` conversion is confined to the
`point_lio_adapter` implementation. The public Point-LIO frontend header no
longer includes the vendor API.

#### P0-6: Standard Super-LIO non-real-time work — resolved in Phase 3

Backend updates, grid insertion, large ROS publications and map accumulation execute on a reliable bounded worker; serialization starts after drain/join. Overload uses explicit backpressure because legacy keyframe acceptance is downstream. Accepted immutable KeyFrames now cross a second reliable scheduler boundary with zero loss in Phase 4 regressions.

### P1 — scalability and maintainability risks

- One relocalization path remains: production KISS/VGICP stays inside `PriorMapLocalization` above the typed scheduler/retriever/verifier seams.
- Point-cloud copies and heap-heavy temporary clouds occur across localization, occupancy and output paths.
- `MapManager` retains accepted KeyFrame payloads without a system-level map
  memory budget; Point-LIO convergence must address this alongside
  prior-localization PCL working-set retention.
- Occupancy storage/building is now ROS-independent and publishes immutable
  snapshots; the transitional `OccupancyGridPublisher` still owns the
  adapter mutex and ground/PCL conversion. Enabled standard configs use a
  latest-only Snapshot rebuild worker. The online-cloud source has product
  comparison coverage, but its lock hold, update-tail latency, queue impact
  and long-run memory growth remain unmeasured; it must not be enabled for a
  deployment until those measurements pass a deployment-specific budget.
- Point/Super/backend queue policies, latest-only optimized-path/grid snapshot
  coalescing and reliable fragment/final map-save IO are explicit. Occupancy
  rebuild cancellation, point limits and scratch-memory rollback are now core
  contracts with unit coverage; long-run map-representation budget measurement
  remains.
- Frontend/relocalization initialization, map-load, update-delay and visualization counters are instance-owned; only a one-time KISS feature-unavailable warning remains process-scoped diagnostic state.
- The `lio` build target has broad dependencies on ROS, GTSAM, PCL and TF; global include directories and source globbing obscure module boundaries.
- Existing tests cover frontend/core domain contracts, backend chain, Snapshot
  concurrency/products, Submap lifecycle, OctVox, point time, ground filter,
  manifold, ring buffer, relocalizer contracts and the ROS-free dataset
  runner. Full recorded standard Super-LIO and Point-LIO replay evidence is
  retained under `log/refactoring_baselines/phase8_20260813_ros_free_dataset/`.
- Super-LIO `OctVoxMap` uses list/hash-iterator storage with pointer chasing
  and per-node allocation. It has not been profiled as a bottleneck and is
  intentionally not an optimization target while Point-LIO convergence is the
  product direction; retire it with Super-LIO if the Point-LIO gates pass.

### P2 — ongoing engineering debt

- Ambiguous or misspelled names such as `caceData` and `voxel_fliter`.
- `bool` failures without an error code or diagnostic context.
- Mixed ROS logging, glog and `std::cout` in algorithm code.
- Sensor/PCL types exposed from the nominally low-level `basic` package.
- Full ROS Path regeneration/publication on each accepted point/keyframe and `erase(begin())` when capped.
- Build-time absolute source `ROOT` remains only on the two compatibility targets
  that preserve relative legacy map-path behavior; broad header installation
  and some global CMake state remain.
- Mutable `shared_ptr` use where unique or immutable ownership would be clearer.

## Existing Strengths to Preserve

- Global corrections are not written back into the Super-LIO/Point-LIO filter or local voxel map.
- Point-LIO moves `FrontendFrame` data through a bounded downstream queue and records drops.
- `PriorMapLocalization` uses a condition-variable worker with controlled shutdown rather than a sleep loop.
- Super-LIO reuses several per-point working arrays and uses TBB in compute-heavy sections.
- `VoxelGridClosest` reuses containers and swaps output buffers.
- `src/basic/include/basic/buffer/RingBuffer.hpp` already provides a bounded close/drain pattern and has tests, although it is not yet used by the SLAM pipeline.

## Audit Limitations

The initial audit was static, but the migration now includes repeatable
two-frontend regressions, full recorded standard typed replays, offline
map-save/readback checks and clean build/install evidence. Exact copy counts
still depend on configuration branches. Long-duration map-representation
scratch/cancellation measurements and real-robot relocalization remain
unvalidated. Snapshot versus online occupancy is an explicitly different
product and is compared by contract rather than byte equality.
