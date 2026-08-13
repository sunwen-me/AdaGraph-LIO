# Refactoring Roadmap

Status: active incremental plan
Last updated: 2026-08-13

This roadmap applies the Strangler Pattern. The existing Super-LIO and Point-LIO executables remain runnable while stable contracts are introduced around them. A phase is complete only when its completion criteria and validation are recorded in `REFACTORING_STATUS.md`.

Before starting any phase:

1. Read `REFACTORING_STATUS.md`.
2. Read the phase definition below.
3. Check `TARGET_ARCHITECTURE.md` when the change touches an invariant.
4. Inspect actual code, `git status`, `git diff` and recent `git log`.
5. Preserve unrelated user changes in the dirty working tree.

After completing meaningful architecture work, update `REFACTORING_STATUS.md`; update `CURRENT_ARCHITECTURE.md` when the implemented architecture has changed.

## Overall Progress

- [x] Phase 0 — Baseline and regression contract
- [x] Phase 1 — Frontend observation seam
- [x] Phase 2 — Core time, pose, sensor and configuration model
- [x] Phase 3 — True frontend/downstream separation and KeyFrame contract
- [x] Phase 4 — Backend/PoseGraph decomposition and scheduling
- [~] Phase 5 — MapManager, MapSnapshot and Submap identity (standard duplicate graph/map owners removed; Point-LIO map-memory budget remains; the temporary Super fragment path is slated for retirement with Super-LIO rather than further optimization)
- [~] Phase 6 — Independent relocalization module (production prior-map owner is immutable; compatibility parity remains)
- [~] Phase 7 — Derived map representations and asynchronous IO (foundation, production evidence and Snapshot rebuild budgets complete; online-path latency profiling remains)
- [~] Phase 8 — ROS adapter/lifecycle and build-target separation (production gates passed; boundary cleanup remains)
- [ ] Phase 9 — Loop closure, multi-session and new constraint sources

## Phase 0 — Baseline and Regression Contract

### Goal

Establish reproducible correctness and performance baselines for the existing Super-LIO and Point-LIO paths before changing runtime boundaries.

### Why

The current audit identified static performance and ownership risks, but exact latency, memory and trajectory behavior depend on configuration and datasets. Refactoring without a baseline would make numerical regressions and hidden point-cloud copies difficult to detect.

### Affected modules

- `scripts/`
- test/evaluation support in `src/super_lio`
- optional low-overhead metrics instrumentation in existing frontends
- `Datasets/*` metadata and ground-truth discovery, without modifying dataset content

### Expected dependency change

No production dependency change. Regression utilities consume existing executables and outputs. A later ROS-independent dataset runner is intentionally deferred until the core boundary exists.

### Work items

- [x] Persist architecture audit, target, roadmap and status in `doc/architecture/`.
- [x] Define a machine-readable baseline manifest for selected dataset/config/frontend combinations.
- [x] Make the existing trajectory comparison/evaluation path reproducible and non-destructive.
- [x] Record output pose count, timestamp range, APE where GT is available and run success/failure.
- [x] Record frontend/backend queue diagnostics that already exist.
- [x] Record peak RSS and available legacy stage mean/max timing. P50/P95/P99 are explicitly deferred to the common Phase 1 observation point because the legacy path does not retain per-frame samples.
- [x] Save the initial baseline report in a generated-output location that is not confused with source configuration.

### Risk

- Instrumentation can perturb scheduling if logging is performed per point/frame.
- Long rosbag runs can create large logs/maps.
- Dataset topic/config mismatches may look like algorithm regressions.

### Validation

- `colcon build` succeeds.
- Existing unit tests pass.
- At least one Super-LIO and one Point-LIO dataset run produces a valid trajectory.
- Ground-truth evaluation commands and thresholds are recorded.
- Baseline output paths are deterministic and do not overwrite user maps.

### Performance considerations

- Metrics collection must be aggregate/in-memory and emitted at low frequency.
- Do not add PointCloud copies for measurement.
- Do not log every point or every nearest-neighbor query.

### Completion criteria

- A future change can be compared against a named baseline with the same dataset, config and frontend.
- Correctness and performance comparisons can distinguish missing output, numerical drift and runtime slowdown.
- Validation results and any unavailable metrics are recorded in `REFACTORING_STATUS.md`.

## Phase 1 — Frontend Observation Seam

### Goal

Introduce a single transitional `FrontendResult` contract emitted by both Super-LIO and Point-LIO without changing their existing downstream behavior or numerical state update.

### Why

No safe strangler migration is possible while downstream modules obtain data through two unrelated frontend-specific paths. The first seam provides an observable contract before moving ownership.

### Affected modules

- new frontend contract headers under `src/super_lio/include/lio/frontend/` during the transitional stage
- `SuperLIO::process` / `SuperLIO::Output`
- Point-LIO `FrontendFrame` adaptation and `PointLIOFrontend::enqueueFrame`
- new unit/contract tests
- CMake test declarations

### Expected dependency change

Initially additive only:

```text
SuperLIO --------┐
                 ├──> FrontendResult -> observer/test
Point-LIO -------┘
```

Legacy backend, localization, grid, ROS and map-output calls remain in place during Phase 1. True dependency removal occurs in Phase 3.

### Work items

- [x] Define transitional timestamp/frame/pose/status types with explicit semantics.
- [x] Define a no-copy current-scan view/handle with documented callback-only lifetime.
- [x] Add a lightweight observer/sink contract that is optional by default.
- [x] Emit exactly one result at the single `SuperLIO::stateProcess` completion point for each processed scan.
- [x] Adapt the existing moved Point-LIO `FrontendFrame` to the same result semantics before queue ownership transfer.
- [x] Verify that the seam borrows original point storage and does not deep-copy the cloud.
- [x] Keep existing nodes and downstream output behavior unchanged.

### Risk

- The two frontends currently disagree in naming of body/IMU/lidar frames.
- A borrowed point-cloud view can outlive its buffer if the observer stores it.
- An observer invoked in the critical path could block frontend processing.

### Validation

- Unit tests verify timestamp, frame metadata and one-result-per-scan behavior.
- Tests or instrumentation verify cloud buffer identity/no new deep copy.
- Legacy Super-LIO and Point-LIO trajectories remain within Phase 0 tolerances.
- Frontend latency does not regress materially.

### Performance considerations

- At most one coarse observer call per accepted scan.
- No per-point virtual dispatch, allocation or locking.
- The Phase 1 observer is synchronous and must be non-owning/non-blocking; cross-thread ownership is introduced only with an explicit immutable cloud handle.

### Completion criteria

- Both current LIO paths can be observed through the same well-documented result contract.
- Existing production outputs are unchanged.
- Phase 0 validation passes and status documentation is updated.

## Phase 2 — Core Time, Pose, Sensor and Configuration Model

### Goal

Introduce ROS-independent, frame-explicit domain types and validated configuration objects at new module boundaries.

### Why

The current duplicate pose/state structures and process-wide `g_*` configuration make frontend substitution and multi-instance testing unsafe.

### Affected modules

- new `common`, `geometry` and `sensor` headers
- frontend result/adapters
- ROS parameter conversion
- Super-LIO and Point-LIO construction adapters
- transform and configuration tests

### Expected dependency change

```text
ROS messages/parameters -> ROS adapter -> core sensor/config values
frontend/backend         -> common/geometry/sensor
```

Legacy algorithm internals may still use old types behind adapters; new public contracts do not expose them.

### Work items

- [x] Introduce `Timestamp`, strong IDs and `T_target_source` pose conventions.
- [x] Introduce ROS-independent `ImuData` and `PointCloudFrame`.
- [x] Define immutable/shared cloud ownership and same-call view rules.
- [x] Add `FrontendConfig`, `BackendConfig` and `RelocalizationConfig` validation.
- [x] Adapt existing `g_*` and vendor ROS parameters into validated configs at construction boundaries. Legacy global loading remains behind the adapter until later ownership phases.
- [x] Stop introducing new direct YAML/ROS parameter reads in algorithms.

### Risk

- Silent transform inversion or incorrect lidar/IMU/base convention.
- Temporary adapter duplication during migration.

### Validation

- Composition/inverse and frame-mismatch unit tests.
- Known static extrinsic tests for current sensors.
- Dataset trajectory parity for both frontends.
- Multiple config instances can coexist in unit tests.

### Performance considerations

Strong IDs/timestamps/poses are value types with no dynamic allocation. Conversions occur once at boundaries, not per point.

### Completion criteria

- New module APIs contain no raw unlabelled transform matrices.
- Core configuration objects can be constructed without ROS.
- No new global configuration dependencies are introduced.

## Phase 3 — True Frontend/Downstream Separation and KeyFrame Contract

### Goal

Make the frontend responsible only for local odometry and move keyframe/downstream orchestration into a system-level pipeline.

### Why

Phase 1 only observes a result; `SuperLIO` and `PointLIOFrontend` still own backend, relocalization, grid and global map work. This phase removes those dependencies.

### Affected modules

- Super-LIO adapter
- Point-LIO adapter
- new `LegacyDownstreamPipeline`/initial system orchestrator
- keyframe policy and `KeyFrame` domain type
- backend/localization/grid invocation sites
- application wiring

### Expected dependency change

Before:

```text
frontend -> backend + relocalization + occupancy + ROS + map IO
```

After:

```text
frontend -> frontend result only
system/legacy downstream -> backend + relocalization + mapping + adapter outputs
```

### Work items

- [x] Define immutable `KeyFrame` with IDs, timestamp, `T_odom_base`, cloud and quality.
- [x] Move keyframe decision policy outside frontend algorithm state.
- [x] Move AdaGraph invocation out of both standard frontend classes.
- [x] Move prior-localization submission out of both standard frontend classes.
- [x] Move occupancy/global map/output orchestration out of both standard frontend classes.
- [x] Preserve current Point-LIO bounded move semantics.
- [x] Preserve a legacy executable/rollback wiring until parity is proven.

### Risk

- Keyframe cadence, Hessian/covariance semantics or output ordering can change.
- Point-cloud lifetime mistakes at the new async boundary.

### Validation

- Same keyframe timestamps/counts and GTSAM factor counts as the legacy paths.
- Dataset trajectory and relocalization parity.
- Thread sanitizer where practical for new ownership boundaries.
- No frontend include/link dependency on backend, localization, occupancy or ROS publishing code.

### Performance considerations

Move payloads; freeze/share only accepted large data. The Super-LIO critical path should improve because downstream work leaves it, while Point-LIO must retain its existing non-blocking downstream behavior.

### Completion criteria

- Both frontends satisfy one contract and can be swapped through composition.
- Neither frontend owns backend, relocalizer, occupancy or global map IO.
- P0-1 and the frontend portion of P0-6 are resolved.

## Phase 4 — Backend/PoseGraph Decomposition and Scheduling

### Goal

Split graph state, constraint creation, optimization and scheduling while first preserving the exact current ISAM2 factor-chain behavior.

### Why

Loop closure, GNSS and optimizer replacement cannot be added cleanly to the current `AdaGraphBackend` class.

### Affected modules

- `AdaGraphBackend`
- new `PoseGraph`, `ConstraintBuilder`, `Optimizer` and `BackendScheduler`
- backend tests
- system/backend wiring

### Expected dependency change

```text
system -> BackendScheduler
BackendScheduler -> PoseGraph + ConstraintBuilder + Optimizer
Optimizer -> immutable graph problem
```

### Work items

- [x] Extract graph node/constraint domain state.
- [x] Wrap current GTSAM behavior behind an optimizer boundary.
- [x] Make graph mutation single-writer.
- [x] Introduce typed odometry/relocalization constraints.
- [x] Schedule backend work outside frontend execution.
- [x] Add empty/disabled loop interfaces without inventing a plugin system.

### Risk

- Asynchronous order can change ISAM2 updates and numerical reproducibility.
- Extracting too many classes can create abstraction overhead without value.

### Validation

- Pure odometry chain numerically reproduces current tests.
- Node/factor counts and update order are deterministic.
- Optimizer tests link without ROS/frontend.
- Frontend latency does not regress.

### Performance considerations

Optimization runs at keyframe/task frequency. Graph snapshots avoid holding graph locks during expensive solve or publication.

### Completion criteria

- PoseGraph is independently testable.
- Optimizer can be replaced without changing frontend.
- Backend does not publish ROS messages or run point-cloud registration.

## Phase 5 — MapManager, MapSnapshot and Submap Identity

### Goal

Create one authoritative map lifecycle and immutable snapshot boundary.

### Why

The current system has multiple mutable maps and pose copies with no consistent version for save, relocalization, grid or visualization.

### Affected modules

- new `MapManager`, `MapSnapshot`, trajectory/session/keyframe/submap IDs
- backend optimization commit
- accumulated point-map handling
- map save and consumers

### Expected dependency change

```text
backend optimization result -> MapManager -> immutable MapSnapshot -> consumers
```

### Work items

- [x] Define authoritative keyframe payload and optimized-pose ownership.
- [x] Publish versioned immutable snapshots.
- [x] Convert optimized-path visualization and comparison map-save output to snapshots; legacy dense output remains rollback until semantic parity is proven.
- [x] Introduce trajectory/session/submap identity, bounded rollover and finished/active metadata.
- [x] Remove the standard Point-LIO accumulated global map owner where equivalent Snapshot-derived output exists; retain only the explicit Super fragment compatibility buffer.
- [x] Remove the duplicate legacy AdaGraph graph from the standard downstream/application path after Snapshot/result parity; retain `adagraph_legacy_backend` only for explicit compatibility and its standalone tests.

### Risk

- Transitional memory duplication.
- Snapshot pose/payload version mismatch.

### Validation

- Snapshot consistency under concurrent reads.
- Save/load/map extent and point-count comparisons.
- Multi-reader tests without locking PoseGraph internals.

### Performance considerations

Clouds are immutable references, not copied per snapshot. Start with light `O(keyframes)` metadata copies; adopt chunking/structural sharing only when profiling requires it.

### Completion criteria

- Relocalization, grid, save and visualization can consume one consistent map version.
- No consumer locks mutable PoseGraph containers.

## Phase 6 — Independent Relocalization Module

### Goal

Keep the single standard prior-map relocalization path frontend-independent.

### Why

The former `SuperLIOReLoc` inheritance path coupled relocation to the complete
frontend. The retained `PriorMapLocalization` adapter still contains
stateful production quality policy that must continue to evolve toward the
generic relocalizer seams without reintroducing a second execution path.

### Affected modules

- `PriorMapLocalization`
- KISS matcher wrapper
- FastVGICP/PCL GICP verifier
- system global-alignment policy
- localization tests

### Expected dependency change

```text
Relocalizer: query + LocalizationMap snapshot -> RelocalizationResult
SlamSystem: result + matching frontend pose -> T_map_odom
```

### Work items

- [x] Separate descriptor/candidate retrieval from registration verification.
- [x] Make queries latest-only/cancellable.
- [x] Return typed `T_map_base`, fitness/inlier/confidence fields without editing frontend state. The legacy adapter currently reports `Confidence::kUnknown` until verifier metrics are adapted.
- [x] Move `T_map_odom` authority to the system/global-state layer for both standard frontend compositions, including optional backend refinement; ROS is a publication cache only.
- [x] Add startup/recovery quality diagnostics and scheduler queue-wait, processing and end-to-end latency gates to the real prior-map regressions for both standard frontends.
- [x] Retire `SuperLIOReLoc`, `relocation_node` and their dedicated
  configuration/launch/build targets. Prior-map operation is now exposed only
  by the standard Super-LIO and Point-LIO compositions.

Implemented foundation: `LocalizationMap`, retrieval/verification interfaces, `ComposedRelocalizer`, cancellable/latest-only `RelocalizationScheduler`, typed update algebra, `GlobalAlignmentState` and `GlobalAlignmentUpdateMailbox` are ROS-independent and unit-tested. Production `PriorMapLocalization` uses the concrete PCL/KISS retriever and PCL GICP/VGICP verifier through `ComposedRelocalizer`, with immutable PCL owners recovered by type identity and no architecture-induced current-scan copy. Global-recovery deployments explicitly use stable KISS voxel reduction and can try both ratio-test policies before the unchanged strict VGICP/GICP quality gate; candidate/verifier diagnostics make startup failures observable. Non-interruptible vendor registration uses the scheduler's finish-active/publish policy while retaining only the newest pending query; cooperative cancellation remains available for cancellable implementations. The stateful quality gate stays above the generic verifier. The standard Super-LIO and Point-LIO downstreams are now the only relocation compositions; structural verification rejects reintroduction of the retired subclass/node/target.

### Risk

- Startup/recovery success rate or quality gating changes.
- Incorrect timestamp pairing between `T_map_base` and `T_odom_base`.

### Validation

- Existing prior maps and arbitrary-start datasets.
- Success rate, false-positive rejection, fitness and recovery latency.
- No ESKF/local-map writeback.

### Performance considerations

Reuse immutable prior-map/index data. Avoid a full current-scan deep copy where an immutable frame handle suffices. Cancellation prevents obsolete expensive registration work.

### Completion criteria

- One relocalization implementation works with both frontends.
- Relocalization can be tested without ROS/backend/frontend internals.
- Global correction has one authority.

## Phase 7 — Derived Map Representations and Asynchronous IO

### Goal

Make occupancy/navigation maps and serialization background consumers of snapshots.

### Why

Grid ray casting, dense-message construction and PCD IO currently contend with real-time or frontend-oriented code and are not reusable outside ROS.

### Affected modules

- `OccupancyGridPublisher`
- ground segmentation and ray casting
- point-map/global-map save
- new ROS-independent occupancy/map IO types

### Expected dependency change

```text
MapSnapshot -> OccupancyGridBuilder / MapSerializer
ROS adapter -> converted grid/save status
```

### Work items

- [x] Extract ROS-independent occupancy storage/building.
- [x] Make publication consume an immutable grid snapshot.
- [x] Move map serialization to reliable asynchronous IO work. Snapshot,
  standard final point-map products and standard Super-LIO fragments/merge use
  `MapSaveScheduler`; binary
  PCD read/merge is FIFO-ordered after fragment writes and installed atomically.
  Production output evidence is recorded by the dedicated Phase 7 regression.
- [x] Add rebuild/incremental/lazy update policy for occupancy evidence and
  publication materialization.
- [x] Keep room for TSDF/ESDF/elevation without changing frontend. `MapSnapshot`
  is the representation input boundary and occupancy remains a concrete
  composed builder; no common virtual representation interface is introduced
  before a second production representation needs it.
- [ ] Before enabling the explicit `online` source for a deployment, measure
  builder/publisher lock-hold and update P50/P95/P99, frontend/downstream/ROS
  queue impact, peak RSS/representation growth and long-run terminal behavior
  at deployment-representative scan rate and map extent. The current Snapshot
  source is the default and already has cancellation/scratch-budget evidence.

Implemented Phase 7 boundary: `OccupancyGridBuilder` is ROS-independent and
supports borrowed-cloud updates plus rebuild from an immutable `MapSnapshot`.
`OccupancyGridPublisher` owns only the ROS-facing adapter state and publishes
an atomically handed-off immutable grid snapshot. Snapshot-source mode is
selected by `lio.occupancy_grid.source: "snapshot"` in the enabled standard
configs; `"online"` remains an explicit compatibility policy, and prior-map
static reset remains available. `MapSaveScheduler` is a reliable bounded IO
worker that accepts immutable snapshots, materialized point-cloud products or
an explicit PCD fragment set, performs deterministic voxelization and binary
PCD read/write with atomic installation. Production save-output and live Nav2
interface evidence now exist in `scripts/phase7_validation_cases.json`.
Snapshot and online occupancy share topic/frame/resolution/value encoding but
are distinct products: Snapshot is optimized-KeyFrame-derived, while online
retains per-scan ray/free evidence. Their extents and cell counts therefore
require explicit product comparison rather than byte equality. Snapshot rebuild
point limits, cooperative cancellation, scratch-memory rollback and check
intervals are now explicit validated core contracts. The recorded-data budget
regression `adagraph_phase7_occupancy_budget_regression_final_20260813` provides
production evidence for real latest-only Snapshot handoffs: Point-LIO completed
216 of 226 requests with 10 superseded cancellations, while Super-LIO
completed all 224 of 224 requests. Both runs had zero shutdown cancellations,
obsolete discards, budget overruns or failures; peak scratch was 689,928 B and 725,896
B, respectively, against a configured 64 MiB budget. Terminal outcomes are
machine-checked by `run_slam_regression.py`.

### Risk

- Occupancy origin/resolution or Nav2 semantics change.
- Snapshot rebuild can temporarily increase memory.

### Validation

- Golden grid tests for fixed clouds/poses.
- Map save/load round trip.
- Nav2 topic/frame compatibility.
- Frontend latency should improve or remain unchanged.

### Performance considerations

No frontend/graph lock during grid building or serialization. Occupancy
publication coalesces through the latest immutable snapshot; map-save work
uses a reliable bounded queue. Snapshot rebuild coalescing, point limits,
cooperative cancellation and scratch-memory rollback are implemented and
unit-tested; the recorded-data budget run also validates cancellation
accounting and scratch usage under high-frequency handoffs. This evidence
covers the Snapshot rebuild path only. Online occupancy must add its own
lock-duration, update-tail-latency, queue-impact and long-run memory evidence
before it becomes a production source.

### Completion criteria

- Occupancy is no longer authoritative SLAM state or owned by a ROS publisher.
- Map save/grid generation cannot block frontend processing.

## Phase 8 — ROS Adapter, Lifecycle and Build-target Separation

### Goal

Make core SLAM ROS-independent, provide mapping/localization lifecycle APIs and expose clean build boundaries.

### Why

The current ROSWrapper and monolithic CMake target prevent ROS replacement, offline execution and independent tests.

### Affected modules

- `ROSWrapper`
- application nodes and launch/config adapters
- CMake/package dependencies
- new `SensorBridge`, `TfBridge`, publishers/services and `DatasetRunner`

### Expected dependency change

```text
adagraph_ros2 -> adagraph_system/core
core -X-> ROS
dataset_runner -> adagraph_system/core
```

Current foundation: `SensorBridge`, `TfBridge`, `ServiceBridge`,
`SlamSystem`, `DatasetRunner`, explicit frontend/vendor/core targets and the
ROS-free `adagraph_legacy_downstream` target now exist. The standard node path
is `SensorBridge -> SlamSystem::AddImu/AddPointCloud -> typed frontend queues
-> SlamSystem::ProcessPending()`. Point-LIO transport policy is now an
explicit snapshot: `SensorBridge` is lossless for range/sampling and reads
`point_lio.input.*` for DDS depth/reliability, leaving vendor filtering to
Small Point-LIO. The retired relocation compatibility path no longer has a
node, launch file or build target; prior-map operation is provided by the
standard `super_lio_node` and `point_lio_node` compositions. `ROSWrapper`
remains a fallback facade for external callers and is not instantiated by an
application. The standard downstream no longer links the old
`AdaGraphBackend`; it reads committed `MapSnapshot` poses for optional backend
refinement. Full recorded-data parity, offline map-save validation and an
empty-tree clean build/install have passed; remaining Phase 8 work is target
boundary cleanup around the explicitly retained legacy algorithm/global
configuration surfaces. The global-parameter loader now has its own
`legacy_global_parameter_bridge.cpp` translation unit and dedicated header, so
the instance-owned ROS adapter no longer exposes or recompiles the global
loader API.

### Work items

- [x] Move standard sensor message conversion to `SensorBridge`; the legacy `ROSWrapper` input path remains only for compatibility.
- [x] Make Point-LIO sensor transport explicit: do not inherit `lio.sensor.*` range/filter parameters; use lossless `SensorBridge` conversion and `point_lio.input.*` QoS/depth values, with a structural boundary guard.
- [x] Move TF and publication to adapter-only components. `TfBridge`, `PublicationBridge`, `ServiceBridge` and injected downstream publication callbacks own active output; `ROSWrapper` is only a delegating compatibility facade.
- [x] Replace standard-node algorithm parameter reads with validated frontend config construction; legacy globals remain in the compatibility downstream/localization path.
- [x] Implement `StartMapping`, `StartLocalization`, `FinishTrajectory`, `SaveMap`, `Stop`, `Reset` through `SlamSystem` and `ServiceBridge`.
- [x] Add a ROS-independent dataset runner, including typed text input, lifecycle handling, final processing pump and real Super-LIO/Point-LIO smoke paths. Full `room_01` rosbag-to-typed-text replay parity, offline map-save products and clean-build evidence are recorded in `REFACTORING_STATUS.md`.
- [~] Split coherent CMake targets and remove global source globs/includes. Core, frontend, vendor, ROS adapter, legacy algorithms and ROS-free transitional downstream targets are explicit. `basic_core` is Eigen/Threads-only and Point-LIO hides its vendor parameter representation; Super legacy algorithm composition/global configuration and the PCL/glog compatibility facade remain.

Active Phase 8 boundary-cleanup work:

- [x] Standard Point-LIO reads no legacy `g_*` globals. `point_lio_node` and
  the ROS-free runner create typed construction snapshots; multi-instance and
  reset-isolation tests cover the frontend. Migrate Super-LIO legacy math
  separately rather than reintroducing global state into Point-LIO.
- [~] Split `basic` into `basic_core` (Eigen/Threads-only manifold math) and
  the PCL/glog `basic` compatibility facade. Migrate remaining legacy
  PCL-facing consumers before the installed facade can be retired.
- [x] Make the Point-LIO frontend consume one project-owned typed config.
  ROS/YAML parsing stays in `point_lio_node`/dataset runner, and only
  `point_lio_adapter` converts to `small_point_lio::Parameters`.
- [x] Audit in-repository callers of `ROSWrapper`, `LegacyInputBridge` and
  `PublicationBridge`. Standard nodes use the explicit bridges and no standard
  path instantiates `ROSWrapper` or `LegacyInputBridge`; retain their isolated
  `adagraph_ros_composition` compatibility target until external callers are
  explicitly retired. No relocation compatibility target remains.

These items are architecture/maintenance work and must preserve the existing
Point-LIO odometry, prior-map relocalization, map-save and occupancy gates.

### Risk

- Launch/install/topic/TF compatibility.
- Over-splitting build targets.

### Validation

- Clean build and install from an empty build directory.
- ROS topic/service/TF contract tests.
- Dataset runner trajectory parity with rosbag execution.
- Individual core tests link without ROS.

### Performance considerations

ROS callbacks convert/enqueue only. Large publication uses latest snapshots and does not allocate/serialize in the frontend worker.

### Completion criteria

- Core targets contain no ROS include/link dependencies.
- ROS 2 can be replaced without changing core algorithms.
- Full recorded-dataset execution works without ROS.

## Phase 9 — Loop Closure, Multi-session and New Constraint Sources

### Goal

Add new SLAM capabilities only after the architectural boundaries that isolate them are proven.

### Why

Adding these features before the preceding phases would deepen the current coupling and create more migration debt.

### Affected modules

- loop candidate retrieval/verification
- descriptor database
- PoseGraph constraint sources
- trajectory/session/submap lifecycle
- GNSS/vision/wheel/semantic adapters as selected

### Expected dependency change

New functionality enters through formal sensor, constraint, descriptor or map-representation boundaries; it does not add dependencies to frontend internals.

### Work items

- [ ] Implement loop candidate retrieval and registration verification.
- [ ] Add descriptor database such as Scan Context when selected.
- [ ] Add multi-session/multi-trajectory graph identity.
- [ ] Add GNSS and future sensors as constraint sources.
- [ ] Add map streaming/partial loading only after submap snapshot semantics are measured.

### Risk

Algorithmic false positives, graph inconsistency and uncontrolled memory growth.

### Validation

- Loop precision/recall and graph-consistency datasets.
- Multi-session load/save and inter-session constraint tests.
- Long-duration memory and latency tests.

### Performance considerations

All expensive search/verification/serialization remains cancellable background work with bounded queues and map/snapshot budgets.

### Completion criteria

- New frontend, optimizer, descriptor, constraint source or map representation can be introduced without cross-cutting changes outside its documented boundary.

## Rollback Policy for Every Phase

- Keep changes in reviewable PR-sized increments; avoid unrelated rewrites.
- Keep the previous executable/wiring available until the new path passes validation.
- Prefer additive adapters followed by dependency removal, not simultaneous replacement.
- Do not delete working legacy behavior until tests demonstrate parity and `REFACTORING_STATUS.md` records the replacement.
- Before handoff, inspect `git diff --check`, `git diff`, build/test results and the status document.
