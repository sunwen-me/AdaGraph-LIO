# Current Refactoring State

Last updated: 2026-08-13
Branch: `feature/point-lio-frontend`
Architecture source-of-truth directory: `doc/architecture/`

This is the dynamic handoff for the architecture refactoring. The code is still the final truth. Before continuing after a context loss, read all four files in this directory and then inspect `git status`, `git diff` and `git log`.

## Current Phase

**Phase 8 — ROS Adapter, Lifecycle and Build-target Separation: production validation complete; target-boundary cleanup remains.**

Latest boundary progress: standard Point-LIO ROS/YAML entry points create
project-owned `PointLIOConfig`; `small_point_lio::Parameters` is private to
`point_lio_adapter`. Point-LIO has explicit typed-config multi-instance and
session-reset isolation coverage. `basic_core` is now Eigen/Threads-only, with
PCL/glog and legacy point/cloud aliases retained by the `basic` compatibility
facade. The remaining Phase 8 work is Super/legacy `g_*` cleanup beyond the
standard Point-LIO boundary and preserving the isolated compatibility facade
until external callers can be retired.

Phases 0–4 are complete; the Phase 5 map/snapshot core, Phase 6 typed
relocalization boundary and Phase 7 occupancy/IO foundation have production
evidence. Phase 8 now has `SensorBridge`, `TfBridge`, `ServiceBridge`,
`SlamSystem`, a ROS-independent `DatasetRunner`, explicit frontend/vendor/core
targets and a ROS-free `adagraph_legacy_downstream` target. The standard path
is `SensorBridge -> SlamSystem::AddImu/AddPointCloud -> typed frontend queues
-> SlamSystem::ProcessPending()`. The standard prior-map relocalization chain
uses the same explicit adapter split:
`PriorMapLocalization -> RelocalizationScheduler -> GlobalAlignmentState` on
both standard compositions; ROS callbacks only enqueue data. `ROSWrapper` is
now a fallback facade for external callers and is not instantiated by any
application. The dataset
runner performs a final system processing pump before lifecycle finish and has
real no-ROS Super-LIO and Point-LIO smoke coverage. Its Point-LIO lifecycle
test checks the bounded newest-only downstream invariant
`enqueued = processed + dropped` after finish, rather than assuming frontend
observations are lossless downstream work. Point-LIO's standard sensor
adapter is explicitly lossless (`blind_squared=0`, large maximum range and
`filter_rate=1`); vendor range/sampling is applied once by Small Point-LIO,
while DDS depth/reliability come from `point_lio.input.*`. The standard
downstream no longer links or instantiates the old `AdaGraphBackend` facade;
`adagraph_legacy_backend` is retained only for explicit compatibility,
external legacy callers and its standalone tests. Full recorded
rosbag/text parity is now demonstrated for both standard frontends: the same
`room_01` bag was converted to typed text and replayed through the ROS-free
runner, with timestamp-aligned translation RMSE of 0.0121 m for Super-LIO and
0.00366 m for Point-LIO against the recorded ROS baselines. Phase 6 now uses
only the standard scheduler-backed prior-map path; its former dedicated
relocation implementation has been retired. Phase 7 now has
explicit core scratch-memory/cancellation budget contracts, unit coverage and
recorded-data long-run measurements for both standard frontends.

## Completed

- [x] Audited repository/package/CMake structure.
- [x] Identified Super-LIO and Point-LIO runtime data paths.
- [x] Audited core classes, ownership, threads, locks, cloud-copy paths, shared ownership, global state and ROS dependencies.
- [x] Classified architecture findings as P0/P1/P2 with concrete code locations.
- [x] Defined target module boundaries, dependency direction, data flow, ownership, frame and threading rules.
- [x] Defined initial core API drafts and abstraction performance contracts.
- [x] Defined the incremental Phase 0–9 roadmap.
- [x] Initialized persistent architecture documents under `doc/architecture/`.
- [x] Added `scripts/slam_regression_cases.json` with named Super-LIO and Point-LIO room_01 cases.
- [x] Added strict trajectory/resource/diagnostics summary generation in `scripts/summarize_tum.py`.
- [x] Extended `scripts/run_m2dgr_eval.sh` to select either frontend, refuse output overwrite, record GNU time resources, run APE and emit `summary.json`.
- [x] Added `scripts/run_slam_regression.py` to execute manifest cases and enforce acceptance gates.
- [x] Built the current working tree and passed all existing `basic`/`super_lio` tests.
- [x] Recorded the pre-seam Phase 0 suite under `log/refactoring_baselines/phase0_20260811_pre_seam/`.
- [x] Added `lio/frontend/frontend_result.h` with implementation/status/frame metadata, explicit `T_target_source` semantics and a non-owning interleaved XYZ view.
- [x] Added default-disabled synchronous result callbacks to `SuperLIO` and `PointLIOFrontend`.
- [x] Super-LIO emits after local-map update and before legacy `Output`; Point-LIO emits before moving the vendor frame into its bounded downstream queue.
- [x] Added no-copy/storage-identity, bounds-safety and frame-semantics unit tests.
- [x] Built and passed the expanded test suite, then recorded post-seam dataset results under `log/refactoring_baselines/phase1_20260811_post_seam/`.
- [x] Added nanosecond `Timestamp`, strong domain IDs and strict frame/time-checked `StampedTransform3d` under `include/adagraph/`.
- [x] Migrated `FrontendResult` and both legacy emitters away from raw `double timestamp_seconds`/`BASIC::SE3` boundary values.
- [x] Added ROS-independent `ImuData` and move-only `PointCloudFrame`; tests prove its vector storage moves without copy.
- [x] Added validated `FrontendConfig`, `BackendConfig` and `RelocalizationConfig` values.
- [x] Super-LIO and Point-LIO translate legacy/vendor parameters into `FrontendConfig` during initialization.
- [x] `AdaGraphBackend` supports explicit independent config instances; its default adapter snapshots existing globals at initialization.
- [x] `PriorMapLocalization` snapshots and validates legacy globals once, then reads its `RelocalizationConfig` during runtime.
- [x] Added time/transform/frame mismatch/known M2DGR extrinsic/sensor/config tests; selected suite passes 8 executables / 61 cases.
- [x] Recorded the final Phase 2 two-frontend regression under `log/refactoring_baselines/phase2_20260812_core_types_final/`; both manifest gates passed.
- [x] Added immutable `KeyFrame`, `KeyFrameQuality` and type-erased immutable cloud ownership; tests prove frame/lifetime/storage identity.
- [x] Added application-owned `LegacyDownstreamPipeline` shared by both frontend compositions without linking Super-LIO to the Point-LIO vendor.
- [x] Adapted the vendor frame to an owning project type by moving both point vectors and copying only fixed-size state/Hessian metadata.
- [x] Removed AdaGraph, prior localization, occupancy/global-map and PCD-save ownership from `PointLIOFrontend`; odometry publication is an injected adapter callback.
- [x] Removed AdaGraph and prior-localization members from `SuperLIO`; standard output/map orchestration now runs in the system pipeline.
- [x] Added a reliable capacity-16 Super-LIO queue with condition-variable drain/backpressure and a reusable cloud/Hessian buffer pool; no accepted/potential keyframe is silently dropped.
- [x] Preserved the Point-LIO capacity-8 latest-frame queue and its existing overflow metrics; Phase 3 regression had zero drops.
- [x] Retired `SuperLIOReLoc`, `relocation_node`, their dedicated CMake target and relocation config/launch/RViz assets; standard Super-LIO and Point-LIO now provide the only supported prior-map relocalization path.
- [x] Recorded the final Phase 3 two-frontend suite under `log/refactoring_baselines/phase3_20260812_frontend_downstream_final/`; both gates passed.
- [x] Added typed graph nodes, odometry/loop/prior/relocalization constraints, optimization results and a version-checked ROS/GTSAM-independent `PoseGraph`.
- [x] Extracted `KeyFramePolicy` and the exact rule/Hessian `OdometryCovarianceEstimator` from `AdaGraphBackend`.
- [x] Added a reliable bounded `BackendScheduler`; its worker is the sole graph writer and drains accepted KeyFrames/constraints before join.
- [x] Added `Optimizer::apply(GraphUpdate)` and a separate GTSAM-only `Isam2Optimizer` target; graph deltas avoid full-graph copies at keyframe frequency.
- [x] Converted legacy `g_odom_robo` once into typed static `T_base_imu`; both frontend observations now report `T_odom_base`.
- [x] Added timed global-pose association and fed prior-map observations into typed relocalization constraints without retimestamping measurements.
- [x] Added minimal disabled loop detector/verifier seams with no runtime plugin system.
- [x] Extended dataset gates to prove reliable drain, keyframe/node equality, `constraints = nodes - 1`, optimizer update parity and pose deltas below `1e-5`.
- [x] Recorded the final Phase 4 suite under `log/refactoring_baselines/phase4_20260812_backend_decomposition_final/`; both frontends passed all graph/optimizer gates.
- [x] Added ROS-independent `MapManager`, `MapSnapshot`, trajectory/submap metadata and atomic immutable publication in `adagraph_mapping_core`.
- [x] Made `BackendScheduler` the single writer of graph topology and map payload/optimized-pose state; snapshots publish only after committed graph/map updates.
- [x] Added bounded Submap lifecycle metadata (`BackendConfig::submap_max_keyframes`): full Submaps are finished and the next KeyFrame opens a monotonic `SubmapId` without changing graph math.
- [x] Added `SnapshotProductBuilder`; trajectory products copy only fixed-size metadata, while point-map materialization explicitly performs one background allocation/transform per point and reports bounds/counts.
- [x] Migrated `/lio/backend/optimized_path` from the legacy `vector<pair<double, SE3>>` to a latest-version Snapshot handoff. The backend callback performs only an atomic `shared_ptr` store; a ROS timer coalesces versions and allocates messages outside the graph writer.
- [x] Removed the unused legacy full-trajectory ISAM2 back-substitution, eliminating an `O(keyframes)` rollback-graph walk on every localized KeyFrame.
- [x] Added final Snapshot PCD comparison output (`<map-stem>_snapshot<ext>`) after drain/join. The legacy every-scan dense map remains primary rollback output because its density semantics differ from an optimized KeyFrame map.
- [x] Added concurrent Snapshot readers, derived point-count/bounds, duplicate optimization, observer isolation/order and Submap rollover tests.
- [x] Recorded `phase5_20260812_submap_lifecycle_final`; both frontends passed trajectory, APE, graph, optimizer, Snapshot, Submap and observer gates.
- [x] Added ROS-independent `LocalizationMap`, `Relocalizer`, `CandidateRetriever`, `RegistrationVerifier`, `ComposedRelocalizer`, `RelocalizationScheduler`, `RelocalizationResult` and strict typed update contracts under `adagraph_localization_core`.
- [x] Added a latest-only/cancellable relocalization scheduler with one active and one replaceable pending query; callback failures and cancellation/replacement counts are isolated and tested.
- [x] Added production `PclKissCandidateRetriever` and `PclRegistrationVerifier` adapters to the typed composition; they recover immutable PCL owners without a boundary-induced point-cloud copy, while the legacy stateful quality gate remains in `PriorMapLocalization`.
- [x] Routed production prior-map queries through `RelocalizationScheduler`; non-interruptible KISS/VGICP calls finish and publish the active result while replacing only pending stale work.
- [x] Prevented LOST-state retry cooldowns from rebuilding/downsampling/registering a local submap; bounded history ingestion continues until the next global retry.
- [x] Added `GlobalAlignmentState` under `adagraph_system_core`; it owns versioned `T_map_odom` and derives it only from same-time typed `T_map_base`/`T_odom_base` pairs without mutating frontend state.
- [x] Added `GlobalAlignmentUpdateMailbox`: the localization worker can only replace a fixed-size typed pending event; the system/downstream execution context commits global alignment and then updates the ROS cache.
- [x] Removed the standard backend/localization/ROS correction feedback ownership cycle. Backend initialization reads `GlobalAlignmentState`, backend refinements return a candidate to the system writer, and `PriorMapLocalization` no longer invokes a map-to-odom/ROS callback.
- [x] Extended dataset diagnostics with alignment version/commit/reject/cache-parity metrics and added an optional regression acceptance gate.
- [x] Validated real prior-map localization with both frontends and validated optional backend localization-factor feedback on Point-LIO; all typed alignment updates were accepted and ROS cache parity remained below numerical noise.
- [x] Extracted ROS-independent occupancy evidence storage/building, including ray casting, inflation, bounded extents and immutable `OccupancyGridSnapshot` products.
- [x] Added `OccupancyGridBuilder::rebuildFromSnapshot` so static occupancy can consume an immutable optimized `MapSnapshot` without ROS/PCL types.
- [x] Changed `OccupancyGridPublisher` to materialize and atomically hand off immutable grid snapshots; ROS message conversion/publication occurs after the adapter lock is released.
- [x] Added explicit occupancy `incremental`, `rebuild` and `lazy` update policies, exposed as `lio.occupancy_grid.update_policy` with `lazy` preserving the current default behavior.
- [x] Added occupancy `source: snapshot|online`; standard enabled configs route optimized `MapSnapshot` handoffs to a dedicated occupancy rebuild worker, while prior-map reset and explicit online compatibility mode remain available.
- [x] Extended ROS-independent reliable `MapSaveScheduler`: snapshot/final point-map products and standard Super fragments/merge are materialized, voxelized/read and atomically installed by a bounded IO worker; callback/status, FIFO merge and queue statistics are tested.
- [x] Added `record_occupancy_summary.py`, `compare_occupancy_summaries.py` and a dedicated Phase 7 validation manifest; enabled Super-LIO and Point-LIO regressions capture `/map` topic/frame/resolution/origin/shape and occupied/free/unknown semantics for Snapshot and online sources.
- [x] Added mapping-time online occupancy Nav2 artifacts: a bounded latest-only
  writer emits atomic PGM, map-server YAML and provenance metadata without
  blocking scan processing. The durable artifact is intentionally restricted
  to the online source because only that product retains ray-carved
  free-space evidence; PGM values/row order, atomic replacement and Nav2
  `map_server` loading are unit- and integration-validated.
- [x] Added production map-save regression support and `inspect_pcd.py`; Super-LIO fragment/final/Snapshot outputs and Point-LIO final/Snapshot outputs are validated as finite binary PCDs and independently loaded with PCL's `pcl::io::loadPCDFile`.
- [x] Replaced frontend and formerly compatibility relocalization function-static counters/flags with per-instance state before retiring the redundant subclass; initialization, map-load, update-delay and visualization counters no longer leak across reset/instances.
- [x] Added explicit `LegacyDownstreamCallbacks`, removed `ROSWrapper` from `LegacyDownstreamPipeline` headers/implementation, and moved it into the ROS-free `adagraph_legacy_downstream` target. Standard ROS nodes now inject publication, occupancy, alignment-cache and lifecycle effects explicitly.
- [x] Added the final `DatasetRunner` processing pump and failure-safe lifecycle close; tests cover EOF draining, active-session finish and pump-failure cleanup.
- [x] Made the production prior-map PCL target immutable after load: `LocalizationMap` is the sole persistent map owner, while crop indexing retains only derived point indices and all PCL access is const.
- [x] Added structural guards requiring standard nodes to use `SensorBridge`/`PublicationBridge` and prohibiting rebinding of `ROSWrapper` legacy publication/occupancy methods.
- [x] Removed the standard Point-LIO per-scan accumulated global PCL owner; final and Snapshot comparison PCDs are now both derived from the immutable `MapSnapshot`. Super-LIO's accumulated buffer remains only for explicit fragment-compatibility saving.
- [x] Reduced `ROSWrapper` to a compatibility facade: `LegacyInputBridge` owns compatibility subscriptions/buffers/prediction pumping, `PublicationBridge` owns publishers/TF/occupancy/map snapshots, and no application instantiates `ROSWrapper`.
- [x] Added the read-only `rosbag_to_typed_text.py` converter with SensorBridge-compatible filtering and deterministic external timestamp ordering; the full `room_01` bag produced 10,057 IMU packets, 728 clouds and 10,785 typed packets for both frontend replays.
- [x] Added `compare_tum_trajectories.py` with timestamp interpolation and parity gates; the recorded ROS/text comparisons cover 494/494 Super-LIO and 1007/1007 Point-LIO reference poses.
- [x] Completed the ROS-free production runner validation: Super-LIO and Point-LIO full typed replays finished with status 0, no rejected packets, and typed backend chains of 100/99 and 101/100 nodes/constraints respectively.
- [x] Fixed offline `--save-map` lifecycle wiring so the runner explicitly submits final Snapshot-derived products after frontend/system drain; both frontends produced finite binary PCDs and independent metadata/readback validation passed.
- [x] Removed the duplicate legacy AdaGraph graph from the standard application/downstream path after Snapshot/result parity; the typed `BackendScheduler`/`MapManager` path is now the only standard graph owner.
- [x] Added a real prior-map Point-LIO case to `scripts/slam_regression_cases.json`, including Snapshot-based backend refinement and a no-legacy-backend execution gate.
- [x] Added ROS-independent occupancy rebuild budget tests covering cooperative cancellation, point limits and scratch-memory limits without partial commit; the scratch estimate is reported by the adapter worker for long-run measurement.
- [x] Completed the recorded-data Phase 7 Snapshot rebuild budget run with real high-frequency `MapSnapshot` handoffs: Point-LIO 226/226 handoffs/requests, 216 completed and 10 superseded cancellations; Super-LIO 224/224 handoffs/requests and 224 completed. Both had zero shutdown cancellations, obsolete discards, budget overruns and failures, with peak scratch below 1 MiB under the configured 64 MiB budget.
- [x] Isolated the process-wide parameter compatibility API in `legacy_global_parameter_bridge.h`; the instance-owned `legacy_parameter_bridge.h` now exposes only validated configuration readers.
- [x] Added `RelocalizationScheduler` queue-wait, processing and end-to-end latency diagnostics, with non-negative timing accounting covered by scheduler unit tests.
- [x] Added prior-map startup/LOST KISS attempt/success/failure counters and first-success delay diagnostics; the Point-LIO snapshot-refinement and Super-LIO prior-map manifests now enforce quality and latency gates.
- [x] Stabilized M2DGR prior-map KISS input: recovery configs explicitly use
  a stable voxel reduction and try the alternate ratio-test policy when
  configured. Candidate and VGICP/GICP verifier outcomes are logged while the
  existing strict registration fitness gate remains unchanged.
- [x] Repeated real Point-LIO prior-map Snapshot refinement three times (two
  with unrestricted TBB) and passed the latency gate every time: startup
  `3/1/2` and first success `10.71 s`. The shared Super-LIO case passed with
  startup `2/1/1` and first success `5.91134 s`.

## In Progress

- [x] Define `MapManager`, versioned immutable `MapSnapshot` and authoritative optimized-pose ownership.
- [x] Add trajectory/session/submap lifecycle identity without changing current single-trajectory graph behavior.
- [x] Publish committed graph/map snapshots without exposing solver or mutable graph containers.
- [x] Migrate optimized-path visualization and comparison map-save output to snapshots while retaining dense-map rollback wiring.
- [x] Migrate production prior-map localization input fully to immutable `LocalizationMap` ownership; the loaded/downsampled registration target is shared through the typed immutable owner, with only a const PCL alias and derived crop index retained by the adapter.
- [x] Remove the duplicate legacy AdaGraph graph from the standard path after snapshot/result parity; retain `adagraph_legacy_backend` only for explicit compatibility and standalone backend tests.
- [x] Establish typed relocalization results and single system authority for global alignment on both standard frontend compositions.
- [x] Adapt the current KISS candidate retrieval and VGICP/GICP verification implementation to `ComposedRelocalizer` without adding an extra current-scan/submap copy.
- [x] Replace the legacy `PriorMapLocalization` busy-drop worker with the tested latest-only scheduler; use finish-active/publish for non-interruptible PCL calls and cooperative cancellation for cancellable implementations.
- [x] Add startup/recovery quality and relocalization scheduler latency gates to real prior-map regressions for both standard frontends.
- [x] Retire `SuperLIOReLoc` and its dedicated application entry point; the supported arbitrary-start/recovery behavior is now the measured standard `PriorMapLocalization` path.
- [x] Establish the Phase 7 occupancy/map-IO core boundary and immutable publication handoffs.
- [x] Replace the standard and compatibility legacy per-scan PCD fragment writer and synchronous fragment merger with the reliable IO worker. Production output evidence now includes binary PCD metadata and independent PCL readback; semantic differences between dense legacy and Snapshot products remain documented before deleting rollback evidence.
- [x] Establish the Phase 8 typed sensor/lifecycle seam: standard nodes route `SensorBridge` values through `SlamSystem`, and `DatasetRunner` drives the same processing boundary without ROS.
- [x] Make Point-LIO transport configuration explicit: `point_lio_node` does not inherit `lio.sensor.*` range/filter values; `SensorBridge` uses lossless transport settings and `point_lio.input.*` QoS/depth values, protected by `verify_core_boundaries.py`.
- [x] Split `LegacyDownstreamPipeline` from `ROSWrapper` through explicit callbacks and place it in the ROS-free `adagraph_legacy_downstream` target; ROS composition now links it explicitly.
- [x] Complete the `ROSWrapper` parameter/compatibility-input ownership migration. `LegacyInputBridge` and `PublicationBridge` now load and own the compatibility adapter configuration; `ROSWrapper` only delegates the legacy API.
- [x] Prove full recorded-dataset rosbag/text trajectory parity for the standard lifecycle paths; these are now the only supported relocation paths.

## Remaining P0 Issues

- [x] **P0-1: Standard frontend/downstream boundary established.** Backend/relocalization/occupancy/map orchestration is application-owned; the inherited relocation frontend was retired.
- [x] **P0-2: ROS directly schedules frontend state.** Standard nodes use `SensorBridge` plus `SlamSystem::ProcessPending()`; ROS callbacks do not run prediction, frontend or relocalization processing.
- [ ] **P0-3: Pose/frame migration remains incomplete.** The standard system path has one typed `T_map_odom` authority and strict timestamp/frame pairing, but legacy pose types remain within the retained frontend and algorithm adapters.
- [x] **P0-4: Backend decomposition established.** Typed graph, constraint builder, dynamic covariance estimator, optimizer adapter, disabled loop seams and reliable single-writer scheduler are independently testable. The legacy graph remains Phase 5 rollback debt, not the target boundary.
- [~] **P0-5: Core depends on ROS and process-wide configuration.** The verified core/frontend/vendor/transitional-downstream targets build without ROS headers/links, standard node configuration is instance-owned at construction, and the full standard recorded-data path runs without ROS. `basic_core` is lightweight, and standard Point-LIO hides the vendor parameter surface; Super/legacy algorithm globals and the PCL/glog compatibility facade remain transitional.
- [x] **P0-6: Standard Super-LIO downstream work extracted.** A bounded reliable worker owns backend, grid, publication and map accumulation/save; normal Phase 3 load had no backpressure.

### Current Work Items — Core and Configuration Boundary Cleanup

These are now active Phase 8 work items. They are structural cleanup tasks,
not current Point-LIO functionality blockers:

- [x] Standard Point-LIO no longer reads legacy `g_*` globals: ROS/YAML entry
  points create `PointLIOConfig` and `LegacyRuntimeConfig` snapshots at
  construction, and multi-instance/session-reset tests prove one Point-LIO
  frontend cannot overwrite another. Super-LIO legacy math remains a separate
  migration item.
- [~] Split `basic`: `basic_core` contains Eigen/Threads-only manifold math;
  `basic` retains PCL/glog and legacy point/cloud aliases as a compatibility
  facade. Migrate remaining legacy PCL consumers before treating all basic
  headers as a lightweight core surface.
- [x] Converge the Small Point-LIO parameter surface: `point_lio_node` and the
  ROS-free dataset runner parse one project-owned typed config, while only
  `point_lio_adapter` converts it to `small_point_lio::Parameters`; public
  frontend headers expose neither vendor nor ROS parameter types.
- [x] Audit in-repository users of `ROSWrapper`, `LegacyInputBridge` and
  `PublicationBridge`: standard nodes use `SensorBridge`/`PublicationBridge`
  directly and do not instantiate the first two; the first two remain only in
  `adagraph_ros_composition` plus compatibility tests. Retain that installed
  facade until external callers are explicitly retired; no relocation layer is
  linked by a standard node or dataset runner.

## Remaining P1 Issues

- [x] Remove the second `SuperLIOReLoc` implementation and make the standard `PriorMapLocalization` path the sole supported relocalizer composition.
- [x] Remove backend/localization/ROS global-correction feedback ownership cycle from the standard Super-LIO and Point-LIO compositions; no second relocation executable remains.
- [x] Introduce explicit immutable point-cloud ownership across async boundaries.
- [~] Bound/remove duplicate accumulated global point maps. Standard Point-LIO no longer retains an accumulated owner and saves from `MapSnapshot`; Super-LIO retains a bounded fragment buffer only for temporary legacy fragment-save compatibility. Do not invest in further Super fragment optimization: remove it with Super-LIO after the recorded Point-LIO convergence gates pass.
- [x] Separate occupancy storage/building from ROS publication and reduce lock scope. The builder remains single-writer under a short adapter mutex; dense snapshot and ROS message conversion are outside that lock.
- [x] Define and validate task-specific queue/backpressure/cancellation policies. Frontend, accepted KeyFrame/backend, relocalization, occupancy latest-only snapshot rebuild/publication and reliable fragment/final map-save policies are explicit; recorded-data budget runs account for every Snapshot rebuild outcome and bound scratch usage.
- [ ] Before enabling `lio.occupancy_grid.source: online` for deployment, record update-time and lock-hold P50/P95/P99, downstream/ROS queue impact, peak RSS/representation growth and long-run terminal behavior under representative scan rate/map extent. Snapshot-source budget evidence is complete; online profiling is a conditional deployment gate, not a current Snapshot-default blocker.
- [x] Remove static-local state that survives reset or contaminates future instances in the retained frontend/localization paths. A one-time KISS feature-unavailable warning remains intentionally process-scoped diagnostic state.
- [x] Split the monolithic `lio` build dependency boundary into explicit core, frontend, vendor, ROS adapter, legacy algorithm, ROS composition and ROS-free downstream targets; `lio` is interface-only compatibility alias.
- [x] Remove the unused legacy-global configuration header/link from the standard `adagraph_super_lio_frontend`; compatibility configuration remains limited to the legacy algorithm/adapter targets.
- [x] Split the global ROS parameter loader into the dedicated `legacy_global_parameter_bridge.cpp` target and header. The instance-owned `legacy_parameter_bridge.cpp` is now compiled only by `adagraph_ros2_adapter`; its public header no longer exposes process-wide loading symbols, and the compatibility global loader no longer macro-switches or recompiles that source.
- [x] Narrow the compatibility `ROOT` source-path definition to `adagraph_legacy_config` and `adagraph_ros2_adapter`; core/frontend targets no longer inherit a directory-wide absolute-path definition.
- [x] Add frontend/system/relocalization/snapshot and non-ROS dataset tests, including full recorded Super-LIO and Point-LIO typed replays; the parity evidence is retained under `log/refactoring_baselines/phase8_20260813_ros_free_dataset/`.
- [~] Profile list/hash-iterator voxel-map cache locality only if Super-LIO must remain after Point-LIO convergence. Otherwise retire `OctVoxMap` and its per-node allocation/pointer-chasing debt with Super-LIO rather than optimizing it.

## Known Technical Debt

- This branch is an intentionally cohesive Point-LIO/relocalization architecture
  change. Do not split follow-up edits away from the recorded boundary and
  validation contracts without reviewing the full diff and status documents.
- `HANDOFF.md` records additional real-robot, prior-map localization and deployment constraints; preserve those during migration.
- The current Point-LIO M2DGR configuration disables prior localization/occupancy/backend localization factors, so its measured trajectory mainly represents local Point-LIO odometry.
- There is no complete loop closure implementation in the current repository.
- The typed `BackendScheduler`/`PoseGraph`/`MapManager` graph is the sole standard graph owner. `adagraph_legacy_backend` remains an explicit compatibility archive for external legacy callers and `test_adagraph_backend`; it is not linked by `adagraph_legacy_downstream`, either standard node or `adagraph_dataset_runner`.
- Process-wide `g_*` loading still prevents complete multi-instance/session
  reset for Super/legacy algorithms. Standard Point-LIO is isolated through
  `PointLIOConfig` and `LegacyRuntimeConfig`; its frontend and downstream do
  not read global state. `AdaGraphBackend` explicit instances are independent,
  and frontend/relocalization runtime counters are instance-owned.
- Remaining core cleanup is limited to Super/legacy migration: replace
  remaining Super frontend/math `g_*` reads with construction-time configs,
  migrate legacy PCL consumers away from the `basic` facade, and retire
  `ROSWrapper`/`LegacyInputBridge` only after external callers are explicitly
  removed. No standard application uses those two facades today.
- The ROS regression script remains the reference producer of recorded baselines, while `adagraph_dataset_runner` now drives the same standard frontend/system/downstream composition from typed text without ROS. The converter is intentionally a read-only ROS bag boundary tool; the runner itself imports no ROS types.
- Snapshot publication currently copies `O(keyframes + constraints + submap metadata)` light records at every committed item. Clouds are shared, not copied; structural sharing/chunking is deferred until long-run profiling justifies it.
- `MapManager` intentionally retains accepted KeyFrame clouds. Point-LIO Phase 5 RSS is about 146 MiB versus 132 MiB in Phase 4. A map memory budget, frozen-Submap compaction and partial unloading remain required before large-scale operation.
- `MapSaveScheduler` queues Snapshot/final point-map products plus explicit Super fragment sets, reads binary PCD fragments in deterministic FIFO merge requests and performs voxelization/atomic installation on a reliable IO worker; save/merge no longer uses synchronous PCL IO.
- Occupancy Snapshot rebuild is available and core-tested. Enabled standard configs now use the Snapshot source and a dedicated rebuild worker. Live enabled regressions prove `/map`, `world` frame and `0.1 m` resolution for both sources on both frontends. Snapshot and online intentionally differ in origin/extent and free/unknown evidence because their input products differ; the comparison JSON records this instead of silently treating it as equality. Prior-map PCL reset remains an explicit compatibility path.
- `PriorMapLocalization` still owns the current PCL/KISS/VGICP tracking implementation and its internal predicted alignment for registration. External semantic authority is `GlobalAlignmentState`; the adapter is scheduler-backed and its startup/recovery and latency diagnostics are gated. Remaining work is to further shrink this production adapter toward the generic retriever/verifier seams.
- Production prior-map loading now uses one immutable `LocalizationMap` owner; adapters retain only a const PCL alias and derived crop indices, while per-query crop/registration clouds remain bounded scratch products.
- Real prior-map Point-LIO runs peak near 203–206 MiB RSS because the prior map, registration temporaries and authoritative KeyFrame clouds coexist; this is measured debt, not attributed to the fixed-size alignment mailbox.
- Failure reporting is mostly `bool`/logging rather than typed `Status`/`Result`.
- Naming and style debt (`caceData`, `voxel_fliter`, mixed logs) is intentionally deferred behind architecture boundaries.

## Architecture Decisions

These decisions are stable review invariants. A change requires an explicit rationale and replacement decision in this section.

### ADR-001: Frontend owns local odometry state

The selected LIO frontend exclusively owns ESKF/IEKF, IMU integration, deskew, scan matching and its local map. Its system output is continuous local odometry `T_odom_base` plus quality/cloud metadata.

### ADR-002: Global corrections do not mutate frontend local state

Backend optimization and relocalization are represented through `T_map_odom` and do not directly reset/overwrite frontend state or its odom-frame local map.

```text
T_map_base = T_map_odom * T_odom_base
```

### ADR-003: Occupancy grid is derived state

Occupancy consumes optimized keyframes/submaps through a consistent map snapshot. It is not authoritative SLAM state and does not belong to the frontend.

### ADR-004: Core is ROS-independent

ROS messages, `rclcpp`, TF, parameter APIs, publishers and services belong to adapter/application targets. Core sensor, pose, map and result types are plain C++.

### ADR-005: Large clouds use explicit immutable sharing only when necessary

Use unique mutable ownership while constructing a cloud, non-owning views for same-call processing and `shared_ptr<const CloudBuffer>` for cross-thread/fan-out lifetime. Do not make every domain object a `shared_ptr`.

### ADR-006: Mutable graph/map state is single-writer and published as snapshots

Background consumers receive `shared_ptr<const MapSnapshot>` instead of locking PoseGraph/MapManager internal containers.

### ADR-007: Expensive non-real-time tasks cannot block frontend processing

Loop/relocalization verification, optimization, occupancy generation, map serialization and heavy ROS visualization use bounded/coalesced background scheduling appropriate to their reliability requirements.

### ADR-008: Coarse runtime interfaces are allowed; hot loops stay concrete

One virtual dispatch per scan/keyframe/task is acceptable. Per-point dynamic dispatch and architecture-induced per-point allocation are not.

### ADR-009: Mapping/localization are system compositions

Mode selection belongs to `SlamSystem`/pipeline composition, not scattered `if (mapping_mode)` branches in algorithms.

### ADR-010: Migration is incremental

Existing runnable nodes stay available until each replacement passes build, unit, dataset and performance validation. New boundaries are added before old dependencies are removed.

### ADR-011: Transitional map-to-odom publication preserves current behavior (superseded by ADR-020)

This decision governed Phases 0–5. Phase 6 introduced the system/global-state layer. ROS remains the single TF publisher, but it is no longer the semantic authority; see ADR-020.

### ADR-012: A stamped pose owns the frontend result timestamp

`FrontendResult::local_pose.timestamp` is the authoritative scan-end/result time during the migration. The result does not carry a second independently mutable timestamp field. Sensor frames retain their own start/end timestamps where those semantics differ.

### ADR-013: Queue reliability follows the semantic acceptance boundary

Point-LIO retains its pre-existing capacity-8 latest-frame queue: overflow may discard an ordinary frame before keyframe acceptance and must be counted. Super-LIO uses a reliable bounded queue because legacy keyframe acceptance occurs in its downstream single writer; it applies explicit backpressure rather than silently dropping a frame that may become a keyframe. Both raw-frame queues drain required work and join on shutdown. Phase 4 added the common post-acceptance boundary, where immutable KeyFrames always use reliable delivery.

### ADR-014: Static extrinsics are not stamped poses

Calibration uses `StaticTransform3d`; runtime poses use `StampedTransform3d`. Legacy `g_odom_robo` is adapted once to typed `T_base_imu`, and frontend/system contracts expose `T_odom_base`. Float legacy rotation matrices are normalized only at that boundary.

### ADR-015: Optimizers consume immutable graph deltas

At keyframe frequency the scheduler sends only added nodes/constraints through `GraphUpdate`; it does not copy the complete graph. `Optimizer` is a coarse virtual boundary and `Isam2Optimizer` is a GTSAM-only adapter. Results carry the input graph version and only the single graph writer may commit them.

### ADR-016: Asynchronous global measurements retain sensor time

Prior-map/relocalization observations retain their measurement timestamp. `BackendScheduler` associates them to the nearest trajectory node within an explicit configured gate; it does not silently rewrite the timestamp. Association misses are counted as policy drops rather than graph corruption.

### ADR-017: Snapshot observers may only perform O(1) handoff

`BackendScheduler` invokes a coarse observer only after a consistent Snapshot is published. The observer may atomically exchange immutable ownership but may not serialize maps, build ROS messages, generate grids or perform registration. Consumer failures are isolated and counted; ROS optimized-path generation is latest-version/coalesced on a timer.

### ADR-018: Submap lifecycle begins as bounded metadata, not a new mapping algorithm

Current single-session trajectories roll to a monotonic `SubmapId` after `BackendConfig::submap_max_keyframes` and mark the old Submap finished. This changes ownership/lifecycle metadata only; it does not invent overlapping local-map optimization or alter frontend scan matching. Spatial/overlapping Submaps require separate validated work.

### ADR-019: Queue policy follows the value of stale work

Accepted KeyFrames and graph constraints use reliable bounded delivery and drain on shutdown. Expensive relocalization queries and unconsumed relocalization results use latest-only replacement because an older global estimate is stale once a newer same-purpose observation exists. Cooperative cancellation is requested for active core relocalization work. ROS visualization and Snapshot-source occupancy publication coalesce immutable Snapshots; rebuild work runs on a dedicated latest-only adapter worker. `MapSaveScheduler` defines a reliable bounded FIFO policy for final products and PCD fragment sets. Snapshot-derived occupancy uses latest-only handoff, cooperative cancellation and explicit scratch/point/materialized-cell budgets; worker shutdown reports cancellation separately from supersession.

### ADR-020: Global alignment has one system authority

`GlobalAlignmentState` is the semantic owner of versioned `T_map_odom` on the standard Super-LIO and Point-LIO paths. A localization worker may only submit a validated, fixed-size `RelocalizationUpdate` to `GlobalAlignmentUpdateMailbox`; the downstream/system execution context commits it. Optional backend refinement returns a candidate to that same writer. `ROSWrapper` receives the committed transform only as a TF/publication cache and is never read as backend/localization authority.

### ADR-021: Snapshot and online occupancy are distinct derived products

`source: "snapshot"` rebuilds occupancy from immutable optimized KeyFrame
payloads and publishes a coalesced representation without retaining per-scan
sensor-origin history. `source: "online"` consumes downstream scan clouds and
can retain ray-cast free evidence. Both modes must preserve the configured ROS
topic, frame, resolution and Nav2 value encoding; width/height/origin and cell
counts are product-specific and require explicit comparison evidence rather
than an assumed byte-for-byte match. No frontend API depends on either mode.

## Dependency Violations Still Present

- `SuperLIO` and `PointLIOFrontend` no longer depend on `ROSWrapper`; active
  applications compose explicit sensor/publication bridges. `ROSWrapper` is a
  fallback facade that delegates to those bridges for external compatibility
  callers.
- `SmallPointLio` vendor types are isolated in `point_lio_adapter`; the
  standard Point-LIO frontend has no ROS/vendor parameter API.
- The `basic` compatibility facade still exposes PCL/glog point/cloud types;
  `basic_core` itself is Eigen/Threads-only.
- Production KISS/VGICP still lives inside `PriorMapLocalization` rather than implementing the new `Relocalizer` composition; this is a dependency migration, not a remaining standard `T_map_odom` ownership cycle.
- `OccupancyGridPublisher` remains a ROS/PCL adapter, while the authoritative
  occupancy representation/building lives in ROS-independent
  `OccupancyGridBuilder`.
- `adagraph_legacy_backend` is an explicit compatibility/rollback target only. The standard graph/map path is exclusively `BackendScheduler` -> `PoseGraph` -> `MapManager` -> `MapSnapshot`.

## Performance Risks Still Present

- Super-LIO deskew performs an approximate `O(points * propagated IMU states)` interval search.
- Super-LIO now pays one queue mutex/CV operation and alternates pooled scan buffers; Phase 3 measured zero queue waits, ~0.15 ms downstream work and about 4.6% peak-RSS increase versus Phase 2 from double buffering.
- Prior localization may deep-copy the current scan and rebuild/allocate the local submap/crop/registration buffers.
- Occupancy update and dense message generation hold a coarse grid mutex.
- Snapshot publication copies light metadata for all KeyFrames/constraints at commit cadence; point storage is immutable-shared. Long-duration scaling has not yet been profiled.
- Point-LIO no longer has a duplicate accumulated point map, but retained
  KeyFrame payloads still need a system-level map-memory budget. The bounded
  Super fragment map is temporary compatibility debt and should be deleted
  with Super-LIO rather than optimized.
- Super OctVox uses a pointer-heavy list/hash-iterator layout; defer its
  profiling/optimization unless Super-LIO remains after Point-LIO convergence.
  Do not assume the separate Point SmallIVox has the same implementation.
- Point-LIO algorithm execution is synchronous on the application-owned `SlamSystem::ProcessPending()` timer thread; ROS callbacks only convert and enqueue typed values. This is a deliberate deterministic processing domain, not a ROS callback execution path.
- Accepted Super keyframes perform one downsampled IMU-to-base cloud transform/copy before immutable sharing. It is outside the frontend thread; Phase 4 measured no queue waits and stable RSS, but long-run allocation behavior still needs profiling.
- Durable KeyFrame cloud retention increased the Phase 5 Point-LIO run from 131672 KiB (Phase 4) to 146248 KiB. This is authoritative map payload rather than per-Snapshot deep copies, but still requires Submap compaction/unloading policy.
- Prior-map Point-LIO validation peaks at 203176–206136 KiB RSS while the immutable KeyFrame map, prior PCL map and registration temporaries coexist. Super-LIO prior-map validation peaks at 137912 KiB.

These remain static audit findings until Phase 0 records measured latency and memory baselines.

## Validation Status

| Validation | Status | Evidence/notes |
| --- | --- | --- |
| Architecture document consistency | Passed | `git diff --check` passed; documents reflect the audited working tree and Phase 0 transition. |
| Current working-tree build | Passed | `cmake --build /home/sw/AdaGraph-LIO/build/super_lio -j2` passed; the known ROS transform-broadcaster deprecation warning remains. |
| Existing unit tests | Passed | `colcon test --packages-select basic super_lio --return-code-on-test-failure`; 6 executables / 49 test cases passed. |
| Super-LIO dataset baseline | Passed | room_01 at 1x: 471 poses, 27.293448 m, APE RMSE 0.415933 m, wall 73.628472 s, peak RSS 107392 KiB, 90 keyframes. |
| Point-LIO dataset baseline | Passed | room_01 at 5x: 1021 odometry messages, 27.982562 m, APE RMSE 0.392974 m, wall 15.347771 s, peak RSS 132368 KiB; downstream 679/679/0. |
| Frontend latency baseline | Partial by design | Legacy Super-LIO exposes stage mean/max only: DownSample 0.310289/0.445852 ms, Observe 0.442535/0.889396 ms, Undistort 0.206492/0.485550 ms, UpdateMap 0.029997/0.079669 ms. Point-LIO exposes downstream mean 0.046057 ms. P50/P95/P99 require Phase 1 common per-frame observation. |
| Peak memory baseline | Passed | GNU time maximum RSS is captured in each case summary. |
| Phase 1 seam tests | Passed | 3 new tests prove storage identity/no copy, safe empty/invalid views and explicit pose/cloud frames. Total suite is now 7 executables / 52 test cases. |
| Phase 1 Super-LIO regression | Passed | room_01 at 1x: path 27.388482 m, APE RMSE 0.413226 m, wall 73.814626 s, peak RSS 107608 KiB. Relative to pre-seam: APE -0.65%, wall +0.25%, RSS +0.20%. |
| Phase 1 Point-LIO regression | Passed | room_01 at 5x: path 27.949183 m, APE RMSE 0.395059 m, wall 15.278430 s, peak RSS 129320 KiB, downstream 713/713/0. Relative to pre-seam: APE +0.53%, wall -0.45%, RSS -2.30%. |
| Phase 2 core/domain tests | Passed | ROS-independent time, composition/inverse/frame mismatch, M2DGR extrinsic, move-only cloud storage, IMU/config validation and independent backend config tests pass. Selected total: 8 executables / 61 cases. |
| Phase 2 Super-LIO regression | Passed | room_01 at 1x: 384 odometry samples, path 27.315567 m, APE RMSE 0.420773 m, wall 73.665035 s, peak RSS 107548 KiB, 82 keyframes. Relative to Phase 1: path -0.27%, APE +1.83%, wall -0.20%, RSS -0.06%. |
| Phase 2 Point-LIO regression | Passed | room_01 at 5x: 904 odometry messages, path 27.766812 m, APE RMSE 0.391140 m, wall 15.285460 s, peak RSS 134940 KiB, downstream 714/714/0. A repeat passed at 133640 KiB; no new point-cloud allocation exists in the changed path. Relative trajectory/wall to Phase 1: path -0.65%, APE -0.99%, wall +0.05%. |
| Phase 3 KeyFrame/cloud tests | Passed | `test_core_domain` now passes 9 cases including immutable cloud storage identity/lifetime and strict `T_odom_base`/base-cloud KeyFrame validation; `test_frontend_result` passes 3 cases. |
| Phase 3 Point downstream extraction | Passed | `phase3_20260812_point_downstream_extract`: room_01 at 5x, 962 odometry messages, path 27.855464 m, APE RMSE 0.395697 m, wall 15.293738 s, peak RSS 131860 KiB; downstream 710/710/0, max depth 1, mean 0.0417424 ms, 100 keyframes. Point frontend no longer owns backend, relocalization, occupancy/global map or PCD save. |
| Phase 3 build and tests | Historical (pre-retirement) | `colcon build --symlink-install --packages-up-to super_lio -DBUILD_TESTING=ON`; 8 selected test executables / 63 test cases passed. At that point `super_lio_node`, `point_lio_node` and the now-retired compatibility node linked successfully. |
| Phase 3 final Super-LIO regression | Passed | `phase3_20260812_frontend_downstream_final`: room_01 at 1x, 416 poses, path 27.257335 m, APE RMSE 0.426561 m, wall 73.631338 s, peak RSS 112504 KiB, 88 keyframes. Reliable downstream 416/416, max depth 1, zero backpressure waits, mean 0.149290 ms. Versus Phase 2: path -0.21%, APE +1.38%, wall -0.05%, RSS +4.61%. |
| Phase 3 final Point-LIO regression | Passed | Same suite: room_01 at 5x, 1065 poses, path 27.943858 m, APE RMSE 0.389245 m, wall 15.272928 s, peak RSS 135484 KiB, 100 keyframes. Downstream 715/715/0, max depth 1, mean 0.0402662 ms. Versus Phase 2: path +0.64%, APE -0.48%, wall -0.08%, RSS +0.40%. |
| Phase 4 build/tests | Passed | Incremental `super_lio` build passed. Selected `basic` and `super_lio` results pass 29 + 55 cases; core domain has 19 cases, legacy AdaGraph 11 and ISAM2 adapter 2. Historical unrelated livox lint XML remains outside the selected package run. |
| Phase 4 Super-LIO final regression | Passed | `phase4_20260812_backend_decomposition_final`: 380 poses, path 27.299311 m, APE RMSE 0.416368 m, wall 73.816140 s, RSS 110576 KiB. Legacy/typed graph 78/78 nodes, 77 constraints, optimizer 78/0 updates/failures, max delta `7.22e-15 m / 4.44e-16 rad`, queue depth one/no waits. |
| Phase 4 Point-LIO final regression | Passed | Same suite: 909 poses, path 27.670180 m, APE RMSE 0.393636 m, wall 15.609358 s, RSS 131672 KiB. Legacy/typed graph 101/101 nodes, 100 constraints, optimizer 101/0, max delta `4.47e-15 m / 7.77e-16 rad`, queue depth one/no drops or waits. |
| Phase 5 build/tests | Passed | `colcon build --symlink-install --packages-select super_lio -DBUILD_TESTING=ON`; 61 `super_lio` tests pass. Snapshot concurrency/products, observer isolation/order and Submap rollover are covered. The immediately preceding selected `basic` suite passed 29 tests. |
| Phase 5 Super-LIO final regression | Passed | `phase5_20260812_submap_lifecycle_final`: 425 poses, path 27.272239 m, APE RMSE 0.412950 m, wall 73.635784 s, RSS 115108 KiB. 86 nodes/85 constraints, Snapshot v86, one active Submap, observer failures zero, queue depth one/no waits. |
| Phase 5 Point-LIO final regression | Passed | Same suite: 922 poses, path 27.782171 m, APE RMSE 0.390696 m, wall 15.271848 s, RSS 146248 KiB. 101 nodes/100 constraints, Snapshot v101, two Submaps/one finished, observer failures zero, queue depth one/no waits. |
| Phase 6 build/core tests | Passed | Incremental build passed; selected `basic`/`super_lio` suites pass 29 + 67 tests. Tests cover typed result validity, exact base-extrinsic algebra, candidate/verification composition, cancellation/replacement, immutable LocalizationMap construction, GlobalAlignmentState versioning and latest-only mailbox behavior. |
| Phase 6 standard disabled-localization regression | Passed | `phase6_20260812_relocalization_contract`: Super-LIO APE 0.413282 m / wall 73.633907 s; Point-LIO APE 0.396845 m / wall 15.274114 s. Existing odometry-only graph/Snapshot gates passed. |
| Phase 6 Super-LIO real prior-map alignment | Passed | `phase6_20260812_alignment_mailbox_super`: 391 poses, path 27.284098 m, APE 0.412392 m, wall 73.654244 s, RSS 137912 KiB. Alignment mailbox/commit/reject = 32/32/0, ROS cache parity `0 m / 9.41e-18 rad`, downstream mean 0.213683 ms and no queue waits. |
| Phase 6 Point-LIO real prior-map alignment | Passed | `phase6_20260812_alignment_single_writer_point`: 1032 poses, path 27.920657 m, APE 0.389065 m, wall 15.472612 s, RSS 203176 KiB. Alignment mailbox/commit/reject = 62/62/0; cache parity `5.55e-17 m / 3.01e-18 rad`. |
| Phase 6 Point-LIO backend-refinement alignment (historical legacy-facade run) | Passed | `phase6_20260812_alignment_single_writer_factor_point` predates removal of the facade from the standard path: 101 nodes/128 constraints, 129 optimizer updates/0 failures, 29 timed global observations dropped by association policy, 59 relocalization events plus backend refinements produced 135 ordered system commits/0 rejects. APE 0.391373 m, wall 15.286564 s, RSS 206136 KiB; ROS cache parity `1.11e-16 m / 2.33e-10 rad`. Retained for historical comparison only. |
| Phase 6 production relocalizer boundary | Passed | Removed policy-gate state from `PclRegistrationVerifier`, kept it in `PriorMapLocalization`, and added LOST retry-cooldown short-circuiting. `colcon build --symlink-install --packages-select super_lio -DBUILD_TESTING=ON` passed; `colcon test --packages-select super_lio --return-code-on-test-failure` passed 68/68 tests; `git diff --check` passed. |
| Phase 6 deterministic prior-map startup/recovery | Passed | The vendored KISS parallel voxel reducer documents run-to-run scheduling variation. M2DGR/MID360 recovery configs now explicitly request stable voxel reduction and dual ratio candidate retrieval; unchanged VGICP/GICP verification selects only a converged candidate satisfying the existing fitness policy. `test_kiss_matcher_sampling` proves stable reduction output. Point-LIO `m2dgr_room01_point_lio_5x_snapshot_backend_factor` passed three times under `OMP_NUM_THREADS=1` (two with unrestricted TBB): 2619-3167 poses, path 29.750840-30.495281 m, APE RMSE 0.374108-0.396743 m, RSS 218520-231984 KiB; every run had startup `3/1/2`, first success `10.71 s`, and passed scheduler/quality gates. Super-LIO `m2dgr_room01_super_lio_1x_prior_map` passed: 477 poses, path 27.366470 m, APE RMSE 0.418355 m, RSS 133420 KiB, startup `2/1/1`, first success `5.91134 s`; scheduler 128/128/128 with 0.090313 ms queue-wait mean and 18.6473 ms end-to-end mean. |
| Phase 7 occupancy/map-IO core extraction | Passed | `OccupancyGridBuilder` borrowed-cloud raycast, reset/config, immutable `MapSnapshot` rebuild, atomic binary-PCD write and FIFO fragment write/merge tests pass. `ctest --test-dir build/super_lio --output-on-failure` passed 7/7 test programs; the current gtest listing contains 66 cases. Outer `livox_ros_driver2` lint failures are pre-existing/out of package scope. |
| Phase 7 Point-LIO occupancy/IO boundary regression | Passed | `phase7_20260812_occupancy_io_point`: room_01 at 5x, 1007 poses, path 27.905317 m, APE RMSE 0.389853 m, wall 16.723096 s, peak RSS 150156 KiB; backend 101/101 nodes, 100 constraints, optimizer 101/0, queue drops 0. The M2DGR config keeps occupancy disabled, so this validates the shared downstream/scheduler boundary rather than live grid frame parity. |
| Phase 7 Super-LIO occupancy/IO boundary regression | Passed | `phase7_20260812_occupancy_io_super`: room_01 at 1x, 494 poses, path 27.377231 m, APE RMSE 0.418780 m, wall 73.654015 s, peak RSS 112992 KiB; backend 91/91 nodes, 90 constraints, optimizer 91/0, downstream 494/494, max queue 1 and zero backpressure waits. The M2DGR config keeps occupancy disabled, so this remains the no-grid boundary baseline. |
| Phase 7 enabled occupancy source regression | Passed with documented product difference | `phase7_20260812_occupancy_parity` and `phase7_20260812_occupancy_point_parity`: both Super-LIO and Point-LIO Snapshot/online runs received transient-local `/map` with `frame_id=world` and `resolution=0.1`. Snapshot/online origin, dimensions and occupied/free/unknown counts are captured in `occupancy_summary.json` and compared by `compare_occupancy_summaries.py`; Snapshot has no per-scan free-ray evidence by design, so byte/cell-count equality is not claimed. |
| Phase 7 production PCD save/readback regression | Passed | `phase7_20260812_map_save_regression`: Super-LIO wrote 21 FIFO fragments plus final merge and Snapshot PCD (`MapSaveScheduler` 23/23/0); Point-LIO wrote final and Snapshot PCD (`2/2/0`). `inspect_pcd.py` found finite binary payloads and matching declared/actual point counts; independent `pcl::io::loadPCDFile` read every output successfully. |
| Phase 7 Snapshot rebuild budget regression | Passed | `adagraph_phase7_occupancy_budget_regression_final_20260813`: Point-LIO at 5x produced 226 handoffs/requests, 216 completed and 10 superseded cancellations; peak scratch `689,928 B`, APE RMSE `0.379507 m`, 4,747 poses. Super-LIO at 1x produced 224 handoffs/requests and 224 completed, with no cancellations/discards/budget overruns/failures; peak scratch `725,896 B`, APE RMSE `0.420714 m`, 441 poses. Both stayed below the configured 64 MiB scratch budget and passed terminal-outcome accounting. |
| Phase 7 mapping-time Nav2 artifact | Passed for format/integration | `Nav2GridArtifact.WritesMapServerCompatiblePgmAndMetadataAtomically` verifies trinary `0/205/254` pixels, top-row PGM order, origin, atomic overwrite and invalid input rejection. A local Nav2 `map_server` configured from the generated YAML returned `resolution=0.5`, origin `(-1.0, 1.5)` and data `[-1, 0, 100, -1]`, proving occupied/free/unknown semantics survive the actual Nav2 reader. The artifact is still default-disabled and online-path long-run profiling remains required before deployment. |
| Phase 8 Point-LIO transport boundary | Passed | Standard `point_lio_node` explicitly sets `SensorBridge` range/filter to lossless values and reads DDS reliability/depth from `point_lio.input.*`; `verify_core_boundaries.py` passed, incremental build passed, and `ROS_LOG_DIR=/tmp/adagraph_lio_ros_logs ctest --test-dir build/super_lio --output-on-failure` passed 13/13 tests. |
| Phase 8 retired relocation compatibility path | Passed | Removed `SuperLIOReLoc`, `relocation_node`, `adagraph_relocation_compat`, the dedicated relocation config/launch/RViz assets and the corresponding configuration reader. Standard `super_lio_node` and `point_lio_node` retain the scheduler-backed prior-map path. `verify_core_boundaries.py` now rejects reintroduction of the retired files and target; incremental build passed on 2026-08-13. |
| Static runtime-state cleanup | Passed | The retained Super-LIO and prior-map localization paths contain no stateful function-local counters/flags for initialization, map loading, update delay or visualization. |
| Phase 8 full rosbag/text Super-LIO parity | Passed | `phase8_20260813_ros_free_dataset/super_lio`: full `room_01` conversion/replay, 10,785 packets, 719 offline poses, typed backend 100/99 nodes/constraints, no rejected packets; timestamp-aligned coverage 494/494, translation RMSE `0.012104 m`, max `0.029173 m`, rotation max `0.008382 rad`. Gate limits were coverage >= 1.0, translation RMSE <= 0.02 m, translation max <= 0.05 m and rotation max <= 0.02 rad. |
| Phase 8 full rosbag/text Point-LIO parity | Passed | `phase8_20260813_ros_free_dataset/point_lio`: full `room_01` conversion/replay, 10,785 packets, 713 offline poses, typed backend 101/100 nodes/constraints, no rejected packets; timestamp-aligned coverage 1007/1007, translation RMSE `0.003662 m`, max `0.012516 m`, rotation max `0.003103 rad`, with the same gates. |
| Phase 8 offline map-save validation | Passed | `adagraph_dataset_runner --save-map` explicitly invokes `saveSuperMap()`/`savePointMap()` after lifecycle drain. Super-LIO produced final, Snapshot and two FIFO fragment PCDs; Point-LIO produced final and Snapshot PCDs. `inspect_pcd.py` found finite binary payloads and matching declared/actual counts. |
| Phase 8 clean build/install and tests | Passed | Empty `/tmp/adagraph_clean_build` and `/tmp/adagraph_clean_install` builds configured `basic` and `super_lio` independently. `super_lio` consumed the freshly installed `basic 2.0.0` via `CMAKE_PREFIX_PATH`; the installed set includes `point_lio_node`, `super_lio_node`, and `adagraph_dataset_runner`, with no retired relocation executable. `ctest` passed `basic` 2/2 and `super_lio` 14/14 under `ROS_LOG_DIR=/tmp/adagraph_clean_ros_logs`; `verify_core_boundaries.py` passed for 50 files/13 targets. |
| Phase 8 snapshot-based backend refinement (latest final) | Passed | `/tmp/adagraph_snapshot_refinement_regression_final/m2dgr_room01_point_lio_5x_snapshot_backend_factor`: standard Point-LIO room_01 at 5x with prior map, `use_localization_factor=true`, `drive_map_to_odom=true`; 2434 poses, path 29.605352 m, APE RMSE 0.389901 m, wall 15.729089 s, peak RSS 225260 KiB. Typed scheduler: 98 nodes/117 constraints, Snapshot v118, 118 optimizer updates/0 failures, 40 global constraints dropped by timestamp policy; 61 relocalization mailbox events yielded 216 completed/published queries and 141 ordered system commits/0 rejects; ROS cache parity `9.1546e-10 m / 1.78001e-10 rad`. `legacy_backend_execution=false`, and the standard log contains no `adagraph_backend.cpp` execution. The manifest gate passed. The earlier 2191-pose run remains under `phase8_20260813_snapshot_backend_factor_point_escalated` for comparison. |

## Next Recommended Action

1. Keep the Snapshot-versus-online occupancy product contract explicit and
   retain the completed rebuild scratch-memory/cancellation budget case in
   regression runs.
2. Before enabling online occupancy in any deployment, add the lock-hold,
   update-tail-latency, queue-impact, RSS and long-run measurements described
   in Remaining P1 Issues; Snapshot remains the standard default meanwhile.
3. Drive Point-LIO memory convergence by profiling simultaneous PCL
   temporary/local-submap/registration ownership and retained KeyFrame/map
   payloads. Keep Super-LIO only as a regression baseline; do not optimize its
   fragment buffer or OctVox layout unless the convergence decision changes.
4. Continue Phase 8 cleanup in this order: explicit instance configuration in
   place of `g_*`; a true lightweight core split from `basic`; explicit
   SmallPointLio construction parameters; then removal of unused external
   compatibility facades.
5. Keep the prior-map quality/latency gate as a regression contract; it now
   has repeatable Point-LIO and Super-LIO evidence, while parallel descriptor
   internals need not have bitwise-identical inlier counts.
6. Continue shrinking `PriorMapLocalization` toward the generic retriever/verifier seams while preserving the existing prior-map quality/latency gates.
7. Retire `adagraph_legacy_backend` after remaining legacy callers no longer require it; the standard duplicate graph owner has already been removed and Snapshot-based refinement is recorded above.

## Recorded Direction — Point-LIO Convergence

The intended product direction is to converge on Point-LIO as the sole
production frontend and retire Super-LIO after a measured Point-LIO memory
reduction. This is a direction, not authorization to delete Super-LIO yet.

Before that retirement, Point-LIO must meet the following gates on the recorded
and deployment-representative prior-map workloads:

- normal operation peak RSS: `<= 140 MiB` (target range `120–140 MiB`);
- prior-map localization peak RSS: target `<= 160 MiB`;
- no regression in trajectory accuracy, prior-map success/false-positive
  rejection, recovery latency, queue terminal accounting or map-save/occupancy
  contracts;
- repeatable real-robot evidence for the selected deployment sensor/config.

The primary investigation targets are avoidable simultaneous ownership of PCL
temporary clouds/local submaps/registration products, prior-map working-set
retention and downstream map-product staging. Maintain the current Super-LIO
regression as a comparison baseline until all gates pass; then deprecate it,
remove its application/configuration/build assets in a separate reviewable
change, and reduce the two-frontend test matrix to Point-LIO.

## Source-of-Truth and Recovery Procedure

When starting a new phase or recovering from context loss:

```text
read CURRENT_ARCHITECTURE.md
read TARGET_ARCHITECTURE.md
read REFACTORING_ROADMAP.md
read REFACTORING_STATUS.md
        ↓
git status / git diff / git log
        ↓
inspect actual code involved in the current phase
        ↓
implement
        ↓
build / test / dataset validation
        ↓
update CURRENT_ARCHITECTURE.md and REFACTORING_STATUS.md
```

If code and documents disagree, do not force code to match stale text. Verify the implementation, correct the documents and record any changed architecture decision above.
