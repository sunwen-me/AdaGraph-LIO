# Point-LIO Submap Relocalization and Recovery Design

Status: implementation design and progress record, 2026-08-14.

For the current code boundary, unfinished work and the required continuation
order, read
[`POINT_LIO_SUBMAP_RECOVERY_HANDOFF.md`](POINT_LIO_SUBMAP_RECOVERY_HANDOFF.md)
first.  This document remains the full architectural rationale.

This document defines the recommended design for extending the current
Point-LIO-only system with persistent submaps, global relocalization, local
frontend recovery and later loop closure. It is a design document. None of
the recovery or loop-closure behavior described here is considered complete
until the implementation and regression evidence exist. This document is the
source of truth for the submap/recovery work; implementation status is recorded
near the end so that the design is not confused with the current runtime.

## Decision Summary

The system should combine the following ideas while keeping Point-LIO as the
only realtime frontend:

- Point-LIO remains responsible for continuous local `odom -> base` motion.
- Cartographer/GLIM-like local submap ownership is used for map storage,
  coordinate isolation and multi-trajectory organization.
- Voxel-SLAM-like failure handling is used for `LOST -> new trajectory ->
  reinitialization -> global reconnect`.
- Super-LIO's map-reuse idea is adopted only as a concept: a validated
  historical submap can be used both for relocalization and as a read-only
  local-map seed. Super-LIO code is not reintroduced.
- CT-ICP-like map-write protection is used to prevent bad registrations from
  contaminating the active map.
- Existing KISS-Matcher and VGICP/GICP adapters remain the registration
  implementation.

The default recovery policy is:

```text
healthy Point-LIO
  -> detect persistent degradation
  -> freeze old local/global map writes
  -> retrieve and geometrically verify a historical submap
  -> create a new Point-LIO trajectory
  -> reset the filter and seed a new local voxel map
  -> verify several frames with the seed read-only
  -> derive map->odom for the new trajectory
  -> enable online map writes after a cooldown
  -> commit the new trajectory to the long-term graph
```

Direct pose correction without a frontend reset is reserved for the case in
which the Point-LIO local state and local map are still healthy and only the
global alignment has drifted.

## Recommended Integrated Design

The system should not select one paper as a complete blueprint. The practical
design is a controlled composition of mechanisms that solve different parts of
the problem:

| Problem | Adopted mechanism | Boundary in this project |
| --- | --- | --- |
| Persistent local map units | Cartographer/GLIM-style submaps | `MapManager` owns immutable `SubmapSnapshot` products |
| Real frontend divergence | Voxel-SLAM-style new trajectory/session | `TrajectoryManager`/frontend recovery owns Point-LIO lifecycle |
| Prior geometry reused by LIO | Super-LIO-style prior-map reuse | Only the matched submap is loaded as a read-only seed; no Super-LIO code |
| Bad scan protection | CT-ICP-style conservative map admission | health/write gate blocks local and global insertion |
| Registration implementation | Existing KISS-Matcher + VGICP/GICP | PCL/vendor code remains in adapters |

The central rule is:

```text
global relocalization result
  = (T_map_base, covariance, submap_id, quality evidence)
  -> new Point-LIO odom trajectory
  -> matched submap transformed into the new odom frame
  -> prior seed is read-only during verification
  -> online points enter a rollbackable provisional layer
  -> stable frames are committed to the graph and map
```

This is an engineering composition, not a claim that one publication already
implements the entire chain. Point-LIO remains the only realtime odometry
frontend, and `map -> odom` remains a derived global alignment rather than a
second hidden state inside the estimator.

### Soft correction versus hard recovery

The runtime must distinguish global drift from a broken local estimator:

| Evidence | Action | Point-LIO state | Map writes |
| --- | --- | --- | --- |
| Local residuals and observability healthy; only global alignment changed | Soft correction | Keep current ESKF and local map | Continue normal admission |
| Short mismatch; no evidence of map pollution; IMU healthy | Retry or bounded soft update | Keep state, increase uncertainty if needed | Delay questionable frames |
| Persistent degeneration or bad scans entered local map | Hard recovery | New trajectory, reset ESKF/queues/local map | Freeze old map; seed is read-only |
| IMU/timestamp health also bad | Full reinitialization | Re-bootstrap gravity, bias and covariance | No writes until verification |

The default for a confirmed Point-LIO loss is hard recovery. A pose-only
`setPose()` is not a recovery API because it leaves the old ESKF covariance,
velocity/bias/gravity state and local map lifecycle inconsistent.

### Recovery transaction

Recovery is treated as one transaction with an explicit commit point:

```text
TRACKING
  -> SUSPECT: hysteresis detects persistent degradation
  -> LOST: freeze local/global map admission
  -> RELOCALIZING: retrieve top-K submaps and geometrically verify
  -> RESTARTING: allocate O_j, reset Point-LIO and load the matched seed
  -> VERIFYING: prior layer read-only, online layer provisional
       | reject -> discard online layer and retry another candidate
       | stable -> RECOVERED
  -> COMMITTED: flush verified keyframes and add graph reconnect constraint
```

The relocalization scan initializes the new trajectory and is not counted a
second time as the first normal Point-LIO LiDAR factor. Normal updates start
from subsequent scans. This avoids double-applying one scan to two different
state transitions.

### Required recovery state

The frontend recovery contract must carry the state that makes Point-LIO
consistent, rather than only a global pose:

```cpp
struct FrontendRecoveryRequest {
  TrajectoryId trajectory_id;
  StampedTransform3d T_map_base;
  StampedTransform3d T_odom_base;
  Eigen::Vector3d velocity;
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d accel_bias;
  Eigen::Vector3d gravity;
  Covariance15d covariance;
  ImmutablePointCloud seed_cloud_odom;
  bool reinitialize_imu = false;
};
```

The exact covariance and vendor types may differ in code, but the contract
must cover ESKF state, covariance, IMU bootstrap policy, queue/timestamp cut,
and local-map replacement atomically. The matched submap identity must travel
with the registration result so that a pose cannot accidentally be paired with
a different seed.

## Current Baseline

The current production path is:

```text
SensorBridge
  -> SlamSystem typed queues
  -> PointLIOFrontend / Small Point-LIO
  -> PointLIOPipeline
       -> BackendScheduler / iSAM2 / MapManager
       -> PriorMapLocalization / RelocalizationScheduler
       -> publication, occupancy and map IO
```

The current ownership boundaries are important:

- `PointLIOFrontend` owns the realtime estimator and vendor local voxel map.
- `PointLIOPipeline` adapts frontend frames into keyframes and global
  observations.
- `MapManager` owns durable keyframe payloads, optimized poses, submap
  membership and graph constraints.
- `MapSnapshot` is the immutable publication boundary.
- `PriorMapLocalization` produces a global registration result and can update
  `map -> odom`; it does not rewrite Point-LIO's ESKF state or vendor map.
- `BackendScheduler` is the single writer for the pose graph and map manager.

Finished `SubmapSnapshot` records now contain an anchor-defined local frame,
immutable downsampled geometry, a seed cloud, bounds and a deterministic
descriptor. Active submaps intentionally retain membership metadata until they
are finished. The current geometry is an initial shared representation for
registration and seeding; separate coarse/fine LODs and persistent descriptor
indexing remain later work.

## Design Goals

The design must:

1. Preserve Point-LIO's realtime behavior and accuracy in the healthy path.
2. Keep `odom` locally continuous during normal operation.
3. Preserve the existing global map when a frontend trajectory is lost.
4. Allow a validated submap to support retrieval, registration and local-map
   seeding.
5. Prevent a false relocalization from writing irreversible data.
6. Support multiple frontend trajectories in one persistent map.
7. Keep ROS, PCL and vendor types outside the typed core contracts.
8. Permit lazy loading and bounded local-map materialization for large maps.
9. Make recovery behavior testable with synthetic fault injection.

Non-goals for the first implementation:

- Replacing Point-LIO's estimator with another odometry algorithm.
- Making submaps independent iSAM2 graph nodes immediately.
- Loading the entire global map into the Point-LIO local map.
- Treating a descriptor match as a valid global pose without registration.
- Using loop closure as a substitute for realtime frontend recovery.

## Coordinate and Ownership Model

Use the following frame convention:

```text
M       persistent global map frame
O_j     local odometry frame for frontend trajectory j
S_i     immutable local frame of submap i
B       robot base frame
```

The frontend owns and updates `O_j -> B`. The global graph owns optimized
poses in `M`. A submap stores geometry in `S_i`; graph optimization changes
`M -> S_i`, not the stored submap points.

For a validated registration at time `t`:

```text
T_M_Oj(t) = T_M_B(t) * inverse(T_Oj_B(t))
```

For a submap seed:

```text
T_Oj_Si = inverse(T_M_Oj) * T_M_Si
point_Oj = T_Oj_Si * point_Si
```

After graph optimization, `map -> odom` is derived again from a current
optimized keyframe:

```text
T_M_Oj = T_M_B_graph * inverse(T_Oj_B_lio)
```

The global alignment must not be treated as an additional uncontrolled state
inside Point-LIO. It is a derived cross-layer transform published through
`GlobalAlignmentState`.

## Map Data Model

### Keyframes

Keyframes remain the authoritative pose-graph nodes for the first version.
They contain:

- `NodeId`, `KeyFrameId`, `SessionId` and `TrajectoryId`;
- `T_odom_base` from Point-LIO;
- optimized `T_map_base` from the backend;
- immutable base-frame cloud payload;
- quality and uncertainty metadata.

The current `KeyFrameSnapshot` already provides most of this contract.

### Submaps

Extend the current submap metadata with a durable logical submap object:

```cpp
struct SubmapSnapshot {
  SubmapId id;
  SessionId session_id;
  TrajectoryId trajectory_id;
  SubmapState state;

  StampedTransform3d T_map_submap;
  ImmutablePointCloud cloud_submap;
  ImmutablePointCloud seed_cloud_submap;
  std::vector<float> descriptor;
  BoundingBox3d bounds;
  std::vector<KeyFrameId> keyframe_ids;

  double static_ratio = 0.0;
  double quality_score = 0.0;
  std::uint64_t version = 0;
};
```

`BoundingBox3d` is illustrative; use the existing geometry types or add a
small ROS-independent type if the repository has no equivalent.

The representations have separate purposes:

| Representation | Purpose | Lifetime |
| --- | --- | --- |
| Descriptor | Place recognition and candidate ranking | Resident in index |
| Coarse cloud | Coarse registration and overlap check | Cache or lazy load |
| Fine cloud | VGICP/GICP verification | Lazy load per candidate |
| Seed cloud | Point-LIO local-map initialization | Materialize on recovery |
| Keyframe payloads | Pose graph, map save and diagnostics | Existing map policy |

The first version should keep the keyframe graph and use submaps as storage,
retrieval and recovery units. A future submap-level graph can be added after
the keyframe implementation is validated.

### Prior and Online Local Layers

The recovered frontend should not use one mutable voxel map for everything.
Use two logical layers:

```text
prior_ivox   validated historical seed, read-only during recovery
online_ivox  points produced by the new frontend trajectory
```

The nearest-neighbor query may read both layers. During verification, only
`online_ivox` is provisional. A failed recovery can discard it without
modifying the historical seed.

## Frontend Health and Map Write Barrier

Add a typed frontend health state owned by the Point-LIO downstream boundary:

```text
TRACKING -> SUSPECT -> LOST -> RELOCALIZING -> RESTARTING
                                      -> VERIFYING -> RECOVERED
```

Health evidence should use the existing feature and uncertainty data:

- inlier/effective update ratio;
- Hessian condition number and trace;
- effective rank;
- effective point count;
- IMU innovation and timestamp continuity;
- short-window motion consistency.

Use hysteresis instead of a single-frame threshold. Initial engineering values
may be configured as follows and must be calibrated against recorded data:

```yaml
frontend_health:
  suspect_bad_frames: 3
  lost_bad_frames: 10
  recover_good_frames: 5
  recovery_cooldown_frames: 20
  min_inlier_ratio: 0.30
  max_condition_number: 10000.0
  min_effective_rank: 4
```

These are starting values, not validated deployment limits.

Write policy:

| State | Point-LIO local write | Global keyframe write |
| --- | --- | --- |
| `TRACKING` | Enabled | Enabled if keyframe quality passes |
| `SUSPECT` | Freeze or delayed admission | Disabled for bad frames |
| `LOST` | Disabled | Disabled |
| `RELOCALIZING` | Historical seed read-only | Disabled |
| `VERIFYING` | Online layer only | Provisional buffer only |
| `RECOVERED` | Enabled | Flush verified buffer, then normal |

The vendor path currently inserts points into its local voxel map during point
processing. To enforce a frame-level health decision, add a bounded pending
frame buffer or a one-frame-lagged map-admission policy. Do not make the
realtime Point-LIO thread wait for VGICP, descriptor retrieval or backend
queue capacity.

## Recovery Protocol

### 1. Enter `LOST`

When the health monitor confirms persistent degradation:

- freeze old local-map insertion;
- stop accepting global keyframes;
- freeze the old `TrajectoryId`;
- retain a bounded LiDAR/IMU recovery window;
- submit the newest query to the relocalization scheduler.

### 2. Retrieve and Verify a Submap

The relocalizer returns a pose and the exact matched submap identity:

```cpp
struct SubmapRelocalizationResult {
  StampedTransform3d T_map_base;
  SessionId session_id;
  SubmapId submap_id;
  Covariance6d covariance;
  double fitness = 0.0;
  double inlier_ratio = 0.0;
  double overlap = 0.0;
  bool observable = false;
};
```

The registration pipeline is:

```text
descriptor index
  -> top-K submaps
  -> KISS-Matcher coarse candidates
  -> VGICP/GICP refinement
  -> overlap / fitness / observability gates
  -> neighboring-keyframe consistency
```

KISS-Matcher is currently pairwise rather than a persistent descriptor index.
It should be used after a submap candidate shortlist, not against every
historical keyframe on every query.

### 3. Start a New Frontend Trajectory

Do not call the existing full-system reset for this operation. Add a
frontend-only recovery path that:

1. stops the old frontend segment;
2. increments `TrajectoryId` while preserving `SessionId` and `MapManager`;
3. clears ESKF/local-map queues and old deskew history;
4. recreates the local voxel map;
5. initializes velocity, biases, gravity and covariance according to a
   recovery policy;
6. loads the matched submap seed in the new `O_j` frame;
7. publishes `T_M_Oj` using the registration result.

The required frontend API is conceptually:

```cpp
struct FrontendRecoveryState {
  StampedTransform3d T_odom_base;
  Eigen::Vector3d velocity;
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d accel_bias;
  Eigen::Vector3d gravity;
  Covariance15d covariance;
};

bool resetAndSeed(const FrontendRecoveryState& state,
                  const ImmutablePointCloud& seed_cloud_odom);
```

The exact state type must match the existing frontend/vendor contract. The
important point is that recovery handles filter state, covariance, IMU
initialization and local map together; it is not a `setPose()` operation.

### 4. Verification Window

The relocalization scan is the initializer and should not also be counted as
an independent first LiDAR factor. Start normal Point-LIO measurement updates
from subsequent scans.

During the verification window:

- `prior_ivox` remains read-only;
- new points go only to `online_ivox`;
- global keyframes are held in a bounded provisional buffer;
- output `map -> odom` may be published for navigation;
- the system checks several consecutive healthy frames.

If verification fails, discard `online_ivox` and retry with another candidate.
If verification succeeds, flush the provisional keyframes in order and add a
global relocalization constraint to the first accepted new keyframe.

## Backend Changes Required Before Loop Closure

### Full optimization result for non-local constraints

`Isam2Optimizer` currently returns only newly added nodes for ordinary relative
constraints. A loop closure adds no node, so its optimized historical poses
would not necessarily be copied to `MapManager`.

Add a graph-update flag such as:

```cpp
bool force_full_estimate = false;
```

Set it for loop closures and large cross-trajectory constraints. The optimizer
must return all affected nodes, and `MapManager` must apply them before the
next `MapSnapshot` publication.

### Derived map alignment

Generalize backend alignment so it is not gated only by prior-map localization.
Whenever a reliable graph update changes the current trajectory:

```text
latest optimized T_map_base
  + latest Point-LIO T_odom_base
  -> derive T_map_odom
  -> commit GlobalAlignmentState
  -> publish map->odom
```

The alignment remains a cross-layer derived value and is not added as an
uncontrolled second pose state inside Point-LIO.

### Loop detector input

The current `LoopDetector` accepts `PoseGraphState`, which has no point-cloud
payload. The production detector should receive a read-only map/submap view,
for example:

```cpp
struct LoopDetectionSnapshot {
  std::shared_ptr<const MapSnapshot> map;
  std::shared_ptr<const SubmapIndex> index;
  NodeId query_node;
};
```

Detection and registration must run on a separate worker. The backend snapshot
observer is an O(1), non-blocking handoff and must never execute registration
on the backend single-writer thread.

## Loop Closure After Recovery

Loop closure is a later consumer of the same submap/index infrastructure:

```text
new keyframe
  -> skip recent temporal neighborhood
  -> retrieve old submap candidates
  -> select historical keyframe(s)
  -> KISS/VGICP verify
  -> makeLoopClosure(match_node, query_node, T_match_query, covariance)
  -> submit to BackendScheduler
  -> full affected-node optimization
```

The first implementation should support same-session and same-trajectory
loops. Cross-trajectory loops can reuse the same constraint path after the
recovery trajectory is stable.

## Proposed Modules

The implementation should be split by ownership:

```text
hybrid_slam/mapping/
  submap.h/.cpp
  submap_index.h/.cpp
  submap_products.h/.cpp

hybrid_slam/localization/
  submap_relocalizer.h/.cpp
  recovery_state_machine.h/.cpp

hybrid_slam/backend/
  loop_closure_scheduler.h/.cpp
  graph_update policy changes

lio/
  frontend_health.h/.cpp
  frontend_recovery.h/.cpp
  pcl_submap_registration_adapters.h/.cpp
```

ROS and PCL conversions remain in adapter modules. The core contracts should
use typed poses, immutable clouds, IDs, covariance and immutable snapshots.

## Current Implementation Status

The work is intentionally incremental. The following status reflects the code
on branch `feature/point-lio-frontend` as of 2026-08-14:

| Area | Status | Evidence or remaining work |
| --- | --- | --- |
| Submap identity and local frame | Implemented | `FrameId::kSubmap`; first keyframe is the anchor |
| Finished-submap geometry | Implemented | `MapManager` builds local cloud, seed cloud, bounds and descriptor |
| Snapshot validation | Implemented | `MapSnapshot::valid()`/`consistent()` validate finished geometry |
| Geometry regression test | Implemented | `BuildsFinishedSubmapGeometryInAnchorFrame` passes in `test_core_domain` |
| Frontend health state | Implemented | ROS/YAML typed thresholds and an instance-owned hysteresis monitor |
| Local/global map-write barrier | Implemented | One-frame-lagged vendor online-iVox gate plus same-frame global keyframe gate |
| Prior/online recovery layers | Implemented foundation | `prior_ivox` is read-only; `online_ivox` can be cleared independently; dual-layer nearest-neighbor query is active |
| Frontend-only reset and seed | Implemented foundation | `resetAndSeed()` rebuilds ESKF, queues and two local layers atomically while ingress is disabled |
| New trajectory lifecycle | Implemented foundation | Pipeline rollover preserves graph/map, derives `map -> odom_j`, and queues a recovery constraint after the first new keyframe |
| Submap descriptor index and retrieval | Implemented foundation | Immutable-snapshot radial-descriptor top-K index plus KISS/VGICP/GICP candidate verification and quality gates; recorded-data validation remains |
| Backend reconnect constraint | Implemented foundation | New trajectory recovery constraint path, deterministic cross-trajectory snapshot propagation and ISAM2 full estimates exist; real recovery fault validation remains |
| Loop closure | Implemented foundation | Detector, verifier, latest-only scheduler, node snapshot handoff and scheduler tests exist; runtime remains disabled by default pending data evidence |

The current implementation checkpoint has been verified with:

```text
cmake --build build/hybrid_slam -j2
ctest --test-dir build/hybrid_slam --output-on-failure
```

These commands verify the current core, Point-LIO lifecycle, submap
relocalizer, loop scheduler and ROS application build. They do not claim that
real-data automatic recovery or loop closure improves trajectory quality.

## Implementation Phases

### Phase 1: Map Contract (implemented)

- Define submap local frame and transform invariants.
- Extend `SubmapSnapshot` with local cloud, seed cloud, descriptor metadata and
  bounds.
- Add snapshot consistency tests.
- Keep keyframes as graph nodes.

Exit criteria: a saved and loaded submap can reconstruct the same local cloud,
metadata and `T_map_submap` without ROS/PCL types in the core contract.

### Phase 2: Health and Write Protection (implemented foundation)

- Add frontend health state and hysteresis.
- Reject low-quality global keyframes.
- Add bounded local-map admission/pending-frame policy.
- Add prior/online local map layers.

Exit criteria: injected bad tracking frames cannot modify the durable global map
or the read-only recovery seed.

The current implementation uses a one-frame-lagged vendor admission decision:
a frame produces health evidence downstream, and the resulting local gate
affects future iVox inserts. This protects the realtime path from blocking on
registration or backend work. Fault-injection coverage for map contamination
remains required before this phase is considered fully validated. Health gates
are therefore disabled by default in shipped YAMLs until their thresholds are
calibrated for the deployed sensor and timestamp behavior.

### Phase 3: Frontend Reset and Seed (implemented foundation)

- Add frontend-only reset separate from full system reset.
- Add trajectory rollover.
- Add `resetAndSeed()` contract.
- Add seed cloud conversion from `S_i` to `O_j`.
- Add `map->odom_j` derivation.

Exit criteria: a synthetic relocalization can start a new trajectory without
deleting the existing MapManager state.

The recovery coordinator now performs the matched-submap identity check,
`S_i -> O_j` seed construction, and a two-phase trajectory rollover:
`prepareRecoveredTrajectory()` runs before frontend reset, while
`commitPreparedRecoveredTrajectory()` runs only after `resetAndSeed()` succeeds.
Unknown submaps and frontend reset failures are covered by tests; recorded-data
and provisional-layer rollback validation of the full transaction remains.

### Phase 4: Submap Relocalization (implemented foundation)

- Build a persistent submap descriptor index.
- Retrieve top-K candidates.
- Reuse KISS-Matcher and VGICP/GICP adapters.
- Return `submap_id` together with pose and quality.
- Add false-candidate rejection and cooldown.

Exit criteria: repeated indoor/outdoor route queries select the correct
submap, and a wrong candidate is rejected before seed creation. The code
foundation is complete; recorded-data and false-candidate fault evidence are
still required.

`SubmapDescriptorIndex` supplies a deterministic, rotation-independent radial
descriptor shortlist. It is intentionally treated as a coarse candidate
source only: its score cannot authorize a pose, a seed or a trajectory
rollover. KISS-Matcher plus VGICP/GICP verification and the quality gates are
implemented; false-candidate fault tests remain required evidence.

### Phase 5: Backend Reconnection (implemented foundation)

- Add global constraint ordering for a new trajectory.
- Force full estimates for loop/cross-trajectory constraints.
- Update MapManager and MapSnapshot with all affected poses.
- Generalize `map->odom` update beyond prior-map mode.

Exit criteria: the new trajectory is visible in the global graph and remains
consistent after graph optimization. The typed path exists; cross-trajectory
fault injection and recorded-data validation remain.

### Phase 6: Loop Closure (implemented foundation)

- Add asynchronous loop scheduler.
- Reuse submap retrieval and geometric verification.
- Add duplicate/cooldown/session policies.
- Add same-session and cross-trajectory loop tests.

Exit criteria: a repeated route reduces drift without blocking Point-LIO or
contaminating the local recovery seed. The asynchronous foundation and unit
tests exist; real route evaluation and runtime enablement remain.

## Evaluation and Acceptance Evidence

The recovery implementation must be compared against at least:

1. No recovery.
2. Global `map->odom` correction only.
3. Frontend reset with empty local map.
4. Frontend reset with matched submap seed.

Inject at least these failures:

- geometric degeneration;
- forced bad local-map insertions;
- LiDAR dropout with continuous IMU;
- temporary IMU/timestamp degradation.

Report:

- relocalization success rate;
- false recovery rate;
- time and distance to recovery;
- post-recovery RPE/ATE;
- map-to-base discontinuity;
- map contamination and rollback count;
- seed construction latency;
- peak memory and CPU during recovery.

The most important comparison is `empty local map` versus `matched submap
seed`. The seed is only justified if it reduces recovery time and post-recovery
drift without increasing false recovery or map contamination.

## Research Basis

The design was informed by the following publicly available systems and
papers. They are references for individual mechanisms, not dependencies of
this project:

- [Voxel-SLAM](https://arxiv.org/abs/2410.08935): divergence handling, new
  session, reinitialization, cross-session reconnection and voxel-map rebuild.
- [Cartographer](https://github.com/cartographer-project/cartographer):
  submap and multi-trajectory organization.
- [GLIM](https://arxiv.org/abs/2407.10344): local submap representation and
  map serialization ideas.
- [Super-LIO](https://doi.org/10.1109/LRA.2026.3653372): reuse of prior
  geometry for relocalization and subsequent LIO matching.
- [CT-ICP](https://arxiv.org/abs/2109.12979): local-map pollution and
  conservative map insertion.
- [Point-LIO](https://doi.org/10.1002/aisy.202200459): Point-LIO frontend
  state and local voxel-map lifecycle.

The exact combination proposed here is an engineering composition. The
project must not claim that one cited paper already validates the complete
`relocalize -> reset -> submap seed -> recover -> graph reconnect` pipeline.
