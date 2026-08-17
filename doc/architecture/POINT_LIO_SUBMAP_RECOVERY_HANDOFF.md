# Point-LIO Submap Recovery Handoff

Status: active implementation record, 2026-08-14.

The master implementation baseline is
[`ADAGRAPH_LIO_MASTER_IMPLEMENTATION_PLAN.md`](ADAGRAPH_LIO_MASTER_IMPLEMENTATION_PLAN.md).
Read it first after a context reset. This is the short operational companion to
[`POINT_LIO_SUBMAP_RECOVERY_DESIGN.md`](POINT_LIO_SUBMAP_RECOVERY_DESIGN.md).
The consolidated implementation plan is
[`ADAGRAPH_LIO_SUBMAP_RECOVERY_LOOP_CLOSURE_PLAN.md`](ADAGRAPH_LIO_SUBMAP_RECOVERY_LOOP_CLOSURE_PLAN.md).
The design document explains why the architecture is shaped this way; this file records what exists in code,
what does not, and the exact next implementation boundary.

## Product Decisions Already Made

- Small Point-LIO is the only realtime local-odometry frontend.  Super-LIO is
  retired and must not be restored as a fallback or compatibility path.
- A historical submap has two legitimate uses: it is a global relocalization
  target and, after geometric verification, a read-only seed for the restarted
  Point-LIO local map.
- A confirmed local frontend failure creates a new `TrajectoryId`; it does not
  overwrite the old ESKF state with `setPose()`.
- `map -> odom` is derived at the system boundary.  It is not a hidden state
  inside Point-LIO.
- Descriptor retrieval is a shortlist only.  It can never by itself start a
  new trajectory, change `map -> odom`, or load a seed.
- No RSS, CPU, or latency number is an acceptance gate unless a concrete
  product requirement is supplied.  Do not recreate retired Super-LIO memory
  thresholds from historical notes.

## Target Runtime Transaction

```text
Point-LIO healthy
  -> health hysteresis marks SUSPECT / LOST
  -> block future local iVox and global-keyframe writes
  -> retrieve top-K finished historical submaps
  -> KISS coarse pose + VGICP/GICP geometric verification
  -> stop SensorBridge ingress
  -> allocate new trajectory O_j and derive M -> O_j
  -> reset Point-LIO ESKF and load matched submap as read-only prior_iVox
  -> verify subsequent frames in a provisional online_iVox layer
  -> commit the first stable new keyframes and reconnect the graph
```

The submap geometric verifier, recovery scheduler and LOST-to-recovery
application handoff are implemented foundations. Automatic recovery remains
disabled by default until recorded-data and fault-injection evidence exists.

## Coordinate Contract

```text
M     persistent map frame
O_j   Point-LIO odom frame for trajectory j
S_i   immutable frame of finished submap i
B     robot base frame
```

The frontend owns `O_j -> B`; the backend owns optimized poses in `M`; a
finished submap stores its points in `S_i`.  The immutable anchor transform is
updated as graph optimization moves the anchor keyframe.

For a verified global pose at recovery time:

```text
T_M_Oj = T_M_B * inverse(T_Oj_B)
T_Oj_Si = inverse(T_M_Oj) * T_M_Si
point_Oj = T_Oj_Si * point_Si
```

`BuildRecoverySeedInOdom()` is the single conversion implementation.  Do not
duplicate this algebra in a PCL adapter.

## Implemented Foundation

| Area | State in code | Primary location |
| --- | --- | --- |
| Finished submap geometry | Anchor-local cloud, seed cloud, bounds, descriptor and `T_map_submap` are stored in immutable snapshots. | `src/hybrid_slam/include/hybrid_slam/mapping/map_snapshot.h` |
| Seed conversion | Converts `S_i` geometry to the new `O_j` with the coordinate contract above. | `src/hybrid_slam/src/mapping/submap_recovery_seed.cpp` |
| Candidate shortlist | Snapshot-local radial descriptor index returns identity-bound top-K candidates. | `src/hybrid_slam/src/mapping/submap_index.cpp` |
| Recovery query window | Accumulates a fixed number of post-LOST frames in odom, then expresses them in the latest IMU-propagated base frame; the default is four frames. | `src/hybrid_slam/src/system/point_lio_pipeline.cpp` |
| Tracking health | Instance-owned hysteresis states are `TRACKING`, `SUSPECT`, `LOST`, `RELOCALIZING`, `RESTARTING`, `VERIFYING`, `RECOVERED`. | `src/hybrid_slam/include/hybrid_slam/frontend/frontend_health.h` |
| Write protection | Bad health blocks global keyframe admission and, with one-frame delay, future vendor online-iVox writes. | `src/hybrid_slam/src/system/point_lio_pipeline.cpp` |
| Dual local map | `prior_ivox` is read-only; `online_ivox` receives new data and can be cleared independently. | `src/hybrid_slam/3rdparty/small_point_lio/src/small_point_lio/small_point_lio.cpp` |
| Frontend reset | ESKF, queues, timestamps and both local layers are reset/seeded while ingress is disabled. | `src/hybrid_slam/src/lio/point_lio_frontend.cpp` |
| New trajectory start | Preserves `MapManager`/backend, switches trajectory identity, derives `map -> odom`, then queues a recovery constraint after the first new keyframe. | `src/hybrid_slam/src/system/point_lio_pipeline.cpp` |
| Recovery transaction shell | Verifies identity, prepares the pipeline, resets/seeds Point-LIO, then commits the new trajectory. Caller owns SensorBridge ingress and resumes it after success. | `src/hybrid_slam/src/lio/point_lio_recovery_coordinator.cpp` |
| Verification rollback | Keeps `prior_ivox` read-only, admits only provisional online points, clears that layer on failure, and returns the lifecycle to `LOST` for a newer retry. | `src/hybrid_slam/src/system/point_lio_pipeline.cpp` |

The health gate is deliberately disabled by default.  Its thresholds have not
been calibrated against a deployed sensor or fault injection, so enabling it
would silently change ordinary odometry behavior.

## Current Execution Checkpoint (2026-08-14)

The latest recorded two-pass replay reaches:

```text
TRACKING -> SUSPECT -> LOST
KISS/VGICP candidate accepted
Point-LIO submap recovery committed
VERIFYING -> LOST
```

This is a real execution of the loss, candidate, and reset/seed path. It is
not a successful relocalization. There is currently no reliable `RECOVERED`
state, no reliable cross-trajectory `kRelocalization` constraint, and no
recorded proof of the first durable keyframe, backend reconnect, or continuous
`map -> odom`/TF after recovery.

The latest fixed four-frame query replay has strictly increasing output
timestamps and the fault relay reports `normalized_loops=1`. The window is
bounded at about 204k points and no longer grows while a candidate is pending.
It still does not reach `RECOVERED`: most shortlisted submaps have too few KISS
coarse correspondences, and one VGICP attempt did not converge. The seeded
two-pass trajectory diverged substantially and must not be used as a success
result.

The immediate debugging order is:

1. Verify descriptor index, finished-submap bounds, `T_map_submap`, query cloud
   base frame and `S_i -> O_j` seed conversion for the fresh scan.
2. Check whether the finished submap actually covers the fresh query position;
   if it does not, expand finished-submap coverage or evaluate adjacent-submap
   joint registration.
3. Inspect KISS coarse candidate correspondences and initial pose quality before
   changing VGICP gates; descriptor retrieval alone must not become acceptance.
4. Re-run build, CTest, boundary checks and the two-pass replay after the
   candidate path is corrected.

Automatic recovery remains disabled by default until the remaining failure
matrix and loop-closure evidence are reviewed. Recorded `RECOVERED`, durable
keyframe, reconnect and TF evidence now exists.

## Recovery Evidence Update (2026-08-15)

The recovery verifier now requires `VGICP/GICP::hasConverged()`.  The prior
compatibility behavior that admitted a finite, non-converged transform when
overlap evidence looked plausible was removed: it contradicted the required
convergence gate and could seed Point-LIO from a false global pose.  This
strict rule is shared by submap recovery and loop verification through
`PclRegistrationVerifier`.

New structured diagnostics are emitted into `summary.json` under
`diagnostics.submap_recovery`, `diagnostics.backend_scheduler`, and
`diagnostics.tf_publication`.  They record the selected target identity,
old/new trajectory IDs, the first durable post-recovery keyframe, reconnect
constraint queue/process/drop counts, verification commit/rollback, final
snapshot trajectory presence, and map-to-odom TF publication continuity.
They provide evidence; they do not turn recovery on by default.

The earlier recorded LiDAR-dropout runs remain useful rejection evidence:

- `/tmp/adagraph-lio-room-recovery-r24-20260815`: two candidates committed
  reset/seed transactions, then both entered `VERIFYING -> LOST`; no durable
  keyframe or `kRelocalization` constraint was admitted.
- `/tmp/adagraph-lio-room-recovery-r26-1x-20260815`: the new per-frame
  verification log showed five frames rising only from approximately 0.1% to
  11.5% frontend inlier ratio, below the unchanged 30% health gate.  The
  provisional online layer rolled back and the original trajectory remained
  intact.
- `/tmp/adagraph-lio-room-recovery-r27-1x-strict-grace20-20260815`: the
  first candidate had `hasConverged=false` and was rejected before trajectory
  rollover.  Backend reconnect counters stayed zero, as required for a
  rejected candidate.

The older r23 log reached `VERIFYING -> RECOVERED`, but it predates the
trajectory/reconnect/TF evidence and the strict convergence rule.  It is not
accepted as proof of successful recovery. The newer r37 run is the accepted
success evidence for the short-dropout path:

- `/tmp/adagraph-lio-room-recovery-r37-clean-one-second-fault-20260815`: the
  pass-boundary LiDAR outage and the injected LiDAR outage both reached
  `VERIFYING -> RECOVERED`; `verification_committed=2`, reconnect constraints
  were enqueued/processed with `2/2/0`, old and new trajectories were present,
  and `map -> odom`/`odom -> base` TF publication had no nonmonotonic stamps.
  The pass-boundary outage is a valid LiDAR-loss event, while the relay also
  reports the intentional drop separately (`dropped_lidar=10`).

Automatic recovery and loop closure remain disabled by default pending the
remaining failure matrix, prior-map replay and loop-closure evaluation.

## Current Follow-up Work

### 1. Complete rejection and rollback evidence

`PclSubmapRelocalizer` now exists as an adapter-side component.  It takes the
current immutable base-frame scan, `MapSnapshot`, and `SubmapDescriptorIndex`;
it returns an optional `SubmapRelocalizationResult` carrying the exact
`session_id`, `trajectory_id`, and `submap_id` of the selected target.

For every descriptor top-K candidate:

1. Find the corresponding finished `SubmapSnapshot`; reject absent,
   unfinished, invalid, or geometry-less candidates.
2. Materialize only that candidate's `cloud_submap` as a PCL target and
   transform it from `S_i` to `M` using `T_map_submap`.
3. Materialize the query scan as a PCL source.  Its declared frame remains
   `B`; use its current `T_odom_base` only where the existing verifier needs
   an odom-frame source.
4. Call `KISSMatcherRelocalizer::estimateCandidates()` for coarse poses.
5. For each coarse pose, use the existing `PclRegistrationVerifier` with a
   `TargetSelector` lambda that returns this candidate target.  Reuse its
   VGICP/GICP refinement instead of implementing another registration path.
6. Gate every output on convergence, finite fitness, overlap/inlier evidence,
   observability, temporal-neighbor consistency, and a recovery cooldown.
7. Convert the accepted pose to `SubmapRelocalizationResult`.  The descriptor
   score is diagnostic only, never a confidence proof.

The current implementation includes fitness, inlier, overlap and observability
gates. The short-dropout path is now recorded as successful. Remaining proof
work is recorded wrong-submap/similar-scene rejection, geometric degeneration,
bad-map insertion, and verification failure followed by provisional-layer
cleanup and retry.

Expected files:

```text
src/hybrid_slam/include/lio/pcl_submap_relocalizer.h
src/hybrid_slam/src/lio/pcl_submap_relocalizer.cpp
src/hybrid_slam/test/test_pcl_submap_relocalizer.cpp
src/hybrid_slam/CMakeLists.txt
```

Keep PCL and `basic` Lie-group conversion private to this adapter.  Core
interfaces remain `ImmutablePointCloud`, typed transforms, covariances and
domain IDs.

### 2. Recovery orchestration evidence

The latest-only worker, LOST/RELOCALIZING query handoff, two-phase transaction,
verification window, provisional online rollback and application-owned ingress
pause/resume are implemented and covered by unit tests. Recorded short-dropout
data now validates the successful transaction; remaining work is to validate:

1. Create a fresh nonzero `TrajectoryId` in the same session.
2. On success, resume ingress and retain write protection during `VERIFYING`.
3. On rejection, discard provisional online state, maintain the old map, and
   retry only after cooldown or a materially newer scan.
4. Add recorded-data and fault-injection evidence before enabling the default.

The coordinator itself does not stop or resume ingress.  That ownership is
intentional: it prevents a map/localization library from manipulating ROS
subscription lifecycle behind the application boundary.

### 3. Prior-map and backend follow-up validation

The first accepted keyframe of the new trajectory has a deferred recovery
constraint path, and the optimizer now returns full estimates for updates that
do not add nodes. Deterministic `BackendScheduler`/`MapManager` and actual
`Isam2Optimizer` tests cover full estimate propagation. The online recovery path
is now validated by r37. Remaining validation is:

- repeat the equivalent recovery with a preloaded prior map;
- retain the existing full-estimate propagation and TF checks in regression;
- compare online-submap seed and prior-map seed behavior.

A single-pass prior-map replay is now recorded at
`/tmp/adagraph-lio-room-prior-map-20260815`. It loaded the saved Point-LIO map
(`10392` raw points, `2910` filtered points), achieved the first KISS success
after 6 attempts, committed 35 global-alignment updates, and produced
`ape_translation_rmse=0.396454 m`. `odom -> base` and `map -> odom` timestamps
were monotonic. The bag's LiDAR-free tail later triggered the configured
watchdog, so this run proves initial prior-map localization, not prior-map
recovery after a deliberate mid-route fault.

### 4. Loop Closure scheduler integration

Loop closure foundations now exist and reuse the same retrieval/registration
components. The remaining implementation boundary is:

```text
new keyframe -> exclude temporal neighbors -> retrieve submaps
             -> verify geometry -> submit graph loop constraint
             -> full estimate propagation -> snapshot publication
```

1. Validate the connected scheduler with the full ROS application build.
2. Confirm the node handoff against a recorded route once loop closure is
   explicitly enabled for evaluation.
3. Begin with same-trajectory loops; enable cross-trajectory loops only after
   reconnect constraints have full-estimate propagation coverage.

The first normalized two-pass `room_01` comparison is recorded at
`/tmp/adagraph-lio-room-loop-closure-two-pass-normalized-20260815` and
`/tmp/adagraph-lio-room-loop-closure-two-pass-baseline-20260815`. The baseline
had 164 nodes/163 constraints and `0.772135 m` APE RMSE. Loop closure had 163
nodes/185 constraints, a maximum optimizer translation delta of `0.0608075 m`,
and `0.788116 m` APE RMSE. This proves the online loop-constraint path executes,
but it did not improve this route, so the default remains disabled and more
routes/false-loop cases are required.

### 5. Structural cleanup still pending

- Move the remaining private PCL/basic bridge out of the `PointLIOPipeline`
  header when strict compile isolation requires it.
- Push PCL/glog aliases from core into adapters and logging.
- Make `FrontendConfig` the project-owned semantic source and reduce duplicate
  compatibility fields in Small Point-LIO `Parameters`.
- Remove remaining legacy global-parameter reads so multiple instances and
  session reset do not share mutable process-wide configuration.

## Required Tests Before Enabling Automatic Recovery

- Correct submap is selected from a known query; a descriptor-only false match
  cannot succeed.
- A geometrically plausible but wrong submap is rejected by quality gates.
- A matched seed has correct `S_i -> O_j` coordinates.
- Recovery rolls to a new trajectory without erasing historical keyframes,
  submaps, or graph state.
- During `SUSPECT`/`LOST` no new local online-iVox points or global keyframes
  are admitted.
- A failed verification discards provisional online data and does not pollute
  the durable map.
- A graph reconnect moves all affected snapshot poses, including old nodes.
- Fault injection: geometric degeneration, bad-map insertion, LiDAR dropout
  with IMU retained, and timestamp/IMU degradation.

The current complete regression command is:

```bash
cmake --build build/hybrid_slam -j2
ctest --test-dir build/hybrid_slam --output-on-failure
```

It validates the unit-test and build foundation. Real short-dropout recovery is
recorded in r37, and the first real loop-constraint comparison is recorded in
the two-pass runs above; neither result is yet sufficient to enable defaults.
The current full build includes the connected loop scheduler and its unit test.

## Continuation Order

1. Add recorded-data fault injection for wrong candidates, degeneration,
   bad-map writes, IMU/timestamp faults and provisional-map rollback.
2. Run prior-map recovery and compare it with online matched-submap seed.
3. Expand loop-closure evaluation to additional routes and false-loop cases.
4. Enable only the behaviors that pass the preceding evidence.

Do not enable `LOST` auto-recovery or loop closure by default before steps 1
through 3 are complete. The short-dropout path is validated, but the remaining
failure and regression matrix still establishes the production safety boundary.
