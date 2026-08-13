# Target Architecture

Status: accepted direction for incremental refactoring
Last reviewed: 2026-08-11

This document defines architecture invariants and the intended module boundaries. It is not permission for a one-shot rewrite. Migration must follow `REFACTORING_ROADMAP.md`, preserve runnable behavior after every phase, and be validated against the code and recorded datasets.

## Design Goals

The framework must remain suitable for a real-time C++ SLAM system while supporting long-term addition or replacement of:

- Super-LIO, Point-LIO and future LIO frontends;
- pose-graph optimizers and constraint sources;
- loop closure and relocalization techniques;
- mapping and localization pipelines;
- keyframes, submaps, multi-session and partial map loading;
- occupancy, voxel, TSDF, ESDF, elevation and semantic representations;
- ROS 1/ROS 2 adapters and a ROS-independent dataset runner.

The target is a small number of cohesive modules with explicit data contracts, not a runtime plugin framework or a hierarchy of tiny abstract classes.

## Architecture Invariants

These rules are mandatory review criteria unless an explicit decision is recorded in `REFACTORING_STATUS.md`.

1. The frontend owns local odometry state and produces `T_odom_base`.
2. Backend/relocalization own global alignment expressed as `T_map_odom` or a result from which the system computes it.
3. Final global pose is composed as:

   ```text
   T_map_base = T_map_odom * T_odom_base
   ```

4. Global optimization or relocalization must not directly mutate the frontend ESKF/IEKF, local voxel map, IMU buffers or matcher internals.
5. Backend must not access frontend internal state. It consumes `FrontendResult`, `KeyFrame` and formal constraints only.
6. Core libraries must not depend on `rclcpp`, ROS messages, TF or ROS parameter APIs.
7. Occupancy grids and other navigation maps are derived representations, not authoritative SLAM state.
8. Map IO, global map construction, occupancy generation, heavy visualization and large ROS serialization must not block the real-time frontend.
9. Mutable containers are private to a single owner. Cross-thread publication uses immutable values/snapshots.
10. A frontend replacement must not require backend, relocalizer, map or ROS algorithm changes when it satisfies the frontend contract.
11. Pose types must encode or document target frame, source frame and timestamp; unlabelled `Eigen::Matrix4d` is not a domain API.
12. No silent loss of accepted keyframes, graph constraints or map-save commands under backpressure.

## Module Boundaries

```text
apps / ros2 adapter / dataset runner
                    │
                    ▼
                 system
          ┌─────────┼──────────┐
          ▼         ▼          ▼
      frontend    backend   localization
                      │          │
                      └────┬─────┘
                           ▼
                    mapping / map
                           │
                           ▼
                          io

frontend/backend/localization/mapping/system
                     │
                     ▼
            sensor / geometry / common
```

Allowed dependency direction is downward only:

```text
apps/adapters
    -> system
    -> frontend/backend/localization/mapping/io
    -> common/geometry/sensor
```

Examples of forbidden dependencies:

- `backend -> ros_node.h`
- `frontend -> OccupancyGridPublisher`
- `mapping -> Frontend::internal_filter()`
- `optimizer -> rclcpp`
- `relocalizer -> ESKF`
- `common -> sensor_msgs`

## Recommended Repository Structure

During migration, the existing `src/super_lio` package remains available as a legacy implementation. The target logical structure is:

```text
src/
  adagraph_core/
    include/adagraph/
      common/
      geometry/
      sensor/
      frontend/
      backend/
      localization/
      mapping/
      io/
      system/
    src/
    test/

  adagraph_frontends/
    super_lio/
    point_lio/

  adagraph_ros2/
    include/adagraph_ros2/
    src/
      ros_node.cpp
      sensor_bridge.cpp
      tf_bridge.cpp
      publisher.cpp
      service_bridge.cpp
      parameter_bridge.cpp

  adagraph_apps/
    src/
      mapping_node.cpp
      localization_node.cpp
      dataset_runner.cpp

  super_lio/             # retained during strangler migration
  livox_ros_driver2/     # transport/vendor package
```

Recommended CMake targets, subject to adjustment after real dependency measurements:

```text
adagraph_common             # common + geometry + sensor; avoid tiny libraries
adagraph_frontend_api
adagraph_frontend_super
adagraph_frontend_point
adagraph_backend
adagraph_localization
adagraph_mapping
adagraph_io
adagraph_system
adagraph_ros2
```

Targets should be cohesive enough for clear incremental compilation without creating a library for every one or two source files.

## Runtime Data Flow

```text
ROS2 SensorBridge or DatasetPlayer
              │
              ▼
      bounded sensor input
              │
              ▼
      selected LioFrontend
              │
              ▼
 FrontendResult {stamped T_odom_base, quality, cloud}
              │
              ▼
        SlamSystem policy
              │
       ┌──────┴─────────┐
       ▼                ▼
local odometry       KeyFrame
                        │
                        ▼
               BackendScheduler
                        │
          ┌─────────────┼─────────────┐
          ▼             ▼             ▼
     PoseGraph      loop workers   Optimizer
          │                             │
          └──────────────┬──────────────┘
                         ▼
              OptimizationResult
                         │
                         ▼
                   MapManager
                         │
                         ▼
       shared_ptr<const MapSnapshot>
        │                 │                  │
        ▼                 ▼                  ▼
  Relocalizer   OccupancyGridBuilder     MapIO / ROS
```

The backend owns graph-global optimization state, while consumers observe consistent immutable snapshots. Consumers do not lock pose-graph internal containers.

## Frontend Responsibility

The frontend's only system responsibility is stable, continuous, high-frequency local odometry from ordered sensor input.

It may internally own:

- ESKF/IEKF and IMU integration;
- LiDAR preprocessing and deskew;
- scan matching and feature extraction;
- local KD-tree/ikd-tree/voxel map;
- local-map update and frontend quality estimation.

It must not own:

- graph optimization or loop closure;
- global relocalization state;
- occupancy/navigation maps;
- ROS publishers, services or TF;
- global map serialization;
- global keyframe/map lifecycle.

Suggested boundary:

```cpp
class Frontend {
public:
    virtual ~Frontend() = default;

    virtual Status AddImu(const ImuData& imu) noexcept = 0;

    virtual Result<std::optional<FrontendResult>>
    AddPointCloud(PointCloudFrame&& frame) = 0;

    virtual Status Reset(const FrontendReset& reset) = 0;
};
```

One virtual dispatch per sensor packet or scan is permitted. Per-point code remains concrete, templated or function-object based and must not perform dynamic dispatch merely for style.

## Backend Responsibility

Backend accepts durable keyframes and constraints, schedules non-real-time graph work and commits versioned optimization results.

```text
Backend
  ├── PoseGraph
  ├── ConstraintBuilder
  ├── LoopDetector
  ├── LoopVerifier
  ├── Optimizer
  └── BackendScheduler
```

Responsibilities:

- convert adjacent keyframes into odometry constraints;
- accept loop, GNSS, relocalization and future sensor constraints;
- schedule detection, verification and optimization;
- serialize graph mutations through a single writer;
- emit `OptimizationResult` and publish map snapshots through `MapManager`.

It does not own frontend sensor state or ROS output.

```cpp
class Backend {
public:
    virtual ~Backend() = default;

    virtual Status EnqueueKeyFrame(KeyFrame&& keyframe) = 0;
    virtual Status AddExternalConstraint(Constraint constraint) = 0;

    virtual std::shared_ptr<const MapSnapshot>
    GetMapSnapshot() const noexcept = 0;
};
```

## PoseGraph Responsibility

`PoseGraph` is the authoritative owner of graph topology and graph pose state, not point-cloud registration or ROS messages.

```cpp
class PoseGraph {
public:
    NodeId AddNode(PoseGraphNode node);
    Status AddConstraint(Constraint constraint);

    PoseGraphState BuildState() const;
    Status Apply(const OptimizationResult& result);
};
```

Rules:

- graph mutation is single-writer;
- `LoopDetector` only proposes candidates;
- `LoopVerifier` only verifies registration and produces measurements/scores;
- `ConstraintBuilder` creates typed constraints;
- `Optimizer` consumes an immutable graph problem and returns an optimization result;
- neither detector nor optimizer mutates `PoseGraph` behind its owner.

## Relocalization Responsibility

Relocalization is independent of both frontend internals and backend storage.

```text
Relocalizer
  ├── DescriptorExtractor
  ├── CandidateRetriever
  ├── RegistrationVerifier
  └── RelocalizationPolicy
```

```cpp
struct RelocalizationQuery {
    Timestamp timestamp;
    ImmutableCloud cloud_base;
    std::optional<Pose3d> prior_map_base;
    std::shared_ptr<const LocalizationMap> map;
};

struct RelocalizationResult {
    bool success = false;
    Timestamp timestamp;
    Pose3d T_map_base;
    double score = 0.0;
    Confidence confidence;
};

class Relocalizer {
public:
    virtual ~Relocalizer() = default;

    virtual Result<RelocalizationResult> Relocalize(
        const RelocalizationQuery& query,
        std::stop_token stop) const = 0;
};
```

The first implementation may wrap the existing KISS candidate and FastVGICP/PCL GICP verifier. A result is submitted to `SlamSystem`, which computes or updates `T_map_odom`. The relocalizer does not call ESKF reset, edit the local voxel map, publish TF or write a ROS-owned correction cache. If the result crosses from a localization worker to the system writer, the handoff is fixed-size and latest-only; it never retains a second cloud.

## Map Responsibility

The map is not one giant point cloud. The authoritative logical state contains:

```text
Map
  ├── trajectories and sessions
  ├── keyframe metadata and immutable payloads
  ├── submaps
  ├── optimized graph poses
  ├── constraints
  ├── descriptors
  └── map metadata/version
```

`MapManager` owns keyframe payload/index state and applies optimization results:

```cpp
class MapManager {
public:
    Status InsertKeyFrame(KeyFrame&& keyframe);
    Status ApplyOptimization(OptimizationResult result);

    std::shared_ptr<const MapSnapshot>
    Snapshot() const noexcept;
};
```

The pose graph need not own large point clouds. Graph nodes reference IDs; `MapManager` owns or references immutable payloads.

Submaps are introduced first as IDs/grouping metadata. Algorithms should not be rewritten merely to create a nominal submap class. Spatial submap behavior is added only after snapshot/keyframe ownership is stable.

## Map Representation Responsibility

Occupancy, TSDF, ESDF and other representations consume an optimized snapshot:

```text
Optimized KeyFrames/Submaps
             │
             ▼
     MapRepresentationBuilder
             │
             ▼
 OccupancyGrid / TSDF / ESDF / ...
```

An occupancy builder is ROS-independent:

```cpp
class OccupancyGridBuilder {
public:
    Result<OccupancyGrid2D> Build(
        const MapSnapshot& snapshot,
        std::stop_token stop) const;

    Status UpdateIncremental(
        const MapSnapshot& snapshot,
        OccupancyGrid2D& grid) const;
};
```

Do not introduce a common virtual `MapRepresentation` interface until at least two implementations need a shared runtime contract. A concrete builder plus composition is the preferred initial design.

## SlamSystem Responsibility

`SlamSystem` is an orchestrator and lifecycle boundary:

- routes sensor data;
- invokes the selected frontend;
- applies mapping or localization policy;
- selects/promotes keyframes;
- submits backend and relocalization work;
- manages trajectory/session/map lifecycle;
- exposes snapshots and asynchronous save operations.

It must not implement ICP, ESKF, graph optimization, voxel filtering or grid ray casting.

```cpp
class SlamSystem {
public:
    Status StartMapping(const MappingOptions& options);
    Status StartLocalization(
        std::shared_ptr<const LocalizationMap> map,
        const LocalizationOptions& options);

    Status AddImu(ImuData imu);
    Status AddPointCloud(PointCloudFrame&& cloud);

    Future<Status> SaveMapAsync(const MapSaveRequest& request);

    Status FinishTrajectory();
    Status Stop();
    Status Reset();

    std::shared_ptr<const SystemSnapshot>
    Snapshot() const noexcept;
};
```

Mapping and localization are system-level component compositions/policies, not `if (mapping_mode)` branches scattered through frontend and backend algorithms.

## ROS Adapter Boundary

Only adapter/application targets may depend on:

```text
rclcpp
sensor_msgs
nav_msgs
geometry_msgs
tf2 / tf2_ros
ROS parameters, timers, services and actions
```

Adapter responsibilities:

```text
RosNode
  ├── SensorBridge      ROS message -> core sensor type
  ├── TfBridge          SystemSnapshot -> TF
  ├── Publisher         core value -> ROS message
  ├── ServiceBridge     lifecycle/save/relocalization commands
  └── ParameterBridge   YAML/ROS params -> validated config values
```

ROS callbacks should convert and enqueue only. The same core must run from `DatasetPlayer -> SlamSystem` without ROS.

## Threading Model

The initial target uses a small number of execution domains rather than one thread per module:

```text
1 frontend real-time worker or deterministic caller thread
1 backend single-writer scheduler
2-3 background workers for loop/relocalization/optimization tasks
1 low-priority IO/map-representation worker
ROS executor callbacks for conversion/enqueue and lightweight publication
```

An optimizer may later receive a dedicated worker when profiling or library thread-safety requires it.

### Queue and backpressure policy

| Work item | Required behavior |
| --- | --- |
| IMU | Time-bounded queue. Overflow is a diagnostics/error event; required IMU samples are not silently dropped. |
| LiDAR scan not yet processed | Bounded. Under overload, drop according to an explicit scan policy and count/report it. |
| Accepted keyframe | Reliable bounded submission. Do not silently drop; throttle keyframe creation before acceptance or surface overload. |
| Loop/relocalization query | Latest-only or coalesced; old work is cancellable. |
| Accepted relocalization result awaiting system commit | Fixed-size latest-only mailbox. System/global-alignment context is the sole semantic writer; no PointCloud is retained. |
| Visualization | Latest-only; dropping is expected. |
| Occupancy rebuild | Coalesce by snapshot version. |
| Map-save command | Reliable asynchronous command with a result. |

Algorithms execute outside queue mutexes. Shutdown follows `request_stop -> close queues -> cancel/coalesce -> drain required work -> join`, without sleep polling.

`GlobalAlignmentState` has one semantic writer in the system scheduling domain. Backend refinement and relocalization do not independently publish `T_map_odom`; they submit typed candidates/results to that writer. ROS/TF only reads the committed immutable state or receives a cache copy for publication.

## Snapshot Model

Mutable graph/map state remains private to its owner. A consistent snapshot is published after a committed change:

```text
single-writer mutable state
          │ commit
          ▼
shared_ptr<const MapSnapshot>
          │ atomic pointer exchange
          ├── relocalizer
          ├── occupancy builder
          ├── map saver
          ├── ROS/visualization
          └── diagnostics
```

```cpp
struct MapSnapshot {
    std::uint64_t version = 0;
    SessionId session_id;
    std::vector<KeyFrameMetadata> keyframes;
    std::vector<OptimizedPose> optimized_poses;
    std::vector<SubmapSnapshot> submaps;
    std::vector<Constraint> constraints;
    std::vector<ImmutableCloud> payloads;
};
```

Large cloud payloads are not deep-copied into every version. Early snapshots may copy `O(keyframes)` pose/metadata values; if measured scaling requires it, later versions may use immutable chunks or structural sharing.

## Pose / Frame Semantics

Every domain transform follows the naming convention:

```text
T_target_source
```

and represents coordinates from `source` expressed in `target`.

```cpp
enum class FrameId : std::uint8_t {
    kMap,
    kOdom,
    kBase,
    kLidar,
    kImu,
};

struct Transform3d {
    FrameId target;
    FrameId source;
    Timestamp timestamp;
    SE3d value;
};
```

Static calibrated extrinsics are explicit, for example `T_base_lidar` and `T_base_imu`. They are not stored in variables named `odom_robo` or passed as an unlabelled matrix.

The frontend publishes continuous `T_odom_base`. The global layer publishes `T_map_odom`. An optimized map pose may jump; `T_odom_base` must remain continuous. Interpolation/smoothing policy applies to publication/global alignment and must not silently change the underlying frontend state.

## Core Domain Types

```cpp
struct FrontendResult {
    // T_odom_base.timestamp is the authoritative result/scan-end timestamp;
    // do not store a second independently mutable copy.
    StampedTransform3d T_odom_base;
    Twist3d body_velocity;
    FrontendStatus status;
    ScanMatchingSummary matching;
    ImmutableCloud deskewed_cloud_base;
};

struct KeyFrame {
    KeyFrameId id;
    TrajectoryId trajectory_id;
    SessionId session_id;
    Timestamp timestamp;
    Transform3d T_odom_base;
    ImmutableCloud cloud_base;
    KeyFrameQuality quality;
};

enum class ConstraintType {
    kOdometry,
    kLoop,
    kRelocalization,
    kGnss,
};

struct Constraint {
    ConstraintId id;
    ConstraintType type;
    NodeId source;
    NodeId target;
    SE3d T_target_source;
    Matrix6d information;
    ConstraintScore score;
};
```

Avoid `pair`, `tuple`, raw matrix and integer-map domain APIs when the fields have SLAM meaning.

## Ownership Rules

1. Components are direct members or `unique_ptr` by default.
2. `shared_ptr` is reserved for immutable, genuinely shared lifetime such as clouds and snapshots.
3. No owning raw pointers.
4. Mutable point-cloud buffers have one writer.
5. Same-call cloud access uses `std::span<const PointXYZIT>` or a concrete view.
6. Cross-thread/fan-out cloud access uses `shared_ptr<const PointCloudBuffer>`.
7. Keyframe and snapshot payloads are immutable after publication.
8. `PoseGraph` and `MapManager` are single-writer owners; readers use snapshots.
9. ROS message ownership ends at the adapter boundary.
10. Configuration objects are constructed and validated before algorithm construction; algorithms do not read YAML/ROS parameters.

## Performance Rules

- No point-cloud deep copy merely to cross a module boundary.
- No per-point virtual dispatch.
- No per-point heap allocation introduced by architecture wrappers.
- Frontend interfaces perform at most one coarse dispatch per sensor packet/frame.
- Queue locks protect move/pointer bookkeeping only; algorithm work runs outside locks.
- Frontend processing never waits for map IO, grid generation, global visualization or large ROS serialization.
- Accepted keyframe clouds are frozen/referenced, not transformed and duplicated for every consumer.
- Frequently reused scratch arrays should retain capacity or come from measured pools/PMR resources.
- Object pools are introduced only after allocation profiling, with explicit reset/lifetime contracts.
- Background status/metrics include frontend latency, scan-match latency, queue depth, optimization latency, relocalization latency, dropped frames, map size and memory.
- Performance decisions require dataset and profiler evidence rather than assuming an abstraction is expensive.

### Expected abstraction costs

| Abstraction | Copy/allocation/dispatch/synchronization contract |
| --- | --- |
| `Timestamp`, IDs and typed poses | Small value copies; no heap, virtual call or mutex. |
| `PointCloudView`/`span` | No ownership, copy or allocation; caller-bound lifetime. |
| `ImmutableCloud` | No deep copy; one control block and atomic refcount at cross-thread/fan-out boundaries. |
| Frontend interface | One virtual call per IMU packet/scan; none per point. |
| Backend/optimizer/relocalizer interfaces | At keyframe/task frequency, outside the real-time point loop. |
| Bounded queue | One short mutex/CV operation and moved payload. |
| `MapSnapshot` | One commit-level allocation plus light metadata; cloud payloads are shared immutable. |
| Occupancy build | Background work, no frontend/PoseGraph lock. |

## Configuration and Error Handling

Configuration is grouped by module:

```text
FrontendConfig
BackendConfig
LoopClosureConfig
RelocalizationConfig
MapConfig
RuntimeConfig
RosConfig
```

Loading and validation are adapter/application responsibilities. Constructors receive validated values.

Use a lightweight `Status`/`Result<T>` with stable error codes at module/lifecycle boundaries. High-frequency math paths remain exception-free. Do not create a generic template framework more complex than the error handling it replaces.

## Testing Contract

The target architecture must allow these without ROS:

- frontend unit and dataset tests;
- backend/PoseGraph deterministic tests;
- relocalizer tests against a fixed localization map;
- MapSnapshot and map IO round trips;
- occupancy golden tests;
- `DatasetPlayer -> SlamSystem -> TrajectoryResult` regression tests.

Each migration phase must preserve a runnable legacy path until its replacement passes build, unit, dataset and performance validation.
