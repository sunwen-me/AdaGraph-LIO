# AdaGraph-LIO Project Handoff

Last updated: 2026-08-13

This file is the cross-session handoff for the AdaGraph-LIO, robot simulation,
real-robot navigation, and deployment work. Read this file before making
changes. The current architecture and its completion evidence are in
`doc/architecture/`; when this file disagrees with that directory, the
architecture documents and current code win. Earlier Jetson-only run records
remain historical evidence, not a current deployment recipe.

## 1. Quick State

### Repositories and machines

| Item | Location / state |
| --- | --- |
| AdaGraph-LIO development checkout | `/home/sw/AdaGraph-LIO` |
| Simulation / Nav2 / MoveIt workspace | `/home/sw/robot-sim` |
| Local ROS | ROS 2 Lyrical |
| Active real-robot target | AidLux X1, SSH alias `aidlux`, ROS 2 Humble |
| X1 checkout | `/home/aidlux/AdaGraph-LIO` |
| X1 prior-map directory | `/home/aidlux/maps/real_20260728_213413` |
| Jetson AdaGraph-LIO checkout | `/home/jetson/AdaGraph-LIO` |
| Jetson auxiliary workspace | `/home/jetson/dev_ws` |
| Jetson Nav2 overlay | `/home/jetson/nav2_ws` |
| Jetson ROS | ROS 2 Jazzy |
| Last working Jetson WiFi address | `jetson@192.168.1.182` |
| AdaGraph-LIO Git remote | `https://github.com/sunwen-me/AdaGraph-LIO.git` |

The Jetson address below is historical and was unreachable from the development
PC on 2026-08-02. Do not assume any ROS process from the 2026-07-30 Jetson test
is still running. The X1 is the current target, but live ROS topology and
physical safety still need to be checked before any motion test.

Do not put SSH, WiFi, GitHub tokens, or passwords into this repository.

### Git state

- Current branch: `feature/point-lio-frontend`.
- Baseline before the Point-LIO architecture refactor: `main@2a2850e`
  (`origin/main`).
- `develop` is still at `9c46e59` and is behind current `main`.
- `fix/ultra-review` points to `a8aa15e` and is retained as the review archive.
- The Point-LIO architecture/lifecycle refactor is documented in
  `doc/architecture/REFACTORING_STATUS.md`; it introduces the standard typed
  runtime path, records validation evidence, and retires the separate
  `SuperLIOReLoc` executable. Inspect `git status`, `git log -1`, and the
  architecture status before making a follow-up change.

The `/home/sw/robot-sim` directory has a newly initialized Git repository with
no commits. Almost the whole workspace is still untracked. Two real-navigation
files were changed during the last test:

```text
/home/sw/robot-sim/src/robot_nav2_bringup/config/nav2_params_real.yaml
/home/sw/robot-sim/src/robot_nav2_bringup/rviz/nav2.rviz
```

## 2. Non-Negotiable Architecture

The prior-map localization design was explicitly chosen by the user. Preserve
these boundaries unless the user asks to redesign them.

1. There is no independent prior-map localization ROS 2 node.
2. Standard Super-LIO and Point-LIO both use
   `PriorMapLocalization -> RelocalizationScheduler -> GlobalAlignmentState`.
   `PriorMapLocalization` is a downstream/system adapter, not frontend-owned
   state in the standard composition.
3. Each frontend owns only local odometry and produces a smooth
   `T_odom_base`; global correction never resets the frontend filter or local
   map.
4. Sensor callbacks only convert and enqueue. `SlamSystem::ProcessPending()`
   runs frontend processing; prior-map crop, KISS, and FastVGICP/PCL GICP run
   on the relocalization worker.
5. FastVGICP never resets or overwrites frontend state.
6. The global correction is:

```text
T_map_odom = T_map_lidar_vgicp * inverse(T_odom_lidar_same_timestamp)
```

7. `GlobalAlignmentState` is the sole semantic owner of `map -> odom`.
   `PublicationBridge`/`TfBridge` publish only its committed cache;
   `ROSWrapper` is a compatibility facade and is not instantiated by a
   standard application.
8. The final navigation TF chain is:

```text
map -> odom -> base_footprint -> base_link -> livox_frame
```

9. Small tracking corrections are smoothed in SE(3): translation uses linear
    interpolation and rotation uses quaternion slerp.
10. Initialization, LOST recovery, and an explicit manual recovery may jump
    immediately after passing quality gates.
11. Quality gating happens before smoothing.
12. Large clouds are passed by shared pointer / move where possible. The full
    prior map is loaded once, voxel filtered once, and spatially indexed; it is
    not deep-copied every frame.

Relevant implementation:

```text
src/super_lio/include/lio/prior_map_localization.h
src/super_lio/src/lio/prior_map_localization.cpp
src/super_lio/src/lio/point_lio_frontend.cpp
src/super_lio/src/system/legacy_downstream_pipeline.cpp
src/super_lio/src/system/global_alignment_state.cpp
```

## 3. Localization Roles

### LIO front end

- ESKF + OctVox provide continuous local odometry.
- The local world used by Super-LIO is semantically `odom`, not `map`.
- `/lio/cloud_world` is therefore published with `frame_id: odom` when Nav2 TF
  mode is enabled.
- AdaGraph currently computes covariance and stores the pose graph, but its
  experimental global localization factors are disabled by default.
- Do not write backend poses back into ESKF. The Ultra review removed that path
  because it would make the ESKF state and odom-frame voxel map inconsistent.

### KISS-Matcher

KISS is the coarse global proposal mechanism. It is not a continuous fixed-Hz
tracker.

It should run only for:

- startup initialization;
- LOST recovery;
- `/lio/relocalization/trigger` manual requests.

Normal tracking uses FastVGICP only.

The current implementation:

- builds a multi-frame local odom submap;
- submits every frame during initialization so the startup submap is filled
  quickly, then returns to `min_frame_interval` while tracking;
- evaluates descriptor ratio-test ON and OFF when
  `adaptive_ratio_test: true`;
- reports processed point, keypoint, match, rotation-inlier, and final-inlier
  counts;
- sends every valid KISS candidate through cropped-map FastVGICP;
- chooses the passing candidate with the lowest FastVGICP fitness;
- automatically retries startup failures rather than waiting forever for a
  manual service call.

Important limitation: KISS was reliable when mapping and localization started
at the same place, but was inconsistent from an arbitrary place in the real
house. Adaptive ratio mode improved candidate handling but did not turn merged
PCD matching into a guaranteed place-recognition system. One stationary real
test needed many retries before finding the correct candidate. Do not weaken
the quality gate merely to make initialization appear successful.

For robust arbitrary-start localization, the next serious options are:

- a manually supplied initial pose that is then verified/refined by VGICP;
- a trajectory-keyframe place-recognition database such as Scan Context;
- a stronger global registration / descriptor database built while mapping.

A merged PCD alone does not preserve map trajectory or observation viewpoints.

### FastVGICP

- CPU FastVGICP is vendored under
  `src/super_lio/3rdparty/fast_gicp`.
- It is part of the Super-LIO package; a system-wide `fast_gicp` installation
  is no longer required for the intended build.
- It is the back-end prior-map registration stage, not the LIO scan-to-map
  front end.
- CUDA is not required. `num_threads` controls CPU parallelism.
- PCL GICP remains a build fallback in the code path, but Jetson validation was
  intended to use the vendored FastVGICP branch.

## 4. Current Localization Parameters

`src/super_lio/config/livox_360.yaml` currently contains:

```yaml
lio.sensor.blind: 0.3

lio.kiss_matcher.voxel_size: 0.2
lio.kiss_matcher.use_quatro: false
lio.kiss_matcher.use_voxel_sampling: true
lio.kiss_matcher.use_ratio_test: true
lio.kiss_matcher.adaptive_ratio_test: true
lio.kiss_matcher.min_final_inliers: 5
lio.kiss_matcher.max_source_points: 30000
lio.kiss_matcher.max_target_points: 120000

lio.prior_map_localization.map_voxel_size: 0.1
lio.prior_map_localization.local_voxel_size: 0.1
lio.prior_map_localization.local_frames: 8
lio.prior_map_localization.min_frame_interval: 5
lio.prior_map_localization.crop_radius: 40.0
lio.prior_map_localization.max_iterations: 100
lio.prior_map_localization.num_threads: 4
lio.prior_map_localization.max_fitness: 0.1
lio.prior_map_localization.initial_max_fitness: 0.01
lio.prior_map_localization.max_translation: 2.0
lio.prior_map_localization.max_rotation_deg: 20.0
lio.prior_map_localization.lost_fitness: 2.0
lio.prior_map_localization.smooth_translation_alpha: 0.1
lio.prior_map_localization.smooth_rotation_alpha: 0.1
```

`initial_max_fitness: 0.01` is intentionally strict. During testing, false
candidates with fitness values around `0.047`, `0.063`, `0.16`, `0.58`, and
`1.37` were rejected. A correct candidate was accepted around `0.0027`.
Tracking commonly stayed around `0.003` to `0.006` while stationary.

`Livox_mid360.py` now exposes independent launch arguments:

```text
localization_voxel_size (VGICP, default 0.10)
kiss_voxel_size         (KISS, default 0.20)
kiss_use_quatro
kiss_use_ratio_test
kiss_adaptive_ratio_test
```

Do not couple the KISS feature scale back to the VGICP voxel scale without a
measured reason.

## 5. TF and Real Sensor Installation

### Real robot geometry

- `base_footprint` is the ground-projected drive-axis reference used by Nav2.
- `base_link` is offset behind the drive axis by about `0.196 m`.
- The Mid-360 is at approximately `x = 0.143 m` and `z = 0.735 m` in
  `base_link`.
- Therefore its x offset relative to `base_footprint` is about `-0.053 m`.
- The real Mid-360 is physically upside down.
- The mounting correction is a 180-degree rotation around **X**, not Y.
- The IMU is treated normally. Do not negate both IMU and LiDAR axes.

Current real configuration:

```yaml
lio.extrinsic.odom_robo: [-0.053, 0.0, 0.735, 180.0, 0.0, 0.0]
```

`Livox_mid360.py` publishes these static transforms unless disabled:

```text
base_footprint -> base_link
base_link -> livox_frame
```

Do not also publish the same transforms from another URDF / static publisher.
Duplicate TF publishers previously caused frame flips and TF buffer failures.

### Simulation geometry

- The simulated IMU is normally oriented.
- RGL does not reliably inherit the orientation semantics expected from the
  intermediate Mid-360 model link.
- The working design attaches the RGL reference directly to `base_link` using
  the same physical connection transform as the real sensor.
- The simulated LiDAR is also rotated by `RotX(pi)`.
- `sim.yaml` contains the matching `lidar_imu` rotation.
- Wheel odometry TF is disabled in
  `/home/sw/robot-sim/src/robot_description/config/ros2_controllers.yaml`:

```yaml
enable_odom_tf: false
```

Super-LIO must be the source of `odom -> base_footprint` during LIO navigation.

### Frame debugging order

When a cloud appears upside down or left/right reversed, inspect each transform
separately before changing an extrinsic:

```bash
ros2 topic echo /livox/lidar --once --field header
ros2 run tf2_ros tf2_echo base_link livox_frame
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

Do not judge the point orientation by changing RViz Fixed Frame repeatedly.
Verify the raw sensor frame, base frame, odom frame, and map frame in order.

## 6. LiDAR Message Types

The real Mid-360 input supports both:

```text
lidar_type:=1  livox_ros_driver2 CustomMsg
lidar_type:=9  sensor_msgs/msg/PointCloud2 using Livox PointXYZRTLT fields
```

For PointCloud2, `livox_ros_driver2` must use `xfer_format=0`, and the actual
field layout and timestamps should be checked once on the real sensor. This is
not because PointCloud2 is unsupported; it is to verify that the installed
driver version publishes the expected per-point timing fields and that LiDAR
and IMU timestamps share a usable clock domain.

See `doc/livox_mid360_pointcloud2.md`.

## 7. Ground Segmentation and Occupancy Grid

The DFKI ground segmentation code is vendored under:

```text
src/super_lio/3rdparty/ground_segmentation
```

The occupancy-grid and ground-filter code is maintained separately from the
main LIO in:

```text
src/super_lio/include/ros/ground_segmentation_filter.h
src/super_lio/src/ros/ground_segmentation_filter.cpp
src/super_lio/include/ros/occupancy_grid_publisher.h
src/super_lio/src/ros/occupancy_grid_publisher.cpp
```

Current real configuration enables both:

```yaml
lio.occupancy_grid.enable: true
lio.occupancy_grid.topic: /map
lio.occupancy_grid.frame_id: map
lio.occupancy_grid.resolution: 0.1
lio.occupancy_grid.use_unknown: true
lio.occupancy_grid.ground_filter.enable: true
lio.occupancy_grid.ground_filter.auto_dist_to_ground: true
lio.occupancy_grid.ground_filter.dist_to_ground: 0.0
```

When `auto_dist_to_ground: true`, `dist_to_ground: 0.0` is only the default /
fallback input. The filter estimates the ground distance from the cloud. If
the seed area is insufficient, it falls back using the configured sensor mount
height.

Important map limitation discovered on the real map:

- A merged PCD contains points, but not the historical sensor origins for
  every scan.
- Ground points alone are not enough to reconstruct all observed free space.
- Many floor and wall points project into the same 2D cells, so obstacle hits
  override floor evidence.
- Online ray casting works after localization because the current sensor origin
  is known, but it only carves the area currently observed.

Last measured real `/map` statistics:

```text
Before map->odom localization / online carving:
  size=516x139, resolution=0.1
  unknown=66718, free=252, occupied=4754

After localization and live ray casting:
  size=516x139, resolution=0.1
  unknown=66620, free=2837, occupied=2267
```

This proves the map publisher and ray carving work, but the static PCD-derived
grid is still too sparse for unrestricted whole-house navigation. The next
mapping run should either:

1. enable occupancy-grid generation while mapping so sensor origins are used
   online, then save the resulting Nav2 grid; or
2. record/replay the mapping bag and rebuild the grid from poses plus scans; or
3. enable the mapping-time grid artifact below during the next mapping run.

Do not solve the black-map problem by setting all unknown cells to free. That
would allow planning through truly unobserved space.

The attempted visual screenshot of the RViz map on 2026-07-30 failed because
XWayland `XGetImage` was unavailable. The numerical map inspection above is
valid; a proper visual review is still pending.

## 8. Real Map Inventory

Last real PCD used for localization:

```text
/home/jetson/maps/real_20260730_105304/map.pcd
```

Recorded properties:

```text
points: 92678
size: about 1.5 MB
x range: -26.453 to 28.939 m
y range: -11.479 to 4.332 m
z range: -0.790 to 4.560 m
```

That directory had no `nav2_map.yaml` or PGM at the time of the last test.
Current code can now generate these files while mapping from online occupancy
evidence; it has not yet been run on the Jetson/real robot.

Older map directories with Nav2 PGM/YAML files existed at:

```text
/home/jetson/maps/real_20260728_213413
/home/jetson/maps/real_20260729_225145
```

Do not mix an old occupancy grid with the newer PCD unless their coordinate
frames are explicitly verified.

## 9. Last Real Localization and Navigation Test

Date: 2026-07-30.

What was verified:

- Livox driver, Super-LIO, DDSM driver, DM IMU, and Xbox packages could run on
  the Jetson.
- KISS eventually proposed the correct initialization and VGICP accepted it.
- `map -> odom` and `odom -> base_footprint` were both available.
- Typical accepted tracking fitness was about `0.003` to `0.006`.
- One observed pose was approximately
  `map -> base_footprint = (0.95, -0.19, yaw 42.5 deg)`. This was only the pose
  at that test time; never use it as a hard-coded next-run initial pose.
- `/lio/cloud_world` ran at approximately 10 Hz with `frame_id: odom`.
- `/lio/odom` ran at approximately 10 Hz.
- Nav2 baseline RPP nodes all reached `active`.
- A plan-only `ComputePathToPose` request from the current pose to approximately
  `(1.17, 0.01)` succeeded with `error_code: 0` and planning time `2.881 ms`.
- No `NavigateToPose` command was sent in that verification, so it did not prove
  full closed-loop vehicle motion along a Nav2 route.

Observed Jetson load during this run:

- `super_lio_node` used roughly one CPU core (`80-90%` in `ps`).
- Livox, DDSM, DM IMU, and Nav2 added additional load.
- `/lio/cloud_world` was normally 10 Hz but occasional scheduler gaps reached
  about 0.35 s.

Therefore the real Nav2 obstacle-source timeout was changed from exactly
`0.10 s` to a tolerant `0.50 s` in both local and global costmaps. This changes
only stale-sensor monitoring; it does not reduce the cloud rate.

## 10. Real Robot Bringup

### Source environment

On Jetson:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jetson/nav2_ws/install/setup.bash
source /home/jetson/AdaGraph-LIO/install/setup.bash
source /home/jetson/dev_ws/install/setup.bash
```

### LiDAR driver

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Before starting another driver, check for an existing process:

```bash
ros2 node list | grep livox
```

### Super-LIO mapping

Choose a new output directory and start without a prior map:

```bash
MAP_DIR=/home/jetson/maps/real_$(date +%Y%m%d_%H%M%S)
mkdir -p "$MAP_DIR"

ros2 launch super_lio Livox_mid360.py \
  rviz:=false \
  lidar_type:=1 \
  use_prior_map:=false \
  save_map:=true \
  save_map_dir:="$MAP_DIR" \
  map_name:=map.pcd
```

Drive slowly, cover corridors in both directions, close loops, and observe
rooms from multiple headings. End Super-LIO with normal `Ctrl+C`; the final PCD
merge happens during shutdown. Do not use `kill -9` for map saving.

### Super-LIO prior-map localization

```bash
ros2 launch super_lio Livox_mid360.py \
  rviz:=false \
  lidar_type:=1 \
  use_prior_map:=true \
  prior_map_file:=/home/jetson/maps/real_20260730_105304/map.pcd \
  localization_voxel_size:=0.10 \
  kiss_voxel_size:=0.20 \
  kiss_adaptive_ratio_test:=true
```

Manual global-relocalization request:

```bash
ros2 service call /lio/relocalization/trigger std_srvs/srv/Trigger "{}"
```

The service response `manual relocalization request cached` only confirms that
the request was queued. Success requires later logs showing a selected KISS
candidate, accepted VGICP result, and a live `map -> odom` transform.

### Nav2 only

Start Nav2 only after both `/map` and the full TF chain exist:

```bash
ros2 launch robot_nav2_bringup nav2.launch.py \
  use_sim_time:=false \
  controller_mode:=rpp \
  nav2_params_file:=/home/jetson/dev_ws/src/robot_nav2_bringup/config/nav2_params_real.yaml \
  cmd_vel_output:=/cmd_vel
```

The real Nav2 output is `geometry_msgs/msg/TwistStamped` on `/cmd_vel`.

Do not currently use `real_system.launch.py` blindly. It has two problems:

1. its default map directory is the stale
   `/home/jetson/maps/real_20260729_225145`;
2. it starts `nav2_map_server` on `/map` while current Super-LIO also publishes
   `/map`, creating two possible global map sources.

Before restoring the one-command launch, choose exactly one `/map` owner:

- stable saved PGM/YAML via `map_server`, with Super-LIO occupancy disabled; or
- Super-LIO live occupancy grid, with `map_server` omitted.

### Desktop RViz for the real robot

The development PC can see Jetson ROS topics on the same network and ROS domain.

```bash
source /opt/ros/lyrical/setup.bash
source /home/sw/nav2_ws/install/setup.bash
source /home/sw/AdaGraph-LIO/install/setup.bash
source /home/sw/robot-sim/install/setup.bash

rviz2 -d /home/sw/robot-sim/src/robot_nav2_bringup/rviz/nav2.rviz
```

The local RViz source config was changed to:

- Fixed Frame: `map`;
- point cloud: `/lio/cloud_world`;
- prior occupancy grid: `/map`;
- local/global Nav2 costmaps enabled;
- RobotModel disabled because the real bringup did not publish
  `/robot_description` to the desktop.

Use `Nav2 Goal` only in a clearly observed nearby free region during the first
motion test.

## 11. DDSM Base Driver

The real measured geometry is:

```text
wheel separation: 0.395 m
wheel diameter:   0.100 m
wheel radius:     0.050 m
```

The generic `ddsm_params.yaml` still used `wheel_separation: 0.3` during one
test and must not be used for the current chassis. The intended file is:

```text
/home/jetson/dev_ws/src/ddsm_driver/config/ddsm_robot_current.yaml
```

Last observed important values:

```yaml
serial_port: /dev/ddsm
protocol: hat_binary
wheel_radius: 0.05
wheel_separation: 0.395
control_rate: 100.0
cmd_vel_timeout: 0.6
enable_stamped_cmd_vel: true
left_motor_sign: 1
right_motor_sign: -1
imu_topic: /imu/data
```

Use the current config explicitly:

```bash
ros2 launch ddsm_driver ddsm_driver.launch.py \
  params_file:=/home/jetson/dev_ws/src/ddsm_driver/config/ddsm_robot_current.yaml \
  serial_port:=/dev/ddsm \
  imu_topic:=/imu/data \
  load_saved_pid:=true
```

Verify the live value rather than trusting the launch command:

```bash
ros2 param get /ddsm_driver_node wheel_separation
ros2 param get /ddsm_driver_node wheel_radius
```

The saved yaw PI overlay was last observed at:

```text
/home/jetson/.config/ddsm_driver/autotuned_pid.yaml
```

with approximately:

```yaml
pid_kp: 0.0532275045422
pid_ki: 0.0532275045422
pid_kd: 0.0
pid_output_limit: 0.7
```

No D term is expected from the current first-order yaw model; the auto tuner is
designed to produce a PI candidate and validate positive/negative rotation.

Auto-tune service sequence:

```bash
ros2 service call /ddsm_driver/auto_tune_yaw std_srvs/srv/Trigger "{}"
ros2 service call /ddsm_driver/auto_tune_status std_srvs/srv/Trigger "{}"
ros2 service call /ddsm_driver/save_tuned_pid std_srvs/srv/Trigger "{}"
```

Run auto tuning only with motors powered, the robot lifted or in a controlled
clear area as required by the procedure, and a working IMU stream.

The front drive axle and about 20 cm of rear overhang were incorporated into
the Nav2 footprint:

```yaml
footprint: "[[-0.36, -0.245], [-0.36, 0.245], [0.07, 0.245], [0.07, -0.245]]"
```

This configuration exists in:

```text
/home/sw/robot-sim/src/robot_nav2_bringup/config/nav2_params_real.yaml
/home/jetson/dev_ws/src/robot_nav2_bringup/config/nav2_params_real.yaml
```

## 12. DM IMU

Two IMU roles must not be confused:

- Super-LIO normally uses the Mid-360 internal IMU on `/livox/imu`.
- DDSM yaw closed-loop / auto tuning uses the separate DM IMU on `/imu/data`.

The standalone DM command used on Jetson was:

```bash
ros2 run dm_imu dm_imu_node --ros-args \
  -p port:=/dev/dm_imu \
  -p baud:=921600
```

The local source package is currently at:

```text
/home/sw/dev_ws/src/dm_imu
```

The Jetson package is under:

```text
/home/jetson/dev_ws/src/dm_imu
```

There is also a ros2_control hardware path, but the direct standalone node is
the intended command for this setup. Do not accidentally start both and
publish duplicate `/imu/data` streams.

Udev aliases intended by the setup include:

```text
/dev/dm_imu
/dev/ddsm
```

Relevant installers:

```text
/home/sw/dev_ws/src/dm_imu/scripts/install_dm_imu_udev.sh
/home/jetson/dev_ws/src/ddsm_driver/scripts/install_ddsm_udev.sh
```

There is an unresolved Jetson boot issue: with only the DM IMU USB device
connected before power-on, the Jetson sometimes failed to boot, while plugging
it in after boot worked. An NVMe-first boot-order change was attempted, but the
apparently successful boot occurred after the DM IMU had been unplugged, so the
fix was never proven. Treat this as unresolved hardware/firmware behavior, not
as a ROS software bug.

## 13. Xbox Teleoperation

The Xbox One controller package lives on Jetson in:

```text
/home/jetson/dev_ws/src/xbox_teleop
```

USB Bluetooth was preferred over onboard Bluetooth for signal quality. The
controller enable/deadman button for the tested configuration is **Y**, not RB.

System services were installed for reconnection and ROS teleop:

```text
xbox-controller-autoconnect.service
xbox-teleop-ros.service
```

The teleop node publishes `TwistStamped` directly to `/cmd_vel`. Nav2's velocity
smoother also publishes to `/cmd_vel`. There is currently no command mux.

Therefore stop Xbox teleop before Nav2 controls the base:

```bash
sudo systemctl stop xbox-teleop-ros.service
```

After stopping Nav2, restore teleop with:

```bash
sudo systemctl start xbox-teleop-ros.service
```

Do not leave both publishers active. Add a proper twist mux / priority arbiter
before expecting seamless manual override during autonomous navigation.

## 14. Simulation Commands

The shell startup already sources the Lyrical, Nav2, MoveIt, AdaGraph-LIO, and
robot-sim overlays. It also defines `clean_sim`.

Manual source sequence:

```bash
source /opt/ros/lyrical/setup.bash
source /home/sw/nav2_ws/install/setup.bash
source /home/sw/ws_moveit2/install/setup.bash
source /home/sw/AdaGraph-LIO/install/setup.bash
source /home/sw/robot-sim/install/setup.bash
```

House world with Super-LIO, without Nav2 or MoveIt:

```bash
ros2 launch robot_bringup full.launch.py \
  world:=aws_house.sdf \
  headless:=false \
  nav2:=false \
  super_lio:=true \
  moveit:=false \
  nav2_rviz:=false \
  moveit_rviz:=false
```

House world only:

```bash
ros2 launch robot_bringup full.launch.py \
  world:=aws_house.sdf \
  headless:=false \
  nav2:=false \
  super_lio:=false \
  moveit:=false \
  nav2_rviz:=false \
  moveit_rviz:=false
```

Keyboard control in simulation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/base_controller/cmd_vel \
  -p stamped:=true \
  -p frame_id:=base_footprint
```

The current `full.launch.py` hard-codes `use_prior_map: false` for its embedded
Super-LIO launch. For prior-map simulation, start Gazebo without Super-LIO,
then launch `super_lio sim.py` separately as documented in
`doc/sim_super_lio_nav2_usage.md`.

Available worlds:

```text
empty.sdf
aws_warehouse.sdf
aws_house.sdf
```

Do not use `empty.sdf` for localization or navigation quality evaluation.

`aws_house.sdf` physics was reduced from 1000 Hz / 1 ms to 500 Hz / 2 ms to
lower simulation CPU load. This reduces physics update cost while retaining a
much higher rate than the 10 Hz LiDAR and ordinary mobile-base control loop.

## 15. Build and Test

### Development PC

```bash
cd /home/sw/AdaGraph-LIO
source /opt/ros/lyrical/setup.bash
colcon build --packages-select livox_ros_driver2 basic super_lio
colcon test --packages-select basic super_lio
colcon test-result --test-result-base build/super_lio --verbose
```

Last test records:

- Ultra review/fixup: 54 tests, 0 failures, documented in
  `doc/ultra_review_fix_report.md`.
- Adaptive KISS / strict VGICP follow-up: scoped Super-LIO result was
  26 tests, 0 failures.
- An unscoped `colcon test-result --verbose` also showed stale unrelated
  Livox lint output. Use package-scoped test result bases before diagnosing a
  regression.

Paper-sequence evaluation is documented in
`doc/super_lio_paper_dataset_evaluation.md`:

```text
M2DGR door_01: repaired RMSE 0.454238 m, original 0.463061 m
M2DGR door_02: repaired RMSE 0.342372 m, original 0.346973 m
```

### Jetson

Never copy x86 `build/`, `install/`, or `log/` directories to Jetson.

```bash
cd /home/jetson/AdaGraph-LIO
source /opt/ros/jazzy/setup.bash
BUILD_JOBS=2 ./scripts/build_jetson.sh
source install/setup.bash
```

Stable tagged deployment:

```bash
BUILD_JOBS=2 ./scripts/deploy_jetson_version.sh jetson-vX.Y.Z
```

See `doc/jetson_development_and_deployment.md` for rollback behavior.

## 16. What Is Complete

- Super-LIO publishes standard Nav2 odom TF.
- Prior-map localization is integrated into the Super-LIO node.
- FastVGICP is isolated on a worker thread.
- `map -> odom` is computed without resetting ESKF.
- KISS startup / LOST / manual recovery path exists.
- KISS ratio-test candidate adaptation and VGICP verification are implemented
  in the current worktree.
- Real Mid-360 upside-down installation and base offsets are represented.
- Livox CustomMsg and PointCloud2 input paths exist.
- Ground segmentation and occupancy grid publishing are integrated.
- Nav2 RPP configuration uses the full chassis footprint and `TwistStamped`.
- Real wheel geometry is known and represented in `ddsm_robot_current.yaml`.
- DDSM yaw PI auto tuning and persistence exist.
- Xbox teleoperation and reconnect services exist.
- Simulation supports house / warehouse, Super-LIO, Nav2, and MoveIt.
- Jetson build/deploy scripts and a private Git remote exist.

## 17. What Is Not Complete

Priorities for the next session:

1. **Validate Point-LIO on the active real robot.** Record same-location
   startup, arbitrary-start startup, intentional LOST/recovery, manual trigger,
   and false-positive rejection with configuration, map checksum, candidate and
   verifier diagnostics, `map -> odom`, RSS, CPU, and queue metrics.
2. **Improve arbitrary-start global localization if that evidence fails.** KISS
   against a merged PCD
   is not yet reliable enough. Do not relax `initial_max_fitness` to hide this.
   Use a descriptor/keyframe place-recognition path such as Scan Context for
   candidate retrieval if strict registration alone cannot make startup robust.
3. **Run the new mapping-time Nav2 grid artifact on the real robot.** Enable
   `lio.occupancy_grid.source: online`, set
   `lio.occupancy_grid.artifact.enable: true` and give
   `lio.occupancy_grid.artifact.yaml_path` an absolute path in the new map
   directory. The writer is format-tested against Nav2 `map_server`, but
   online update/lock/RSS long-run behavior still needs a representative run.
4. **Perform an actual closed-loop Nav2 drive in an observed safe area.** The last test proved planning,
   TF, costmaps, and command wiring, but did not send a NavigateToPose goal.
5. **Fix the one-command real launch.** Remove the duplicate `/map` source and
   stale default map path.
6. **Add a command mux.** Xbox teleop and Nav2 currently cannot safely coexist.
7. **Validate DDSM at higher angular speed.** The PI gains were tuned at low
   yaw rate; high-speed behavior and tail swing still need a controlled test.
8. **Visually inspect and quantify the latest grid.** Check wall alignment,
   free-space connectivity, and whether LIO/map irregularity is from sparse
   observations, drift, or grid projection.
9. **Resolve the DM IMU boot interaction.** Confirm boot order with the IMU
   continuously connected before claiming NVMe-first solved it.
10. **Put robot-sim under real version control.** It currently has no commits.
11. **Mechanical arm work remains.** Navigation is far along, but arm hardware,
    MoveIt calibration, grasping, and task-level behavior are separate unfinished
    workstreams.
12. **Profile Point-LIO prior-map memory before changing ownership.** Measure
    `MapManager` KeyFrames, prior-map PCL, local-submap/registration temporary
    clouds and map-product staging; add a budget/compaction design only after
    the dominant owners are known. Super-LIO remains a regression baseline
    until Point-LIO meets the documented RSS, accuracy, recovery and real-robot
    retirement gates.
13. **Do not enable AdaGraph absolute localization factors on the real robot
    yet.** `lio.backend.use_localization_factor` and
    `lio.backend.drive_map_to_odom` remain experimental and default false.

## 18. New Session Startup Checklist

Give the next coding session this instruction:

```text
Read /home/sw/AdaGraph-LIO/HANDOFF.md first. Then inspect git status and the
latest architecture commit before editing. Treat main@2a2850e as the
pre-refactor baseline. Read all four files under `doc/architecture/` before
changing a boundary. Before real-robot work, use the active AidLux X1 target
and re-check live ROS nodes, TF, wheel_separation, map source, and `/cmd_vel`
publishers. Never start duplicate Livox, Point-LIO/Super-LIO, map-server,
odom-TF, or cmd_vel publishers.
```

First commands:

```bash
cd /home/sw/AdaGraph-LIO
git status --short --branch
git diff --check
git diff --stat
sed -n '1,260p' HANDOFF.md
```

Before moving the real robot:

```bash
ros2 node list
ros2 topic info /cmd_vel -v
ros2 param get /ddsm_driver_node wheel_separation
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once --field info
```

Expected safety invariants:

```text
one odom -> base_footprint publisher
one map -> odom publisher
one /map owner
one active /cmd_vel publisher
wheel_separation = 0.395
map -> odom -> base_footprint available before Nav2 activation
```

## 19. Detailed Documents

- `doc/sim_super_lio_nav2_usage.md`: simulation, mapping, prior-map loading,
  Nav2 commands, TF debugging.
- `doc/jetson_development_and_deployment.md`: ARM64 build, tags, deployment,
  rollback.
- `doc/ultra_review_fix_report.md`: complete Fable5/Ultra finding-to-fix map,
  residual risks, and tests.
- `doc/super_lio_paper_dataset_evaluation.md`: M2DGR reproduction and original
  versus repaired accuracy.
- `doc/livox_mid360_pointcloud2.md`: real PointCloud2 integration.
- `doc/adagraph_lio_project_proposal.md`: competition-project AdaGraph-LIO
  technical narrative.
- `README.md`: project overview and build entry points.

## 20. X1 Deployment

The active ARM64 target is now the AidLux X1:

```text
SSH alias: aidlux
management address: 192.168.1.166
user/home: aidlux / /home/aidlux
ROS: Humble
LiDAR bridge: br-lan, 192.168.5.5/24
Mid-360 address: 192.168.5.170
```

Deployed workspaces and maps:

```text
/home/aidlux/AdaGraph-LIO
/home/aidlux/DDSM_Driver_ros2
/home/aidlux/nav2_ws
/home/aidlux/robot_ws
/home/aidlux/maps/real_20260728_213413
```

`robot_ws` contains `dm_imu`, `xbox_teleop`, and `robot_nav2_bringup`.
The DM-IMU package intentionally contains only the standalone serial node;
its ros2_control hardware plugin and controller launch path were removed.
Stable serial aliases are `/dev/ddsm` and `/dev/dm_imu`. The `aidlux` user is
in the `dialout` and `input` groups.

Xbox auto-connect and teleoperation are enabled as system services:

```bash
systemctl status xbox-controller-autoconnect.service
systemctl status xbox-teleop-ros.service
```

Real-system launch:

```bash
ros2 launch robot_nav2_bringup real_system.launch.py \
  map_dir:=/home/aidlux/maps/real_20260728_213413 rviz:=false
```

The X1 smoke test loaded the PCD with FastVGICP, loaded the Nav2 PGM/YAML,
and created the RPP/Navfn/behavior plugins. Without live LiDAR packets it
correctly waits for `odom -> base_footprint`, so full lifecycle activation
must be verified once the Mid-360 is connected.

MoveIt planning assets are also installed in `robot_ws`:

```text
so101_sim
robot_description
robot_moveit_config
```

The X1 uses the Humble-compatible OMPL adapter format and defaults
`use_sim_time` to `false`. A smoke test reached `You can start planning now!`.
Gazebo worlds are intentionally kept on the development computer. The current
MoveIt controller names still require a real SO-101 hardware driver before
trajectory execution can move either arm.
