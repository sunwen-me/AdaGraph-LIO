# Point-LIO Operation

HybridSLAM uses the vendored
[Small Point-LIO](https://github.com/Yancey2023/small_point_lio) frontend in
process. Point-LIO owns IMU propagation, deskew, point-to-plane updates and
its local voxel map. Accepted point-to-plane observations are forwarded to the
HybridSLAM backend without reconstructing them from ROS odometry messages.

The downstream path builds keyframes and map snapshots, publishes map products
and runs optional prior-map localization. It does not write registration output
back into Point-LIO's odom-frame estimator map. A successful registration
updates the `map -> odom` alignment instead.

## Launch

Mid-360 input:

```bash
source install/setup.bash
ros2 launch hybrid_slam PointLIO_mid360.py
```

M2DGR input:

```bash
ros2 launch hybrid_slam PointLIO_m2dgr.py
```

The M2DGR launch expects `/velodyne_points` and `/handsfree/imu`.

## Prior-Map Localization

```bash
ros2 launch hybrid_slam PointLIO_mid360.py \
  use_prior_map:=true \
  prior_map_file:=/absolute/path/to/map.pcd
```

KISS-Matcher supplies an optional global initialization candidate; FastVGICP
validates and tracks against a cropped prior-map region. The resulting pose is
committed as:

```text
T_map_odom = T_map_base_registration * inverse(T_odom_base_same_timestamp)
```

The `/lio/relocalization/trigger` service requests a new global localization
attempt when prior-map localization is enabled.

## Map Saving

```bash
ros2 launch hybrid_slam PointLIO_mid360.py \
  save_map:=true \
  save_map_dir:=/absolute/path/to/output \
  map_name:=point_lio_map.pcd
```

Map saving uses immutable backend snapshots. The output directory must resolve
to an absolute path; normal node shutdown drains the save scheduler.

## Parameters

Point-LIO frontend parameters are at the top of:

- `src/hybrid_slam/config/point_lio_mid360.yaml`
- `src/hybrid_slam/config/point_lio_m2dgr.yaml`

Important frontend parameters include `point_filter_num`, `min_distance`,
`max_distance`, `space_downsample`, `space_downsample_leaf_size`,
`map_resolution`, `init_map_size`, `extrinsic_T`, `extrinsic_R`,
`laser_point_cov` and `match_sqaured`.

`publish_odometry_without_downsample: true` exposes Point-LIO's point-level
state updates. Use `lio.output.odometry_rate_hz` to bound the externally
published `/lio/odom`, TF and path rate without slowing the estimator; the
shipped configurations use `100.0`. Set it to `0` only when every frontend
output is required, because a dense spinning lidar can otherwise publish tens
of thousands of messages per second.

`point_lio.input.*` controls DDS transport and bounded input queues. `lio.*`
controls HybridSLAM, prior-map localization, TF, occupancy and map saving.
