# HybridSLAM

HybridSLAM is a ROS 2 LiDAR-inertial SLAM system built around the vendored
Small Point-LIO frontend. It combines Point-LIO odometry with an incremental
GTSAM/iSAM2 pose-graph backend, optional prior-map localization,
occupancy-grid output and asynchronous PCD map saving.

## Features

- Small Point-LIO local odometry with IMU fusion and local voxel mapping.
- Hessian-based covariance estimation and incremental pose-graph optimization.
- Prior-map localization using KISS-Matcher initialization and FastVGICP
  tracking, publishing `map -> odom` without rewriting Point-LIO's odom map.
- ROS 2 odometry, path, point-cloud, occupancy-grid and Nav2 TF outputs.
- M2DGR and Oxford Spires recorded-data regression support.

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
INSTALL_DEPS=1 ./scripts/build_jetson.sh
source install/setup.bash
```

The first configuration can download dependencies used by the vendored
KISS-Matcher implementation.

## Run

Mid-360:

```bash
ros2 launch hybrid_slam PointLIO_mid360.py
```

M2DGR:

```bash
ros2 launch hybrid_slam PointLIO_m2dgr.py
ros2 bag play Datasets/room_01_ros2 \
  --delay 1.0 --wait-for-all-acked 5 \
  --topics /velodyne_points /handsfree/imu
```

Oxford Spires:

The Oxford converter streams PCD files from the downloaded archive and writes
MCAP by default. MCAP is preferred for large deterministic replay; the
original SQLite bag can still be selected explicitly with `--storage-id
sqlite3`.

```bash
python3 scripts/convert_oxford_spires.py \
  --lidar-zip Datasets/oxford_spires/keble-college-02/sequences/2024-03-12-keble-college-02/raw/lidar-clouds.zip \
  --imu-csv Datasets/oxford_spires/keble-college-02/sequences/2024-03-12-keble-college-02/raw/imu.csv \
  --output Datasets/oxford_spires/keble-college-02/keble-college-02_mcap

scripts/run_oxford_spires_eval.sh keble-college-02 1.0 \
  /tmp/adagraph-lio-oxford-keble02-eval
```

The Oxford runner records both the frontend `/lio/odom` trajectory and the
latest backend `/lio/backend/optimized_path` snapshot. The frontend trajectory
does not include later graph corrections; use the backend trajectory when
evaluating loop-closure corrections.

## Documentation

- [Point-LIO operation and configuration](doc/point_lio_frontend.md)
- [Livox Mid-360 PointCloud2 input](doc/livox_mid360_pointcloud2.md)
- [Jetson development and deployment](doc/jetson_development_and_deployment.md)
- [Third-party licenses](doc/LICENSES.md)

## License

Vendored components retain their own licenses. See
[doc/LICENSES.md](doc/LICENSES.md).
