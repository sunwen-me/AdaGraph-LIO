# AdaGraph-LIO v1.0

Adaptive SLAM system with selectable Super-LIO or Point-LIO frontends,
Hessian/rule-based SE(3) covariance estimation, and a GTSAM/iSAM2 backend.

## Features

- **Super-LIO Frontend**: Robust LiDAR-Inertial Odometry
- **Point-LIO Frontend Variant**: In-process Small Point-LIO with the same
  AdaGraph, prior-map localization, TF, and Nav2 output contracts
- **Adaptive Covariance**: Hessian covariance with rule-based degeneracy fallback
- **GTSAM Backend**: Incremental pose graph optimization. Optionally fuses
  accepted prior-map registrations as absolute factors
  (`lio.backend.use_localization_factor`, default off) and publishes the
  optimized map-frame keyframe trajectory on `/lio/backend/optimized_path`;
  experimental, not yet validated on a real robot
- **Prior-map Localization**: KISS-Matcher initialization and FastVGICP tracking
- **Navigation Output**: Ground segmentation and ROS 2 occupancy grid
- **ROS 2 Integration**: ROS 2 nodes, launch files, and Nav2-ready outputs

## Dependencies

- Ubuntu 26.04 with ROS 2 Lyrical (the distro used by the docs and all
  testing so far; the build scripts follow whatever `$ROS_DISTRO` is sourced,
  but other distros are untested)
- GTSAM 4.x
- Eigen3
- PCL
- TBB and OpenMP

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
INSTALL_DEPS=1 ./scripts/build_jetson.sh
```

Note: the first CMake configure needs internet access — the vendored ROBIN
library downloads its `pmc` and `xenium` dependencies at configure time.

## Run

Super-LIO version:

```bash
source install/setup.bash
ros2 launch super_lio Livox_mid360.py
```

Point-LIO version:

```bash
source install/setup.bash
ros2 launch super_lio PointLIO_mid360.py
```

仓库内 M2DGR 数据集可使用专用入口：

```bash
ros2 launch super_lio PointLIO_m2dgr.py
ros2 bag play Datasets/room_01_ros2 \
  --delay 1.0 --wait-for-all-acked 5 \
  --topics /velodyne_points /handsfree/imu
```

## Documentation

- [Jetson development, deployment, updates, and rollback](doc/jetson_development_and_deployment.md)
- [Simulation, Super-LIO, and Nav2 usage](doc/sim_super_lio_nav2_usage.md)
- [Livox Mid-360 PointCloud2 support](doc/livox_mid360_pointcloud2.md)
- [Point-LIO frontend version](doc/point_lio_frontend.md)

## License

This repository is a derivative of upstream
[Super-LIO](https://arxiv.org/abs/2509.05723) (see `doc/参考文献.md`); the
upstream license terms are still being confirmed, so no top-level LICENSE file
is provided yet. The `super_lio` and `basic` package manifests carry the GPLv3
declaration inherited from upstream. Vendored third-party components
(Small Point-LIO, fast_gicp, KISS-Matcher, ROBIN, ground_segmentation, robin-map,
livox_ros_driver2, and others) keep their own licenses — see
[doc/LICENSES.md](doc/LICENSES.md).

## Citation

A publication for AdaGraph-LIO is in preparation. Until it is available,
please cite this repository by its URL, and cite upstream Super-LIO
(<https://arxiv.org/abs/2509.05723>) when using the Super-LIO frontend. For the
Point-LIO version, also cite the upstream Small Point-LIO and Point-LIO work.
