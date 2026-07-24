# AdaGraph-LIO v1.0

Adaptive SLAM system with a Super-LIO frontend, Hessian/rule-based
SE(3) covariance estimation, and a GTSAM/iSAM2 backend.

## Features

- **Super-LIO Frontend**: Robust LiDAR-Inertial Odometry
- **Adaptive Covariance**: Hessian covariance with rule-based degeneracy fallback
- **GTSAM Backend**: Incremental pose graph optimization
- **Prior-map Localization**: KISS-Matcher initialization and FastVGICP tracking
- **Navigation Output**: Ground segmentation and ROS 2 occupancy grid
- **ROS2 Integration**: Full ROS2 Humble/Jazzy support

## Dependencies

- Ubuntu 22.04/24.04
- ROS2 Humble/Jazzy
- GTSAM 4.x
- Eigen3
- PCL
- TBB and OpenMP

## Build

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
INSTALL_DEPS=1 ./scripts/build_jetson.sh
```

## Run

```bash
source install/setup.bash
ros2 launch super_lio Livox_mid360.py
```

## Documentation

- [Jetson development, deployment, updates, and rollback](doc/jetson_development_and_deployment.md)
- [Simulation, Super-LIO, and Nav2 usage](doc/sim_super_lio_nav2_usage.md)
- [Livox Mid-360 PointCloud2 support](doc/livox_mid360_pointcloud2.md)

## Citation

If you use this work, please cite:

```bibtex
@article{adagraph_lio2026,
  title={AdaGraph-LIO: Adaptive SLAM with Hessian Covariance Estimation},
  author={Your Name},
  year={2026}
}
```
