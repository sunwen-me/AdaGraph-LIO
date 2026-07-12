# AdaGraph-LIO v1.0

Adaptive SLAM system with Super-LIO frontend, KalmanNet-inspired temporal covariance estimator, and GTSAM/iSAM2 backend.

## Features

- **Super-LIO Frontend**: Robust LiDAR-Inertial Odometry
- **Adaptive Weights**: GRU-based temporal covariance estimation
- **GTSAM Backend**: Incremental pose graph optimization
- **ROS2 Integration**: Full ROS2 Humble/Jazzy support

## Dependencies

- Ubuntu 22.04/24.04
- ROS2 Humble/Jazzy
- GTSAM 4.x
- LibTorch (C++ API)
- Eigen3
- PCL

## Build

```bash
# Install dependencies
sudo apt install libgoogle-glog-dev libtbb-dev

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/your-repo/AdaGraph-LIO.git
cd ..
colcon build --packages-select ada_graph_lio
```

## Run

```bash
source install/setup.bash
ros2 launch ada_graph_lio adagraph_lio.launch.py
```

## Training

```bash
cd training
python train_covariance_estimator.py --dataset path/to/dataset --output models/covariance_estimator.pth
python export_model.py
```

## Architecture

See [docs/architecture.md](docs/architecture.md) for detailed system architecture.

## Citation

If you use this work, please cite:

```bibtex
@article{adagraph_lio2026,
  title={AdaGraph-LIO: Adaptive SLAM with Learned Covariance Estimation},
  author={Your Name},
  year={2026}
}
```
