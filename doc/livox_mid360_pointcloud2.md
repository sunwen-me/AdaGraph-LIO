# Livox Mid-360 PointCloud2 Input

Point-LIO accepts Livox CustomMsg and Livox Driver2 PointCloud2 input through
the `lidar_type` launch argument:

```bash
ros2 launch hybrid_slam PointLIO_mid360.py \
  lidar_type:=livox_pointcloud2
```

For Livox Driver2 PointCloud2 input, configure `xfer_format=0`. The message
must provide `x`, `y`, `z`, `intensity`, `tag`, `line` and `timestamp` fields.
The Point-LIO adapter preserves the raw input stream and applies the frontend
range and sampling configuration itself.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 launch hybrid_slam PointLIO_mid360.py lidar_type:=livox_pointcloud2
```

Check the configured topics before starting localization:

```bash
ros2 topic type /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
```
