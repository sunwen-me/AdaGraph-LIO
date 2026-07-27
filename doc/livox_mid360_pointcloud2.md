# Livox Mid-360 PointCloud2 接入说明

Super-LIO 支持两种 Livox Mid-360 输入：

| `lio.sensor.lidar_type` | ROS 2 消息 | Driver2 `xfer_format` | 逐点时间 |
|---|---|---|---|
| `1` | `livox_ros_driver2/msg/CustomMsg` | `1` | `offset_time`，纳秒 |
| `9` | `sensor_msgs/msg/PointCloud2` | `0` | `timestamp`，帧内纳秒偏移 |

类型 `9` 只接受 Livox Driver2 的 PointXYZRTLT 字段布局：

```text
x          FLOAT32
y          FLOAT32
z          FLOAT32
intensity  FLOAT32
tag        UINT8
line       UINT8
timestamp  FLOAT64
```

`line` 字段由驱动发布，但 Super-LIO 当前不参与计算。Livox Driver2
1.2.x 将 `offset_time` 直接写入 `timestamp`，Super-LIO 按纳秒换算：

```text
offset_time = point.timestamp * 1e-9
```

解析器也兼容部分转换数据集使用的绝对纳秒时间戳。解析时继续应用
Livox `tag` 有效性过滤、盲区过滤、最大量程过滤和 `filter_rate`
抽样。扫描结束时间取全部有效逐点时间中的最大偏移，因此 ESKF
可以继续执行逐点运动畸变补偿。

## 编译

```bash
cd /home/sw/AdaGraph-LIO
source /opt/ros/lyrical/setup.bash
colcon build --packages-select super_lio --symlink-install
source install/setup.bash
```

## 启动 PointCloud2 驱动

仓库自带的 `rviz_MID360_launch.py` 使用 `xfer_format=0`：

```bash
source /opt/ros/lyrical/setup.bash
source /home/sw/AdaGraph-LIO/install/setup.bash

ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

如果使用自定义 Livox Driver2 launch 文件，必须设置：

```text
xfer_format = 0
```

不要使用 `xfer_format=2`。格式 2 虽然也是 `PointCloud2`，但只有普通 `PointXYZI` 字段，没有逐点时间，无法完成完整去畸变。

## 启动 Super-LIO

```bash
source /opt/ros/lyrical/setup.bash
source /home/sw/AdaGraph-LIO/install/setup.bash

ros2 launch super_lio Livox_mid360.py \
  lidar_type:=9 \
  rviz:=false
```

`Livox_mid360.py` 默认仍为 `lidar_type:=1`，因此原有 CustomMsg 使用方式不受影响。

## 启动前检查

```bash
ros2 topic type /livox/lidar
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --field fields
ros2 topic hz /livox/imu
```

正确结果应包括：

```text
sensor_msgs/msg/PointCloud2
x y z intensity tag line timestamp
```

Super-LIO 启动日志应显示：

```text
Using Lidar type: LIVOX_POINTCLOUD2
```

如果字段缺失、类型不匹配、消息头时间无效或逐点时间不是纳秒，Super-LIO 会拒绝该帧并输出节流错误日志，不会将错误点云静默送入 ESKF。
