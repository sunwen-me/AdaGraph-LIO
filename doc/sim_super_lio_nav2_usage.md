# Super-LIO 仿真、建图、加载地图与 Nav2 使用手册

本文档记录当前工程中 Gazebo/RGL Mid360、Super-LIO、先验地图定位和 Nav2 的常用操作。

当前目录约定：

- Super-LIO 工程：`/home/sw/AdaGraph-LIO`
- 仿真工程：`/home/sw/robot-sim`
- Super-LIO 仿真配置：`/home/sw/AdaGraph-LIO/src/super_lio/config/sim.yaml`
- 推荐地图保存目录：`/home/sw/AdaGraph-LIO/src/super_lio/map/<map_name>/`

## 1. 环境 source

每个新终端先 source：

```bash
source /opt/ros/lyrical/setup.bash
source /home/sw/nav2_ws/install/setup.bash
source /home/sw/ws_moveit2/install/setup.bash
source /home/sw/AdaGraph-LIO/install/setup.bash
source /home/sw/robot-sim/install/setup.bash
```

如果修改了源码或配置后，需要重新编译：

```bash
cd /home/sw/AdaGraph-LIO
source /opt/ros/lyrical/setup.bash
colcon build --packages-select super_lio

cd /home/sw/robot-sim
source /opt/ros/lyrical/setup.bash
colcon build
```

## 2. 当前坐标系约定

标准 TF 应保持：

```text
map -> odom -> base_footprint/base_link
```

当前仿真传感器约定：

- IMU：正常安装，不翻转。
- RGL/Mid360：相对 `base_link` 绕 X 轴 180 度倒装。
- RGL 点云 frame：`mid360_rgl_frame`
- Super-LIO 输出 odom：`odom -> base_footprint`
- 先验地图定位输出全局校正：`map -> odom`

`sim.yaml` 中关键外参：

```yaml
lio.sensor.imu_acc_scale: [1.0, 1.0, 1.0]
lio.sensor.imu_gyro_scale: [1.0, 1.0, 1.0]

lio.extrinsic.lidar_imu: [0.0, 0.0, 0.0,
                          1.0, 0.0, 0.0,
                          0.0, -1.0, 0.0,
                          0.0, 0.0, -1.0]
lio.extrinsic.odom_robo: [0.143, 0.0, 0.735, 0.0, 0.0, 0.0]
```

这里的 `lidar_imu` 是绕 X 轴 180 度：

```text
[ 1   0   0
  0  -1   0
  0   0  -1 ]
```

## 3. 清理旧仿真进程

如果 TF、clock、点云方向或者参数表现不对，先清掉旧进程：

```bash
pkill -9 -f "ros2 launch robot_bringup|gz-sim|gz sim|super_lio_node|robot_state_publisher|parameter_bridge|ros_gz_bridge|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|lifecycle_manager"

ros2 daemon stop
sleep 1
ros2 daemon start
```

## 4. 启动仿真

启动 warehouse world，不启动 Nav2，只启动 Super-LIO：

```bash
ros2 launch robot_bringup full.launch.py \
  world:=aws_warehouse.sdf \
  headless:=false \
  nav2:=false \
  super_lio:=true \
  moveit:=false \
  nav2_rviz:=false \
  moveit_rviz:=false
```

只启动 Gazebo，不启动 Super-LIO 和 Nav2：

```bash
ros2 launch robot_bringup full.launch.py \
  world:=aws_warehouse.sdf \
  headless:=false \
  nav2:=false \
  super_lio:=false \
  moveit:=false \
  nav2_rviz:=false \
  moveit_rviz:=false
```

可用 world：

```text
empty.sdf
aws_warehouse.sdf
aws_house.sdf
```

## 5. 遥控机器人建图

仿真启动后，用键盘遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r cmd_vel:=/base_controller/cmd_vel \
  -p stamped:=true \
  -p frame_id:=base_footprint
```

建图时建议：

- 先低速直行，确认 `/lio/odom` 方向正确。
- 再小角速度转向，确认点云不会转向跑飞。
- 建图路线尽量闭合，覆盖 Nav2 需要走的区域。
- 不要用空白 world 测导航，空场景对定位和地图质量都不友好。

## 6. 保存 PCD 地图

Super-LIO 的保存地图不是 service。它由参数 `lio.map.save_map` 控制，在节点正常退出时保存最终 PCD。

推荐单独启动 Super-LIO 建图，这样可以直接用参数覆盖：

```bash
ros2 run super_lio super_lio_node \
  --ros-args \
  --params-file /home/sw/AdaGraph-LIO/install/super_lio/share/super_lio/config/sim.yaml \
  -p use_sim_time:=true \
  -p lio.map.save_map:=true \
  -p lio.map.if_filter:=true \
  -p lio.map.save_map_dir:=map/aws_warehouse \
  -p lio.map.map_name:=aws_warehouse.pcd \
  -p lio.map.ds_size:=0.2 \
  -p lio.map.save_interval:=500
```

注意：`lio.map.save_map_dir` 在代码中会拼到 Super-LIO 源码包目录下面，所以推荐写相对路径。

上面的最终地图路径是：

```text
/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/aws_warehouse.pcd
```

建图过程中会先保存分片：

```text
/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/PCD/scans_0.pcd
/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/PCD/scans_1.pcd
...
```

结束建图时，用 `Ctrl+C` 正常关闭 `super_lio_node`。关闭时会自动合并分片并保存最终地图。

如果 `lio.map.save_interval < 0`，不会保存分片。一般不建议这样用。

## 7. 加载先验地图定位

加载地图时关闭保存地图，打开先验地图定位和 KISS-Matcher：

```bash
ros2 launch super_lio sim.py \
  use_sim_time:=true \
  rviz:=true \
  use_prior_map:=true \
  map_file:=/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/aws_warehouse.pcd
```

`sim.py` 会自动设置：

```yaml
lio.prior_map_localization.enable: true
lio.prior_map_localization.map_file: <map_file>
lio.kiss_matcher.enable: true
```

也可以直接 `ros2 run`：

```bash
ros2 run super_lio super_lio_node \
  --ros-args \
  --params-file /home/sw/AdaGraph-LIO/install/super_lio/share/super_lio/config/sim.yaml \
  -p use_sim_time:=true \
  -p lio.map.save_map:=false \
  -p lio.prior_map_localization.enable:=true \
  -p lio.prior_map_localization.map_file:=/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/aws_warehouse.pcd \
  -p lio.kiss_matcher.enable:=true
```

定位流程：

```text
Super-LIO 当前点云
-> 转到 odom 坐标系并累积短时局部子地图
-> 根据预测全局位姿裁剪先验地图
-> GICP/VGICP 配准
-> 得到 T_map_lidar
-> 查询同一时间戳 T_odom_lidar
-> T_map_odom = T_map_lidar * inverse(T_odom_lidar)
-> 质量门控
-> SE(3) 平滑
-> 发布 map -> odom
```

正常状态下：

- KISS-Matcher 不按固定 Hz 跑。
- KISS-Matcher 用于初始化、LOST 恢复、人工触发。
- KISS-Matcher 触发前会等待局部子地图至少累计 `lio.prior_map_localization.min_kiss_frames` 帧。
- 正常连续定位主要跑 GICP/VGICP。
- GICP/VGICP 在独立工作线程中运行，不阻塞 Super-LIO 主处理线程。
- VGICP/GICP 不重置 ESKF，只更新 `map -> odom`。

## 8. 手动触发重定位

当先验地图定位已经启用时，可以手动触发一次全局重定位：

```bash
ros2 service call /super_lio_node/manual_relocalization std_srvs/srv/Trigger {}
```

这个 service 只置位请求，真正的 KISS-Matcher 初始化/恢复在 Super-LIO 主流程提交局部子地图后执行。

## 9. 启动 Nav2 导航

推荐顺序：先启动仿真，再启动带先验地图的 Super-LIO，最后启动 Nav2。

这样 Nav2 启动时 `/clock`、`/map` 和 `map -> odom -> base_footprint` 都已经存在，比较不容易遇到 TF 空缓存。

终端 1，启动仿真：

```bash
ros2 launch robot_bringup full.launch.py \
  world:=aws_warehouse.sdf \
  headless:=false \
  nav2:=false \
  super_lio:=false \
  moveit:=false \
  nav2_rviz:=false \
  moveit_rviz:=false
```

终端 2，启动 Super-LIO 先验地图定位：

```bash
ros2 launch super_lio sim.py \
  use_sim_time:=true \
  rviz:=false \
  use_prior_map:=true \
  map_file:=/home/sw/AdaGraph-LIO/src/super_lio/map/aws_warehouse/aws_warehouse.pcd
```

终端 3，启动 Nav2：

```bash
ros2 launch robot_nav2_bringup nav2.launch.py use_sim_time:=true
```

需要 RViz 时：

```bash
rviz2 -d /home/sw/robot-sim/install/robot_nav2_bringup/share/robot_nav2_bringup/rviz/nav2.rviz
```

此时 TF 应为：

```text
map -> odom -> base_footprint
```

`/map` 栅格地图由 Super-LIO 的 occupancy grid 发布，Nav2 使用它做全局地图。

## 10. 栅格地图参数

`sim.yaml` 中当前启用了 occupancy grid：

```yaml
lio.occupancy_grid.enable: true
lio.occupancy_grid.topic: "/map"
lio.occupancy_grid.frame_id: "map"
lio.occupancy_grid.resolution: 0.05
lio.occupancy_grid.publish_period: 1.0
```

地面分割相关参数在同一段配置下面：

```yaml
lio.occupancy_grid.ground_filter.enable: true/false
lio.occupancy_grid.ground_filter.auto_dist_to_ground: true
lio.occupancy_grid.ground_filter.dist_to_ground: 0.0
```

当 `auto_dist_to_ground: true` 时，`dist_to_ground` 只是默认值/兜底值，实际地面高度会从局部点云中估计。

## 11. 常用检查命令

检查点云 frame：

```bash
ros2 topic echo /mid360/points --once --field header
```

期望：

```text
frame_id: mid360_rgl_frame
```

检查 RGL 相对 base：

```bash
ros2 run tf2_ros tf2_echo base_link mid360_rgl_frame
```

期望大致：

```text
translation: x=0.143, y=0.0, z=0.735
rotation: roll=180 deg
```

检查 IMU 频率：

```bash
ros2 topic hz /imu
```

检查雷达频率：

```bash
ros2 topic hz /mid360/points
```

检查 LIO odom：

```bash
ros2 topic hz /lio/odom
ros2 topic echo /lio/odom --once
```

检查 TF：

```bash
ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo map odom
```

检查地图：

```bash
ros2 topic echo /map --once --field header
ros2 topic hz /map
```

检查 Super-LIO 参数：

```bash
ros2 param get /super_lio_node lio.prior_map_localization.enable
ros2 param get /super_lio_node lio.prior_map_localization.map_file
ros2 param get /super_lio_node lio.kiss_matcher.enable
ros2 param get /super_lio_node lio.map.save_map
```

## 12. 常见问题

### `/lio/cloud_world` 为什么不是 `world` frame？

原始 Super-LIO 代码把自己的本地世界系叫 `world`。接入 Nav2 后，这个本地世界系语义上就是 `odom`。

当前逻辑：

- `lio.nav2_tf.enable: true` 时，对外发布 header 使用 `odom`。
- `lio.nav2_tf.enable: false` 时，保留原始 `world`。

### 为什么 `full.launch.py` 加载不了先验地图？

当前 `/home/sw/robot-sim/src/robot_bringup/launch/full.launch.py` 里面把 Super-LIO 的 `use_prior_map` 写死成了 `false`。

所以先验地图定位推荐单独启动：

```bash
ros2 launch super_lio sim.py use_prior_map:=true map_file:=...
```

或者后续把 `full.launch.py` 扩展出顶层参数：

```text
use_prior_map
map_file
```

### 为什么保存地图要正常 Ctrl+C？

Super-LIO 的最终 PCD 合并和保存发生在节点退出流程里。如果直接 `kill -9`，可能只留下分片，最终地图不会生成。

### 点云在 base 下是对的，但 odom 下反了怎么办？

先检查三件事：

```bash
ros2 topic echo /mid360/points --once --field header
ros2 run tf2_ros tf2_echo base_link mid360_rgl_frame
ros2 topic echo /imu --once
```

当前正确语义是：

- IMU 正常，不做 acc/gyro 反号。
- 雷达相对 base 绕 X 轴 180 度。
- LIO 用 `lidar_imu = RotX(pi)` 把雷达点转到 IMU。

不要同时翻 IMU 和雷达，否则 yaw/转向很容易反。

### Nav2 报 TF jump back in time 怎么办？

常见原因是旧仿真进程、旧 `/clock` 或多个 Gazebo 同时在跑。

先清进程：

```bash
pkill -9 -f "ros2 launch robot_bringup|gz-sim|gz sim|super_lio_node|robot_state_publisher|parameter_bridge|ros_gz_bridge|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|lifecycle_manager"
ros2 daemon stop
sleep 1
ros2 daemon start
```

然后重新 source 并启动。

## 13. 推荐完整流程

第一次建图：

```text
1. 清理旧进程
2. 启动 Gazebo world，super_lio:=false，nav2:=false
3. 单独 ros2 run super_lio_node，打开 lio.map.save_map
4. teleop 跑完整环境
5. Ctrl+C 正常关闭 super_lio_node，生成 PCD
```

第二次定位导航：

```text
1. 清理旧进程
2. 启动 Gazebo，super_lio:=false，nav2:=false
3. 单独启动 Super-LIO 先验地图定位
4. 启动 Nav2
5. 检查 map -> odom -> base_footprint
6. 在 RViz 里给 Nav2 goal
```
