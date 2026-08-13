# Point-LIO 前端版本

本版本在独立分支 `feature/point-lio-frontend` 上把局部里程计前端替换为
[Small Point-LIO](https://github.com/Yancey2023/small_point_lio)，同时保留原来的
Super-LIO 可执行程序和启动文件。使用的上游基线是 ROS 2 分支 commit
`b87754309d5a8603dfb8ec7631bf7665dc067b21`。

## 保留与替换的边界

替换内容：

- IMU/点云时序处理、Point-LIO ESKF、逐点点到平面更新和局部体素地图；
- 前端发布频率和 Point-LIO 滤波参数。

保留内容：

- AdaGraph 自适应协方差和 GTSAM/iSAM2 后端；
- KISS-Matcher 初始化、FastVGICP 先验图跟踪和 `map -> odom` 修正；
- `/lio/odom`、`/lio/path`、`/lio/cloud_world`、`/map` 等输出；
- `map -> odom -> base_footprint -> base_link -> livox_frame` TF 约定；
- 地面分割、占据栅格、Nav2 输出和退出时 PCD 地图保存。

Point-LIO 与 AdaGraph 在同一进程内运行。适配层直接读取被 Point-LIO 接受的
点到平面 Jacobian 和残差，按 AdaGraph 的 `[rotation, translation]` 顺序形成
6x6 Hessian；没有通过 ROS 里程计消息反推或使用固定协方差。逐点生成的注册点
会按原始 LiDAR 消息边界聚合后再提交给 AdaGraph、先验图定位和占据栅格。

为避免完整系统反压前端，传感器回调只执行 Point-LIO 滤波、发布里程计并把帧
快照以移动语义放入有界队列。AdaGraph、先验图定位、点云发布、占据栅格和地图
累计由独立串行 worker 消费。只有实际启用对应功能或 AdaGraph 选中关键帧时才
构造 PCL/机体系点云。队列过载时保留最新帧并统计丢弃数，不阻塞滤波器；退出时
会排空队列，并打印输入消息数、入队/处理/丢弃帧数、最大队列深度和平均下游耗时。

## 编译

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
./scripts/build_jetson.sh
```

Small Point-LIO 已固定版本放在
`src/super_lio/3rdparty/small_point_lio`，无需再单独 clone。新增编译依赖为
OpenMP；其余 Eigen、ROS 2、livox_ros_driver2 等依赖已属于现有工程。

## Mid-360 运行

```bash
source install/setup.bash
ros2 launch super_lio PointLIO_mid360.py
```

默认输入为：

- `/livox/lidar`：`livox_ros_driver2/msg/CustomMsg`；
- `/livox/imu`：`sensor_msgs/msg/Imu`；
- `livox_ros_driver2` 的加速度单位按驱动实际输出设置为 `g`，因此默认
  `acc_norm: 1.0`。

Point-LIO 上游支持的适配器可通过启动参数选择：

```bash
ros2 launch super_lio PointLIO_mid360.py \
  lidar_type:=livox_custom_msg
```

可选值为 `livox_custom_msg`、`livox_pointcloud2`、
`custom_mid360_driver`、`unilidar` 和 `velodyne_pointcloud2`。PointCloud2 模式
要求消息字段与对应适配器一致；普通 XYZI 点云不能直接冒充 Livox PointCloud2。

## M2DGR 数据集

仓库内 `Datasets` 目录是 M2DGR ROS 2 bag。专用配置读取：

- `/velodyne_points`：VLP-32C `x/y/z/intensity/ring/time` 点云；
- `/handsfree/imu`：Handsfree A9 IMU；
- float32 `time` 是相对消息时间戳的逐点秒偏移；
- LiDAR 到 IMU 的官方平移标定为
  `[0.27255, -0.00053, 0.17954]`，旋转为单位阵。

终端一启动节点：

```bash
source install/setup.bash
ros2 launch super_lio PointLIO_m2dgr.py
```

终端二只回放需要的两个话题：

```bash
ros2 bag play Datasets/room_01_ros2 \
  --delay 1.0 --wait-for-all-acked 5 \
  --topics /velodyne_points /handsfree/imu
```

对应参数文件是 `src/super_lio/config/point_lio_m2dgr.yaml`。GT 文本遵循 TUM
八列格式，可使用 evo 对输出轨迹做时间关联和 SE(3) 对齐。

## 先验图定位

```bash
ros2 launch super_lio PointLIO_mid360.py \
  use_prior_map:=true \
  prior_map_file:=/absolute/path/to/map.pcd
```

定位仍遵循：

```text
T_map_odom = T_map_body_registration * inverse(T_odom_body_same_timestamp)
```

FastVGICP 不会写回 Point-LIO ESKF 或其 odom-frame 体素地图。手动全局重定位服务
仍为 `/lio/relocalization/trigger`。

## 保存地图

```bash
ros2 launch super_lio PointLIO_mid360.py \
  save_map:=true \
  save_map_dir:=/absolute/path/to/output \
  map_name:=point_lio_map.pcd
```

用 Ctrl-C 正常结束后，节点将累计的 odom-frame 注册点云按
`lio.map.ds_size` 降采样并保存。为避免路径误配置，运行时只接受解析后的绝对输出
目录。

## 主要参数

Point-LIO 参数位于 `src/super_lio/config/point_lio_mid360.yaml` 顶部，包括：

- `point_filter_num`、`min_distance`、`max_distance`；
- `space_downsample`、`space_downsample_leaf_size`；
- `map_resolution`、`init_map_size`；
- `extrinsic_T`、`extrinsic_R`、`extrinsic_est_en`；
- `laser_point_cov`、IMU 测量噪声和过程噪声；
- `plane_threshold`、上游原名 `match_sqaured`。

接入层运行参数：

- `point_lio.input.reliable`：离线 bag 使用可靠传输，实时 Livox 默认 best-effort；
- `point_lio.input.lidar_queue_depth`、`point_lio.input.imu_queue_depth`：DDS 输入深度；
- `point_lio.backend.queue_capacity`：完整下游 worker 的有界帧队列容量。

同一文件内的 `lio.*` 参数继续控制 AdaGraph、先验图定位、TF、栅格和输出。
`lio.sensor.*` 中仅共享的机器人安装几何和输出相关参数会影响本版本，Super-LIO
自身的 ESKF/OctVox 参数不会参与 Point-LIO 局部里程计。

## 当前验证范围

- ROS 2 Lyrical 下完整编译通过；
- 启动文件可在无传感器数据时正常初始化和退出；
- 使用合成 Livox CustomMsg + IMU 数据完成 Point-LIO 初始化，验证
  `/lio/odom`、`/lio/cloud_world` 和 `/map` 均产生输出；
- M2DGR `room_01` 以原 0.3 m 参数做两次 1x 回放：两次均完整收到
  `728` 帧 LiDAR 和 `10057` 条 IMU、处理 `714` 个下游帧且零丢弃，输出的
  `713` 行 TUM 文件 SHA-256 完全相同；平移 APE RMSE 为 `0.397958 m`；
- 5x 完整下游压测（世界点云、逐帧占据栅格和 AdaGraph 同时开启）：完整收到
  `728/10057` 条输入并处理 `714` 帧，后台零丢弃、最大队列深度 `1`、平均下游
  耗时 `3.57 ms`，AdaGraph 生成 `101` 个关键帧；APE RMSE 为 `0.398283 m`，
  相对 1x 轨迹的未对齐平移 RMSE 仅 `0.000356 m`；
- M2DGR 专用体素参数进一步收敛到 0.2 m：最终 1x 回放继续完整收到
  `728/10057` 条输入、处理 `714` 帧且零丢弃，平移 APE RMSE 为
  `0.396434 m`，轨迹长度从 0.3 m 参数的 `28.063 m` 降到 `27.538 m`
  （GT `27.171 m`）；5x APE RMSE 为 `0.394166 m`，相对最终 1x 轨迹的
  未对齐平移 RMSE 为 `0.001107 m`。0.15 m 在 5x 下无法及时清空逐点队列，
  未采用；
- 真实 Mid-360 轨迹精度、长时间内存和先验图定位质量仍需在机器人上标定验证。
