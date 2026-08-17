# AdaGraph-LIO large_loop 回环问题交接

更新时间：2026-08-17（Asia/Shanghai）

## run22 最新结论（覆盖下文旧的“下一步”判断）

已定位并修复真正的后端权重根因：`OdometryCovarianceEstimator` 过去直接使用未标定、强相关的逐点 Hessian 逆矩阵，可能把连续里程计因子压到约 `0.0001 m / 0.00003 rad` 的虚假精度。固定真实轨迹重放证明，这会令四条正确回环残留约 `0.68--0.89 m`；使用标定下限 `0.035 m / 0.012 rad` 时残差仅约 3 cm。

修复后 Hessian 只能放大弱方向不确定度，不能令协方差比标定规则更紧；同时 iSAM2 会在后续每次图更新后刷新已接受回环的残差统计。

完整实验：

```text
log/large_loop_cartographer_sequence_20260817_run22_odom_covariance_floor
```

run22 接受三条 GT 可解释回环，最终最大因子残差：

```text
translation 0.0125161 m
rotation    0.00121325 rad
```

三条独立残差为 `0.0105 / 0.0125 / 0.0035 m`。最终指标：

```text
共享走廊 PCD overlap             92.85%
PCD NN 中位距离                  0.0706 m
PCD 匹配点 |dZ| 中位数           0.0236 m
轨迹回访 |dZ| 中位数             0.0141 m
APE RMSE                         11.1271 m
GT 对齐后最终点误差              1.5323 m
```

run20 对应的 NN 中位距离为 `0.1637 m`、PCD `|dZ|` 中位数为 `0.0722 m`、最大因子残差为 `0.555 m`，因此 run22 已显著改善局部双轮廓。22/22 测试目标、166 个测试全部通过。

第四个候选 `4444->244` 仍因 FastVGICP `hasConverged=false` 被严格拒绝，没有绕过验收；前三条已经足以把最终重访区压紧。当前不应再调整 Huber、放宽配准或恢复兜底。下一步若继续，应处理 BEV/VGICP 的运行非确定性和子图内部剩余形变，而不是后端信息权重。

## 给新会话的直接指令

接手 `/home/sw/AdaGraph-LIO` 的 UrbanV2X `large_loop` 3D LiDAR SLAM 回环问题，直接在当前工作区继续实现、编译、测试和实验，不要停留在计划或泛泛分析。

必须保留当前脏工作区。不要执行 `git reset`、`git checkout --`、批量删除或回滚其他人的改动。当前根仓库状态：

- 分支：`feature/point-lio-frontend`
- HEAD：`8c082df`
- `git status --short` 约 407 项；其中 `src/hybrid_slam/` 当前整体显示为未跟踪。
- 这些改动并不全属于本次回环任务，不要试图清理。

不要杀用户已有的 BEVPlace2 sidecar。当前 sidecar：

```text
PID 317024
/home/sw/Software/anaconda3/envs/mibot/bin/python
  /home/sw/AdaGraph-LIO/scripts/bevplace2_descriptor_service.py
  --socket /tmp/adagraph_bevplace2.sock
  --source /run/media/sw/仓库/BEVPlace2
  --checkpoint /run/media/sw/仓库/BEVPlace2/runs/Aug08_10-17-29/model_best.pth.tar
```

目前没有遗留的 `large_loop`、`ros2 bag play` 或 `hybrid_slam_point_lio` 实验进程。

## 原始目标与判定标准

系统前端为 Point-LIO，后端为 iSAM2 + GTSAM Pose3，候选检索主要使用 BEVPlace2。问题是开回环后的地图仍可能出现重影、Z 分层和“双球”。

必须区分：

1. 外观检索找到候选；
2. 子图 3D 几何验证成功；
3. 回环 Pose3 因子进入 iSAM2；
4. iSAM2 实际满足回环约束；
5. 最终 PCD 在重访区域视觉和定量上收敛。

不能只看 APE。只有回环边可由 GT/几何解释，且最终 PCD 的双层与重影明显改善，才可以说回环有效。

UrbanV2X 数据：

```text
BAG_DIR=/run/media/sw/仓库/urbanv2x/large_loop/large_loop_ros2_mcap
GROUND_TRUTH=/run/media/sw/仓库/urbanv2x/large_loop/gt_large_loop_aligned.tum
BAG_TOPICS='/velodyne_points /imu/data'
PARAMS_FILE=/home/sw/AdaGraph-LIO/src/hybrid_slam/config/point_lio_urbanv2x.yaml
```

正确 IMU 话题是 `/imu/data`，不是 `/handsfree/imu`。

## 已完成的核心设计

### 坐标链

回环测量统一定义为：

```text
T_match_query = inverse(T_map_match) * T_map_query
p_match = T_match_query * p_query
```

配准 source 是 query 侧子图（query base 表达），target 是 match 侧子图（match base 表达）。FastVGICP 输出 `T_match_query`，后端插入：

```text
BetweenFactor(match, query, T_match_query)
```

子图锚点重表达使用：

```text
T_base_submap = inverse(T_odom_base(endpoint)) * T_odom_base(anchor)
```

合成 6DoF 测试已验证 Z、roll、pitch 和变换方向。已知变换：

```text
XYZ expected  [4, -3, 1.8]
XYZ recovered [3.999827, -3.000032, 1.800029]
RPY expected  [0.08, -0.06, 0.35]
RPY recovered [0.080000, -0.059998, 0.350003]
```

因此此前约 8–10 m 的 Z 修正是累计高度漂移，不是矩阵方向写反。

### Cartographer 风格滚动子图

当前主要参数在：

- `src/hybrid_slam/config/point_lio_urbanv2x.yaml`
- `src/hybrid_slam/src/lio/pcl_loop_closure.cpp`
- `src/hybrid_slam/src/backend/loop_closure_scheduler.cpp`
- `src/hybrid_slam/src/lio/bevplace2_loop_detector.cpp`

当前方案：

- 每 35 个关键帧完成一个基础子图；
- 相邻两个基础子图组成约 70 帧滚动支持面，步长 35，约 50% 重叠；
- 在固定 odom 锚点局部坐标中融合；
- 使用体素质心，不再达到点数上限后截断时间序列尾部；
- query/match 两侧分别裁剪 40 m；
- BEVPlace2 仅负责候选、序列一致性和 XY/yaw 粗初值；
- 最终只有 `joint_submaps` 的严格 3D 验证可以产生回环边；
- raw keyframe 和 `submap_pair` 最终兜底均已移除；
- 多初值包括 graph、BEV RANSAC 和 ±8 m uniqueness probes；
- 使用粗 VGICP + 细 FastVGICP；
- 同时检查双向 overlap、互近邻、双向 RMSE、空间范围、Z 范围、协方差特征值、空间体素、多初值一致性、图预测修正、相对 Z/roll/pitch；
- 失败直接拒绝，不能靠放宽 fitness 或忽略 `hasConverged` 接受。

相关当前参数：

```text
lio.backend.submap_max_keyframes: 35
lio.loop_closure.registration_crop_radius_m: 40.0
lio.loop_closure.coarse_registration_voxel_size_m: 4.0
lio.loop_closure.maximum_bidirectional_inlier_rmse_m: 0.75
lio.loop_closure.minimum_support_extent_m: 15.0
lio.loop_closure.minimum_support_z_extent_m: 2.0
lio.loop_closure.minimum_support_voxels: 20
lio.loop_closure.minimum_pose_free_sequence_consensus: 3
lio.loop_closure.maximum_pose_free_graph_correction_m: 3.0
lio.loop_closure.maximum_pose_free_graph_correction_rad: 0.15
lio.loop_closure.translation_sigma_m: 0.25
lio.loop_closure.rotation_sigma_rad: 0.04
```

注意 YAML 中静态默认仍可能写着 `lio.loop_closure.enable: false` 和 `plugin: scan_context`；完整实验脚本会覆盖为本次需要的 BEVPlace2 回环设置。不要仅凭 YAML 这两行判断实验未开回环。

### 后端位姿协方差

此前把约 0.55 m 的逐点配准 RMSE 直接当成整个位姿标准差，导致数千个几何对应点估计出的真回环因子过松。现已改为根据空间支持体素估计有效独立观测数，同时保留系统误差下限：

```text
translation sigma floor = 0.25 m
rotation sigma floor = 0.04 rad
```

日志会输出：

```text
covariance_sigma=...
point_rmse=...
support_voxels=...
pose_standard_error=...
```

相关代码：

- `src/hybrid_slam/src/lio/pcl_loop_closure.cpp`
- `src/hybrid_slam/test/test_isam2_optimizer.cpp`

新增的长里程计链回归测试模拟 4200 条里程计累计约 10 m 高度漂移，要求严格验证回环加入后残差 `< 0.30 m`。修改后全量测试通过。

### 回环专用 Huber 阈值

严格 3D 验证后的真回环，加入时可能距离漂移图 8–10 m。普通 Huber `delta=1.345` 会长期把剩余 2–3 sigma 修正降权。

已加入：

```text
lio.backend.loop_closure_huber_delta: 3.0
```

相关代码：

- `src/hybrid_slam/include/hybrid_slam/backend/backend_config.h`
- `src/hybrid_slam/src/backend/isam2_optimizer.cpp`
- `src/hybrid_slam/src/ros/parameter_bridge.cpp`
- `src/hybrid_slam/config/point_lio_urbanv2x.yaml`

普通相对因子仍使用 1.345；仅 `ConstraintType::kLoopClosure` 使用配置的 3.0。仍保留鲁棒核，没有改成普通高斯。

## 测试状态

最近一次构建和测试：

```text
22/22 tests passed
```

典型命令：

```bash
source /opt/ros/lyrical/setup.bash
cd /home/sw/AdaGraph-LIO
colcon build --packages-select hybrid_slam --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source /home/sw/AdaGraph-LIO/install/setup.bash
colcon test --packages-select hybrid_slam \
  --event-handlers console_cohesion+ --return-code-on-test-failure
colcon test-result --test-result-base build/hybrid_slam --verbose
```

## 可信实验结果

### loop-off 基线

目录：

```text
log/large_loop_cartographer_sequence_20260817_run12_loop_off_endpoint_pcd
```

指标：

```text
APE RMSE                         12.0817 m
GT 对齐后最终点误差              4.5025 m
首尾轨迹 XY 中位差               7.2696 m
首尾轨迹 |dZ| 中位数             7.9139 m
共享走廊 PCD 双向 overlap         9.853%
尾段到首段 PCD NN 中位距离        4.8512 m
```

### run19：四条真回环，旧位姿级权重

目录：

```text
log/large_loop_cartographer_sequence_20260817_run19_backend_refined
```

四条回环均可由 GT 解释，最终 PCD 已明显收敛：

```text
PCD 双向 overlap                89.93%
PCD NN 中位距离                 0.304 m
PCD 匹配点 |dZ| 中位数          0.129 m
最大后端回环平移残差             0.746 m
```

球体等清晰边缘仍可见双轮廓，因此继续查后端权重。

### run20：当前最可信、最佳完整结果

目录：

```text
log/large_loop_cartographer_sequence_20260817_run20_pose_covariance
```

接受四条回环：

```text
4304 -> 104
  measurement t=[1.45268, 0.72889, 0.0310943]
  rpy=[-0.0192213, -0.0332754, 0.88561]
  GT translation error=0.4486 m
  GT rotation error=0.03797 rad

4339 -> 139
  measurement t=[0.135579, -6.53708, 0.0330462]
  rpy=[-0.00615399, -0.0105892, -0.00890057]
  GT translation error=0.1517 m
  GT rotation error=0.00588 rad

4374 -> 174
  measurement t=[0.509354, -6.62893, 0.0179513]
  rpy=[0.0107256, -0.000612914, -0.175658]
  GT translation error=0.3436 m
  GT rotation error=0.01157 rad

4444 -> 244
  measurement t=[-3.0247, -9.75579, -0.0955032]
  rpy=[0.00569911, -0.0216941, 0.0434836]
  GT translation error=0.1237 m
  GT rotation error=0.00607 rad
```

优化后四条回环平移残差约为：

```text
0.290 m, 0.164 m, 0.127 m, 0.555 m
```

整体最大值由 run19 的 0.746 m 降到 0.555 m，说明位姿协方差修正方向正确，但第四条真边仍被 Huber 降权。

run20 最终指标：

```text
APE RMSE                         11.0376 m
GT 对齐后最终点误差              1.2545 m
首尾轨迹 XY 中位差               1.0238 m
首尾轨迹 |dZ| 中位数             0.0963 m
共享走廊 PCD 双向 overlap         93.164%
尾段到首段 PCD NN 中位距离        0.1637 m
PCD 匹配点 |dZ| 中位数           0.0722 m
```

这是目前最好的、可解释的完整结果。最终 PCD：

```text
log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/map/large_loop_cartographer_rolling_final.pcd
SHA256 07eead910a06f02dbdd2a462d8d42a28440dab3f00c37db36b9cb648409e7fd8
```

关键产物：

- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/shared_corridor_comparison.json`
- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/shared_corridor_comparison.png`
- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/local_vertical_windows.png`
- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/loop_pose_vs_gt.json`
- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/backend_comparison.json`
- `log/large_loop_cartographer_sequence_20260817_run20_pose_covariance/point_lio.log`

### run21：不能用于验证 Huber 3.0

目录：

```text
log/large_loop_cartographer_sequence_20260817_run21_loop_huber3
```

原会话在 run21 回放进行到约 266/601 秒时因 Codex 响应流断开。实验进程随后自行完整退出，最终 PCD 和轨迹均保存成功；不是 SLAM 进程崩溃。

但 run21 仅接受一条回环：

```text
4304 -> 104
measurement t=[0.518227, 1.47696, 0.00912866]
rpy=[-0.0182872, -0.0403385, 0.871794]
GT translation error=0.4093 m
GT rotation error=0.03695 rad
optimized residual=0.112683 m / 0.0287864 rad
```

其他三条没有复现：

- `4409 -> 209` 的 graph 初始化最终 `registration_failed`；
- `4444 -> 244` 的 BEV pose 只有 `sequence_consensus=2`，而 pose-free 路径要求 3；
- 其余连续候选也受实时调度/前端关键帧非确定性影响，没有形成与 run20 相同的四条约束。

因此 run21 不是“Huber 3.0 失败”的证据，也不是有效的 run20 对照。它根本没有对同一组四条回环测量进行优化。

run21 指标：

```text
accepted loops                    1
APE RMSE                          11.1514 m
GT 对齐后最终点误差               4.7629 m
首尾轨迹 |dZ| 中位数              0.8950 m
共享走廊 PCD 双向 overlap          70.746%
尾段到首段 PCD NN 中位距离         0.4390 m
PCD 匹配点 |dZ| 中位数            0.1383 m
```

最终 PCD：

```text
log/large_loop_cartographer_sequence_20260817_run21_loop_huber3/map/large_loop_cartographer_rolling_final.pcd
SHA256 701f8666cffa20086b4f7659cfcb3c167b798ed51d5d648f67769bba6a784c9e
```

## 下一步：必须先做受控后端对照

不要立即再跑一次完整 bag 来“赌”四条边复现。当前最大干扰是 BEV/关键帧/调度非确定性，而不是缺少更多完整回放。

优先实现一个后端固定约束重放测试或离线工具：

1. 固定 run20 的 odometry/node 轨迹与上面四条回环测量；
2. 使用完全相同的四条 Pose3 约束、协方差和初始化；
3. 只切换 `loop_closure_huber_delta=1.345` 与 `3.0`；
4. 比较每条因子的优化后平移/旋转残差、总图误差、节点位姿差；
5. 目标是确认第四条 `4444->244` 的 0.555 m 是否能压到约 0.2–0.3 m，同时不让其他三条或里程计链异常；
6. 把这个结果固化为回归测试，再决定是否保留 Huber 3.0；
7. 只有受控测试成立后，再跑一次完整 large_loop 验证最终 PCD 和“双球”。

如果无法直接从运行时导出完整 factor graph，可以先在测试中按真实节点数构造与 `test_isam2_optimizer.cpp` 相同的长链，并插入 run20 的四条固定回环。更好的方案是给 backend recorder 增加约束序列导出/重放，保证完整实验可重复。

不要通过以下方式掩盖问题：

- 降低 BEV minimum pose inliers；
- 降低 pose-free sequence consensus；
- 放宽 VGICP fitness/correspondence distance；
- 忽略 `hasConverged`；
- 恢复 raw keyframe 或 `submap_pair` 兜底；
- 仅凭单条回环残差或 APE 宣称成功。

## 常用分析命令/脚本

完整实验脚本：

```bash
cd /home/sw/AdaGraph-LIO
setsid /home/sw/AdaGraph-LIO/scripts/run_large_loop_experiment.sh \
  /home/sw/AdaGraph-LIO/log/large_loop_cartographer_sequence_YYYYMMDD_runXX_name \
  > /tmp/adagraph_runXX_launcher.log 2>&1 &
```

分析脚本：

- `scripts/compare_loop_pose_to_gt.py`
- `scripts/analyze_backend_loop_trajectory.py`
- `scripts/compare_endpoint_loop_maps.py`
- `scripts/plot_endpoint_local_windows.py`
- `scripts/analyze_registration_debug.py`

检查回环和后端最终状态：

```bash
rg -n '\[LoopClosure\]: accepted|loop factor residuals|ERROR|FATAL' \
  log/<run>/point_lio.log
```

检查实验是否完整保存：

```bash
rg -n 'MapSaveScheduler|submitted/processed/failed' log/<run>/point_lio.log
find log/<run>/map -maxdepth 1 -type f -printf '%s %p\n'
```

## 原会话记录

完整 Codex JSONL（约 51.2 MB、6288 条记录）：

```text
/home/sw/.codex/sessions/2026/08/17/rollout-2026-08-17T12-50-50-01a00e0e-df6a-7863-b8cf-8352dca8ad90.jsonl
```

该会话中间多次发生 `stream disconnected before completion`。不要把这些 Codex 连接错误误判成构建、ROS 或 SLAM 进程失败；以实验目录里的日志和最终文件为准。

## 当前结论

子图构建和 3D 几何验证已经从“明显错误/不可靠”改善到可用：run20 的四条边可由 GT 解释，最终共享走廊 PCD overlap 达 93.16%，Z 分层和大范围重影显著收敛。

尚未完全解决的是严格真回环在 iSAM2 中的最终执行残差，以及由此在球体、护栏等清晰局部结构上留下的约 0.1–0.5 m 双轮廓。当前最需要的是固定四条回环的受控后端实验，而不是继续调前端阈值或重复非确定性完整回放。
