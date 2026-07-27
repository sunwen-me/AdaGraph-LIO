# Super-LIO 论文数据集复现实验

日期：2026-07-27

## 实验口径

- 数据集：M2DGR `door_01`、`door_02`，对应论文表 I 的 `m2d1`、`m2d2`。
- 配置：`M2DGR.yaml`，IESKF 最大迭代 4 次、点抽样率 3、扫描和地图体素
  0.5 m。
- 回放速度：5 倍，与论文 X86 精度实验一致。
- 指标：`evo_ape tum ... -a --t_max_diff 0.1`，SE(3) Umeyama
  对齐后的平移 APE RMSE。
- 本机：Intel Core i5-12490F，6 核 12 线程，Ubuntu 26.04，
  ROS 2 Lyrical。
- 代码状态：`fix/ultra-review`，HEAD `4c6e67c`，包含尚未提交的人工
  复核修复。

论文结果来源：
[Super-LIO arXiv Table I](https://arxiv.org/html/2509.05723#S4.SS2)。

## 精度结果

| 序列 | 估计位姿 | 匹配位姿 | 本次 RMSE | 论文 RMSE | 差值 |
|---|---:|---:|---:|---:|---:|
| `m2d1` / `door_01` | 2521 | 1541 | 0.454238 m | 0.456 m | -0.001762 m |
| `m2d2` / `door_02` | 612 | 585 | 0.342372 m | 0.321 m | +0.021372 m |

`m2d2` 首次独立运行得到 0.345651 m，第二次为 0.342372 m，
两次相差约 3 mm，没有出现发散。

## 修复前后对比

修复前版本使用隔离 worktree 中的 `main/develop` 共同基线
`9c46e59`，没有切换或修改当前修复分支。数据、配置的有效参数、
回放速度和 evo 命令均相同。

| 序列 | 修复前 RMSE | 修复后 RMSE | 改善 | 论文 RMSE |
|---|---:|---:|---:|---:|
| `m2d1` | 0.463061 m | 0.454238 m | 1.91% | 0.456 m |
| `m2d2` | 0.346973 m | 0.342372 m | 1.33% | 0.321 m |

两条序列中，修复后精度均小幅提高，没有为了修复系统接口而牺牲
LIO 前端轨迹精度。

## 核心阶段耗时

以下是代码内部 `Timer` 覆盖的四个核心阶段，不包含消息同步、AdaGraph
关键帧处理和 ROS 发布，因此不能直接等同于论文表 II 的端到端耗时。

| 序列 | 去畸变 | 降采样 | 状态更新 | 地图更新 | 四阶段合计 |
|---|---:|---:|---:|---:|---:|
| `m2d1` | 0.248148 ms | 0.406768 ms | 1.211970 ms | 0.140735 ms | 2.007621 ms |
| `m2d2` | 0.235080 ms | 0.415652 ms | 1.311560 ms | 0.150880 ms | 2.113172 ms |

修复前四阶段合计分别为 2.000909 ms 和 2.112618 ms，变化均小于
0.4%。5 倍回放中，修复前/后输出位姿数分别为 2711/2521 (`m2d1`)
和 785/612 (`m2d2`)。由于 `m2d2` 两次修复后运行本身也有 612/664
的调度波动，不能只凭该计数认定确定性性能回归；仍应使用固定 CPU
频率、缓存状态和多次重复实验，测量 Timer 未覆盖的同步、AdaGraph
和 ROS 发布开销。

论文在 AMD 5800H 上报告 M2DGR door 组端到端平均耗时 2.89 ms。

## 复现命令

```bash
cd /home/sw/AdaGraph-LIO
source /opt/ros/lyrical/setup.bash
source install/setup.bash

./scripts/run_m2dgr_eval.sh door_01 5.0 log/paper_eval/m2d1
evo_ape tum Datasets/door_01_ros2/door_01.txt \
  log/paper_eval/m2d1/estimated.tum \
  -a --t_max_diff 0.1 \
  --save_results log/paper_eval/m2d1/ape.zip --no_warnings

./scripts/run_m2dgr_eval.sh door_02 5.0 log/paper_eval/m2d2
evo_ape tum Datasets/door_02_ros2/door_02.txt \
  log/paper_eval/m2d2/estimated.tum \
  -a --t_max_diff 0.1 \
  --save_results log/paper_eval/m2d2/ape.zip --no_warnings
```

每个输出目录包含：

- `estimated.tum`：Super-LIO 估计轨迹。
- `ape.zip`：evo 误差数组、统计和对齐变换。
- `ape_plot_map.png`、`ape_plot_raw.png`：轨迹误差图和随时间变化的
  原始 APE 曲线。
- `super_lio.log`：初始化、AdaGraph 和阶段耗时日志。
- `bag_play.log`、`recorder.log`：数据回放和轨迹记录日志。
- `wall_time_seconds.txt`：数据回放墙钟时间。

两次最终运行均完成 KF/地图初始化，日志中未发现
`ERROR`、`FATAL`、异常、NaN 或 IMU 静止门拒绝。

修复前结果保存在 `log/paper_eval/original/m2d1` 和
`log/paper_eval/original/m2d2`。原版两次运行也没有算法错误，但均有
`Logging before InitGoogleLogging()` 警告，该问题已在修复分支消除。
