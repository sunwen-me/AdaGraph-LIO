# Ultra-Review 修复报告

分支:`fix/ultra-review`(基线存档标签:`archive/pre-ultra-fix-20260726`)。日期:2026-07-27。
本报告面向项目作者与答辩评审,内容以各工作包决策日志与验收记录为准,可逐条对照代码核查。
**本轮全部改动仍在工作区,未 commit。**

---

## 1. 概述

### 1.1 审查来源

本轮修复的输入是一次 **44 智能体多维审查 + 对抗验证**:

- 按维度并行审查:前端/ESKF、AdaGraph 后端、数学基础库(basic)、先验地图定位、
  导航输出(占据栅格/地面分割)、数据结构(OctVoxMap)、ROS 集成、构建/配置/文档,另有跨维度 critic;
- 每条发现均经独立对抗验证者确认或修正(claim + correction),仅确认后的条目进入修复工单;
- 修复分 8 个并行工作包(Wave 1)+ 1 个特性工作包(Wave 2)+ 编译验证 + 单元测试补充 +
  一轮验收返工(fixup)执行,各工作包互不越界(OWNED FILES 制),接口冻结经编译验证。

### 1.2 修复规模

`git diff --stat archive/pre-ultra-fix-20260726`:

- **53 个文件改动,+2335 / −1047 行**;
- 另有未跟踪新增:`doc/LICENSES.md`、4 个 gtest 测试文件(共 883 行,
  `src/basic/test/`、`src/super_lio/test/`);
- 工单条目共约 90 条(backend 12、basic 11、config 12、frontend 15、localization 11、
  nav 9、octvox 6、ros 17),验收全部通过(3 条 partial、1 条 skipped_ok);
  验收发现的回归/联动问题经 fixup 轮 11 项(A–K)处置;
- 新增可调参数 21 个(见 §3),其中 2 个为实验特性旗标,默认全部保持修复前行为
  (除 §5 明确列出的默认行为变化)。

---

## 2. 发现 → 修复映射表

处置标记:fixed = 已修;partial = 部分修复(残余已记录);skip = 未修(附理由)。
行号为修复前基线(`archive/pre-ultra-fix-20260726`)位置。

### 2.1 WP-basic(数学基础库)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | Gamma_2 第三项丢 K·K 因子(Manifold.cpp:79) | major | fixed | 照闭式修正,补小角一阶项 SKEW/6;随 #9 移入 Manifold.h;级数对照误差 <1e-12 |
| 2 | SE3::update() 不刷新 R_/t_ 缓存(Manifold.cpp:453) | major | fixed | 与 updateRhs() 一致同步刷新;grep 确认仓内无调用者依赖旧行为 |
| 3 | SE3 MatrixBase 模板构造不初始化 R_/t_(Manifold.h:154) | major | fixed | 构造体内从 T_ 提取;保持非 explicit(接口冻结) |
| 4 | glog 从未 InitGoogleLogging(logs.cpp) | major | fixed | logs.cpp 增幂等 `ensure_glog_initialized()`(static bool + IsGoogleLoggingInitialized);init_log/init_log_dir 完整设置输出模式并自建目录;签名不变 |
| 5 | SO3::adjoint() 返回 R^T(Manifold.cpp:348) | — | fixed | 改返回 R;grep 全仓零调用者;测试验证 hat(Rw)=Ad·hat(w)·Adᵀ |
| 6 | 免费函数 Log(R) 近 π 数值错误(Manifold.cpp:125) | — | fixed | 重写为四元数路径(与 SO3::log 同算法),θ∈[1e-9,π] 往返误差 <1e-7 |
| 7 | SO3::coeffs() 四元数不归一化(Manifold.cpp:287) | — | fixed | 返回前 normalized();下游 ROSWrapper/ds.h 只受益 |
| 8 | A_matrix 小角门限精度问题(Manifold.cpp:48) | — | fixed | 核实后原 claim 部分不成立(计算本在 double);改泰勒分支消除门限争议,误差 <1e-6 |
| 9 | 头文件声明的模板在 .cpp 定义、外部不可用(Manifold.h:24 系列) | — | fixed | SKEW/Exp/Log/Gamma_1/Gamma_2/RotMtoEuler/tolerance/hat6/vee6 全部移入头文件 inline;修复 T=double 不可实例化问题 |
| 10 | RingBuffer 两种 push 溢出语义矛盾 + waitAndPop 无关闭路径 | — | fixed | 统一 drop-oldest(丢 1 个);增 close()/closed_,waitAndPop 返回 bool(签名变更为工单明示例外,buffer 套件全仓零调用者,文件顶注明"目前无使用者") |
| 11 | logs.h `using namespace google` 污染包含者 | — | fixed | 删除 using-directive;颜色宏保留;全仓 grep 确认无非限定使用 |
| 附 | (测试轮发现)SE3::log() 小角度 k1 灾难性消去,θ<2e-8 时 NaN(Manifold.cpp) | major | fixed | θ<1e-2 改稳定级数 1/12+o²/720+o⁴/30240;回归测试覆盖 θ∈[1e-9,1e-3] |

未采纳:M8 宏加前缀(破坏非本包调用点)、M6 删除/接线 buffer 套件(超授权)。

### 2.2 WP-frontend(Super-LIO 前端 / ESKF)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | 后端优化位姿写回 ESKF(super_lio.cpp:628-635) | critical | fixed | 整块删除(链式图下恒等;真修正会与 odom 系体素地图失配);调用点留地图一致性约束注释 |
| 2 | 固定 20000 定长缓冲越界 + Observe() 函数局部 static 跨实例共享(super_lio.cpp:495 等) | major | fixed | effect_mask_ 等改 `std::vector<uint8_t>` 按帧 resize、容量常驻复用;static 缓冲改成员;reloc 子类兼容性已编译验证 |
| 3 | ESKF 状态未传播到扫描末端、死分支(ESKF.cpp:206) | major | fixed | 删不可达分支;新增 `PredictToObsTime()` 尾段传播(FAST-LIO 系标准做法),纯增量接口 |
| 4 | 去畸变插值 dt==0 时 0/0 NaN(super_lio.cpp:451) | — | fixed | s = clamp(tau/dt,0,1) 加零保护;Predict 失败不入队;连带补 propagate_states_ 少于 2 元素的迭代器保护 |
| 5 | 扫描驱动 Predict 缺 dt 钳制(ESKF.cpp:210) | — | fixed | dt<0 或 >0.2 s 时告警、推进时间锚点并 return false,不把空洞积分进 P |
| 6 | F_W 噪声雅可比注释 bg/ba 写反(ESKF.cpp:234) | minor | fixed | 仅交换两行注释 |
| 7 | 死代码(重复 A_matrix、17 维 STATE_DOF、g_residual_type)+ 魔数 1000 | — | fixed | 全部删除(grep 零调用);权重提取为 `kLidarPointWeight = 1000.0`(= 1/σ², σ≈0.032 m) |
| 8 | IMU 初始化无保护(super_lio.cpp:138-183) | — | fixed | mean_acce.norm()<0.1 拒绝初始化(防除零 NaN);Welford 静止性检查(gyro 方差 >0.01、加计相对方差 >4e-3 则重等);注释对齐实际 50 样本 |
| 9 | `system("rm -rf ...")` 危险调用(super_lio.cpp:283-287) | — | partial→fixed | 改 std::filesystem + 绝对路径/归一化/`<save_root>/PCD` 校验;验收发现尾斜杠误杀,fixup A 已修 |
| 10 | ProcessCaceMap 关机时全图入内存(super_lio.cpp:302-330) | — | fixed | g_if_filter 时逐 fragment 先降采样再合并(峰值内存降为降采样全图+单 fragment);顺带加固目录不存在/空云崩溃路径 |
| 11 | 占据栅格更新被可视化节流门控(super_lio.cpp:652-672) | — | fixed | 移出 pub_step 早退,独立 `g_grid_update_step`(默认每帧);参数接线由 fixup G 完成;输入云选择由 fixup E 修正为随 dense 旗标 |
| 12 | timer.h Evaluate 无界追加 | — | fixed | 改滚动统计(count/mean/max,Welford),打印格式兼容并追加 max |
| 13 | planar 地面约束调平 odom/IMU 系而非 base 系(super_lio.cpp:244) | — | fixed | 按 correction:取 base 系旋转提取 yaw 再回推;单位安装旋转下与原行为相同 |
| 14 | params 死参数删除 + ROOT 编译期路径 | — | fixed/skip | 删 10 个死全局(与 ros/config 三方按同一清单);ROOT 子项 skip(读者在非本包文件)→ fixup H 在 ROSWrapper 完成"绝对路径不拼 ROOT" |
| 15 | super_lio_node.cpp 无 glog 初始化 | — | fixed | rclcpp::init 后接线 `BASIC::init_log()` |

### 2.3 WP-backend(AdaGraph 后端 / 特征提取)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | 退化判据方向反了,Hessian 路径死代码(adagraph_backend.cpp:154) | critical | fixed | 重写 predictSE3Covariance:对累积 H 特征分解,方向级混合(λ≥`hessian_eig_min_`=1e3 用 1/λ,低于注入规则协方差投影);全退化/空 terms 才整体走规则协方差 |
| 2 | H 取平均应为求和(adagraph_backend.cpp:147) | major | fixed | 删 `H /= size()`(信息可加);当前 N=1 无行为变化 |
| 3 | 假"残差统计"(单元素梯度范数冒充分布统计) | major | fixed | fillResidualStats → fillGradientStats,只记真实 gradient_norm;删 8 个假字段 + residual_cluster_count + scan_matching_residual;删 residual_mean 膨胀规则;日志写明 Input 冻结拿不到逐点残差及将来前端需暴露的内容 |
| 4 | 散布矩阵冒充 Hessian(fillGeometryStats) | major | fixed | 从 Input.hessian_terms 求真 6×6 H 填 hessian_eig/trace/condition/effective_rank;散布特征改名 scene_scatter_*;规则阈值按真 Hessian 量级重设(注明未标定工程初值) |
| 5 | motion_distortion_score 恒 0 但规则在用 | — | fixed(删除) | 无扫描起止时间无法真算;硬编码周期只是 angular_velocity_norm 的常数倍冗余,连字段带规则删除 |
| 6 | 7 个恒 0 占位字段 | — | fixed | normal_entropy/local_map_density/voxel_occupancy/point_age_mean/eskf_iteration_count/convergence_state/prediction_error 全删,grep 无残留引用 |
| 7 | effective_point_number<20000 规则永真 | — | fixed | 阈值改 1500(0.5 m 降采样后典型数千点),注明依据与未标定性质 |
| 8 | optimized 标志不诚实(纯链式图优化不改估计) | — | fixed | 恒置 false,注释说明;有独立信息因子(Wave 2)后才置 true |
| 9 | keyframe 时间戳被丢弃 | — | fixed | 新增 keyframe_timestamps_ 与 keyframe_poses_ 平行存储 |
| 10 | calculateEstimate() 全量 O(N);keyframe_poses_ 无界 | — | fixed | 改单键 `calculateEstimate<Pose3>(key)`;keyframe_poses_ 保留并注释增长量级(数小时数万条) |
| 11 | adagraph_log.h 用 std::cout 不 include iostream | — | fixed | 全部宏改 glog(LOG(INFO)/LOG(WARNING)) |
| 12 | fillRelativePose 混系混单位(feature_extractor:18) | — | fixed | 平移改上一关键帧 body 系 R_lastᵀ·Δt;innovation_norm 删除(米+弧度直加无意义,且无规则消费) |

明确未动(记录在日志):M2 between 噪声切空间混系(纯链式图下不影响估计,Wave 2 后需处理)。

### 2.4 WP-localization(先验地图定位 / 重定位)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | 重定位后 map->odom 双重应用(super_lio_reloc.cpp:266-283) | critical | fixed | ESKF 保持 odom 系,重定位结果全部进 map->odom = T_map_imu·T_odom_imu⁻¹;ivox 灌入随之移到 kf_init 成功路径并按 T_odom_map 变换;决策日志附修复后 TF 链推导;非 nav2 配置保留旧语义 |
| 2 | 手动重定位被跳变门丢弃(prior_map_localization.cpp:480-489) | major | fixed | force_jump 在 fitness 达标时绕过 max_translation/max_rotation 门 |
| 3 | LOST 后 KISS 只试一次(:251) | major | fixed | 冷却重试:5 s × min(失败数+1, 6)(最长 30 s),每次 LOG(INFO),恢复/新 LOST 清零 |
| 4 | TRACKING 楔死(:292) | major | fixed | 连续 10 次一致的 delta 门拒绝 → LOG(WARNING) 并 enterLost() 触发 KISS |
| 5 | update_map=false 不冻结地图(super_lio_reloc.cpp:298) | — | fixed | UpdateMap() 开头早退,false 时完全不向 ivox 插入 |
| 6 | map_odom 往返恒等重算 + Frame::timestamp 未用(:462) | minor | fixed | 删恒等重算;timestamp 接入重试时钟与接受日志(TF 时间戳属公共接口,未动,已记录) |
| 7 | setMaxCorrespondenceDistance 对 FastVGICP 无效(:431) | — | fixed | 删 FastVGICP 分支该调用(对应关系由 setResolution 决定);PCL GICP 回退分支保留 |
| 8 | relocation_node.cpp SigHandle 死代码 + 无 glog 初始化 | — | fixed | 删死代码;main 接线 BASIC::init_log() |
| 9 | 跳变门作用在 odom 原点参数化(:481) | minor | fixed/skip | delta 改机器人处共轭度量;smoothMapToOdom 的原点参数化插值 skip(仅小修正时生效,风险大于收益) |
| 10 | cropPriorMap 每次全图线性扫描 | minor | fixed | 加载时建 10 m 粗体素桶索引,裁剪只扫 AABB 覆盖桶 |
| 11 | enterLost() 尾部死赋值 | minor | fixed | 删除;顺带清零失败计数 |

### 2.5 WP-nav(占据栅格 / 地面分割)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | 未知空间当 free 发布 + 障碍永不清除(occupancy_grid_publisher.cpp:276) | major | fixed | 证据分数模型(hit +3 / miss −1,clamp [−3,6]),缺席 = unknown(−1);insertBodyCloud 路径 2D Bresenham 光线投射;use_unknown 默认 false→true;无原点路径保持旧行为并注释 |
| 2 | 高度过滤固定世界系 z 带(:316) | major | fixed | 改相对当前 body 原点(rel_z = z − origin_z);t=0 时与旧语义重合;yaml 数值换算由 fixup D 完成 |
| 3 | 先验地图绕过地面分割(resetFromCloud:161) | major | fixed | 新增 segmentWorldMap()(map 系逐格低分位数地面模型),reset 时先分割再入格 |
| 4 | odom 系数据配 map frame_id(:251) | major | partial→fixed | 新增 setFrameMode(SLAM 模式强制 odom 系 frame_id);验收发现无调用点,fixup B 已在 ROSWrapper::setupIO 接线。残余:重定位模式在线增量仍在 odom 系(已声明) |
| 5 | 无距离门控、边界单调膨胀(:220) | — | fixed | 新参数 max_range(30 m)/max_extent(400 m);超限拒绝扩张并节流告警;顺带修同路径 double→int32 索引截断 UB |
| 6 | 时间戳回退用 wall clock(:252) | minor | fixed | 存节点 clock,尊重 use_sim_time |
| 7 | 重置后空图不发布、陈旧 latched 栅格永驻(:173) | — | fixed | reset 后立即发布(空图发 1×1 unknown 最小栅格覆盖旧样本) |
| 8 | 全量重建跑在传感器回调组(:105) | minor | partial | dirty 门控(无新插入不重建不重发)+ 端点去重;独立回调组/增量 patch 发布 skip(线程模型属 ROSWrapper 接线决策、改动面大),验收判定 skip 理由成立 |
| 9 | segment() 静默吞异常 + dist_to_ground 回退 0(ground_segmentation_filter.cpp:151/180) | — | fixed | catch 改 RCLCPP_ERROR_THROTTLE 带 what();回退值改 −sensor_mount_height(新参数,默认 0.5 m) |

### 2.6 WP-octvox(OctVoxMap 数据结构)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | KNN 提前退出阈值写死 0.5 m 分辨率(OctVoxMap.hpp:376) | major | fixed | SetOptions 按 (res/0.5)² 预计算成员表,内层零新增分支;实证:res 0.3 下修复前 9/2000 查询 5-NN 非最优,修复后 0/2000 |
| 2 | decrease_max_group() 无调用者(:204) | — | fixed(删除) | 接线需实机调参且无设计线索,选最小删除;reset_max_group()/group_idx_max_ 保留(外部调用点不在本包) |
| 3 | KNNHeap 5 路展开硬编码但模板参数 K(:68) | — | fixed | 类作用域 `static_assert(K == 5)`,K=4 实例化编译期报错验证 |
| 4 | 死序列化 API getMap/resetMap/getTopK_VN/saveMap(:413) | — | fixed(删除) | grep 确认零调用后连同附属状态/表/include 删除 |
| 5 | VoxelGridClosest 15-bit key 超界静默混叠(VoxelGridFilter.h:59) | — | fixed | 入格前范围守卫(单次比较),超界丢弃 + 一次性 LOG(WARNING);注释写明 key 布局与有效范围;出厂配置行为不变 |
| 6 | LRU 1e8 容量实际禁用(:294) | minor | fixed(仅注释) | 默认值处注明"每体素约 150 B、1e8≈15 GB、实际永不淘汰,大范围建图需下调" |

### 2.7 WP-ros(ROSWrapper)

| # | 发现(位置) | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | 外参旋转按列主序装载(转置)(ROSWrapper.cpp:117) | major | fixed | 逐元素按行主序装载;恒等/对角外参行为不变,MCD_ATH 类非对称外参从此正确 |
| 2 | 断流时 lidar/imu 缓冲无界增长(:919) | major | fixed | 新参数 lidar_buffer_capacity(100)/imu_buffer_capacity(2000),超限丢最旧 + 节流告警;lidar 侧同时复位 lidar_pushed_ 防错位 |
| 3 | 时间回退不复位(bag 循环)(:913) | — | fixed | lidar 回退 >1 s 判定 loopback,清缓冲重锚;ESKF 彻底复位仍为 todo(非本包文件,见 §6) |
| 4 | sync_measure 空 IMU 队列返回 true(:925) | — | fixed | 空队列丢帧 return false,杜绝 .back() UB |
| 5 | VELO16/32 空云 .back()(:768) | — | fixed | 各分支空云先行返回;HESAI16 补 points[0] 保护 |
| 6 | 未知 lidar_type 静默丢点(:895) | — | fixed | 构造时白名单校验,非法值 LOG(FATAL) fail-fast |
| 7 | 参数向量长度未校验(:110) | — | fixed | lidar_imu/odom_robo/init_pose 长度校验回退默认;filter_rate clamp ≥1(消除除零) |
| 8 | path_ 无界增长(:1005) | — | fixed | 新参数 path_max_poses(5000),FIFO 淘汰,格式不变 |
| 9 | 全局地图每 10 s 永久全量重发(:1207) | — | fixed | 改 latched QoS + 仅 dirty/新订阅时重发,周期可配;静态 publisher/timer 成员化 |
| 10 | odometry twist 世界系(:979) | — | fixed | 按 REP-105 转 child 系(base/imu);/lio/imu/odom 同步修正;杆臂未补偿(毫米/秒级,已记录) |
| 11 | getLatestImuData 无锁读(ROSWrapper.h:66) | — | fixed | 新增 buffer_mutex_ 覆盖全部缓冲触点;单线程下无行为变化 |
| 12 | 函数局部 static ROS 实体(:1249) | — | fixed | publisher/timer/订阅全部成员化;删无定义的声明;/initialpose 入 cb_sensor_ 组 |
| 13 | 死代码 livox2pcl(par_unseq 竞争)(:398) | — | fixed | grep 零调用后删除 |
| 14 | end_time 取自最后被接受点 + HESAI16 时钟域(:697-703) | — | fixed | 各分支 end_time = start + max_offset(过滤前求);HESAI16 start_time 统一到 header.stamp 时钟域(与 IMU 同域) |
| 15 | 缺 LiDAR-IMU 时间偏移参数 | — | fixed | 新参数 lio.sensor.time_offset(默认 0.0),imuHandler 应用 |
| 16 | 死参数 declare/get 删除 | — | fixed | 按三方共享清单删 10 个,grep 确认无读者 |
| 17 | SLAM 模式 map->odom 缺失致栅格 TF 孤儿(:1082) | minor | skipped_ok | 核查后本文件无需改(栅格 frame_id 在 occupancy_grid_publisher);未发明恒等 map->odom,与 WP-nav 约定一致 |

### 2.8 WP-config(配置 / 文档 / 构建)

| # | 发现 | 严重度 | 处置 | 修复方式/位置 |
|---|---|---|---|---|
| 1 | relocation.yaml filter_rate: 3.0(int 声明,启动即抛异常) | critical | fixed | 改 3;并以脚本比对 10 个 yaml × 108 个声明参数类型,无其他错配 |
| 2 | relocation.yaml 键名拼错 nbg(陀螺偏置噪声静默为 0) | major | fixed | 改 lio.sensor.imu_nbg: 0.0001 |
| 3 | 未声明键残留(lio.hash_map.insert_resolution 等) | minor | fixed | 删除;全 yaml 扫描无其他未声明键 |
| 4 | 死参数 yaml 键(共享清单 10 项) | — | fixed | 全部 yaml 删除;删除前确认无读者 |
| 5 | doc §8 服务名错误 | major | fixed | 改 `/lio/relocalization/trigger`;顺带核对全文档话题/参数/launch 名 |
| 6 | README 占位 bibtex / ROS 发行版口径 / 无 License 说明 | — | fixed | 删占位引用;如实声明 Ubuntu 26.04 + ROS 2 Lyrical 开发测试;新增 License 一节(上游许可待确认) |
| 7 | 缺第三方许可清单 | — | fixed | 新建 doc/LICENSES.md 逐组件列许可;kiss_matcher/pmc/xenium 如实标"待确认";未发明仓库 LICENSE |
| 8 | CMake 外部依赖未钉版 / ROOT 无说明 / 3rdparty 全量安装 | — | partial→fixed | install 排除 tests/matlab/docs;ROOT 加注释;xenium GIT_TAG main 钉版超出 OWNED 范围先 skip → fixup J 已钉到 commit 1c449ae9 |
| 9 | launch 默认起 rviz2 但无依赖 | — | fixed | package.xml 加 rviz2 exec_depend(保持 launch 默认,不使文档过期);全量 include-依赖核对无其他缺失 |
| 10 | 部署脚本无回滚 | — | fixed | deploy_jetson_version.sh 记录当前 ref,构建失败自动回滚重建;build_jetson.sh 核查后无需改 |
| 11 | 文档与代码漂移 | — | fixed | 三份使用文档逐条 grep 验证并修正 |
| 12 | 新参数缺文档 | — | fixed | 先留 §14 占位 → fixup K 定稿三张参数表 + z_min/z_max 语义换算说明 |

### 2.9 验收回归 → fixup 轮(A–K)

| # | 验收发现 | 严重度 | 处置 |
|---|---|---|---|
| A | PCD 路径守卫误杀尾斜杠 save_map_dir | major | fixed:无 filename 时取 parent_path 再校验,5 种输入覆盖测试 |
| B | setFrameMode 全仓无调用点(V4 修复失活) | major | fixed:ROSWrapper::setupIO 接线 |
| C | sim/livox_360.yaml 显式 use_unknown: false 覆盖新默认 | major | fixed:改 true |
| D | z_min/z_max 语义变更未同步配置 | critical | fixed:sim.yaml 换算为相对值(−0.585/1.265)并注释;resetFromCloud 的地面高度基准连带修正(+sensorMountHeight());补 sensor_mount_height: 0.735 |
| E | 占据栅格输入被固定为 0.3 m 降采样云(dense 语义丢失) | major | fixed:恢复随 g_visual_dense 选择稠密/降采样云 |
| F | kf_init 静止门无超时兜底(振动平台可能永不启动) | major | fixed(部分):连续拒绝 20 窗(≈10 s)后告警降级用当前窗初始化;阈值参数化 deferred |
| G | grid_update_step 未接线 yaml | — | fixed:declare/get + yaml 键(默认 1) |
| H | save_map_dir 绝对路径仍拼 ROOT | — | fixed:仅相对路径拼接 |
| I | localization 4 个新参数未入装载路径 | — | fixed:params + ROSWrapper declare/get + init() 赋值 |
| J | xenium GIT_TAG main 未钉版 | — | fixed:钉到 commit 1c449ae9(本轮被测检出) |
| K | doc §14 参数文档占位未定稿 | — | fixed:定稿 |

---

## 3. 新增参数总表

全部经现有参数装载路径(ROSWrapper declare/get)接入;除标注者外,默认值即修复前行为。

| 参数 | 类型 | 默认 | 语义 | 模块 |
|---|---|---|---|---|
| `lio.backend.use_localization_factor` | bool | **false** | **实验旗标**:VGICP 配准作 unary 因子进图,发布 /lio/backend/optimized_path(见 §4) | backend |
| `lio.backend.drive_map_to_odom` | bool | **false** | **实验旗标**:后端派生 T_map_odom 喂现有跳变门/平滑通道(需上一旗标开启) | backend |
| `lio.backend.loc_factor_sigma_t` | double | 0.1 | unary 因子基准平移 σ [m] | backend |
| `lio.backend.loc_factor_sigma_r` | double | 0.02 | unary 因子基准旋转 σ [rad] | backend |
| `lio.backend.loc_factor_fitness_ref` | double | 0.1 | 使 σ 翻倍的 fitness 参考值 [m²] | backend |
| `lio.backend.loc_factor_max_time_diff` | double | 0.15 | 观测-关键帧时间关联门限 [s] | backend |
| `lio.sensor.time_offset` | double | 0.0 | imu_time += offset,IMU 对齐 lidar 时钟 [s] | ros |
| `lio.ros.lidar_buffer_capacity` | int | 100 | lidar 缓冲最大帧数(≈10 s@10 Hz),超限丢最旧 | ros |
| `lio.ros.imu_buffer_capacity` | int | 2000 | IMU 缓冲最大样本数(≈10 s@200 Hz),超限丢最旧 | ros |
| `lio.output.path_max_poses` | int | 5000 | /lio/path 最大位姿数,FIFO 淘汰 | ros |
| `lio.output.global_map_pub_period` | double | 10.0 | 全局地图重发检查周期 [s],仅 dirty/新订阅时发 | ros |
| `lio.output.grid_update_step` | int | 1 | 占据栅格更新节拍(帧),与可视化 pub_step 解耦 | frontend/fixup |
| `lio.occupancy_grid.max_range` | double | 30.0 | 扫描入格 3D 射程门控 [m](仅 insertBodyCloud 路径) | nav |
| `lio.occupancy_grid.max_extent` | double | 400.0 | 栅格每轴总跨度上限 [m],超限拒绝扩张并节流告警 | nav |
| `lio.occupancy_grid.raycast.enable` | bool | true | 沿传感器→命中点射线雕刻 free 空间 | nav |
| `lio.occupancy_grid.raycast.max_ray_length` | double | 15.0 | 超此 2D 长度不做 free 雕刻(命中仍记录),控 CPU | nav |
| `lio.occupancy_grid.use_unknown` | bool | **true(原 false,默认行为变化)** | 未观测格发布为 −1(unknown) | nav |
| `<prefix>.ground_filter.sensor_mount_height` | double | 0.5(sim.yaml 实配 0.735) | 传感器离地高度 [m];auto dist_to_ground 失败时地面种子取 −该值 | nav |
| `lio.prior_map_localization.kiss_retry_cooldown_sec` | double | 5.0 | LOST 期 KISS 重试基础冷却 [s] | localization |
| `lio.prior_map_localization.kiss_retry_max_backoff` | int | 6 | 连续失败冷却倍率上限(最长 5×6=30 s) | localization |
| `lio.prior_map_localization.max_delta_rejections` | int | 10 | 连续一致跳变门拒绝达该数即转 LOST 触发 KISS | localization |
| `lio.prior_map_localization.crop_grid_cell_size` | double | 10.0 | 先验地图裁剪索引桶尺寸 [m] | localization |

另:观测队列容量为编译期常量 `kMapObservationQueueCap = 8`(prior_map_localization.cpp);
kf_init 静止门常量 kInitMaxGyroVar=0.01、kInitMaxAcceVarRel=4e-3、kInitMaxStillRejects=20 未参数化(deferred,见 §6)。

---

## 4. 实验特性:先验地图定位结果作为因子进图

**背景**:Wave 1 确认修复前后端因子图为"先验 + 顺序 BetweenFactor 链"——纯链式图的解恒等于
里程计输入,后端对估计零影响(optimized 标志已相应诚实化为 false)。本特性是后端第一次
真正参与估计:把 PriorMapLocalization 的 VGICP 配准结果(map 系绝对位姿 + fitness)作为
unary PriorFactor 加入图,形成"里程计链 + 绝对观测"的有冗余图。

**设计要点**(详见决策日志 feature.md):

- 双旗标门控,**默认全 false,全 false 时与修复后基线行为一致**(经等价性核查);
- 旗标开启时关键帧状态为 map 系位姿;Between 度量取 odom 系相对位姿(与系无关),
  噪声用 Wave 1 修好的自适应协方差;unary 噪声 σ = σ_base·(1 + fitness/fitness_ref),外包 Huber(1.345);
- x0 先验在旗标开启时改弱规范先验(σ 1.0 rad / 10.0 m)——map 系须由绝对观测定义;
- 数据通路:互斥锁保护的 8 深有界队列,锁内只做交换;按时间戳 lower_bound 关联最近关键帧,|dt|>0.15 s 丢弃并计数;
- 输出:优化轨迹发布为 `/lio/backend/optimized_path`(nav_msgs/Path);
  可选经第二旗标把 T_map_odom_smoothed 喂现有平滑通道。**绝不写回 ESKF、绝不改 odom 系状态与体素地图**;
- iSAM2 update/calculateEstimate 全部 try/catch:链 update 失败则后端永久停用(前端不受影响),unary 失败仅丢本批。

**告诫(必读)**:

1. **该特性全部未经实机验证**,所有 σ/映射参数是未标定工程初值;
2. fitness→σ 是经验置信度映射,不是配准自身协方差;fitness 对"自信但错"的配准无鉴别力,Huber + 跳变门是唯一防线;
3. 定位初始化前插入的关键帧初值有误,初始化跳变大时 iSAM2 收敛性未验证;
4. unary 观测无运动补偿(≤0.15 s × 速度的系统误差);稀疏关键帧时观测丢弃率偏高;
5. 全量轨迹回代与 Path 发布为 O(n)/关键帧,多小时运行开销未评估;
6. 链 update 异常后后端本次运行内永久停用,无图重建恢复路径。

**实机部署建议保持两个旗标为 false**,先在数据集/仿真上评估。

---

## 5. 行为变化清单(默认配置下与修复前的差异)

给实机部署者:以下为**不改任何配置**时行为即发生变化的条目。

**定位/里程计核心**

1. 后端不再写回 ESKF(链式图下写回本是恒等,理论上轨迹 bit 级不变;消除 float 往返扰动路径)。
2. ESKF 新增尾段传播到扫描末端:去畸变覆盖完整扫描,快速旋转下建图质量理论上只会变好。
3. 扫描驱动 Predict 对 dt<0 或 >0.2 s 的样本跳过积分并重锚时间(原样把空洞积分进 P);高频前向路径同样受 0.2 s 门限保护。
4. IMU 初始化:零加计拒绝初始化;静止性门(gyro 方差 0.01、加计相对方差 4e-3)不满足时重等,连续 20 窗(≈10 s)不满足则告警降级初始化。怠速振动平台注意观察初始化日志。
5. planar 约束调平对象改为 base 系(安装旋转为单位阵的配置无变化)。
6. 外参旋转按 yaml 书写序(行主序)装载:**非对称旋转外参的配置行为改变**(此前被转置);恒等/对角外参不受影响。
7. HESAI16 start_time 统一到 header.stamp 时钟域;各雷达 end_time 改为全部点最大时标(过滤前),同步窗口更准确。
8. 重定位节点(nav2 配置):ESKF 保持 odom 系,map->odom 只应用一次匹配位姿(修复双重应用);/lio/odometry 等话题 frame_id 与数据所在系一致。
9. LOST 后 KISS 周期性重试(5–30 s 退避)而非只试一次;TRACKING 楔死会自动转 LOST 脱出;手动重定位触发立即生效(绕过跳变门)。
10. update_map=false 现在真正冻结先验地图(此前仍持续插入)。
11. sim.yaml(vox_resolution=0.3)下 KNN 提前退出界修正:近邻更准(实测消除 9/2000 的错误 5-NN),每点搜索开销略增。

**导航输出**

12. 占据栅格三态化:unknown(−1)/free(0)/occupied(100),默认发布 unknown;raycast 雕刻 free,瞬态障碍(行人/搬动家具)约 3–6 帧内清除,**障碍不再永久驻留**。Nav2 代价地图需按三态语义确认 track_unknown_space 配置。
13. 高度带 z_min/z_max 语义改为**相对传感器原点**(sim.yaml 已换算,自有 yaml 需按 `新值 = 旧离地值 − 传感器离地高度` 换算)。
14. 栅格更新与可视化 pub_step 解耦:默认每帧更新(原每 pub_step 帧),Jetson 需实测耗时,超预算调大 lio.output.grid_update_step。
15. SLAM 模式(无先验地图)栅格 frame_id 改为 odom 系名(原挂 map 系造成 TF 孤儿)。
16. 新增 30 m 射程门控与 400 m 栅格跨度上限;reset 后立即发布(空图发 1×1 unknown);无新插入时定时器不再重建重发。
17. 先验地图入格前先过地面分割(原直接入格,地面点全变障碍)。

**ROS 接口**

18. odometry twist 改 child 系(REP-105);下游速度消费方(EKF/速度平滑)需确认。
19. 全局地图改 latched QoS,仅地图变化或新订阅时重发:**首帧发布延迟可达 global_map_pub_period(默认 10 s)**,稳态不再周期性全量重发。
20. 未知 lidar_type 启动即 LOG(FATAL)(原静默丢弃所有点云)。
21. lidar/imu 缓冲、/lio/path 均有上限(丢最旧/FIFO);bag 循环播放自动清缓冲恢复(ESKF 彻底复位仍待做,见 §6)。
22. glog 已初始化:日志走 stderr(可选文件),不再有 "Logging before InitGoogleLogging" 前缀;AdaGraph 日志并入 glog。
23. save_map_dir 为绝对路径时不再拼接编译期 ROOT;地图删除改 std::filesystem 且仅允许删 `<save_root>/PCD`。
24. relocation.yaml 可以启动了(filter_rate 类型修复);其陀螺偏置随机游走噪声从静默 0 恢复为 0.0001(键名修复)。

---

## 6. 遗留事项与建议

### 6.1 验收标记的未闭合问题(建议排期处理)

| 问题 | 位置 | 说明 |
|---|---|---|
| buildPriorMapGrid 对含 NaN 的先验地图点 floor→int(UB) | prior_map_localization.cpp | 先验 PCD 未过滤 NaN 时未定义行为;建议入索引前 isfinite 过滤 |
| segmentWorldMap 对有限但巨大坐标无防护 double→int32 截断(UB) | ground_segmentation_filter.cpp | 损坏的先验 PCD 触发;occupancy 侧同类问题已修,此处未修 |
| resetFromCloud:ground_filter 启用但先验图点数 < min_input_points(30)的边界行为 | occupancy_grid_publisher.cpp | 小点云先验图路径未覆盖 |
| max_extent 跨度检查以首个入格格为锚 | occupancy_grid_publisher.cpp | insertCloud(无原点)路径无射程门控,reset 后首点为离群点时 400 m 窗口锚在离群位置 |
| ivox 灌入移至 kf_init 成功路径 | super_lio_reloc.cpp | 站点级地图(数百万点)在重定位成功瞬间做全图 SE3 变换 + 插入,一次性延迟需实测 |
| submitBackendMapToOdom 读-改-写竞态(后写者胜) | prior_map_localization.cpp | 仅实验旗标开启时存在;两写者均过门与平滑,量级受限,判定良性,仍建议后续统一 |
| 重定位节点 odom 系化后,基类 caceData()/saveMap() 若在该节点启用会存出 odom 系地图 | super_lio_reloc.cpp | 定位模式通常不存图;如需存图须先变换回 map 系 |
| set_global_map 新订阅检测用 sub_count 增量 | ROSWrapper.cpp | 两次 tick 之间"加入又离开/换手"的订阅者可能错过重发;latched QoS 已覆盖 transient_local 订阅者 |
| 全局地图首帧发布延迟至多 10 s(原 ~1 s) | ROSWrapper.cpp | 见 §5-19;可调小 global_map_pub_period 缓解 |
| VoxelGrid 告警文本/注释的有效范围与 round() 装箱差半格 | VoxelGridFilter.h:63-65 | 守卫本身正确(按 OR 检查),仅文案量程表述不精确 |
| Gamma_2 小角分支门限 1e-5 按 double 调,T=float 实例化欠佳 | Manifold.h | 当前调用均为项目 scalar;泛型 float 使用需复核门限 |
| computeRuleBasedScales 注释"(frontend buffers cap at 20000)"已陈旧 | adagraph_feature_extractor.cpp | 前端缓冲已改动态 resize;仅注释过期 |
| kRankEigMin(1e3)与 hessian_eig_min_(adagraph_backend.h:85)语义重复的两处常量 | backend | 建议合并为单一来源 |
| Wave 2 fillOptimizedTrajectory 每关键帧全量 isam2 回代 O(n) | adagraph_backend.cpp | 仅实验旗标开启时;多小时运行需改增量/抽稀 |

### 6.2 明确 deferred / skip(理由成立,维持现状)

- occupancy 独立回调组 + MultiThreadedExecutor、增量 OccupancyGridUpdate 发布(nav 8-M1):线程模型变更风险大于收益;
- 后端位姿跳变时清 local_frames_odom_ 的接口:触发源(写回)已删除,无需新增;
- bag 循环后 ESKF 彻底复位(原代码 `// eskf_->Reset()` todo):无既定设计,现有 dt 保护带可过渡;
- kf_init 静止门阈值参数化:失效模式已被 20 窗超时兜底消除;
- smoothMapToOdom 的 odom 原点参数化插值:仅小修正时生效,重推导风险大于收益;
- between 因子噪声的切空间混系(backend M2):纯链式图下不影响估计,实验特性常态化前须处理;
- iSAM2 图/keyframe_poses_ 无界增长:fixed-lag/边缘化属图结构改动,留后续;
- 宏加前缀、buffer 套件接线或删除:接口冻结/超授权。

### 6.3 需实机/数据集验证清单(汇总)

1. 建图回归:修复后轨迹与基线对照(理论 bit 级相同);快速旋转下尾段传播效果;sim 0.3 分辨率 KNN 修复后定位指标。
2. 重定位:先验地图与 scan 在 map 系对齐、tf2_echo map→base_link 与匹配位姿一致;手动触发纠偏;遮挡制造 LOST 观察重试与恢复;update_map=false 下内存不增长。
3. Nav2 联调:栅格三态 + unknown 语义、低矮障碍出现与清除时延、SLAM 模式 frame_id、twist 改 body 系后速度消费方行为。
4. Jetson 性能:稠密云每帧入栅格耗时(超预算调 grid_update_step)、raycast CPU 占用、cropPriorMap 桶索引提速、kf_init 处 ivox 灌入延迟。
5. 数据集:MCD_ATH(外参行主序)、HESAI(时钟域统一后去畸变质量)、bag 循环播放自动恢复、断流内存封顶。
6. 阈值标定:hessian_eig_min_、condition/trace/点数阈值、静止门阈值(怠速振动平台)、20 窗超时降级后的初始化质量。
7. Jetson 端 glog 版本:logs.cpp 使用 `google::IsGoogleLoggingInitialized()`(glog ≥0.5.0,JetPack 6 满足;JetPack 5 需改纯 static bool)。
8. 部署脚本:故意用坏 tag 验证一次自动回滚路径。

### 6.4 范围与许可

- **livox_ros_driver2 未纳入本轮审查与构建**(仅其 LICENSE 已录入 doc/LICENSES.md,MIT);
- **上游 Super-LIO 许可待确认**:仓库未发明 LICENSE 文件,README 已如实声明;package.xml 的 GPLv3 为上游继承;
- kiss_matcher 源码头标 MIT 但树内缺许可文本,需作者从上游补入;pmc、xenium 许可树内不可确认,已标"待确认";
- 外部下载依赖已钉版(pmc 4bbd40ab、xenium 1c449ae9);首次 CMake configure 需联网(已写入 README)。

---

## 7. 编译与测试状态

- 环境:Ubuntu 26.04 / ROS 2 Lyrical(本机);`colcon build --packages-select basic super_lio`。
- **干净全量重建通过,0 编译错误**(删除 build/install 缓存后重建,1 min 53 s);
  8 个工作包改动相互兼容,跨包协调性修改数为 0。stderr 仅存量警告
  (3rdparty/robin 的 -Wsign-compare 等、tf2_ros 弃用 #warning、CMake 版本弃用),均不属本轮修复范围。
- 单元测试:新增 4 个 gtest 文件(basic:Manifold 16 例 + RingBuffer 11 例;
  super_lio:adagraph_backend 协方差逻辑 9 例 + OctVoxMap KNN 对照 4 例),
  `colcon test` **44 tests, 0 errors, 0 failures, 0 skipped**。
  测试编写过程中发现并修复了 `SE3::log()` 小角度数值灾难(§2.1 附)——测试先于实机暴露真实 bug。
- 每个工作包另做了 `-fsyntax-only`/增量 make 自查与 grep 无悬空符号检查;
  基础数学修复有独立数值 harness(级数/积分对照、暴力 KNN 对照)佐证。
- fixup 轮后复跑构建与全部测试:全绿(结果文件时间戳确认为最终工作区状态)。
- livox_ros_driver2 按约定未纳入构建。
- 实验特性(§4)仅编译与逻辑验证,**无实机验证**;两旗标默认关闭。
