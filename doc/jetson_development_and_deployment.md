# AdaGraph-LIO Jetson 开发与部署

## 1. 基本原则

- 开发电脑负责修改源码、仿真、测试、提交和发布版本。
- Jetson 只运行带 `jetson-vX.Y.Z` 标签的稳定版本。
- 不在两台机器之间复制 `build/`、`install/`、`log/`。
- x86_64 和 aarch64 必须分别从源码编译。
- 实机参数、标定结果和先验地图应单独备份，不要覆盖为开发电脑配置。

## 2. 配置远程仓库

当前仓库需要先在 GitHub、GitLab、Gitee 或局域网 Git 服务中创建一个私有空仓库，然后在开发电脑执行：

```bash
cd /home/sw/AdaGraph-LIO
git remote add origin <REMOTE_URL>
git push -u origin main
git push origin develop
git push origin --tags
```

检查配置：

```bash
git remote -v
```

## 3. 日常开发

日常工作从 `develop` 创建功能分支：

```bash
git switch develop
git switch -c feature/my-change

# 修改、编译和测试
git add .
git commit -m "Implement my change"

git switch develop
git merge --ff-only feature/my-change
git push origin develop
```

`main` 只接收已经通过仿真和实机测试的版本：

```bash
git switch main
git merge --ff-only develop
git tag -a jetson-v0.2.0 -m "Jetson release v0.2.0"
git push origin main
git push origin jetson-v0.2.0
git switch develop
```

如果 `main` 与 `develop` 已经分叉，不要强行执行 `--ff-only`，应先检查提交历史并通过普通合并解决。

## 4. Jetson 首次安装

Jetson 上只克隆源码：

```bash
git clone <REMOTE_URL> ~/AdaGraph-LIO
cd ~/AdaGraph-LIO
git checkout jetson-v0.1.0

source /opt/ros/<ROS_DISTRO>/setup.bash
INSTALL_DEPS=1 BUILD_JOBS=2 ./scripts/build_jetson.sh
source install/setup.bash
```

`BUILD_JOBS=2` 用于避免 Jetson 首次编译 PCL、KISS-Matcher 和 FastVGICP 时内存压力过大。内存充足时可以提高到 `4`。

工程内置 FastVGICP 的 CPU 实现，不要求 CUDA。运行时并行度由以下参数控制：

```yaml
lio.prior_map_localization.num_threads: 4
```

## 5. Jetson 更新稳定版本

开发电脑推送新标签后，在 Jetson 执行：

```bash
cd ~/AdaGraph-LIO
source /opt/ros/<ROS_DISTRO>/setup.bash
BUILD_JOBS=2 ./scripts/deploy_jetson_version.sh jetson-v0.2.0
source install/setup.bash
```

部署脚本会：

1. 拒绝覆盖 Jetson 上未提交的本地修改。
2. 从 `origin` 获取标签。
3. 切换到指定的稳定版本。
4. 清理 CMake 缓存并重新编译 Super-LIO 依赖链。

## 6. 回滚

新版本出现问题时，切回上一个标签：

```bash
cd ~/AdaGraph-LIO
source /opt/ros/<ROS_DISTRO>/setup.bash
BUILD_JOBS=2 ./scripts/deploy_jetson_version.sh jetson-v0.1.0
source install/setup.bash
```

## 7. Jetson 本机配置

建议将以下内容放在 Jetson 的独立配置或启动覆盖文件中：

- Mid-360 IP、点云和 IMU 话题。
- LiDAR 到 IMU、IMU 到 `base_link` 的外参。
- FastVGICP 线程数。
- 地面分割和栅格地图参数。
- 先验 PCD 地图路径。
- Nav2 和 MoveIt 实机参数。

先验地图、rosbag 和日志默认不进入 Git。发布新代码前，应分别备份这些运行数据。
