# 第三方组件许可清单

本仓库基于上游 Super-LIO(<https://arxiv.org/abs/2509.05723>,见
`doc/参考文献.md`)改造。上游 Super-LIO 的许可条款尚待确认,因此仓库暂不提供
顶层 LICENSE 文件;`src/super_lio/package.xml` 与 `src/basic/package.xml` 中的
GPLv3 声明继承自上游。下表列出仓库内 vendored 的第三方组件及其各自许可,
以树内实际携带的许可文本 / 源码头注释为准;无法在树内确认的如实标注"待确认"。

## vendored 组件

| 组件 | 树内位置 | 来源 | 许可 |
|---|---|---|---|
| fast_gicp | `src/super_lio/3rdparty/fast_gicp` | <https://github.com/SMRT-AIST/fast_gicp> | BSD-3-Clause(树内 `LICENSE`,Copyright 2020 SMRT-AIST) |
| KISS-Matcher | `src/super_lio/3rdparty/kiss_matcher` | <https://github.com/MIT-SPARK/KISS-Matcher> | 源码头标注 "Copyright 2024 MIT ... See LICENSE",但 vendored 副本未携带 LICENSE 文件;上游仓库为 MIT,**待确认并需补齐许可文本** |
| KISS-Matcher 内嵌 kdtree/points | `src/super_lio/3rdparty/kiss_matcher/core/kiss_matcher/{kdtree,points}` | small_gicp(Kenji Koide) | MIT(源码头 SPDX: `SPDX-License-Identifier: MIT`,Copyright 2024 Kenji Koide) |
| KISS-Matcher 内嵌 tsl robin-map | `src/super_lio/3rdparty/kiss_matcher/core/kiss_matcher/tsl` | <https://github.com/Tessil/robin-map> | MIT(许可文本保留在各头文件内,Copyright 2017 Thibaut Goetghebuer-Planchon) |
| ROBIN | `src/super_lio/3rdparty/robin` | <https://github.com/MIT-SPARK/ROBIN> | 树内 `LICENSE.md`:BSD-2-Clause 风格的 "ROBIN License"(Copyright 2020 MIT),另附 Catch2 的 Boost Software License 1.0 条款(仅测试用) |
| ground_segmentation | `src/super_lio/3rdparty/ground_segmentation` | <https://github.com/dfki-ric/ground_segmentation>(commit `4ce23005`) | BSD-3-Clause(树内 `licenses/LICENSE.txt`,Copyright 2023 DFKI Planning) |
| tsl robin-map | `src/super_lio/include/OctVoxMap/tsl/robin_map.h` 等(`robin_set.h`、`robin_hash.h`、`robin_growth_policy.h`) | <https://github.com/Tessil/robin-map> | MIT(许可文本保留在各头文件内,Copyright 2017 Thibaut Goetghebuer-Planchon) |
| robin_hood hashing | `src/super_lio/include/OctVoxMap/tsl/robin_hood.h` | <https://github.com/martinus/robin-hood-hashing> | MIT(源码头 SPDX,Copyright 2018-2021 Martin Ankerl) |
| livox_ros_driver2 | `src/livox_ros_driver2` | <https://github.com/Livox-SDK/livox_ros_driver2> | MIT(树内 `LICENSE.txt`,按其列出的文件范围适用) |

## 构建期下载的依赖(不在仓库内)

以下依赖在 CMake configure 阶段联网下载,许可以各自上游为准:

| 依赖 | 下载脚本 | 版本 | 许可 |
|---|---|---|---|
| pmc | `src/super_lio/3rdparty/robin/cmake/DownloadExternal.cmake` | MIT-SPARK/pmc,commit `4bbd40ab` | 待确认(上游 pmc 许可未在树内携带) |
| xenium | `src/super_lio/3rdparty/robin/cmake/DownloadExternal.cmake` | mpoeter/xenium,commit `1c449ae9`(已钉版) | 待确认 |
| Eigen 3.4.0 | `robin/cmake/DownloadEigen.cmake`、`kiss_matcher/3rdparty/eigen/eigen.cmake`(仅当系统 Eigen 不可用时) | 3.4.0 | MPL-2.0 |
| oneTBB | `kiss_matcher/3rdparty/tbb/tbb.cmake`(仅当系统 TBB 不可用时) | v2022.0.0 | Apache-2.0 |

注:本工程默认 `USE_SYSTEM_EIGEN3=ON`、`USE_SYSTEM_TBB=ON`,正常构建不会触发
Eigen/oneTBB 下载;pmc 与 xenium 则总会在首次 configure 时下载。
