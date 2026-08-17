#!/usr/bin/env python3

"""Verify that HybridSLAM builds only the Point-LIO production path."""

from __future__ import annotations

import re
import sys
from pathlib import Path


CORE_PATHS = (
    Path("src/hybrid_slam/include/hybrid_slam"),
    Path("src/hybrid_slam/src/backend"),
    Path("src/hybrid_slam/src/io"),
    Path("src/hybrid_slam/src/localization"),
    Path("src/hybrid_slam/src/mapping"),
    Path("src/hybrid_slam/src/system/dataset_runner.cpp"),
    Path("src/hybrid_slam/src/system/global_alignment_state.cpp"),
    Path("src/hybrid_slam/src/system/slam_system.cpp"),
)

POINT_LIO_VENDOR_SOURCES = (
    "3rdparty/small_point_lio/src/small_point_lio/estimator.cpp",
    "3rdparty/small_point_lio/src/small_point_lio/parameters.cpp",
    "3rdparty/small_point_lio/src/small_point_lio/preprocess.cpp",
    "3rdparty/small_point_lio/src/small_point_lio/small_point_lio.cpp",
    "3rdparty/small_point_lio/src/util/voxelgrid_sampling.cpp",
)

POINT_LIO_ALGORITHM_SOURCES = (
    "src/lio/hybrid_slam_feature_extractor.cpp",
    "src/lio/kiss_matcher_relocalizer.cpp",
    "src/lio/pcl_relocalizer_adapters.cpp",
    "src/lio/prior_map_localization.cpp",
    "src/lio/runtime_memory.cpp",
)

RETIRED_ROOTS = (
    "src/super_lio",
    "src/hybrid_slam/include/adagraph",
)

RETIRED_PATHS = (
    "src/hybrid_slam/src/apps/super_lio_node.cpp",
    "src/hybrid_slam/src/lio/ESKF.cpp",
    "src/hybrid_slam/include/lio/ESKF.h",
    "src/hybrid_slam/src/lio/super_lio.cpp",
    "src/hybrid_slam/include/lio/super_lio.h",
    "src/hybrid_slam/include/common/legacy_state.h",
    "src/hybrid_slam/include/lio/system/legacy_downstream_frame.h",
    "src/hybrid_slam/include/lio/system/legacy_downstream_pipeline.h",
    "src/hybrid_slam/include/lio/system/legacy_point_frame.h",
    "src/hybrid_slam/include/lio/system/legacy_runtime_config.h",
    "src/hybrid_slam/include/OctVoxMap",
    "src/hybrid_slam/src/lio/params.cpp",
    "src/hybrid_slam/include/lio/params.h",
    "src/hybrid_slam/src/lio/adagraph_backend.cpp",
    "src/hybrid_slam/include/lio/adagraph_backend.h",
    "src/hybrid_slam/src/lio/legacy_algorithm_config_adapter.cpp",
    "src/hybrid_slam/src/ros/legacy_global_parameter_bridge.cpp",
    "src/hybrid_slam/include/ros/legacy_global_parameter_bridge.h",
    "src/hybrid_slam/src/ros/ROSWrapper.cpp",
    "src/hybrid_slam/include/ros/ROSWrapper.h",
    "src/hybrid_slam/src/ros/legacy_input_bridge.cpp",
    "src/hybrid_slam/include/ros/legacy_input_bridge.h",
    "src/hybrid_slam/include/ros/legacy_parameter_bridge.h",
    "src/hybrid_slam/src/ros/legacy_parameter_bridge.cpp",
    "src/hybrid_slam/src/system/legacy_downstream_pipeline.cpp",
    "src/hybrid_slam/test/test_adagraph_backend.cpp",
    "src/hybrid_slam/test/test_octvoxmap.cpp",
    "src/hybrid_slam/rviz/lio.rviz",
    "src/hybrid_slam/config/M2DGR.yaml",
    "src/hybrid_slam/config/NCLT.yaml",
    "src/hybrid_slam/config/NTU.yaml",
    "src/hybrid_slam/config/sim.yaml",
    "src/hybrid_slam/config/hesai.yaml",
    "src/hybrid_slam/config/r3live.yaml",
    "src/hybrid_slam/config/MCD_ATH.yaml",
    "src/hybrid_slam/config/livox_360.yaml",
    "src/hybrid_slam/launch/M2DGR.py",
    "src/hybrid_slam/launch/NCLT.py",
    "src/hybrid_slam/launch/NTU.py",
    "src/hybrid_slam/launch/sim.py",
    "src/hybrid_slam/launch/hesai.py",
    "src/hybrid_slam/launch/r3live.py",
    "src/hybrid_slam/launch/MCD_ATH.py",
    "src/hybrid_slam/launch/Livox_mid360.py",
    "src/hybrid_slam/msg/CloudPose.msg",
    "src/hybrid_slam/msg/CloudPose2.msg",
    "src/hybrid_slam/src/lio/super_lio_reloc.cpp",
    "src/hybrid_slam/include/lio/super_lio_reloc.h",
    "src/hybrid_slam/src/apps/relocation_node.cpp",
    "src/hybrid_slam/include/ros/legacy_relocation_config.h",
)

RETIRED_CMAKE_TOKENS = (
    "adagraph_super_lio_frontend",
    "adagraph_legacy_config",
    "adagraph_legacy_global_ros_compat",
    "adagraph_legacy_backend",
    "adagraph_legacy_compat",
    "adagraph_ros_composition",
    "add_library(lio INTERFACE)",
    "super_lio_node",
    "src/lio/super_lio.cpp",
    "src/lio/ESKF.cpp",
    "src/ros/ROSWrapper.cpp",
    "src/ros/legacy_input_bridge.cpp",
    "src/ros/legacy_global_parameter_bridge.cpp",
    "SUPER_LIO_",
    "adagraph_relocation_compat",
    "relocation_node",
)

FORBIDDEN_ROS_INCLUDE = re.compile(
    r"^[ \t]*#[ \t]*include[ \t]+(?:<|\")"
    r"(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|visualization_msgs|"
    r"tf2|tf2_ros|std_srvs|livox_ros_driver2|ros/)",
    re.MULTILINE,
)

ROS_DEPENDENCY = re.compile(
    r"\b(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|visualization_msgs|"
    r"tf2|tf2_ros|std_srvs|pcl_ros|pcl_conversions|livox_ros_driver2)\b"
)

RETIRED_BASIC_COMPAT_INCLUDE = re.compile(
    r'#include\s+[<"]basic/(?:alias|logs)\.h[>"]'
)


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for relative in CORE_PATHS:
        path = root / relative
        if path.is_file():
            files.append(path)
        elif path.is_dir():
            files.extend(
                child
                for child in path.rglob("*")
                if child.suffix in {".h", ".hpp", ".cpp", ".cc", ".cxx"}
            )
    return sorted(files)


def add_library_block(cmake: str, target: str) -> str:
    match = re.search(
        rf"add_library\(\s*{re.escape(target)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else ""


def target_block(cmake: str, target: str) -> str:
    match = re.search(
        rf"target_link_libraries\(\s*{re.escape(target)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else ""


def require_sources(
    cmake: str, target: str, sources: tuple[str, ...], failures: list[str]
) -> None:
    block = add_library_block(cmake, target)
    if not block:
        failures.append(f"missing add_library definition for {target}")
        return
    expanded_block = block
    for variable in re.findall(r"\$\{([A-Z0-9_]+)\}", block):
        definition = re.search(
            rf"set\(\s*{re.escape(variable)}\b(?P<body>.*?)\)",
            cmake,
            re.DOTALL,
        )
        if definition:
            expanded_block += definition.group("body")
    for source in sources:
        if source not in expanded_block:
            failures.append(f"{target} does not compile {source}")


def check(root: Path) -> list[str]:
    failures: list[str] = []
    cmake_path = root / "src/hybrid_slam/CMakeLists.txt"
    if not cmake_path.is_file():
        return [f"missing {cmake_path}"]
    cmake = cmake_path.read_text(encoding="utf-8")

    for relative in RETIRED_ROOTS:
        if (root / relative).exists():
            failures.append(f"retired root exists: {relative}")

    for path in source_files(root):
        content = path.read_text(encoding="utf-8")
        if FORBIDDEN_ROS_INCLUDE.search(content):
            failures.append(f"ROS include leaked into core: {path.relative_to(root)}")

    hybrid_slam_root = root / "src/hybrid_slam"
    for path in sorted(hybrid_slam_root.rglob("*")):
        if path.suffix not in {".h", ".hpp", ".cpp", ".cc", ".cxx"}:
            continue
        content = path.read_text(encoding="utf-8")
        if RETIRED_BASIC_COMPAT_INCLUDE.search(content):
            failures.append(
                "retired basic PCL/logging compatibility include exists: "
                f"{path.relative_to(root)}"
            )

    for relative in RETIRED_PATHS:
        if (root / relative).exists():
            failures.append(f"retired path exists: {relative}")
    for token in RETIRED_CMAKE_TOKENS:
        if token in cmake:
            failures.append(f"retired CMake token exists: {token}")

    require_sources(cmake, "point_lio_vendor", POINT_LIO_VENDOR_SOURCES, failures)
    require_sources(
        cmake,
        "hybrid_slam_point_lio_algorithms",
        POINT_LIO_ALGORITHM_SOURCES,
        failures,
    )
    require_sources(
        cmake,
        "hybrid_slam_point_lio_pipeline",
        ("src/system/point_lio_pipeline.cpp",),
        failures,
    )
    require_sources(
        cmake, "point_lio_adapter", ("src/lio/point_lio_frontend.cpp",), failures
    )

    adapter_links = target_block(cmake, "point_lio_adapter")
    for required in ("point_lio_vendor", "basic::basic_core", "Eigen3::Eigen"):
        if required not in adapter_links:
            failures.append(f"point_lio_adapter must link {required}")
    for forbidden in (
        "adagraph_legacy_config",
        "adagraph_legacy_global_ros_compat",
        "adagraph_ros_composition",
        "${basic_LIBRARIES}",
    ):
        if forbidden in adapter_links:
            failures.append(f"point_lio_adapter links retired compatibility target {forbidden}")

    downstream_block = add_library_block(cmake, "hybrid_slam_point_lio_pipeline")
    downstream_links = target_block(cmake, "hybrid_slam_point_lio_pipeline")
    if ROS_DEPENDENCY.search(downstream_links):
        failures.append("ROS dependency leaked into Point-LIO downstream")
    for forbidden in (
        "src/lio/adagraph_backend.cpp",
        "src/lio/params.cpp",
        "src/lio/super_lio.cpp",
        "src/ros/ROSWrapper.cpp",
        "src/ros/legacy_input_bridge.cpp",
        "src/system/legacy_downstream_pipeline.cpp",
    ):
        if forbidden in downstream_block:
            failures.append(f"retired source leaked into downstream: {forbidden}")

    point_node_links = target_block(cmake, "point_lio_node")
    for required in (
        "point_lio_adapter",
        "hybrid_slam_ros2_adapter",
        "hybrid_slam_ros_publication",
        "hybrid_slam_system_core",
        "hybrid_slam_point_lio_pipeline",
    ):
        if required not in point_node_links:
            failures.append(f"point_lio_node must link {required}")

    runner_links = target_block(cmake, "hybrid_slam_dataset_runner")
    for required in ("point_lio_adapter", "hybrid_slam_point_lio_pipeline"):
        if required not in runner_links:
            failures.append(f"hybrid_slam_dataset_runner must link {required}")
    if "adagraph_super_lio_frontend" in runner_links:
        failures.append("dataset runner retains retired frontend link")

    installed_apps = re.search(
        r"install\(TARGETS\s*(?P<body>.*?)\s*DESTINATION lib/\$\{PROJECT_NAME\}",
        cmake,
        re.DOTALL,
    )
    if not installed_apps:
        failures.append("missing application install block")
    else:
        apps = installed_apps.group("body")
        for required in ("point_lio_node", "hybrid_slam_dataset_runner"):
            if required not in apps:
                failures.append(f"application install omits {required}")
        if "super_lio_node" in apps:
            failures.append("application install retains super_lio_node")

    standard_app = root / "src/hybrid_slam/src/apps/point_lio_node.cpp"
    if not standard_app.is_file():
        failures.append("missing Point-LIO application source")
    else:
        content = standard_app.read_text(encoding="utf-8")
        for required in (
            '"ros/parameter_bridge.h"',
            '"ros/sensor_bridge.h"',
            '"ros/publication_bridge.h"',
            "sensor_config.blind_squared = 0.0;",
            "sensor_config.max_range_squared = 1.0e12;",
            "sensor_config.filter_rate = 1;",
        ):
            if required not in content:
                failures.append(f"point_lio_node is missing {required}")
        for forbidden in (
            "ROSWrapper",
            "LegacyInputBridge",
            '"lio/params.h"',
            '"ros/legacy_parameter_bridge.h"',
        ):
            if forbidden in content:
                failures.append(f"point_lio_node retains {forbidden}")

    for relative in (
        "src/hybrid_slam/src/lio/point_lio_frontend.cpp",
        "src/hybrid_slam/include/lio/point_lio_frontend.h",
    ):
        path = root / relative
        if not path.is_file():
            failures.append(f"missing Point-LIO adapter file: {relative}")
            continue
        content = path.read_text(encoding="utf-8")
        for forbidden in (
            '"lio/params.h"',
            '"ros/legacy_global_parameter_bridge.h"',
            '"lio/super_lio.h"',
            "Implementation::",
        ):
            if forbidden in content:
                failures.append(f"Point-LIO adapter retains {forbidden}: {relative}")

    return failures


def main() -> int:
    root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path.cwd()
    failures = check(root)
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    print("PASS: Point-LIO-only core boundaries verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
