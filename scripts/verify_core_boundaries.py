#!/usr/bin/env python3
"""Verify that ROS-independent AdaGraph targets stay free of adapter headers.

The source lists below intentionally mirror the sources compiled by the
transitional CMake targets.  In particular, the vendored Point-LIO ROS node
and sensor adapters are not part of ``point_lio_vendor`` and must not make a
clean vendor-core check fail merely because they exist in the repository.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path
import sys


CORE_DIRS = (
    Path("src/super_lio/include/adagraph"),
    Path("src/super_lio/src/backend"),
    Path("src/super_lio/src/io"),
    Path("src/super_lio/src/localization"),
    Path("src/super_lio/src/mapping"),
    Path("src/super_lio/src/system/dataset_runner.cpp"),
    Path("src/super_lio/src/system/global_alignment_state.cpp"),
    Path("src/super_lio/src/system/slam_system.cpp"),
)

LEGACY_DOWNSTREAM_SOURCES = (
    Path("src/super_lio/src/system/legacy_downstream_pipeline.cpp"),
)

SUPER_LIO_FRONTEND_SOURCES = (
    Path("src/super_lio/src/lio/ESKF.cpp"),
    Path("src/super_lio/src/lio/super_lio.cpp"),
)

LEGACY_CONFIG_SOURCES = (
    Path("src/super_lio/src/lio/params.cpp"),
)

LEGACY_BACKEND_SOURCES = (
    Path("src/super_lio/src/lio/adagraph_backend.cpp"),
)

LEGACY_COMPAT_SOURCES = (
    Path("src/super_lio/src/lio/legacy_algorithm_config_adapter.cpp"),
)

LEGACY_GLOBAL_ROS_COMPAT_SOURCES = (
    Path("src/super_lio/src/ros/legacy_global_parameter_bridge.cpp"),
)

LEGACY_GLOBAL_ROS_COMPAT_HEADER = Path(
    "src/super_lio/include/ros/legacy_global_parameter_bridge.h"
)

POINT_LIO_VENDOR_SOURCES = (
    Path("src/super_lio/3rdparty/small_point_lio/src/small_point_lio/estimator.cpp"),
    Path("src/super_lio/3rdparty/small_point_lio/src/small_point_lio/parameters.cpp"),
    Path("src/super_lio/3rdparty/small_point_lio/src/small_point_lio/preprocess.cpp"),
    Path("src/super_lio/3rdparty/small_point_lio/src/small_point_lio/small_point_lio.cpp"),
    Path("src/super_lio/3rdparty/small_point_lio/src/util/voxelgrid_sampling.cpp"),
)

POINT_LIO_ADAPTER_SOURCES = (
    Path("src/super_lio/src/lio/point_lio_frontend.cpp"),
)

STANDARD_APPLICATION_SOURCES = (
    Path("src/super_lio/src/apps/super_lio_node.cpp"),
    Path("src/super_lio/src/apps/point_lio_node.cpp"),
)

BASIC_CORE_SOURCES = (
    Path("src/basic/src/Manifold.cpp"),
)

BASIC_LEGACY_SOURCES = (
    Path("src/basic/src/logs.cpp"),
)

FORBIDDEN_ROS_INCLUDE = re.compile(
    r"^[ \t]*#[ \t]*include[ \t]+(?:<|\")"
    r"(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|"
    r"visualization_msgs|tf2|tf2_ros|std_srvs|"
    r"livox_ros_driver2|ros/)",
    re.MULTILINE,
)

CORE_TARGETS = (
    "adagraph_mapping_core",
    "adagraph_map_io",
    "adagraph_localization_core",
    "adagraph_system_core",
    "adagraph_backend_core",
    "adagraph_isam2_optimizer",
)

ROS_DEPENDENCY = re.compile(
    r"\b(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|"
    r"visualization_msgs|tf2|tf2_ros|std_srvs|pcl_ros|"
    r"pcl_conversions|livox_ros_driver2)\b"
)


def source_files(root: Path) -> list[Path]:
    files: list[Path] = []
    for relative in CORE_DIRS:
        directory = root / relative
        if not directory.exists():
            continue
        if directory.is_file():
            files.append(directory)
        else:
            files.extend(
                path
                for path in directory.rglob("*")
                if path.suffix in {".h", ".hpp", ".cpp", ".cc", ".cxx"}
            )
    return sorted(files)


def target_block(cmake: str, target: str) -> str | None:
    match = re.search(
        rf"target_link_libraries\(\s*{re.escape(target)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def target_compile_definitions_block(cmake: str, target: str) -> str | None:
    match = re.search(
        rf"target_compile_definitions\(\s*{re.escape(target)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def add_library_block(cmake: str, target: str) -> str | None:
    match = re.search(
        rf"add_library\(\s*{re.escape(target)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def project_library_block(cmake: str, project_name: str) -> str | None:
    """Match add_library(${PROJECT_NAME} ...) after project() indirection."""
    match = re.search(
        rf"add_library\(\s*\$\{{PROJECT_NAME\}}(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def project_target_block(cmake: str, project_name: str) -> str | None:
    """Match target_link_libraries(${PROJECT_NAME} ...) after project()."""
    match = re.search(
        rf"target_link_libraries\(\s*\$\{{PROJECT_NAME\}}(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def variable_block(cmake: str, variable: str) -> str | None:
    match = re.search(
        rf"set\(\s*{re.escape(variable)}\b(?P<body>.*?)\)",
        cmake,
        re.DOTALL,
    )
    return match.group("body") if match else None


def expanded_add_library_block(cmake: str, target: str) -> str:
    """Return an add_library body with referenced local source lists expanded."""
    body = add_library_block(cmake, target) or ""
    expanded = [body]
    for variable in re.findall(r"\$\{([^}]+)\}", body):
        value = variable_block(cmake, variable)
        if value is not None:
            expanded.append(value)
    return "\n".join(expanded)


def check_compiled_sources(
    root: Path, cmake: str, basic_cmake: str, failures: list[str]
) -> None:
    expected = {
        "adagraph_super_lio_frontend": SUPER_LIO_FRONTEND_SOURCES,
        "adagraph_legacy_config": LEGACY_CONFIG_SOURCES,
        "adagraph_legacy_backend": LEGACY_BACKEND_SOURCES,
        "adagraph_legacy_compat": LEGACY_COMPAT_SOURCES,
        "adagraph_legacy_global_ros_compat": LEGACY_GLOBAL_ROS_COMPAT_SOURCES,
        "point_lio_vendor": POINT_LIO_VENDOR_SOURCES,
        "point_lio_adapter": POINT_LIO_ADAPTER_SOURCES,
    }
    for target, sources in expected.items():
        body = expanded_add_library_block(cmake, target)
        if body is None:
            failures.append(f"missing add_library definition for {target}")
            continue
        for source in sources:
            cmake_source = source.relative_to(Path("src/super_lio")).as_posix()
            if cmake_source not in body:
                failures.append(
                    f"{cmake_source} is not explicitly compiled into {target}"
                )
            path = root / source
            if not path.is_file():
                failures.append(f"compiled source is missing: {source}")

        for source in sources:
            path = root / source
            if not path.is_file():
                continue
            content = path.read_text(encoding="utf-8")
            if target != "adagraph_legacy_global_ros_compat" and \
                    FORBIDDEN_ROS_INCLUDE.search(content):
                failures.append(
                    f"forbidden ROS include in {source} ({target})"
                )

    # Phase 8 keeps the legacy algorithms and ROS/downstream composition as
    # separate explicit targets.  This is intentionally a structural check:
    # both targets still contain transitional legacy code, but the algorithm
    # archive must not compile ROSWrapper/publisher/downstream sources and the
    # ROS composition archive must list those sources explicitly.
    legacy_algorithms = expanded_add_library_block(
        cmake, "adagraph_legacy_algorithms"
    )
    if not legacy_algorithms:
        failures.append("missing add_library definition for adagraph_legacy_algorithms")
    else:
        for source in (
            "src/lio/adagraph_feature_extractor.cpp",
            "src/lio/kiss_matcher_relocalizer.cpp",
            "src/lio/pcl_relocalizer_adapters.cpp",
            "src/lio/prior_map_localization.cpp",
        ):
            if source not in legacy_algorithms:
                failures.append(
                    f"{source} is not explicitly compiled into "
                    "adagraph_legacy_algorithms"
                )
        for forbidden in (
            "src/lio/adagraph_backend.cpp",
            "src/lio/params.cpp",
            "src/lio/legacy_algorithm_config_adapter.cpp",
            "src/ros/ROSWrapper.cpp",
            "src/ros/occupancy_grid_publisher.cpp",
            "src/system/legacy_downstream_pipeline.cpp",
        ):
            if forbidden in legacy_algorithms:
                failures.append(
                    f"ROS/downstream source {forbidden} leaked into "
                    "adagraph_legacy_algorithms"
                )

    legacy_backend = expanded_add_library_block(
        cmake, "adagraph_legacy_backend"
    )
    if not legacy_backend:
        failures.append("missing add_library definition for adagraph_legacy_backend")
    else:
        if "src/lio/adagraph_backend.cpp" not in legacy_backend:
            failures.append(
                "adagraph_backend.cpp is not explicitly compiled into "
                "adagraph_legacy_backend"
            )
        if "adagraph_legacy_algorithms" not in (
            target_block(cmake, "adagraph_legacy_backend") or ""
        ):
            failures.append(
                "adagraph_legacy_backend must link the legacy algorithm adapters"
            )

    legacy_compat = expanded_add_library_block(cmake, "adagraph_legacy_compat")
    if not legacy_compat:
        failures.append("missing add_library definition for adagraph_legacy_compat")
    else:
        if "src/lio/legacy_algorithm_config_adapter.cpp" not in legacy_compat:
            failures.append(
                "legacy config adapter is not explicitly compiled into "
                "adagraph_legacy_compat"
            )
        if "adagraph_legacy_config" not in (target_block(cmake, "adagraph_legacy_compat") or ""):
            failures.append(
                "adagraph_legacy_compat must link adagraph_legacy_config"
            )

    global_ros_compat = expanded_add_library_block(
        cmake, "adagraph_legacy_global_ros_compat"
    )
    if not global_ros_compat:
        failures.append(
            "missing add_library definition for adagraph_legacy_global_ros_compat"
        )
    else:
        if "src/ros/legacy_global_parameter_bridge.cpp" not in global_ros_compat:
            failures.append(
                "legacy global parameter source is not explicitly compiled into "
                "adagraph_legacy_global_ros_compat"
            )
        definitions = target_compile_definitions_block(
            cmake, "adagraph_legacy_global_ros_compat"
        ) or ""
        if "ADAGRAPH_ENABLE_LEGACY_GLOBAL_PARAMETER_BRIDGE" in definitions:
            failures.append(
                "adagraph_legacy_global_ros_compat must use the dedicated "
                "global parameter translation unit rather than a macro-switched "
                "duplicate source"
            )
        if "adagraph_legacy_config" not in (
            target_block(cmake, "adagraph_legacy_global_ros_compat") or ""
        ):
            failures.append(
                "adagraph_legacy_global_ros_compat must link "
                "adagraph_legacy_config"
            )

    global_bridge_header = root / LEGACY_GLOBAL_ROS_COMPAT_HEADER
    if not global_bridge_header.is_file():
        failures.append(
            "missing dedicated legacy global parameter bridge header"
        )
    else:
        header_content = global_bridge_header.read_text(encoding="utf-8")
        for required in ("LoadLegacyParametersFromRos", "LoadParamFromRos"):
            if required not in header_content:
                failures.append(
                    "legacy global parameter bridge header is missing "
                    f"{required}"
                )

    instance_parameter_source = root / (
        "src/super_lio/src/ros/legacy_parameter_bridge.cpp"
    )
    instance_parameter_header = root / (
        "src/super_lio/include/ros/legacy_parameter_bridge.h"
    )
    for path in (instance_parameter_source, instance_parameter_header):
        if not path.is_file():
            continue
        content = path.read_text(encoding="utf-8")
        for forbidden in (
            "legacy_global_parameter_bridge.h",
            "LoadLegacyParametersFromRos",
            "LoadParamFromRos",
            '"lio/params.h"',
        ):
            if forbidden in content:
                failures.append(
                    f"instance-owned parameter bridge retains global API/state: "
                    f"{forbidden} in {path.relative_to(root)}"
                )

    global_parameter_source = root / (
        "src/super_lio/src/ros/legacy_global_parameter_bridge.cpp"
    )
    if global_parameter_source.is_file():
        content = global_parameter_source.read_text(encoding="utf-8")
        if '"ros/legacy_global_parameter_bridge.h"' not in content:
            failures.append(
                "global parameter implementation must include its dedicated header"
            )

    ros_publication = expanded_add_library_block(cmake, "adagraph_ros_publication")
    if not ros_publication:
        failures.append("missing add_library definition for adagraph_ros_publication")
    else:
        for source in (
            "src/ros/ground_segmentation_filter.cpp",
            "src/ros/occupancy_grid_publisher.cpp",
            "src/ros/publication_bridge.cpp",
        ):
            if source not in ros_publication:
                failures.append(
                    f"{source} is not explicitly compiled into "
                    "adagraph_ros_publication"
                )
        if "src/ros/ROSWrapper.cpp" in ros_publication:
            failures.append(
                "ROSWrapper.cpp leaked into adagraph_ros_publication"
            )

    ros_composition = expanded_add_library_block(cmake, "adagraph_ros_composition")
    if not ros_composition:
        failures.append("missing add_library definition for adagraph_ros_composition")
    else:
        if "src/ros/ROSWrapper.cpp" not in ros_composition:
            failures.append(
                "src/ros/ROSWrapper.cpp is not explicitly compiled into "
                "adagraph_ros_composition"
            )
        if "src/system/legacy_downstream_pipeline.cpp" in ros_composition:
            failures.append(
                "ROS/downstream source src/system/legacy_downstream_pipeline.cpp "
                "leaked into adagraph_ros_composition"
            )

    # The compatibility ROS facade may retain output and parameter APIs, but
    # input ownership is centralized in LegacyInputBridge.  ROSWrapper must
    # not grow a second sensor buffer, subscription callback, initial-pose
    # capture or estimator-prediction queue. Prediction is an explicit
    # application-context pump, rather than a callback-owned estimator action.
    ros_wrapper = root / "src/super_lio/src/ros/ROSWrapper.cpp"
    ros_wrapper_header = root / "src/super_lio/include/ros/ROSWrapper.h"
    if ros_wrapper.is_file() and ros_wrapper_header.is_file():
        wrapper_content = ros_wrapper.read_text(encoding="utf-8")
        wrapper_header_content = ros_wrapper_header.read_text(encoding="utf-8")
        if '"ros/legacy_input_bridge.h"' not in wrapper_content:
            failures.append(
                "ROSWrapper must include the centralized LegacyInputBridge"
            )
        for forbidden in (
            "SensorBridge",
            "sensor_bridge_",
            "handleImuMeasurement",
            "pushLidarData",
            "std::deque<IMUData>",
            "pending_imu_prediction_buffer_",
            "create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>",
        ):
            if forbidden in wrapper_content or forbidden in wrapper_header_content:
                failures.append(
                    f"ROSWrapper retains duplicate compatibility input implementation: {forbidden}"
                )
        if "legacy_input_bridge_" not in wrapper_header_content or \
                "legacy_input_bridge_" not in wrapper_content:
            failures.append(
                "ROSWrapper must delegate compatibility input ownership to "
                "legacy_input_bridge_"
            )
        if "legacy_input_bridge_->processPendingImuPredictions()" not in wrapper_content:
            failures.append(
                "ROSWrapper lacks the explicit LegacyInputBridge prediction pump"
            )
        if not re.search(
            r"legacy_input_bridge_->sync_measure\s*\(\s*\w+\s*\)",
            wrapper_content,
        ):
            failures.append(
                "ROSWrapper sync_measure must delegate to LegacyInputBridge"
            )
        if "legacy_input_bridge_->setLegacyImuPredictor" not in wrapper_content:
            failures.append(
                "ROSWrapper predictor setter must delegate to LegacyInputBridge"
            )

    legacy_input_header = root / "src/super_lio/include/ros/legacy_input_bridge.h"
    legacy_input_source = root / "src/super_lio/src/ros/legacy_input_bridge.cpp"
    if legacy_input_header.is_file() and legacy_input_source.is_file():
        legacy_input_content = legacy_input_source.read_text(encoding="utf-8")
        if "processPendingImuPredictions" not in legacy_input_content:
            failures.append(
                "LegacyInputBridge lacks an explicit IMU prediction processing pump"
            )
        handle_match = re.search(
            r"void\s+LegacyInputBridge::handleImuMeasurement\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
            legacy_input_content,
            re.DOTALL,
        )
        if not handle_match:
            failures.append("LegacyInputBridge::handleImuMeasurement is missing")
        elif re.search(
            r"\b(?:imu_predictor_|processPendingImuPredictions|Predict)\b",
            handle_match.group("body"),
        ):
            failures.append(
                "LegacyInputBridge IMU callback directly schedules prediction"
            )

    # Standard applications must use the explicit ROS output/input adapters.
    # ROSWrapper remains available for parameters and compatibility services,
    # but its legacy publisher/occupancy ownership must not be rebound by a
    # standard node.
    for source in STANDARD_APPLICATION_SOURCES:
        path = root / source
        if not path.is_file():
            failures.append(f"standard application source is missing: {source}")
            continue
        content = path.read_text(encoding="utf-8")
        for required in (
            '"ros/legacy_parameter_bridge.h"',
            '"ros/sensor_bridge.h"',
            '"ros/publication_bridge.h"',
            "SensorBridge",
            "PublicationBridge",
        ):
            if required not in content:
                failures.append(
                    f"{source} does not use the explicit ROS adapter {required}"
                )
        if re.search(
            r"(?:data_wrapper|wrapper)->"
            r"(?:pub_|set_global_map|set_prior_map_localization_active|"
            r"set_map_to_odom|get_map_to_odom|update_occupancy_grid|"
            r"occupancy_grid_enabled)",
            content,
        ):
            failures.append(
                f"{source} rebinds a legacy ROSWrapper output/occupancy interface"
            )
        if re.search(r'"ros/ROSWrapper\.h"|\bROSWrapper\b', content):
            failures.append(
                f"{source} retains a direct ROSWrapper dependency"
            )

        # Point-LIO owns its vendor-side range and sampling semantics.  The
        # shared YAML may still contain lio.sensor.* values for Super-LIO, but
        # point_lio_node must configure SensorBridge losslessly and read only
        # point_lio.input.* for DDS transport policy.  Otherwise the ROS
        # adapter silently filters a scan before SmallPointLio sees it.
        if source == Path("src/super_lio/src/apps/point_lio_node.cpp"):
            for required in (
                "sensor_config.blind_squared = 0.0;",
                "sensor_config.max_range_squared = 1.0e12;",
                "sensor_config.filter_rate = 1;",
                '"point_lio.input.imu_queue_depth"',
                '"point_lio.input.lidar_queue_depth"',
                '"point_lio.input.reliable"',
            ):
                if required not in content:
                    failures.append(
                        f"{source} is missing explicit Point-LIO transport "
                        f"boundary: {required}"
                    )
            for forbidden in (
                '"lio.sensor.blind"',
                '"lio.sensor.maxrange"',
                '"lio.sensor.filter_rate"',
            ):
                if forbidden in content:
                    failures.append(
                        f"{source} must not read Super-LIO transport filter "
                        f"parameter {forbidden}"
                    )

    legacy_downstream = expanded_add_library_block(
        cmake, "adagraph_legacy_downstream"
    )
    if not legacy_downstream:
        failures.append("missing add_library definition for adagraph_legacy_downstream")
    else:
        for source in LEGACY_DOWNSTREAM_SOURCES:
            cmake_source = source.relative_to(Path("src/super_lio")).as_posix()
            if cmake_source not in legacy_downstream:
                failures.append(
                    f"{cmake_source} is not explicitly compiled into "
                    "adagraph_legacy_downstream"
                )
            path = root / source
            if not path.is_file():
                failures.append(f"compiled source is missing: {source}")
            elif FORBIDDEN_ROS_INCLUDE.search(path.read_text(encoding="utf-8")):
                failures.append(
                    f"forbidden ROS include in {source} "
                    "(adagraph_legacy_downstream)"
                )
            else:
                content = path.read_text(encoding="utf-8")
                if '"lio/params.h"' in content or \
                        "MakeLegacyRuntimeConfigFromGlobals" in content:
                    failures.append(
                        "legacy downstream must consume an explicit "
                        "LegacyRuntimeConfig rather than process-wide g_* state"
                    )
                if re.search(r'lio/adagraph_backend\.h|\bAdaGraphBackend\b', content):
                    failures.append(
                        "standard legacy downstream must not include or instantiate "
                        "the duplicate AdaGraph backend"
                    )
        if ROS_DEPENDENCY.search(legacy_downstream):
            failures.append("ROS/transport dependency leaked into adagraph_legacy_downstream")

    # The compatibility lio target is now an interface-only alias.  It must
    # not silently regain an implementation source list during migration.
    lio_body = add_library_block(cmake, "lio") or ""
    if lio_body.strip() and not re.search(r"\bINTERFACE\b", lio_body):
        failures.append("lio compatibility target must remain interface-only")

    adapter_links = target_block(cmake, "point_lio_adapter") or ""
    if re.search(r"\blio\b", adapter_links):
        failures.append("point_lio_adapter must not link the monolithic lio target")
    if "basic::basic_core" not in adapter_links:
        failures.append(
            "point_lio_adapter must link the lightweight basic::basic_core target"
        )
    for forbidden in (
        "${basic_LIBRARIES}",
        "adagraph_legacy_config",
        "adagraph_legacy_global_ros_compat",
        "adagraph_ros_composition",
    ):
        if forbidden in adapter_links:
            failures.append(
                "point_lio_adapter must not link legacy/PCL/glog compatibility "
                f"surface: {forbidden}"
            )

    point_adapter_source = root / "src/super_lio/src/lio/point_lio_frontend.cpp"
    point_adapter_header = root / "src/super_lio/include/lio/point_lio_frontend.h"
    for path in (point_adapter_source, point_adapter_header):
        if not path.is_file():
            failures.append(f"missing Point-LIO adapter file: {path.relative_to(root)}")
            continue
        content = path.read_text(encoding="utf-8")
        for forbidden in (
            '"lio/params.h"',
            '"ros/legacy_global_parameter_bridge.h"',
            '"basic/alias.h"',
            '"basic/logs.h"',
            '"common/ds.h"',
            "#include <pcl",
            "#include <glog",
        ):
            if forbidden in content:
                failures.append(
                    "Point-LIO adapter must stay free of legacy global/PCL/glog "
                    f"dependency {forbidden} in {path.relative_to(root)}"
                )
    if point_adapter_header.is_file() and (
        "small_point_lio" in point_adapter_header.read_text(encoding="utf-8")
    ):
        failures.append(
            "Point-LIO public frontend header must not expose the vendor API"
        )

    ros_adapter_links = target_block(cmake, "adagraph_ros2_adapter") or ""
    if re.search(r"\badagraph_legacy_config\b", ros_adapter_links):
        failures.append(
            "adagraph_ros2_adapter must not link the legacy global config target"
        )

    # Frontend implementations are selected by applications, not smuggled
    # through the transitional downstream composition. This is especially
    # important for point_lio_node: linking Super-LIO here creates unresolved
    # ESKF/global-parameter references in an otherwise Point-LIO executable.
    lio_body = add_library_block(cmake, "lio") or ""
    for forbidden in (
        "src/lio/ESKF.cpp",
        "src/lio/super_lio.cpp",
    ):
        if forbidden in lio_body:
            failures.append(f"lio must not compile frontend source {forbidden}")
    lio_links = target_block(cmake, "lio") or ""
    if not re.search(r"\badagraph_legacy_compat\b", lio_links):
        failures.append("lio compatibility target must link adagraph_legacy_compat")
    if re.search(r"\badagraph_super_lio_frontend\b", lio_links):
        failures.append("lio must not link adagraph_super_lio_frontend")

    super_frontend_links = target_block(
        cmake, "adagraph_super_lio_frontend"
    ) or ""
    if re.search(r"\badagraph_map_io\b", super_frontend_links):
        failures.append(
            "adagraph_super_lio_frontend must not link compatibility map IO"
        )

    super_node_links = target_block(cmake, "super_lio_node") or ""
    if not re.search(r"\badagraph_super_lio_frontend\b", super_node_links):
        failures.append(
            "super_lio_node must explicitly link adagraph_super_lio_frontend"
        )
    point_node_links = target_block(cmake, "point_lio_node") or ""
    if re.search(r"\badagraph_super_lio_frontend\b", point_node_links):
        failures.append("point_lio_node must not link adagraph_super_lio_frontend")
    if re.search(r"\badagraph_legacy_global_ros_compat\b", point_node_links):
        failures.append(
            "point_lio_node must not link the legacy global parameter bridge"
        )
    # The old AdaGraph facade is an explicit rollback/compatibility surface.
    # Standard applications and the ROS-free downstream must use the typed
    # BackendScheduler/MapManager path and must not acquire the second graph
    # transitively.
    dataset_runner_links = target_block(cmake, "adagraph_dataset_runner") or ""
    for target_name, links in (
        ("adagraph_legacy_downstream", target_block(cmake, "adagraph_legacy_downstream") or ""),
        ("super_lio_node", super_node_links),
        ("point_lio_node", point_node_links),
        ("adagraph_dataset_runner", dataset_runner_links),
    ):
        if re.search(r"\badagraph_legacy_backend\b", links):
            failures.append(
                f"{target_name} must not link the legacy AdaGraph facade; "
                "use the typed backend/map snapshot path"
            )
    for target_name, links in (
        ("super_lio_node", super_node_links),
        ("point_lio_node", point_node_links),
    ):
        if re.search(r"\badagraph_ros_composition\b", links):
            failures.append(
                f"{target_name} must not link compatibility "
                "adagraph_ros_composition"
            )

    basic_core_body = add_library_block(basic_cmake, "basic_core")
    basic_core_sources = variable_block(basic_cmake, "BASIC_CORE_SOURCES")
    if basic_core_body is None:
        failures.append("missing add_library definition for basic_core")
    if basic_core_sources is None:
        failures.append("missing explicit BASIC_CORE_SOURCES list")
    else:
        for source in BASIC_CORE_SOURCES:
            cmake_source = source.relative_to(Path("src/basic")).as_posix()
            if cmake_source not in basic_core_sources:
                failures.append(
                    f"{cmake_source} is not explicitly compiled into basic_core"
                )
    core_links = target_block(basic_cmake, "basic_core") or ""
    for forbidden in ("PCL", "glog", "gflags"):
        if forbidden in core_links:
            failures.append(
                f"basic_core must not link legacy PCL/glog surface: {forbidden}"
            )

    basic_body = project_library_block(basic_cmake, "basic")
    basic_legacy_sources = variable_block(basic_cmake, "BASIC_LEGACY_SOURCES")
    if basic_body is None:
        failures.append("missing add_library definition for basic compatibility facade")
    if basic_legacy_sources is None:
        failures.append("missing explicit BASIC_LEGACY_SOURCES list")
    else:
        for source in BASIC_LEGACY_SOURCES:
            cmake_source = source.relative_to(Path("src/basic")).as_posix()
            if cmake_source not in basic_legacy_sources:
                failures.append(
                    f"{cmake_source} is not explicitly compiled into basic facade"
                )
    basic_links = project_target_block(basic_cmake, "basic") or ""
    if "basic_core" not in basic_links:
        failures.append("basic compatibility facade must link basic_core")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()

    failures: list[str] = []
    for path in source_files(root):
        content = path.read_text(encoding="utf-8")
        if FORBIDDEN_ROS_INCLUDE.search(content):
            failures.append(
                f"forbidden adapter include in {path.relative_to(root)}"
            )

    cmake_path = root / "src/super_lio/CMakeLists.txt"
    cmake = cmake_path.read_text(encoding="utf-8")
    retired_relocation_paths = (
        Path("src/super_lio/include/lio/super_lio_reloc.h"),
        Path("src/super_lio/src/lio/super_lio_reloc.cpp"),
        Path("src/super_lio/src/apps/relocation_node.cpp"),
        Path("src/super_lio/include/lio/system/legacy_relocation_config.h"),
        Path("src/super_lio/config/relocation.yaml"),
        Path("src/super_lio/launch/relocation.py"),
        Path("src/super_lio/rviz/relocation.rviz"),
    )
    for retired_path in retired_relocation_paths:
        if (root / retired_path).exists():
            failures.append(f"retired relocation path was reintroduced: {retired_path}")
    for retired_target in (
        "adagraph_relocation_compat",
        "relocation_node",
        "super_lio_reloc.cpp",
    ):
        if retired_target in cmake:
            failures.append(
                f"retired relocation target/source was reintroduced: {retired_target}"
            )
    if re.search(r"(?m)^\s*add_definitions\([^\n]*\bROOT\b", cmake):
        failures.append(
            "ROOT must not be exported through a directory-wide add_definitions"
        )
    for target in ("adagraph_legacy_config", "adagraph_ros2_adapter"):
        definitions = target_compile_definitions_block(cmake, target) or ""
        if not re.search(r"\bROOT=", definitions):
            failures.append(
                f"{target} must carry the compatibility-only ROOT definition"
            )
    if 'option(SUPER_LIO_ENABLE_KISS_MATCHER' not in cmake:
        failures.append(
            "CMake must expose SUPER_LIO_ENABLE_KISS_MATCHER for offline "
            "clean builds without the optional network dependency"
        )
    basic_cmake = (root / "src/basic/CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    for target in CORE_TARGETS:
        body = target_block(cmake, target)
        if body is None:
            failures.append(f"missing target_link_libraries for {target}")
            continue
        if ROS_DEPENDENCY.search(body):
            failures.append(f"ROS/transport dependency leaked into {target}")

    check_compiled_sources(root, cmake, basic_cmake, failures)

    basic_package = (root / "src/basic/package.xml").read_text(
        encoding="utf-8"
    )
    if re.search(
        r"\b(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|"
        r"visualization_msgs|tf2|tf2_ros|std_srvs|livox_ros_driver2)\b",
        basic_cmake,
    ):
        failures.append("ROS dependency leaked into src/basic/CMakeLists.txt")
    if re.search(
        r"<(?:depend|build_depend|exec_depend|test_depend)>"
        r"[^<]*(?:rclcpp|sensor_msgs|geometry_msgs|nav_msgs|"
        r"visualization_msgs|tf2|tf2_ros|std_srvs|livox_ros_driver2)",
        basic_package,
    ):
        failures.append("ROS dependency leaked into src/basic/package.xml")

    # The optimizer adapter is intentionally GTSAM-only; it must remain a
    # separate target instead of making the generic backend core depend on it.
    backend_body = target_block(cmake, "adagraph_backend_core") or ""
    if re.search(r"\bgtsam\b", backend_body):
        failures.append("GTSAM dependency leaked into adagraph_backend_core")

    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(
        f"core boundary verification passed: {len(source_files(root))} files, "
        f"{len(CORE_TARGETS) + 7} core/frontend targets"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
