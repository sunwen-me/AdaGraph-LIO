#!/usr/bin/env python3

"""Run named SLAM regression cases and aggregate their JSON summaries."""

import argparse
import datetime
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    default_root = script_dir.parent
    timestamp = datetime.datetime.now(datetime.timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ"
    )

    parser = argparse.ArgumentParser(
        description="Run named AdaGraph-LIO regression cases."
    )
    parser.add_argument(
        "cases",
        nargs="*",
        help="Case IDs to run; omit to run every manifest case.",
    )
    parser.add_argument(
        "--manifest",
        type=Path,
        default=script_dir / "slam_regression_cases.json",
    )
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=default_root,
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=default_root / "log" / "refactoring_baselines" / f"phase0_{timestamp}",
    )
    return parser.parse_args()


def load_manifest(path: Path) -> Dict[str, object]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError(f"unsupported manifest schema in {path}")
    if not isinstance(data.get("cases"), list):
        raise ValueError(f"manifest has no case list: {path}")
    return data


def select_cases(
    manifest_cases: Sequence[Dict[str, object]], requested: Sequence[str]
) -> List[Dict[str, object]]:
    by_id = {str(case["id"]): case for case in manifest_cases}
    if not requested:
        return list(manifest_cases)

    missing = [case_id for case_id in requested if case_id not in by_id]
    if missing:
        raise ValueError("unknown regression cases: " + ", ".join(missing))
    return [by_id[case_id] for case_id in requested]


def evaluate_acceptance(
    case: Dict[str, object], summary: Dict[str, object]
) -> List[str]:
    acceptance = case.get("acceptance", {})
    if not isinstance(acceptance, dict):
        return ["manifest acceptance field is not an object"]

    trajectory = summary.get("trajectory", {})
    ape = summary.get("ape_translation_m", {})
    if not isinstance(trajectory, dict) or not isinstance(ape, dict):
        return ["summary is missing trajectory or APE objects"]

    failures: List[str] = []
    pose_count = int(trajectory.get("pose_count", 0))
    path_length = float(trajectory.get("path_length_m", 0.0))

    minimum_pose_count = acceptance.get("minimum_pose_count")
    if minimum_pose_count is not None and pose_count < int(minimum_pose_count):
        failures.append(
            f"pose_count {pose_count} < minimum {int(minimum_pose_count)}"
        )

    minimum_path_length = acceptance.get("minimum_path_length_m")
    if minimum_path_length is not None and path_length < float(minimum_path_length):
        failures.append(
            f"path_length_m {path_length:.6f} < minimum "
            f"{float(minimum_path_length):.6f}"
        )

    maximum_path_length = acceptance.get("maximum_path_length_m")
    if maximum_path_length is not None and path_length > float(maximum_path_length):
        failures.append(
            f"path_length_m {path_length:.6f} > maximum "
            f"{float(maximum_path_length):.6f}"
        )

    maximum_ape = acceptance.get("maximum_ape_rmse_m")
    ape_rmse = ape.get("rmse")
    if maximum_ape is not None:
        if ape_rmse is None:
            failures.append("APE RMSE is missing")
        elif float(ape_rmse) > float(maximum_ape):
            failures.append(
                f"ape_rmse_m {float(ape_rmse):.6f} > maximum "
                f"{float(maximum_ape):.6f}"
            )

    if acceptance.get("require_backend_chain_consistency", False):
        diagnostics = summary.get("diagnostics", {})
        scheduler = (
            diagnostics.get("backend_scheduler", {})
            if isinstance(diagnostics, dict)
            else {}
        )
        if not isinstance(scheduler, dict) or not scheduler:
            failures.append("backend scheduler diagnostics are missing")
        else:
            enqueued = int(scheduler.get("enqueued", -1))
            processed = int(scheduler.get("processed", -1))
            failed = int(scheduler.get("failed", -1))
            node_count = int(scheduler.get("node_count", -1))
            constraint_count = int(scheduler.get("constraint_count", -1))
            expected_constraints = max(0, node_count - 1)
            if enqueued != processed or failed != 0:
                failures.append(
                    "backend scheduler did not reliably drain: "
                    f"enqueued/processed/failed={enqueued}/{processed}/{failed}"
                )
            if node_count != enqueued:
                failures.append(
                    f"backend graph nodes {node_count} != accepted items {enqueued}"
                )
            if constraint_count != expected_constraints:
                failures.append(
                    "backend odometry chain is inconsistent: "
                    f"constraints={constraint_count}, expected={expected_constraints}"
                )
            keyframe_count = (
                int(diagnostics.get("keyframe_count", -1))
                if isinstance(diagnostics, dict)
                else -1
            )
            if keyframe_count != node_count:
                failures.append(
                    f"legacy keyframes {keyframe_count} != graph nodes {node_count}"
                )
            snapshot_version = int(
                scheduler.get("map_snapshot_version", -1)
            )
            if snapshot_version != node_count:
                failures.append(
                    f"map snapshot version {snapshot_version} != graph nodes "
                    f"{node_count} for the odometry-only regression"
                )
            submap_count = int(scheduler.get("submap_count", -1))
            finished_submap_count = int(
                scheduler.get("finished_submap_count", -1)
            )
            if node_count > 0 and not (
                submap_count > 0
                and 0 <= finished_submap_count <= submap_count
            ):
                failures.append(
                    "submap lifecycle diagnostics are inconsistent: "
                    f"submaps/finished={submap_count}/{finished_submap_count}"
                )
            snapshot_observer_failures = int(
                scheduler.get("snapshot_observer_failures", -1)
            )
            if snapshot_observer_failures != 0:
                failures.append(
                    "snapshot observer handoff failed: "
                    f"failures={snapshot_observer_failures}"
                )
            optimizer_updates = int(scheduler.get("optimizer_updates", -1))
            optimizer_failures = int(
                scheduler.get("optimizer_failures", -1)
            )
            if optimizer_updates != node_count or optimizer_failures != 0:
                failures.append(
                    "optimizer update chain is inconsistent: "
                    f"updates/failures={optimizer_updates}/{optimizer_failures}, "
                    f"nodes={node_count}"
                )

            maximum_optimizer_translation_delta = acceptance.get(
                "maximum_optimizer_translation_delta_m"
            )
            actual_translation_delta = scheduler.get(
                "maximum_optimizer_translation_delta_m"
            )
            if maximum_optimizer_translation_delta is not None:
                if actual_translation_delta is None:
                    failures.append("optimizer translation parity metric is missing")
                elif float(actual_translation_delta) > float(
                    maximum_optimizer_translation_delta
                ):
                    failures.append(
                        "optimizer translation delta "
                        f"{float(actual_translation_delta):.9g} > maximum "
                        f"{float(maximum_optimizer_translation_delta):.9g}"
                    )

            maximum_optimizer_rotation_delta = acceptance.get(
                "maximum_optimizer_rotation_delta_rad"
            )
            actual_rotation_delta = scheduler.get(
                "maximum_optimizer_rotation_delta_rad"
            )
            if maximum_optimizer_rotation_delta is not None:
                if actual_rotation_delta is None:
                    failures.append("optimizer rotation parity metric is missing")
                elif float(actual_rotation_delta) > float(
                    maximum_optimizer_rotation_delta
                ):
                    failures.append(
                        "optimizer rotation delta "
                        f"{float(actual_rotation_delta):.9g} > maximum "
                        f"{float(maximum_optimizer_rotation_delta):.9g}"
                    )

    if acceptance.get("require_global_alignment_consistency", False):
        diagnostics = summary.get("diagnostics", {})
        alignment = (
            diagnostics.get("global_alignment", {})
            if isinstance(diagnostics, dict)
            else {}
        )
        queue = (
            diagnostics.get("global_alignment_queue", {})
            if isinstance(diagnostics, dict)
            else {}
        )
        if not isinstance(alignment, dict) or not alignment:
            failures.append("global alignment diagnostics are missing")
        if not isinstance(queue, dict) or not queue:
            failures.append("global alignment queue diagnostics are missing")
        if isinstance(alignment, dict) and alignment and isinstance(queue, dict) and queue:
            version = int(alignment.get("version", -1))
            committed = int(queue.get("committed", -1))
            rejected = int(queue.get("rejected", -1))
            if version != committed:
                failures.append(
                    "global alignment version/commit mismatch: "
                    f"version={version}, committed={committed}"
                )
            if rejected != 0:
                failures.append(
                    f"global alignment updates rejected={rejected}"
                )
            maximum_translation_delta = float(
                acceptance.get(
                    "maximum_ros_alignment_translation_delta_m", 1e-9
                )
            )
            maximum_rotation_delta = float(
                acceptance.get(
                    "maximum_ros_alignment_rotation_delta_rad", 1e-8
                )
            )
            translation_delta = float(
                alignment.get("ros_cache_translation_delta_m", float("inf"))
            )
            rotation_delta = float(
                alignment.get("ros_cache_rotation_delta_rad", float("inf"))
            )
            if translation_delta > maximum_translation_delta:
                failures.append(
                    "ROS alignment translation cache delta "
                    f"{translation_delta:.9g} > maximum "
                    f"{maximum_translation_delta:.9g}"
                )
            if rotation_delta > maximum_rotation_delta:
                failures.append(
                    "ROS alignment rotation cache delta "
                    f"{rotation_delta:.9g} > maximum "
                    f"{maximum_rotation_delta:.9g}"
                )

    occupancy_acceptance = acceptance.get("occupancy")
    if occupancy_acceptance is not None:
        if not isinstance(occupancy_acceptance, dict):
            failures.append("occupancy acceptance field is not an object")
        else:
            occupancy = summary.get("occupancy", {})
            if not isinstance(occupancy, dict) or not occupancy:
                failures.append("occupancy summary is missing")
            elif occupancy.get("status") != "received":
                failures.append(
                    "occupancy topic did not produce a message: "
                    f"status={occupancy.get('status')}"
                )
            else:
                expected_topic = occupancy_acceptance.get("topic")
                if expected_topic is not None and occupancy.get("topic") != expected_topic:
                    failures.append(
                        "occupancy topic mismatch: "
                        f"{occupancy.get('topic')} != {expected_topic}"
                    )
                expected_frame = occupancy_acceptance.get("frame_id")
                if expected_frame is not None and occupancy.get("frame_id") != expected_frame:
                    failures.append(
                        "occupancy frame mismatch: "
                        f"{occupancy.get('frame_id')} != {expected_frame}"
                    )
                minimum_width = occupancy_acceptance.get("minimum_width", 1)
                minimum_height = occupancy_acceptance.get("minimum_height", 1)
                if int(occupancy.get("width", 0)) < int(minimum_width):
                    failures.append(
                        f"occupancy width {occupancy.get('width')} < minimum {minimum_width}"
                    )
                if int(occupancy.get("height", 0)) < int(minimum_height):
                    failures.append(
                        f"occupancy height {occupancy.get('height')} < minimum {minimum_height}"
                    )
                semantics = occupancy.get("cell_semantics", {})
                if not isinstance(semantics, dict):
                    failures.append("occupancy cell semantics are missing")
                else:
                    for key in (
                        "occupied_positive",
                        "free_zero",
                        "unknown_negative",
                    ):
                        if key not in semantics:
                            failures.append(
                                f"occupancy cell semantics missing {key}"
                            )

    occupancy_rebuild_acceptance = acceptance.get("occupancy_snapshot_rebuild")
    if occupancy_rebuild_acceptance is not None:
        if not isinstance(occupancy_rebuild_acceptance, dict):
            failures.append("occupancy snapshot rebuild acceptance is not an object")
        else:
            diagnostics = summary.get("diagnostics", {})
            rebuild = (
                diagnostics.get("occupancy_snapshot_rebuild", {})
                if isinstance(diagnostics, dict)
                else {}
            )
            budget = (
                diagnostics.get("occupancy_rebuild_budget", {})
                if isinstance(diagnostics, dict)
                else {}
            )
            if not isinstance(rebuild, dict) or not rebuild:
                failures.append("occupancy snapshot rebuild diagnostics are missing")
            else:
                for field, label in (
                    ("handoffs", "handoffs"),
                    ("requested", "requested rebuilds"),
                    ("completed", "completed rebuilds"),
                    ("cancelled_superseded", "superseded cancellations"),
                ):
                    minimum = occupancy_rebuild_acceptance.get(f"minimum_{field}")
                    if minimum is not None and int(rebuild.get(field, 0)) < int(minimum):
                        failures.append(
                            f"occupancy snapshot {label} {rebuild.get(field)} "
                            f"< minimum {minimum}"
                        )

                for field, label in (
                    ("cancelled_shutdown", "shutdown cancellations"),
                    ("discarded_obsolete", "obsolete rebuild discards"),
                    ("budget_exceeded", "budget exceedances"),
                    ("failed", "failures"),
                    ("peak_scratch_bytes", "peak scratch bytes"),
                ):
                    maximum = occupancy_rebuild_acceptance.get(f"maximum_{field}")
                    if maximum is not None and int(rebuild.get(field, 0)) > int(maximum):
                        failures.append(
                            f"occupancy snapshot {label} {rebuild.get(field)} "
                            f"> maximum {maximum}"
                        )

                terminal_rebuilds = sum(
                    int(rebuild.get(field, 0))
                    for field in (
                        "completed",
                        "cancelled_superseded",
                        "cancelled_shutdown",
                        "discarded_obsolete",
                        "budget_exceeded",
                        "failed",
                    )
                )
                if terminal_rebuilds != int(rebuild.get("requested", -1)):
                    failures.append(
                        "occupancy snapshot rebuild accounting is inconsistent: "
                        f"requested={rebuild.get('requested')}, terminals={terminal_rebuilds}"
                    )

                minimum_budget = occupancy_rebuild_acceptance.get(
                    "minimum_configured_scratch_bytes"
                )
                if minimum_budget is not None:
                    if not isinstance(budget, dict) or not budget:
                        failures.append("occupancy rebuild budget diagnostics are missing")
                    elif int(budget.get("max_scratch_bytes", 0)) < int(minimum_budget):
                        failures.append(
                            "occupancy configured scratch budget "
                            f"{budget.get('max_scratch_bytes')} < minimum {minimum_budget}"
                        )

    pcd_acceptance = acceptance.get("pcd")
    if pcd_acceptance is not None:
        if not isinstance(pcd_acceptance, dict):
            failures.append("PCD acceptance field is not an object")
        else:
            pcd = summary.get("pcd", {})
            files = pcd.get("files", []) if isinstance(pcd, dict) else []
            if not isinstance(files, list):
                failures.append("PCD summary files field is not a list")
            else:
                minimum_files = int(pcd_acceptance.get("minimum_files", 1))
                if len(files) < minimum_files:
                    failures.append(
                        f"PCD file count {len(files)} < minimum {minimum_files}"
                    )
                minimum_points = int(pcd_acceptance.get("minimum_points", 1))
                for entry in files:
                    if not isinstance(entry, dict):
                        failures.append("PCD summary contains a non-object entry")
                        continue
                    if not entry.get("finite", False):
                        failures.append(
                            f"PCD contains non-finite points: {entry.get('path')}"
                        )
                    if int(entry.get("point_count", 0)) < minimum_points:
                        failures.append(
                            "PCD point count "
                            f"{entry.get('point_count')} < minimum {minimum_points}: "
                            f"{entry.get('path')}"
                        )

    if acceptance.get("require_relocalization_scheduler", False):
        diagnostics = summary.get("diagnostics", {})
        scheduler = (
            diagnostics.get("relocalization_scheduler", {})
            if isinstance(diagnostics, dict)
            else {}
        )
        if not isinstance(scheduler, dict) or not scheduler:
            failures.append("relocalization scheduler diagnostics are missing")
        else:
            minimum_submitted = int(
                acceptance.get("minimum_relocalization_submitted", 1)
            )
            minimum_completed = int(
                acceptance.get("minimum_relocalization_completed", 1)
            )
            minimum_published = int(
                acceptance.get("minimum_relocalization_published", 1)
            )
            if int(scheduler.get("submitted", 0)) < minimum_submitted:
                failures.append(
                    "relocalization submissions "
                    f"{scheduler.get('submitted')} < minimum {minimum_submitted}"
                )
            if int(scheduler.get("completed", 0)) < minimum_completed:
                failures.append(
                    "relocalization completions "
                    f"{scheduler.get('completed')} < minimum {minimum_completed}"
                )
            if int(scheduler.get("published", 0)) < minimum_published:
                failures.append(
                    "relocalization publications "
                    f"{scheduler.get('published')} < minimum {minimum_published}"
                )
            if int(scheduler.get("callback_failures", 0)) != 0:
                failures.append(
                    "relocalization callback failures="
                    f"{scheduler.get('callback_failures')}"
                )
            timing_samples = scheduler.get("timing_samples")
            if timing_samples is None:
                failures.append("relocalization scheduler timing diagnostics are missing")
            elif int(timing_samples) != int(scheduler.get("completed", 0)):
                failures.append(
                    "relocalization scheduler timing samples/completions mismatch: "
                    f"{timing_samples}/{scheduler.get('completed')}"
                )
            for field, label in (
                ("queue_wait_mean_ms", "queue wait mean"),
                ("queue_wait_max_ms", "queue wait max"),
                ("processing_mean_ms", "processing mean"),
                ("processing_max_ms", "processing max"),
                ("end_to_end_mean_ms", "end-to-end mean"),
                ("end_to_end_max_ms", "end-to-end max"),
            ):
                if field in scheduler and float(scheduler[field]) < 0.0:
                    failures.append(
                        f"relocalization scheduler {label} is negative: "
                        f"{scheduler[field]}"
                    )
            for field, label in (
                ("maximum_relocalization_queue_wait_mean_ms", "queue wait mean"),
                ("maximum_relocalization_queue_wait_max_ms", "queue wait max"),
                ("maximum_relocalization_processing_mean_ms", "processing mean"),
                ("maximum_relocalization_processing_max_ms", "processing max"),
                ("maximum_relocalization_end_to_end_mean_ms", "end-to-end mean"),
                ("maximum_relocalization_end_to_end_max_ms", "end-to-end max"),
            ):
                maximum = acceptance.get(field)
                metric = field.removeprefix("maximum_relocalization_")
                if maximum is None:
                    continue
                actual = scheduler.get(metric)
                if actual is None:
                    failures.append(
                        f"relocalization scheduler {label} metric is missing"
                    )
                elif float(actual) > float(maximum):
                    failures.append(
                        f"relocalization scheduler {label} "
                        f"{float(actual):.6f} ms > maximum {float(maximum):.6f} ms"
                    )

    if acceptance.get("require_prior_map_quality", False):
        diagnostics = summary.get("diagnostics", {})
        prior_map = (
            diagnostics.get("prior_map_localization", {})
            if isinstance(diagnostics, dict)
            else {}
        )
        if not isinstance(prior_map, dict) or not prior_map:
            failures.append("prior-map localization quality diagnostics are missing")
        else:
            for field, label in (
                ("startup_kiss_attempts", "startup KISS attempts"),
                ("startup_kiss_successes", "startup KISS successes"),
                ("lost_recovery_attempts", "LOST recovery attempts"),
            ):
                minimum = acceptance.get(f"minimum_{field}")
                if minimum is not None and int(prior_map.get(field, 0)) < int(minimum):
                    failures.append(
                        f"{label} {prior_map.get(field, 0)} < minimum {minimum}"
                    )
            if int(prior_map.get("startup_kiss_successes", 0)) <= 0:
                failures.append("startup KISS produced no successful relocalization")
            for prefix, label in (
                ("startup_kiss", "startup KISS"),
                ("lost_recovery", "LOST recovery"),
            ):
                attempts = int(prior_map.get(f"{prefix}_attempts", 0))
                successes = int(prior_map.get(f"{prefix}_successes", 0))
                failed = int(prior_map.get(f"{prefix}_failures", 0))
                if successes > attempts or failed > attempts:
                    failures.append(
                        f"{label} outcomes exceed attempts: "
                        f"{attempts}/{successes}/{failed}"
                    )
                minimum_successes = acceptance.get(
                    f"minimum_{prefix}_successes"
                )
                if minimum_successes is not None and successes < int(
                    minimum_successes
                ):
                    failures.append(
                        f"{label} successes {successes} < minimum "
                        f"{minimum_successes}"
                    )
            if int(prior_map.get("startup_kiss_failures", 0)) < 0:
                failures.append("startup KISS failure count is invalid")
            if int(prior_map.get("lost_recovery_failures", 0)) < 0:
                failures.append("LOST recovery failure count is invalid")
            first_success_delay = prior_map.get("first_success_delay_s")
            if first_success_delay is None or float(first_success_delay) < 0.0:
                failures.append("first successful relocalization delay is missing or invalid")
            maximum_delay = acceptance.get("maximum_first_success_delay_s")
            if maximum_delay is not None and first_success_delay is not None:
                if float(first_success_delay) > float(maximum_delay):
                    failures.append(
                        "first successful relocalization delay "
                        f"{float(first_success_delay):.6f} s > maximum "
                        f"{float(maximum_delay):.6f} s"
                    )

    if acceptance.get("require_snapshot_refinement", False):
        diagnostics = summary.get("diagnostics", {})
        if not isinstance(diagnostics, dict):
            diagnostics = {}
        backend = diagnostics.get("backend_scheduler", {})
        alignment = diagnostics.get("global_alignment", {})
        queue = diagnostics.get("global_alignment_queue", {})
        if not isinstance(backend, dict) or not backend:
            failures.append("snapshot refinement requires backend scheduler diagnostics")
        else:
            accepted_keyframes = int(backend.get("accepted_keyframes", 0))
            snapshot_version = int(backend.get("map_snapshot_version", 0))
            optimizer_updates = int(backend.get("optimizer_updates", 0))
            if accepted_keyframes <= 0 or snapshot_version <= 0:
                failures.append(
                    "snapshot refinement requires a non-empty committed MapSnapshot"
                )
            if optimizer_updates <= accepted_keyframes:
                failures.append(
                    "snapshot refinement requires optimizer updates beyond keyframe commits"
                )
            if int(backend.get("failed", 0)) != 0 or int(
                backend.get("optimizer_failures", 0)
            ) != 0:
                failures.append("snapshot refinement backend/optimizer failures are non-zero")

        if not isinstance(alignment, dict) or not alignment:
            failures.append("snapshot refinement requires global alignment diagnostics")
        if not isinstance(queue, dict) or not queue:
            failures.append("snapshot refinement requires global alignment queue diagnostics")
        elif int(queue.get("committed", 0)) <= 0:
            failures.append("snapshot refinement produced no committed global alignment")
        elif int(queue.get("rejected", 0)) != 0:
            failures.append(
                "snapshot refinement global alignment rejects="
                f"{queue.get('rejected')}"
            )

        minimum_snapshot_version = acceptance.get("minimum_snapshot_version")
        if isinstance(backend, dict) and minimum_snapshot_version is not None:
            if int(backend.get("map_snapshot_version", 0)) < int(
                minimum_snapshot_version
            ):
                failures.append(
                    "map snapshot version "
                    f"{backend.get('map_snapshot_version')} < minimum "
                    f"{minimum_snapshot_version}"
                )

    if acceptance.get("forbid_legacy_backend_execution", False):
        # summarize_tum records this negative execution assertion from the
        # actual node log; require explicit false evidence for this gate.
        if summary.get("diagnostics", {}).get("legacy_backend_execution", False):
            failures.append("legacy AdaGraph backend execution marker was found")
        elif "legacy_backend_execution" not in summary.get("diagnostics", {}):
            failures.append("legacy backend execution evidence is missing")

    return failures


def run_case(
    repo_root: Path,
    output_root: Path,
    case: Dict[str, object],
) -> Tuple[Dict[str, object], bool]:
    case_id = str(case["id"])
    case_output = output_root / case_id
    params = repo_root / str(case["params"])
    command = [
        str(repo_root / "scripts" / "run_m2dgr_eval.sh"),
        str(case["sequence"]),
        str(case["replay_rate"]),
        str(case_output),
    ]
    environment = os.environ.copy()
    environment["SUPER_LIO_ROOT"] = str(repo_root)
    environment["FRONTEND"] = str(case["frontend"])
    environment["PARAMS_FILE"] = str(params)

    occupancy = case.get("occupancy")
    if isinstance(occupancy, dict):
        for field, variable in (
            ("enable", "OCCUPANCY_ENABLE"),
            ("source", "OCCUPANCY_SOURCE"),
            ("update_policy", "OCCUPANCY_UPDATE_POLICY"),
            ("topic", "OCCUPANCY_TOPIC"),
            ("resolution", "OCCUPANCY_RESOLUTION"),
            ("max_extent", "OCCUPANCY_MAX_EXTENT"),
        ):
            if field in occupancy:
                environment[variable] = str(occupancy[field]).lower()
        if occupancy.get("capture", True):
            environment["CAPTURE_OCCUPANCY"] = "true"
        rebuild = occupancy.get("rebuild")
        if rebuild is not None:
            if not isinstance(rebuild, dict):
                raise ValueError(f"case {case_id}: occupancy.rebuild is not an object")
            for field, variable in (
                ("max_scratch_memory_mb", "OCCUPANCY_REBUILD_MAX_SCRATCH_MEMORY_MB"),
                ("max_points", "OCCUPANCY_REBUILD_MAX_POINTS"),
                ("cancellation_check_interval", "OCCUPANCY_REBUILD_CANCELLATION_CHECK_INTERVAL"),
            ):
                if field in rebuild:
                    environment[variable] = str(rebuild[field])

    node_cpu_affinity = case.get("node_cpu_affinity")
    if node_cpu_affinity is not None:
        environment["NODE_CPU_AFFINITY"] = str(node_cpu_affinity)

    backend = case.get("backend")
    if backend is not None:
        if not isinstance(backend, dict):
            raise ValueError(f"case {case_id}: backend is not an object")
        for field, variable in (
            ("keyframe_translation_m", "BACKEND_KEYFRAME_TRANSLATION_M"),
            ("keyframe_rotation_rad", "BACKEND_KEYFRAME_ROTATION_RAD"),
            ("keyframe_max_frames", "BACKEND_KEYFRAME_MAX_FRAMES"),
        ):
            if field in backend:
                environment[variable] = str(backend[field])

    replay = case.get("replay")
    if replay is not None:
        if not isinstance(replay, dict):
            raise ValueError(f"case {case_id}: replay is not an object")
        if "loop" in replay:
            environment["BAG_LOOP"] = str(replay["loop"]).lower()
        if "duration_seconds" in replay:
            environment["BAG_PLAYBACK_DURATION_SECONDS"] = str(
                replay["duration_seconds"]
            )

    save_map = case.get("save_map")
    if isinstance(save_map, dict) and save_map.get("enable", True):
        environment["SAVE_MAP"] = "true"
        environment["SAVE_MAP_DIR"] = str((case_output / "map").resolve())
        for field, variable in (
            ("map_name", "MAP_NAME"),
            ("save_interval", "MAP_SAVE_INTERVAL"),
            ("if_filter", "MAP_IF_FILTER"),
            ("ds_size", "MAP_DS_SIZE"),
        ):
            if field in save_map:
                environment[variable] = str(save_map[field]).lower()

    if case.get("backend_localization_factor", False):
        environment["ENABLE_BACKEND_LOCALIZATION_FACTOR"] = "true"

    prior_map_file = case.get("prior_map_file")
    if prior_map_file is not None:
        prior_path = Path(str(prior_map_file))
        if not prior_path.is_absolute():
            prior_path = repo_root / prior_path
        environment["PRIOR_MAP_FILE"] = str(prior_path.resolve())

    print(f"[ RUN      ] {case_id}", flush=True)
    process = subprocess.run(command, env=environment, check=False)

    result: Dict[str, object] = {
        "id": case_id,
        "frontend": case["frontend"],
        "sequence": case["sequence"],
        "replay_rate": case["replay_rate"],
        "output": str(case_output.resolve()),
        "runner_exit_code": process.returncode,
        "acceptance": case.get("acceptance", {}),
    }

    summary_path = case_output / "summary.json"
    if not summary_path.is_file():
        result["status"] = "failed"
        result["failures"] = ["summary.json was not produced"]
        print(f"[  FAILED  ] {case_id}: no summary", flush=True)
        return result, False

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    failures = evaluate_acceptance(case, summary)
    if process.returncode != 0:
        failures.insert(0, f"runner exit code {process.returncode}")

    result["summary"] = summary
    result["failures"] = failures
    result["status"] = "passed" if not failures else "failed"
    passed = not failures
    label = "       OK" if passed else "  FAILED"
    print(f"[{label}] {case_id}", flush=True)
    for failure in failures:
        print(f"             {failure}", flush=True)
    return result, passed


def write_suite_summary(
    path: Path,
    manifest: Path,
    results: Sequence[Dict[str, object]],
) -> None:
    document = {
        "schema_version": 1,
        "manifest": str(manifest.resolve()),
        "status": (
            "passed"
            if results and all(result.get("status") == "passed" for result in results)
            else "failed"
        ),
        "results": list(results),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    repo_root = args.repo_root.resolve()
    output_root = args.output_root.resolve()
    manifest_path = args.manifest.resolve()

    manifest = load_manifest(manifest_path)
    manifest_cases = manifest["cases"]
    if not isinstance(manifest_cases, list):
        raise ValueError("manifest cases are not a list")
    cases = select_cases(manifest_cases, args.cases)

    if output_root.exists() and any(output_root.iterdir()):
        raise ValueError(f"refusing to overwrite non-empty output root: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)

    results: List[Dict[str, object]] = []
    all_passed = True
    for case in cases:
        result, passed = run_case(repo_root, output_root, case)
        results.append(result)
        all_passed = all_passed and passed

    summary_path = output_root / "suite_summary.json"
    write_suite_summary(summary_path, manifest_path, results)
    print(f"suite_summary={summary_path}", flush=True)
    return 0 if all_passed and results else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(2)
