#!/usr/bin/env python3

"""Validate a TUM trajectory and write a machine-readable regression summary."""

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


Pose = Tuple[float, float, float, float, float, float, float, float]


def parse_tum(path: Path) -> Tuple[List[Pose], Dict[str, object]]:
    poses: List[Pose] = []
    quaternion_norm_error_max = 0.0

    with path.open("r", encoding="ascii") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue

            fields = line.split()
            if len(fields) != 8:
                raise ValueError(
                    f"{path}:{line_number}: expected 8 TUM fields, got {len(fields)}"
                )

            try:
                values = tuple(float(value) for value in fields)
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: non-numeric TUM field"
                ) from error

            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{path}:{line_number}: non-finite TUM value")

            pose = values  # type: ignore[assignment]
            if poses and pose[0] <= poses[-1][0]:
                raise ValueError(
                    f"{path}:{line_number}: timestamps are not strictly increasing"
                )

            quaternion_norm = math.sqrt(sum(value * value for value in pose[4:8]))
            if quaternion_norm < 1.0e-12:
                raise ValueError(f"{path}:{line_number}: zero-length quaternion")
            quaternion_norm_error_max = max(
                quaternion_norm_error_max, abs(quaternion_norm - 1.0)
            )
            poses.append(pose)

    if not poses:
        raise ValueError(f"{path}: trajectory contains no poses")

    path_length = 0.0
    for previous, current in zip(poses, poses[1:]):
        path_length += math.dist(previous[1:4], current[1:4])

    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    summary: Dict[str, object] = {
        "path": str(path.resolve()),
        "sha256": digest,
        "pose_count": len(poses),
        "timestamp_first": poses[0][0],
        "timestamp_last": poses[-1][0],
        "duration_seconds": poses[-1][0] - poses[0][0],
        "path_length_m": path_length,
        "start_position_m": list(poses[0][1:4]),
        "end_position_m": list(poses[-1][1:4]),
        "quaternion_norm_error_max": quaternion_norm_error_max,
        "timestamps_strictly_increasing": True,
        "finite": True,
    }
    return poses, summary


def parse_ape(path: Optional[Path]) -> Dict[str, float]:
    if path is None or not path.is_file():
        return {}

    stats: Dict[str, float] = {}
    pattern = re.compile(
        r"^\s*(max|mean|median|min|rmse|sse|std)\s+"
        r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$"
    )
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = pattern.match(line)
        if match:
            stats[match.group(1)] = float(match.group(2))
    return stats


def parse_resource_usage(path: Optional[Path]) -> Dict[str, object]:
    if path is None or not path.is_file():
        return {}

    result: Dict[str, object] = {}
    patterns = {
        "maximum_resident_set_kib": re.compile(
            r"^\s*Maximum resident set size \(kbytes\):\s*(\d+)\s*$"
        ),
        "user_cpu_seconds": re.compile(
            r"^\s*User time \(seconds\):\s*([-+0-9.eE]+)\s*$"
        ),
        "system_cpu_seconds": re.compile(
            r"^\s*System time \(seconds\):\s*([-+0-9.eE]+)\s*$"
        ),
        "cpu_percent": re.compile(
            r"^\s*Percent of CPU this job got:\s*([^\s]+)\s*$"
        ),
    }
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        for key, pattern in patterns.items():
            match = pattern.match(line)
            if not match:
                continue
            if key == "maximum_resident_set_kib":
                result[key] = int(match.group(1))
            elif key == "cpu_percent":
                result[key] = match.group(1)
            else:
                result[key] = float(match.group(1))
    return result


def parse_node_diagnostics(path: Optional[Path]) -> Dict[str, object]:
    if path is None or not path.is_file():
        return {}

    text = path.read_text(encoding="utf-8", errors="replace")
    diagnostics: Dict[str, object] = {}

    # The standard compositions must use the typed BackendScheduler.  Keep a
    # machine-readable negative assertion in the summary so a regression case
    # can fail if a compatibility AdaGraph facade is accidentally reintroduced.
    diagnostics["legacy_backend_execution"] = bool(
        re.search(r"(?:adagraph_backend\.cpp|\[AdaGraph\])", text)
    )

    keyframes = [int(value) for value in re.findall(r"Keyframe\s+(\d+)", text)]
    if keyframes:
        diagnostics["keyframe_count"] = max(keyframes)

    accepted_keyframe_matches = re.findall(
        r"accepted_keyframes=(\d+)", text
    )
    if accepted_keyframe_matches:
        # The typed scheduler remains authoritative even when the legacy
        # AdaGraph graph is disabled and therefore emits no "Keyframe N"
        # lines.  Prefer the explicit accepted count when both diagnostics
        # styles are present during the compatibility transition.
        diagnostics["keyframe_count"] = int(accepted_keyframe_matches[-1])

    point_pattern = re.compile(
        r"input lidar/imu=(\d+)/(\d+),\s*"
        r"downstream enqueued/processed/dropped=(\d+)/(\d+)/(\d+),\s*"
        r"max_queue=(\d+),\s*mean_downstream=([-+0-9.eE]+)\s*ms"
    )
    point_matches = list(point_pattern.finditer(text))
    if point_matches:
        match = point_matches[-1]
        diagnostics["point_lio"] = {
            "lidar_input_count": int(match.group(1)),
            "imu_input_count": int(match.group(2)),
            "downstream_enqueued": int(match.group(3)),
            "downstream_processed": int(match.group(4)),
            "downstream_dropped": int(match.group(5)),
            "downstream_max_queue_depth": int(match.group(6)),
            "downstream_mean_ms": float(match.group(7)),
        }

    point_memory_pattern = re.compile(
        r"\[PointLIOMemory\]:\s*"
        r"ivox\(entries/buckets/capacity/estimated\)="
        r"(\d+)/(\d+)/(\d+)/(\d+),\s*"
        r"preprocess\(queued_points/queued_imus/vectors\)="
        r"(\d+)/(\d+)/(\d+),\s*frame_vectors=(\d+)\s*bytes"
    )
    point_memory_matches = list(point_memory_pattern.finditer(text))
    if point_memory_matches:
        match = point_memory_matches[-1]
        diagnostics["point_lio_memory"] = {
            "ivox_entries": int(match.group(1)),
            "ivox_buckets": int(match.group(2)),
            "ivox_capacity_limit": int(match.group(3)),
            "ivox_estimated_bytes": int(match.group(4)),
            "preprocess_queued_points": int(match.group(5)),
            "preprocess_queued_imus": int(match.group(6)),
            "preprocess_vector_backing_bytes": int(match.group(7)),
            "frame_vector_backing_bytes": int(match.group(8)),
        }

    super_downstream_pattern = re.compile(
        r"Super-LIO downstream enqueued/processed=(\d+)/(\d+),\s*"
        r"max_queue=(\d+),\s*backpressure_waits=(\d+),\s*"
        r"mean=([-+0-9.eE]+)\s*ms"
    )
    super_downstream_matches = list(super_downstream_pattern.finditer(text))
    if super_downstream_matches:
        match = super_downstream_matches[-1]
        diagnostics["super_lio_downstream"] = {
            "downstream_enqueued": int(match.group(1)),
            "downstream_processed": int(match.group(2)),
            "downstream_max_queue_depth": int(match.group(3)),
            "backpressure_wait_count": int(match.group(4)),
            "downstream_mean_ms": float(match.group(5)),
        }

    backend_scheduler_pattern = re.compile(
        r"\[BackendScheduler\]:\s*enqueued/processed/failed="
        r"(\d+)/(\d+)/(\d+),\s*nodes/constraints=(\d+)/(\d+),\s*"
        r"max_queue=(\d+),\s*backpressure_waits=(\d+),\s*"
        r"optimizer updates/failures=(\d+)/(\d+),\s*"
        r"optimizer max pose delta=([-+0-9.eE]+)\s*m/"
        r"([-+0-9.eE]+)\s*rad,\s*global_constraints_dropped=(\d+),\s*"
        r"map_snapshot_version=(\d+),\s*submaps/finished=(\d+)/(\d+),\s*"
        r"accepted_keyframes=(\d+),\s*snapshot_observer_failures=(\d+)"
    )
    backend_scheduler_matches = list(
        backend_scheduler_pattern.finditer(text)
    )
    if backend_scheduler_matches:
        match = backend_scheduler_matches[-1]
        diagnostics["backend_scheduler"] = {
            "enqueued": int(match.group(1)),
            "processed": int(match.group(2)),
            "failed": int(match.group(3)),
            "node_count": int(match.group(4)),
            "constraint_count": int(match.group(5)),
            "maximum_queue_depth": int(match.group(6)),
            "capacity_wait_count": int(match.group(7)),
            "optimizer_updates": int(match.group(8)),
            "optimizer_failures": int(match.group(9)),
            "maximum_optimizer_translation_delta_m": float(match.group(10)),
            "maximum_optimizer_rotation_delta_rad": float(match.group(11)),
            "global_constraints_dropped": int(match.group(12)),
            "map_snapshot_version": int(match.group(13)),
            "submap_count": int(match.group(14)),
            "finished_submap_count": int(match.group(15)),
            "accepted_keyframes": int(match.group(16)),
            "snapshot_observer_failures": int(match.group(17)),
        }

    backend_memory_pattern = re.compile(
        r"\[BackendScheduler\]:[^\n]*"
        r"keyframe_clouds\(unique/points/logical/backing\)="
        r"(\d+)/(\d+)/(\d+)/(\d+)"
    )
    backend_memory_matches = list(backend_memory_pattern.finditer(text))
    if backend_memory_matches:
        match = backend_memory_matches[-1]
        diagnostics["map_keyframe_memory"] = {
            "unique_cloud_count": int(match.group(1)),
            "unique_cloud_points": int(match.group(2)),
            "unique_cloud_logical_bytes": int(match.group(3)),
            "unique_cloud_backing_bytes": int(match.group(4)),
        }

    global_alignment_pattern = re.compile(
        r"\[GlobalAlignmentState\]:\s*version=(\d+),\s*"
        r"legacy parity=([-+0-9.eE]+)\s*m/([-+0-9.eE]+)\s*rad"
    )
    global_alignment_matches = list(global_alignment_pattern.finditer(text))
    if global_alignment_matches:
        match = global_alignment_matches[-1]
        diagnostics["global_alignment"] = {
            "version": int(match.group(1)),
            "ros_cache_translation_delta_m": float(match.group(2)),
            "ros_cache_rotation_delta_rad": float(match.group(3)),
        }

    global_alignment_queue_pattern = re.compile(
        r"\[GlobalAlignmentQueue\]:\s*enqueued/replaced/committed/"
        r"rejected=(\d+)/(\d+)/(\d+)/(\d+)"
    )
    global_alignment_queue_matches = list(
        global_alignment_queue_pattern.finditer(text)
    )
    if global_alignment_queue_matches:
        match = global_alignment_queue_matches[-1]
        diagnostics["global_alignment_queue"] = {
            "enqueued": int(match.group(1)),
            "replaced": int(match.group(2)),
            "committed": int(match.group(3)),
            "rejected": int(match.group(4)),
        }

    relocalization_scheduler_pattern = re.compile(
        r"\[RelocalizationScheduler\]:\s*submitted/replaced/cancelled/"
        r"completed/published/callback_failures="
        r"(\d+)/(\d+)/(\d+)/(\d+)/(\d+)/(\d+)"
        r"(?:,\s*timing samples=(\d+),\s*"
        r"queue_wait mean/max=([-+0-9.eE]+)/([-+0-9.eE]+)\s*ms,\s*"
        r"processing mean/max=([-+0-9.eE]+)/([-+0-9.eE]+)\s*ms,\s*"
        r"end_to_end mean/max=([-+0-9.eE]+)/([-+0-9.eE]+)\s*ms)?"
    )
    relocalization_scheduler_matches = list(
        relocalization_scheduler_pattern.finditer(text)
    )
    if relocalization_scheduler_matches:
        match = relocalization_scheduler_matches[-1]
        diagnostics["relocalization_scheduler"] = {
            "submitted": int(match.group(1)),
            "replaced_pending": int(match.group(2)),
            "cancellation_requests": int(match.group(3)),
            "completed": int(match.group(4)),
            "published": int(match.group(5)),
            "callback_failures": int(match.group(6)),
        }
        if match.group(7) is not None:
            diagnostics["relocalization_scheduler"].update(
                {
                    "timing_samples": int(match.group(7)),
                    "queue_wait_mean_ms": float(match.group(8)),
                    "queue_wait_max_ms": float(match.group(9)),
                    "processing_mean_ms": float(match.group(10)),
                    "processing_max_ms": float(match.group(11)),
                    "end_to_end_mean_ms": float(match.group(12)),
                    "end_to_end_max_ms": float(match.group(13)),
                }
            )

    prior_map_diagnostics_pattern = re.compile(
        r"\[PriorMapLocalization\]:\s*diagnostics\s+"
        r"startup_kiss\s+attempts/successes/failures=(\d+)/(\d+)/(\d+),\s*"
        r"lost_recovery\s+attempts/successes/failures=(\d+)/(\d+)/(\d+),\s*"
        r"first_success_delay_s=([-+0-9.eE]+)"
    )
    prior_map_diagnostics_matches = list(
        prior_map_diagnostics_pattern.finditer(text)
    )
    if prior_map_diagnostics_matches:
        match = prior_map_diagnostics_matches[-1]
        diagnostics["prior_map_localization"] = {
            "startup_kiss_attempts": int(match.group(1)),
            "startup_kiss_successes": int(match.group(2)),
            "startup_kiss_failures": int(match.group(3)),
            "lost_recovery_attempts": int(match.group(4)),
            "lost_recovery_successes": int(match.group(5)),
            "lost_recovery_failures": int(match.group(6)),
            "first_success_delay_s": float(match.group(7)),
        }

    prior_map_memory_pattern = re.compile(
        r"\[PriorMapLocalizationMemory\]:\s*"
        r"prior\(points/backing\)=(\d+)/(\d+),\s*"
        r"crop_index\(cells/indices/payload\)=(\d+)/(\d+)/(\d+),\s*"
        r"local_frames\(count/points/backing\)=(\d+)/(\d+)/(\d+),\s*"
        r"peak_local_submap\(points/backing\)=(\d+)/(\d+),\s*"
        r"peak_crop\(points/backing\)=(\d+)/(\d+),\s*"
        r"peak_pcl\(source_odom/aligned\)=(\d+)/(\d+),\s*"
        r"peak_runtime\(kiss_rss/allocated/retained\)=(\d+)/(\d+)/(\d+),\s*"
        r"registration_rss/allocated/retained=(\d+)/(\d+)/(\d+),\s*"
        r"peak_explicit_working_set=(\d+)\s*bytes"
    )
    prior_map_memory_matches = list(prior_map_memory_pattern.finditer(text))
    if prior_map_memory_matches:
        match = prior_map_memory_matches[-1]
        diagnostics["prior_map_memory"] = {
            "prior_map_points": int(match.group(1)),
            "prior_map_backing_bytes": int(match.group(2)),
            "crop_index_cells": int(match.group(3)),
            "crop_index_point_indices": int(match.group(4)),
            "crop_index_index_payload_bytes": int(match.group(5)),
            "local_frame_count": int(match.group(6)),
            "local_frame_points": int(match.group(7)),
            "local_frame_backing_bytes": int(match.group(8)),
            "peak_local_submap_points": int(match.group(9)),
            "peak_local_submap_backing_bytes": int(match.group(10)),
            "peak_crop_points": int(match.group(11)),
            "peak_crop_backing_bytes": int(match.group(12)),
            "peak_pcl_odom_conversion_backing_bytes": int(match.group(13)),
            "peak_pcl_registration_aligned_backing_bytes": int(match.group(14)),
            "peak_kiss_resident_kib": int(match.group(15)),
            "peak_kiss_glibc_allocated_bytes": int(match.group(16)),
            "peak_kiss_glibc_retained_bytes": int(match.group(17)),
            "peak_registration_resident_kib": int(match.group(18)),
            "peak_registration_glibc_allocated_bytes": int(match.group(19)),
            "peak_registration_glibc_retained_bytes": int(match.group(20)),
            "peak_explicit_working_set_bytes": int(match.group(21)),
        }

    occupancy_rebuild_pattern = re.compile(
        r"Occupancy snapshot handoffs/rebuilds requested/completed/"
        r"cancelled_superseded/cancelled_shutdown/discarded_obsolete/"
        r"budget_exceeded/failed="
        r"(\d+)/(\d+)/(\d+)/(\d+)/(\d+)/(\d+)/(\d+)/(\d+),\s*"
        r"peak_scratch=(\d+) bytes"
    )
    occupancy_rebuild_matches = list(occupancy_rebuild_pattern.finditer(text))
    if occupancy_rebuild_matches:
        match = occupancy_rebuild_matches[-1]
        diagnostics["occupancy_snapshot_rebuild"] = {
            "handoffs": int(match.group(1)),
            "requested": int(match.group(2)),
            "completed": int(match.group(3)),
            "cancelled_superseded": int(match.group(4)),
            "cancelled_shutdown": int(match.group(5)),
            "discarded_obsolete": int(match.group(6)),
            "budget_exceeded": int(match.group(7)),
            "failed": int(match.group(8)),
            "peak_scratch_bytes": int(match.group(9)),
        }

    occupancy_budget_pattern = re.compile(
        r"Occupancy rebuild budget:\s*scratch=(\d+) bytes,\s*"
        r"points=(\d+),\s*cancel_check=(\d+),\s*max_cells=(\d+)"
    )
    occupancy_budget_matches = list(occupancy_budget_pattern.finditer(text))
    if occupancy_budget_matches:
        match = occupancy_budget_matches[-1]
        diagnostics["occupancy_rebuild_budget"] = {
            "max_scratch_bytes": int(match.group(1)),
            "max_points": int(match.group(2)),
            "cancellation_check_interval": int(match.group(3)),
            "max_materialized_cells": int(match.group(4)),
        }

    timer_pattern = re.compile(
        r"\[\s*\[([^\]]+)\]\s*\]\s*average time usage:\s*"
        r"([-+0-9.eE]+)\s*ms\s*,\s*called times:\s*(\d+)\s*,\s*"
        r"max:\s*([-+0-9.eE]+)\s*ms"
    )
    timers: Dict[str, object] = {}
    for match in timer_pattern.finditer(text):
        timers[match.group(1).strip()] = {
            "mean_ms": float(match.group(2)),
            "count": int(match.group(3)),
            "max_ms": float(match.group(4)),
        }
    if timers:
        diagnostics["super_lio_timers"] = timers

    return diagnostics


def sha256_if_file(path: Optional[Path]) -> Optional[str]:
    if path is None or not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_occupancy_summary(path: Optional[Path]) -> Dict[str, object]:
    if path is None or not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: occupancy summary must be a JSON object")
    return value


def parse_pcd_summary(path: Optional[Path]) -> Dict[str, object]:
    if path is None or not path.is_file():
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path}: PCD summary must be a JSON object")
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a TUM trajectory and create a regression summary."
    )
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ground-truth", type=Path)
    parser.add_argument("--ape", type=Path)
    parser.add_argument("--resource-usage", type=Path)
    parser.add_argument("--node-log", type=Path)
    parser.add_argument("--occupancy-summary", type=Path)
    parser.add_argument("--pcd-summary", type=Path)
    parser.add_argument("--params", type=Path)
    parser.add_argument("--frontend", required=True)
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--rate", required=True, type=float)
    parser.add_argument("--wall-seconds", type=float)
    parser.add_argument("--git-commit")
    parser.add_argument(
        "--runtime-override",
        action="append",
        default=[],
        help="Effective key=value runtime override; repeat for each value.",
    )
    parser.add_argument("--git-dirty", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _, trajectory = parse_tum(args.trajectory)

    ground_truth: Optional[Dict[str, object]] = None
    if args.ground_truth is not None:
        _, ground_truth = parse_tum(args.ground_truth)

    summary: Dict[str, object] = {
        "schema_version": 1,
        "run": {
            "frontend": args.frontend,
            "sequence": args.sequence,
            "replay_rate": args.rate,
            "wall_seconds": args.wall_seconds,
            "git_commit": args.git_commit,
            "git_dirty": args.git_dirty,
            "runtime_overrides": args.runtime_override,
            "params_path": (
                str(args.params.resolve()) if args.params is not None else None
            ),
            "params_sha256": sha256_if_file(args.params),
        },
        "trajectory": trajectory,
        "ground_truth": ground_truth,
        "ape_translation_m": parse_ape(args.ape),
        "resources": parse_resource_usage(args.resource_usage),
        "diagnostics": parse_node_diagnostics(args.node_log),
        "occupancy": parse_occupancy_summary(args.occupancy_summary),
        "pcd": parse_pcd_summary(args.pcd_summary),
    }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(args.output)

    print(
        "trajectory "
        f"poses={trajectory['pose_count']} "
        f"duration={trajectory['duration_seconds']:.6f}s "
        f"length={trajectory['path_length_m']:.6f}m"
    )
    ape = summary["ape_translation_m"]
    if isinstance(ape, dict) and "rmse" in ape:
        print(f"ape_translation_rmse={ape['rmse']:.6f}m")


if __name__ == "__main__":
    main()
