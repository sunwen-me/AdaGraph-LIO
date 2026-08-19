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


def parse_tum(
    path: Path, *, allow_position_only: bool = False
) -> Tuple[List[Pose], Dict[str, object]]:
    poses: List[Pose] = []
    quaternion_norm_error_max = 0.0
    orientation_available = True

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
                if not allow_position_only:
                    raise ValueError(f"{path}:{line_number}: zero-length quaternion")
                orientation_available = False
            else:
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
        "orientation_available": orientation_available,
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
            if key == "cpu_percent":
                result[key] = match.group(1)
            else:
                result[key] = float(match.group(1))
    return result


def parse_node_diagnostics(path: Optional[Path]) -> Dict[str, object]:
    if path is None or not path.is_file():
        return {}

    text = path.read_text(encoding="utf-8", errors="replace")
    diagnostics: Dict[str, object] = {}

    keyframes = [int(value) for value in re.findall(r"Keyframe\s+(\d+)", text)]
    if keyframes:
        diagnostics["keyframe_count"] = max(keyframes)

    accepted_keyframe_matches = re.findall(
        r"accepted_keyframes=(\d+)", text
    )
    if accepted_keyframe_matches:
        # Prefer the explicit accepted count to optional per-keyframe logs.
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

    pipeline_memory_pattern = re.compile(
        r"\[PipelineRuntimeMemory\]:\s*"
        r"rss/allocated/retained=(\d+)/(\d+)/(\d+)"
    )
    pipeline_memory_matches = list(pipeline_memory_pattern.finditer(text))
    if pipeline_memory_matches:
        match = pipeline_memory_matches[-1]
        diagnostics["pipeline_runtime_memory"] = {
            "resident_kib": int(match.group(1)),
            "glibc_allocated_bytes": int(match.group(2)),
            "glibc_retained_bytes": int(match.group(3)),
        }

    backend_scheduler_pattern = re.compile(
        r"\[BackendScheduler\]:\s*enqueued/processed/failed="
        r"(\d+)/(\d+)/(\d+),\s*nodes/constraints=(\d+)/(\d+),\s*"
        r"max_queue=(\d+),\s*backpressure_waits=(\d+),\s*"
        r"optimizer updates/failures=(\d+)/(\d+),\s*"
        r"optimizer max pose delta=([-+0-9.eE]+)\s*m/"
        r"([-+0-9.eE]+)\s*rad,\s*global_constraints_dropped=(\d+),\s*"
        r"(?:relocalization_constraints enqueued/processed/dropped="
        r"(\d+)/(\d+)/(\d+),\s*)?"
        r"map_snapshot_version=(\d+),\s*submaps/finished=(\d+)/(\d+),\s*"
        r"accepted_keyframes=(\d+),\s*snapshot_observer_failures=(\d+)"
        r"(?:,\s*evaluation_drift constraints/distance/yaw="
        r"(\d+)/([-+0-9.eE]+)\s*m/([-+0-9.eE]+)\s*rad)?"
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
            "map_snapshot_version": int(match.group(16)),
            "submap_count": int(match.group(17)),
            "finished_submap_count": int(match.group(18)),
            "accepted_keyframes": int(match.group(19)),
            "snapshot_observer_failures": int(match.group(20)),
        }
        if match.group(13) is not None:
            diagnostics["backend_scheduler"].update(
                {
                    "relocalization_constraints_enqueued": int(match.group(13)),
                    "relocalization_constraints_processed": int(match.group(14)),
                    "relocalization_constraints_dropped": int(match.group(15)),
                }
            )
        if match.group(21) is not None:
            diagnostics["backend_scheduler"].update(
                {
                    "evaluation_drifted_odometry_constraints": int(
                        match.group(21)
                    ),
                    "evaluation_drifted_distance_m": float(match.group(22)),
                    "evaluation_drifted_yaw_rad": float(match.group(23)),
                }
            )

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
        r"publication delta=([-+0-9.eE]+)\s*m/([-+0-9.eE]+)\s*rad"
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

    tf_pattern = re.compile(
        r"\[PointLIOTf\]:\s*odom_base_published=(\d+),\s*"
        r"map_odom_published=(\d+),\s*"
        r"nonmonotonic_odom_base_stamps=(\d+),\s*"
        r"nonmonotonic_map_odom_stamps=(\d+)"
    )
    tf_matches = list(tf_pattern.finditer(text))
    if tf_matches:
        match = tf_matches[-1]
        diagnostics["tf_publication"] = {
            "odom_base_published": int(match.group(1)),
            "map_odom_published": int(match.group(2)),
            "nonmonotonic_odom_base_stamps": int(match.group(3)),
            "nonmonotonic_map_odom_stamps": int(match.group(4)),
        }

    recovery: Dict[str, object] = {}
    candidate_pattern = re.compile(
        r"Submap recovery candidate:\s*target_session=(\d+),\s*"
        r"target_trajectory=(\d+),\s*target_submap=(\d+),\s*"
        r"new_trajectory=(\d+),\s*query_timestamp=([-+0-9.eE]+)"
    )
    candidate_matches = list(candidate_pattern.finditer(text))
    if candidate_matches:
        match = candidate_matches[-1]
        recovery["candidate"] = {
            "target_session": int(match.group(1)),
            "target_trajectory": int(match.group(2)),
            "target_submap": int(match.group(3)),
            "new_trajectory": int(match.group(4)),
            "query_timestamp": float(match.group(5)),
        }

    commit_pattern = re.compile(
        r"\[PointLIORecovery\]:\s*committed old_trajectory=(\d+),\s*"
        r"new_trajectory=(\d+),\s*session=(\d+),\s*"
        r"timestamp=([-+0-9.eE]+),\s*first_durable_keyframe_pending=true"
    )
    commit_matches = list(commit_pattern.finditer(text))
    if commit_matches:
        match = commit_matches[-1]
        recovery["transaction"] = {
            "old_trajectory": int(match.group(1)),
            "new_trajectory": int(match.group(2)),
            "session": int(match.group(3)),
            "timestamp": float(match.group(4)),
        }

    reconnect_pattern = re.compile(
        r"\[PointLIORecovery\]:\s*reconnect_constraint_enqueued\s+"
        r"trajectory=(\d+),\s*session=(\d+),\s*timestamp=([-+0-9.eE]+)"
    )
    reconnect_matches = list(reconnect_pattern.finditer(text))
    if reconnect_matches:
        match = reconnect_matches[-1]
        recovery["reconnect_constraint"] = {
            "trajectory": int(match.group(1)),
            "session": int(match.group(2)),
            "timestamp": float(match.group(3)),
        }

    durable_pattern = re.compile(
        r"\[PointLIORecovery\]:\s*first_durable_keyframe=(\d+),\s*"
        r"trajectory=(\d+),\s*session=(\d+),\s*timestamp=([-+0-9.eE]+)"
    )
    durable_matches = list(durable_pattern.finditer(text))
    if durable_matches:
        match = durable_matches[-1]
        recovery["first_durable_keyframe"] = {
            "keyframe": int(match.group(1)),
            "trajectory": int(match.group(2)),
            "session": int(match.group(3)),
            "timestamp": float(match.group(4)),
        }

    recovery["verification_committed"] = len(re.findall(
        r"\[PointLIORecovery\]:\s*verification_committed", text))
    recovery["verification_rollbacks"] = len(re.findall(
        r"\[PointLIORecovery\]:\s*verification_rollback", text))

    snapshot_pattern = re.compile(
        r"\[PointLIORecovery\]:\s*final_snapshot old_trajectory=(\d+),\s*"
        r"new_trajectory=(\d+),\s*old_present=(\d+),\s*"
        r"new_present=(\d+),\s*relocalization_constraints=(\d+)"
    )
    snapshot_matches = list(snapshot_pattern.finditer(text))
    if snapshot_matches:
        match = snapshot_matches[-1]
        recovery["final_snapshot"] = {
            "old_trajectory": int(match.group(1)),
            "new_trajectory": int(match.group(2)),
            "old_present": bool(int(match.group(3))),
            "new_present": bool(int(match.group(4))),
            "relocalization_constraints": int(match.group(5)),
        }
    if any(recovery.values()):
        diagnostics["submap_recovery"] = recovery

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

    loop_closure_scheduler_pattern = re.compile(
        r"\[LoopClosureScheduler\]:\s*submitted/replaced/cancelled/"
        r"completed/published/callback_failures="
        r"(\d+)/(\d+)/(\d+)/(\d+)/(\d+)/(\d+)"
    )
    loop_closure_scheduler_matches = list(
        loop_closure_scheduler_pattern.finditer(text)
    )
    if loop_closure_scheduler_matches:
        match = loop_closure_scheduler_matches[-1]
        diagnostics["loop_closure_scheduler"] = {
            "submitted": int(match.group(1)),
            "replaced_pending": int(match.group(2)),
            "cancellation_requests": int(match.group(3)),
            "completed": int(match.group(4)),
            "constraints_published": int(match.group(5)),
            "callback_failures": int(match.group(6)),
        }

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
    parser.add_argument("--backend-trajectory", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ground-truth", type=Path)
    parser.add_argument("--ape", type=Path)
    parser.add_argument("--backend-ape", type=Path)
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

    backend_trajectory: Optional[Dict[str, object]] = None
    if args.backend_trajectory is not None:
        _, backend_trajectory = parse_tum(args.backend_trajectory)

    ground_truth: Optional[Dict[str, object]] = None
    if args.ground_truth is not None:
        _, ground_truth = parse_tum(args.ground_truth, allow_position_only=True)

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
        "backend_trajectory": backend_trajectory,
        "ground_truth": ground_truth,
        "ape_translation_m": parse_ape(args.ape),
        "backend_ape_translation_m": parse_ape(args.backend_ape),
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
