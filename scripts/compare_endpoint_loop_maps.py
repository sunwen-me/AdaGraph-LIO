#!/usr/bin/env python3

"""Compare first/last keyframe PCDs only along their shared loop corridor."""

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", "/tmp/adagraph-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from analyze_loop_map_layers import iter_xyz, read_pcd_layout


def parse_case(value: str) -> tuple[str, Path, Path, Path]:
    parts = value.split("|", 3)
    if len(parts) != 4 or not parts[0]:
        raise argparse.ArgumentTypeError(
            "case must be LABEL|trajectory.tum|head.pcd|tail.pcd"
        )
    paths = tuple(Path(item) for item in parts[1:])
    if not all(path.is_absolute() for path in paths):
        raise argparse.ArgumentTypeError("all case paths must be absolute")
    return parts[0], paths[0], paths[1], paths[2]


def load_cloud(path: Path) -> np.ndarray:
    cloud, _ = read_pcd_layout(path)
    chunks = list(iter_xyz(cloud))
    if not chunks:
        raise ValueError(f"{path}: no finite points")
    return np.concatenate(chunks)


def crop_to_shared_corridor(
    trajectory: np.ndarray,
    head: np.ndarray,
    tail: np.ndarray,
    endpoint_poses: int,
    pose_gate_m: float,
    corridor_radius_m: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    if trajectory.ndim != 2 or trajectory.shape[1] != 8:
        raise ValueError("trajectory must have eight TUM columns")
    count = min(endpoint_poses, trajectory.shape[0] // 2)
    if count < 2:
        raise ValueError("trajectory is too short for endpoint comparison")
    head_poses = trajectory[:count, 1:4]
    tail_poses = trajectory[-count:, 1:4]
    pose_distance, head_index = cKDTree(head_poses[:, :2]).query(
        tail_poses[:, :2], workers=-1
    )
    shared_tail = pose_distance <= pose_gate_m
    if not np.any(shared_tail):
        raise ValueError("endpoint trajectories have no shared XY corridor")
    shared_head_xy = head_poses[np.unique(head_index[shared_tail]), :2]
    shared_tail_xy = tail_poses[shared_tail, :2]
    head_distance = cKDTree(shared_head_xy).query(head[:, :2], workers=-1)[0]
    tail_distance = cKDTree(shared_tail_xy).query(tail[:, :2], workers=-1)[0]
    cropped_head = head[head_distance <= corridor_radius_m]
    cropped_tail = tail[tail_distance <= corridor_radius_m]
    return cropped_head, cropped_tail, {
        "endpoint_pose_count": count,
        "shared_tail_pose_count": int(np.count_nonzero(shared_tail)),
        "trajectory_xy_distance_median_m": float(
            np.median(pose_distance[shared_tail])
        ),
    }


def nearest_metrics(source: np.ndarray, target: np.ndarray,
                    gate_m: float) -> tuple[dict[str, Any], np.ndarray]:
    distance, index = cKDTree(target).query(source, workers=-1)
    inlier = distance <= gate_m
    if not np.any(inlier):
        return {
            "overlap": 0.0,
            "inlier_nn_rmse_m": math.inf,
            "inlier_abs_dz_median_m": math.inf,
            "inlier_abs_dz_p95_m": math.inf,
            "nn_distance_median_m": float(np.median(distance)),
        }, inlier
    dz = np.abs(source[inlier, 2] - target[index[inlier], 2])
    return {
        "overlap": float(np.mean(inlier)),
        "inlier_nn_rmse_m": float(np.sqrt(np.mean(distance[inlier] ** 2))),
        "inlier_abs_dz_median_m": float(np.median(dz)),
        "inlier_abs_dz_p95_m": float(np.percentile(dz, 95)),
        "nn_distance_median_m": float(np.median(distance)),
    }, inlier


def sample(points: np.ndarray, limit: int, seed: int) -> np.ndarray:
    if points.shape[0] <= limit:
        return points
    indices = np.random.default_rng(seed).choice(
        points.shape[0], limit, replace=False
    )
    return points[indices]


def analyze_case(case: tuple[str, Path, Path, Path], args: argparse.Namespace):
    label, trajectory_path, head_path, tail_path = case
    for path in (trajectory_path, head_path, tail_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    trajectory = np.loadtxt(trajectory_path, dtype=np.float64)
    head = load_cloud(head_path)
    tail = load_cloud(tail_path)
    head, tail, trajectory_metrics = crop_to_shared_corridor(
        trajectory, head, tail, args.endpoint_poses,
        args.pose_gate_m, args.corridor_radius_m
    )
    tail_to_head, _ = nearest_metrics(tail, head, args.point_gate_m)
    head_to_tail, _ = nearest_metrics(head, tail, args.point_gate_m)
    result = {
        "label": label,
        "trajectory_path": str(trajectory_path.resolve()),
        "head_pcd_path": str(head_path.resolve()),
        "tail_pcd_path": str(tail_path.resolve()),
        "pose_gate_m": args.pose_gate_m,
        "corridor_radius_m": args.corridor_radius_m,
        "point_gate_m": args.point_gate_m,
        "cropped_head_points": int(head.shape[0]),
        "cropped_tail_points": int(tail.shape[0]),
        **trajectory_metrics,
        "tail_to_head": tail_to_head,
        "head_to_tail": head_to_tail,
        "bidirectional_overlap": min(
            tail_to_head["overlap"], head_to_tail["overlap"]
        ),
        "bidirectional_inlier_nn_rmse_m": max(
            tail_to_head["inlier_nn_rmse_m"],
            head_to_tail["inlier_nn_rmse_m"],
        ),
    }
    return result, head, tail


def plot_cases(analyses, output: Path) -> None:
    figure, axes = plt.subplots(
        len(analyses), 2, figsize=(14, 5.5 * len(analyses)),
        squeeze=False, constrained_layout=True
    )
    for row, (result, head, tail) in enumerate(analyses):
        head_plot = sample(head, 180_000, 17)
        tail_plot = sample(tail, 180_000, 23)
        title = (
            f"{result['label']}: shared-corridor PCD\n"
            f"bidirectional overlap={100 * result['bidirectional_overlap']:.1f}%, "
            f"NN median={result['tail_to_head']['nn_distance_median_m']:.3f} m"
        )
        axes[row, 0].scatter(
            head_plot[:, 0], head_plot[:, 1], s=0.25, alpha=0.28,
            label="first 500 keyframes"
        )
        axes[row, 0].scatter(
            tail_plot[:, 0], tail_plot[:, 1], s=0.25, alpha=0.28,
            label="last 500 keyframes"
        )
        axes[row, 0].set_title(title + " (XY)")
        axes[row, 0].set_aspect("equal", adjustable="box")
        axes[row, 0].set_xlabel("map X [m]")
        axes[row, 0].set_ylabel("map Y [m]")
        axes[row, 0].legend(markerscale=8)

        axes[row, 1].scatter(
            head_plot[:, 0], head_plot[:, 2], s=0.25, alpha=0.28,
            label="first 500 keyframes"
        )
        axes[row, 1].scatter(
            tail_plot[:, 0], tail_plot[:, 2], s=0.25, alpha=0.28,
            label="last 500 keyframes"
        )
        axes[row, 1].set_title(
            title + " (XZ)\n"
            f"inlier |dZ| median="
            f"{result['tail_to_head']['inlier_abs_dz_median_m']:.3f} m, "
            f"P95={result['tail_to_head']['inlier_abs_dz_p95_m']:.3f} m"
        )
        axes[row, 1].set_xlabel("map X [m]")
        axes[row, 1].set_ylabel("map Z [m]")
        axes[row, 1].legend(markerscale=8)
        for axis in axes[row]:
            axis.grid(alpha=0.15)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", required=True,
                        type=parse_case)
    parser.add_argument("--endpoint-poses", type=int, default=500)
    parser.add_argument("--pose-gate-m", type=float, default=10.0)
    parser.add_argument("--corridor-radius-m", type=float, default=20.0)
    parser.add_argument("--point-gate-m", type=float, default=1.0)
    parser.add_argument("--output-json", required=True, type=Path)
    parser.add_argument("--output-png", required=True, type=Path)
    args = parser.parse_args()
    if (args.endpoint_poses < 2 or min(
            args.pose_gate_m, args.corridor_radius_m,
            args.point_gate_m) <= 0.0):
        parser.error("counts and distance gates must be positive")
    analyses = [analyze_case(case, args) for case in args.case]
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps({"schema_version": 1,
                    "cases": [item[0] for item in analyses]},
                   indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_cases(analyses, args.output_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
