#!/usr/bin/env python3

"""Measure aligned GT error and first/last-pass closure for backend TUM paths."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def parse_case(text: str) -> tuple[str, Path]:
    parts = text.split("|", 1)
    if len(parts) != 2 or not parts[0]:
        raise argparse.ArgumentTypeError("case must be LABEL|trajectory.tum")
    path = Path(parts[1])
    if not path.is_absolute():
        raise argparse.ArgumentTypeError("trajectory path must be absolute")
    return parts[0], path


def load_tum(path: Path) -> np.ndarray:
    trajectory = np.loadtxt(path, dtype=np.float64)
    if (trajectory.ndim != 2 or trajectory.shape[1] != 8 or
            trajectory.shape[0] < 2 or not np.all(np.isfinite(trajectory))):
        raise ValueError(f"{path}: expected a finite eight-column TUM trajectory")
    if not np.all(np.diff(trajectory[:, 0]) > 0.0):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return trajectory


def interpolate_positions(reference: np.ndarray,
                          timestamps: np.ndarray) -> np.ndarray:
    if timestamps[0] < reference[0, 0] or timestamps[-1] > reference[-1, 0]:
        raise ValueError("trajectory timestamps fall outside ground-truth coverage")
    indices = np.searchsorted(reference[:, 0], timestamps)
    indices = np.clip(indices, 1, reference.shape[0] - 1)
    left = reference[indices - 1]
    right = reference[indices]
    alpha = ((timestamps - left[:, 0]) /
             (right[:, 0] - left[:, 0]))[:, None]
    return (1.0 - alpha) * left[:, 1:4] + alpha * right[:, 1:4]


def rigid_align(source: np.ndarray,
                target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right_transpose = np.linalg.svd(covariance)
    rotation = right_transpose.T @ left.T
    if np.linalg.det(rotation) < 0.0:
        right_transpose[-1] *= -1.0
        rotation = right_transpose.T @ left.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def summarize_case(label: str, path: Path, ground_truth: np.ndarray,
                   endpoint_poses: int, pose_gate_m: float) -> dict:
    trajectory = load_tum(path)
    truth = interpolate_positions(ground_truth, trajectory[:, 0])
    rotation, translation = rigid_align(trajectory[:, 1:4], truth)
    aligned = trajectory[:, 1:4] @ rotation.T + translation
    errors = np.linalg.norm(aligned - truth, axis=1)

    count = min(endpoint_poses, trajectory.shape[0] // 2)
    head = trajectory[:count, 1:4]
    tail = trajectory[-count:, 1:4]
    xy_distance, head_index = cKDTree(head[:, :2]).query(tail[:, :2], workers=-1)
    shared = xy_distance <= pose_gate_m
    if not np.any(shared):
        raise ValueError(f"{path}: no shared endpoint corridor")
    dz = tail[shared, 2] - head[head_index[shared], 2]
    abs_dz = np.abs(dz)

    return {
        "label": label,
        "trajectory_path": str(path.resolve()),
        "pose_count": int(trajectory.shape[0]),
        "raw_endpoint_m": trajectory[-1, 1:4].tolist(),
        "ape_se3_aligned_m": {
            "rmse": float(np.sqrt(np.mean(errors ** 2))),
            "mean": float(np.mean(errors)),
            "median": float(np.median(errors)),
            "maximum": float(np.max(errors)),
            "final": float(errors[-1]),
            "final_10_median": float(np.median(errors[-10:])),
        },
        "endpoint_revisit": {
            "endpoint_pose_count": count,
            "pose_gate_m": pose_gate_m,
            "shared_tail_pose_count": int(np.count_nonzero(shared)),
            "xy_distance_median_m": float(np.median(xy_distance[shared])),
            "signed_dz_p10_m": float(np.percentile(dz, 10)),
            "signed_dz_median_m": float(np.median(dz)),
            "signed_dz_p90_m": float(np.percentile(dz, 90)),
            "abs_dz_median_m": float(np.median(abs_dz)),
            "abs_dz_p90_m": float(np.percentile(abs_dz, 90)),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--case", required=True, action="append", type=parse_case)
    parser.add_argument("--endpoint-poses", type=int, default=500)
    parser.add_argument("--pose-gate-m", type=float, default=10.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.endpoint_poses < 2 or args.pose_gate_m <= 0.0:
        raise ValueError("endpoint limits must be positive")

    ground_truth = load_tum(args.ground_truth)
    result = {
        "schema_version": 1,
        "ground_truth_path": str(args.ground_truth.resolve()),
        "cases": [
            summarize_case(label, path, ground_truth, args.endpoint_poses,
                           args.pose_gate_m)
            for label, path in args.case
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
