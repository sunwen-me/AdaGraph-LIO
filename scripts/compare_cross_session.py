#!/usr/bin/env python3
"""Evaluate a cross-session map trajectory in one shared GT frame."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def read_tum(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 4 or fields[0].startswith("#"):
            continue
        rows.append([float(fields[0]), *map(float, fields[1:4])])
    values = np.asarray(rows, dtype=float)
    if len(values) < 3:
        raise ValueError(f"not enough TUM poses: {path}")
    return values[:, 0], values[:, 1:4]


def interpolate(times: np.ndarray, points: np.ndarray, query: np.ndarray) -> np.ndarray:
    result = np.column_stack(
        [np.interp(query, times, points[:, axis]) for axis in range(3)]
    )
    return result


def fit_rigid(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def error_stats(estimated: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    errors = np.linalg.norm(estimated - truth, axis=1)
    return {
        "count": int(len(errors)),
        "rmse_m": float(np.sqrt(np.mean(errors**2))),
        "mean_m": float(np.mean(errors)),
        "median_m": float(np.median(errors)),
        "max_m": float(np.max(errors)),
        "first_m": float(errors[0]),
        "last_m": float(errors[-1]),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-session", type=Path, required=True)
    parser.add_argument("--map-ground-truth", type=Path, required=True)
    parser.add_argument("--query-trajectory", type=Path, required=True)
    parser.add_argument("--query-ground-truth", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--relocalization-time", type=float)
    parser.add_argument(
        "--fit-window-seconds",
        type=float,
        help="fit the shared frame using only this initial map-session interval",
    )
    args = parser.parse_args()

    map_time, map_trajectory = read_tum(args.map_session)
    map_gt_time, map_gt = read_tum(args.map_ground_truth)
    fit_query = map_gt_time[(map_gt_time >= map_time[0]) & (map_gt_time <= map_time[-1])]
    if args.fit_window_seconds is not None:
        if args.fit_window_seconds <= 0:
            raise ValueError("--fit-window-seconds must be positive")
        fit_query = fit_query[fit_query <= map_time[0] + args.fit_window_seconds]
    fit_source = interpolate(map_time, map_trajectory, fit_query)
    fit_target = interpolate(map_gt_time, map_gt, fit_query)
    rotation, translation = fit_rigid(fit_source, fit_target)

    query_time, query_trajectory = read_tum(args.query_trajectory)
    query_gt_time, query_gt = read_tum(args.query_ground_truth)
    common_time = query_gt_time[
        (query_gt_time >= query_time[0]) & (query_gt_time <= query_time[-1])
    ]
    query_map = interpolate(query_time, query_trajectory, common_time)
    query_estimated_global = (rotation @ query_map.T).T + translation
    query_truth = interpolate(query_gt_time, query_gt, common_time)

    result = {
        "map_alignment": {
            "fit_pose_count": int(len(fit_source)),
            "fit_rmse_m": error_stats((rotation @ fit_source.T).T + translation, fit_target)["rmse_m"],
        },
        "query": error_stats(query_estimated_global, query_truth),
        "query_time_first": float(common_time[0]),
        "query_time_last": float(common_time[-1]),
        "query_endpoint_estimated_m": query_estimated_global[-1].tolist(),
        "query_endpoint_ground_truth_m": query_truth[-1].tolist(),
    }
    if args.relocalization_time is not None:
        index = int(np.argmin(np.abs(common_time - args.relocalization_time)))
        result["relocalization"] = {
            "requested_time": args.relocalization_time,
            "matched_time": float(common_time[index]),
            "error_m": float(np.linalg.norm(query_estimated_global[index] - query_truth[index])),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
