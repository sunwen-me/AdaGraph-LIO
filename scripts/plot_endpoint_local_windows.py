#!/usr/bin/env python3

"""Render first/last-pass PCD overlays in local vertical windows."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/adagraph-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from analyze_loop_map_layers import iter_xyz, read_pcd_layout


def load_cloud(path: Path) -> np.ndarray:
    cloud, _ = read_pcd_layout(path)
    chunks = list(iter_xyz(cloud))
    if not chunks:
        raise ValueError(f"{path}: no finite points")
    return np.concatenate(chunks)


def sample(points: np.ndarray, limit: int, seed: int) -> np.ndarray:
    if points.shape[0] <= limit:
        return points
    indices = np.random.default_rng(seed).choice(
        points.shape[0], limit, replace=False
    )
    return points[indices]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--head", required=True, type=Path)
    parser.add_argument("--tail", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--start-node", type=int, default=-200)
    parser.add_argument("--end-node", type=int, default=-1)
    parser.add_argument("--windows", type=int, default=12)
    parser.add_argument("--radius-m", type=float, default=12.0)
    parser.add_argument("--points-per-pass", type=int, default=45_000)
    args = parser.parse_args()
    if args.windows < 2 or args.radius_m <= 0.0 or args.points_per_pass < 1:
        parser.error("window count, radius and point budget must be positive")

    trajectory = np.loadtxt(args.trajectory, dtype=np.float64)
    if trajectory.ndim != 2 or trajectory.shape[1] != 8:
        raise ValueError("trajectory must have eight TUM columns")
    start = args.start_node % trajectory.shape[0]
    end = args.end_node % trajectory.shape[0]
    if start >= end:
        raise ValueError("resolved start node must precede end node")
    nodes = np.linspace(start, end, args.windows, dtype=np.int64)

    head = load_cloud(args.head)
    tail = load_cloud(args.tail)
    head_tree = cKDTree(head[:, :2])
    tail_tree = cKDTree(tail[:, :2])

    rows = (args.windows + 1) // 2
    figure, axes = plt.subplots(
        rows, 4, figsize=(16, 4.2 * rows), squeeze=False,
        constrained_layout=True
    )
    for position, node in enumerate(nodes):
        center = trajectory[node, 1:4]
        rotation = Rotation.from_quat(trajectory[node, 4:8])
        forward = rotation.apply([1.0, 0.0, 0.0])[:2]
        norm = np.linalg.norm(forward)
        if norm < 1e-9:
            forward = np.array([1.0, 0.0])
        else:
            forward /= norm
        lateral = np.array([-forward[1], forward[0]])
        head_local = head[head_tree.query_ball_point(center[:2], args.radius_m)]
        tail_local = tail[tail_tree.query_ball_point(center[:2], args.radius_m)]
        head_local = sample(head_local, args.points_per_pass, 1000 + int(node))
        tail_local = sample(tail_local, args.points_per_pass, 2000 + int(node))
        row = position // 2
        column = 2 * (position % 2)
        for offset, (axis_vector, label) in enumerate(
            ((forward, "longitudinal"), (lateral, "lateral"))
        ):
            axis = axes[row, column + offset]
            for points, color, name in (
                (head_local, "tab:blue", "first pass"),
                (tail_local, "tab:orange", "last pass"),
            ):
                coordinate = (points[:, :2] - center[:2]) @ axis_vector
                axis.scatter(coordinate, points[:, 2], s=0.35, alpha=0.22,
                             color=color, label=name)
            axis.set_xlim(-args.radius_m, args.radius_m)
            axis.set_ylim(center[2] - 4.0, center[2] + 16.0)
            axis.set_title(
                f"node {node}, center=({center[0]:.1f},{center[1]:.1f},"
                f"{center[2]:.1f}) {label}"
            )
            axis.grid(alpha=0.15)
            if row == 0 and column + offset == 0:
                axis.legend(markerscale=8)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
