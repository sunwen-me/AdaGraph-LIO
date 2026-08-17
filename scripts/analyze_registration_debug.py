#!/usr/bin/env python3

"""Quantify and plot saved source/target/aligned loop-registration clouds."""

import argparse
import json
import math
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/adagraph-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree

from analyze_loop_map_layers import iter_xyz, read_pcd_layout


def load_xyz(path: Path) -> np.ndarray:
    cloud, _ = read_pcd_layout(path)
    chunks = list(iter_xyz(cloud))
    if not chunks:
        raise ValueError(f"{path}: no finite points")
    return np.concatenate(chunks).astype(np.float64, copy=False)


def bounds(points: np.ndarray) -> dict[str, list[float]]:
    return {
        "minimum": [float(value) for value in points.min(axis=0)],
        "maximum": [float(value) for value in points.max(axis=0)],
        "extent": [float(value) for value in np.ptp(points, axis=0)],
    }


def nearest_metrics(source: np.ndarray, target: np.ndarray,
                    gate_m: float) -> tuple[dict[str, float], np.ndarray, np.ndarray]:
    distance, index = cKDTree(target).query(source, workers=-1)
    dz = source[:, 2] - target[index, 2]
    inlier = distance <= gate_m
    selected_distance = distance[inlier]
    selected_dz = np.abs(dz[inlier])
    metrics = {
        "overlap": float(np.mean(inlier)),
        "nn_distance_median_m": float(np.median(distance)),
        "nn_distance_p95_m": float(np.percentile(distance, 95)),
        "inlier_nn_rmse_m": (
            float(np.sqrt(np.mean(selected_distance ** 2)))
            if selected_distance.size else math.inf
        ),
        "inlier_abs_dz_median_m": (
            float(np.median(selected_dz)) if selected_dz.size else math.inf
        ),
        "inlier_abs_dz_p95_m": (
            float(np.percentile(selected_dz, 95))
            if selected_dz.size else math.inf
        ),
    }
    return metrics, distance, dz


def sample(points: np.ndarray, limit: int = 40_000) -> np.ndarray:
    if points.shape[0] <= limit:
        return points
    indices = np.random.default_rng(11).choice(points.shape[0], limit,
                                                replace=False)
    return points[indices]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--target", type=Path, required=True)
    parser.add_argument("--aligned", type=Path, required=True)
    parser.add_argument("--gate-m", type=float, default=1.0)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.gate_m) or args.gate_m <= 0.0:
        parser.error("gate-m must be finite and positive")
    for path in (args.source, args.target, args.aligned):
        if not path.is_file():
            parser.error(f"missing PCD: {path}")

    source = load_xyz(args.source)
    target = load_xyz(args.target)
    aligned = load_xyz(args.aligned)
    aligned_to_target, distance, dz = nearest_metrics(
        aligned, target, args.gate_m
    )
    target_to_aligned, _, _ = nearest_metrics(target, aligned, args.gate_m)
    result = {
        "schema_version": 1,
        "gate_m": args.gate_m,
        "source": {"path": str(args.source.resolve()),
                   "points": int(source.shape[0]), **bounds(source)},
        "target": {"path": str(args.target.resolve()),
                   "points": int(target.shape[0]), **bounds(target)},
        "aligned": {"path": str(args.aligned.resolve()),
                    "points": int(aligned.shape[0]), **bounds(aligned)},
        "aligned_to_target": aligned_to_target,
        "target_to_aligned": target_to_aligned,
        "bidirectional_overlap": min(
            aligned_to_target["overlap"], target_to_aligned["overlap"]
        ),
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    source_plot = sample(source)
    target_plot = sample(target)
    aligned_plot = sample(aligned)
    figure, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    axes[0, 0].scatter(source_plot[:, 0], source_plot[:, 1], s=0.4,
                       alpha=0.25, label="source (query)")
    axes[0, 0].scatter(target_plot[:, 0], target_plot[:, 1], s=0.4,
                       alpha=0.25, label="target (match)")
    axes[0, 0].set_title("Prepared clouds before alignment (XY)")
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].legend(markerscale=8)

    axes[0, 1].scatter(target_plot[:, 0], target_plot[:, 1], s=0.4,
                       alpha=0.25, label="target")
    axes[0, 1].scatter(aligned_plot[:, 0], aligned_plot[:, 1], s=0.4,
                       alpha=0.25, label="aligned source")
    axes[0, 1].set_title("Fine registration result (XY)")
    axes[0, 1].set_aspect("equal", adjustable="box")
    axes[0, 1].legend(markerscale=8)

    axes[1, 0].scatter(target_plot[:, 0], target_plot[:, 2], s=0.4,
                       alpha=0.25, label="target")
    axes[1, 0].scatter(aligned_plot[:, 0], aligned_plot[:, 2], s=0.4,
                       alpha=0.25, label="aligned source")
    axes[1, 0].set_title("Fine registration result (XZ)")
    axes[1, 0].set_xlabel("X [m]")
    axes[1, 0].set_ylabel("Z [m]")
    axes[1, 0].legend(markerscale=8)

    finite = np.isfinite(distance) & np.isfinite(dz)
    axes[1, 1].hist2d(distance[finite], np.abs(dz[finite]), bins=100,
                      range=((0.0, min(3.0, float(np.percentile(distance, 99)))),
                             (0.0, min(3.0, float(np.percentile(np.abs(dz), 99))))),
                      cmap="viridis")
    axes[1, 1].axvline(args.gate_m, color="red", linestyle="--",
                       label=f"gate {args.gate_m:g} m")
    axes[1, 1].set_title("Aligned→target nearest-neighbor residual")
    axes[1, 1].set_xlabel("3D distance [m]")
    axes[1, 1].set_ylabel("absolute Z residual [m]")
    axes[1, 1].legend()
    for axis in axes.flat:
        axis.grid(alpha=0.15)
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output_png, dpi=180)
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
