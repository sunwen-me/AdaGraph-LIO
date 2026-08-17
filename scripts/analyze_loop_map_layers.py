#!/usr/bin/env python3

"""Measure and visualize repeated height layers near a known loop corridor.

The reference corridor is taken from an early section of the backend trajectory.
Map points are assigned to their nearest reference pose in XY, then expressed as
height relative to that pose.  A later traversal that is vertically inconsistent
with the early traversal appears as a second dense band in arc-length/height.
"""

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple

os.environ.setdefault("MPLCONFIGDIR", "/tmp/adagraph-matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.signal import find_peaks
from scipy.spatial import cKDTree


def read_pcd_layout(path: Path) -> Tuple[np.memmap, int]:
    header: Dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: PCD header ended before DATA")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            words = text.split()
            tag = words[0].upper()
            if tag == "DATA":
                if len(words) != 2 or words[1].lower() != "binary":
                    raise ValueError(f"{path}: only binary PCD is supported")
                data_offset = stream.tell()
                break
            header[tag] = words[1:]

    names = header["FIELDS"]
    sizes = [int(value) for value in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(value) for value in header.get("COUNT", ["1"] * len(names))]
    if not (len(names) == len(sizes) == len(types) == len(counts)):
        raise ValueError(f"{path}: inconsistent field metadata")
    if any(count != 1 for count in counts):
        raise ValueError(f"{path}: vector PCD fields are unsupported")
    formats = {
        ("F", 4): "<f4", ("F", 8): "<f8", ("I", 1): "i1",
        ("I", 2): "<i2", ("I", 4): "<i4", ("I", 8): "<i8",
        ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
        ("U", 8): "<u8",
    }
    offsets = []
    dtype_fields = []
    record_size = 0
    for name, size, type_code in zip(names, sizes, types):
        try:
            dtype_fields.append((name, formats[(type_code, size)]))
        except KeyError as error:
            raise ValueError(
                f"{path}: unsupported PCD field {name} {type_code}/{size}"
            ) from error
        offsets.append(record_size)
        record_size += size
    if not all(axis in names for axis in ("x", "y", "z")):
        raise ValueError(f"{path}: x/y/z fields are required")
    dtype = np.dtype({
        "names": names,
        "formats": [field[1] for field in dtype_fields],
        "offsets": offsets,
        "itemsize": record_size,
    })
    point_count = int(header.get("POINTS", [header["WIDTH"][0]])[0])
    expected_size = data_offset + point_count * record_size
    if path.stat().st_size != expected_size:
        raise ValueError(
            f"{path}: size {path.stat().st_size} != expected {expected_size}"
        )
    return np.memmap(path, mode="r", dtype=dtype, offset=data_offset,
                     shape=(point_count,)), point_count


def load_tum(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 8 or values.shape[0] < 2:
        raise ValueError(f"{path}: expected a TUM trajectory with 8 columns")
    if not np.isfinite(values).all():
        raise ValueError(f"{path}: non-finite trajectory values")
    return values


def parse_case(value: str) -> Tuple[str, Path, Path]:
    parts = value.split("|", 2)
    if len(parts) != 3 or not parts[0]:
        raise argparse.ArgumentTypeError(
            "case must be LABEL|/absolute/map.pcd|/absolute/trajectory.tum"
        )
    label, pcd, trajectory = parts[0], Path(parts[1]), Path(parts[2])
    if not pcd.is_absolute() or not trajectory.is_absolute():
        raise argparse.ArgumentTypeError("case paths must be absolute")
    return label, pcd, trajectory


def iter_xyz(cloud: np.memmap, chunk_size: int = 1_000_000) -> Iterable[np.ndarray]:
    for start in range(0, cloud.shape[0], chunk_size):
        chunk = cloud[start:start + chunk_size]
        xyz = np.column_stack((chunk["x"], chunk["y"], chunk["z"]))
        finite = np.isfinite(xyz).all(axis=1)
        yield xyz[finite].astype(np.float32, copy=False)


def collect_corridor_points(
    cloud: np.memmap,
    reference_xyz: np.ndarray,
    reference_s: np.ndarray,
    corridor_radius_m: float,
    z_min_m: float,
    z_max_m: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    tree = cKDTree(reference_xyz[:, :2])
    along_parts = []
    height_parts = []
    distance_parts = []
    xy_parts = []
    for xyz in iter_xyz(cloud):
        distance, nearest = tree.query(xyz[:, :2], workers=-1)
        relative_z = xyz[:, 2] - reference_xyz[nearest, 2]
        selected = (
            (distance <= corridor_radius_m)
            & (relative_z >= z_min_m)
            & (relative_z <= z_max_m)
        )
        if not np.any(selected):
            continue
        along_parts.append(reference_s[nearest[selected]].astype(np.float32))
        height_parts.append(relative_z[selected].astype(np.float32))
        distance_parts.append(distance[selected].astype(np.float32))
        xy_parts.append(xyz[selected, :2])
    if not along_parts:
        empty = np.empty(0, dtype=np.float32)
        return empty, empty, empty, np.empty((0, 2), dtype=np.float32)
    return (
        np.concatenate(along_parts),
        np.concatenate(height_parts),
        np.concatenate(distance_parts),
        np.concatenate(xy_parts),
    )


def detect_layered_path_bins(
    along: np.ndarray,
    height: np.ndarray,
    path_length_m: float,
    along_bin_m: float,
    height_bin_m: float,
    z_min_m: float,
    z_max_m: float,
) -> Dict[str, Any]:
    along_edges = np.arange(0.0, path_length_m + along_bin_m, along_bin_m)
    height_edges = np.arange(z_min_m, z_max_m + height_bin_m, height_bin_m)
    histogram, _, _ = np.histogram2d(along, height,
                                     bins=(along_edges, height_edges))
    height_centers = 0.5 * (height_edges[:-1] + height_edges[1:])
    evaluated = 0
    layered = 0
    separations = []
    secondary_ratios = []
    dominant_modes = []
    bin_details = []
    minimum_peak_distance = max(1, int(round(2.0 / height_bin_m)))
    for index, counts in enumerate(histogram):
        total = int(counts.sum())
        if total < 200:
            continue
        smoothed = gaussian_filter1d(counts.astype(np.float64), sigma=1.0)
        maximum = float(smoothed.max())
        if maximum <= 0.0:
            continue
        peaks, properties = find_peaks(
            smoothed,
            distance=minimum_peak_distance,
            prominence=max(5.0, maximum * 0.06),
            height=max(5.0, maximum * 0.08),
        )
        evaluated += 1
        if peaks.size:
            dominant = int(peaks[np.argmax(smoothed[peaks])])
            dominant_modes.append(float(height_centers[dominant]))
        best_pair = None
        for left_pos in range(peaks.size):
            for right_pos in range(left_pos + 1, peaks.size):
                left = int(peaks[left_pos])
                right = int(peaks[right_pos])
                separation = float(height_centers[right] - height_centers[left])
                if separation < 2.0 or separation > 14.0:
                    continue
                lower_peak = min(smoothed[left], smoothed[right])
                higher_peak = max(smoothed[left], smoothed[right])
                ratio = float(lower_peak / higher_peak)
                valley = float(smoothed[left:right + 1].min())
                if ratio < 0.15 or valley > 0.45 * lower_peak:
                    continue
                score = ratio * separation
                if best_pair is None or score > best_pair[0]:
                    best_pair = (score, separation, ratio,
                                 float(height_centers[left]),
                                 float(height_centers[right]))
        if best_pair is not None:
            layered += 1
            separations.append(best_pair[1])
            secondary_ratios.append(best_pair[2])
            bin_details.append({
                "arc_start_m": float(along_edges[index]),
                "arc_end_m": float(along_edges[index + 1]),
                "lower_mode_m": best_pair[3],
                "upper_mode_m": best_pair[4],
                "separation_m": best_pair[1],
                "secondary_to_primary_ratio": best_pair[2],
                "points": total,
            })
    fraction = float(layered / evaluated) if evaluated else 0.0
    return {
        "evaluated_path_bins": evaluated,
        "layered_path_bins": layered,
        "layered_path_fraction": fraction,
        "median_layer_separation_m": (
            float(np.median(separations)) if separations else None
        ),
        "median_secondary_to_primary_ratio": (
            float(np.median(secondary_ratios)) if secondary_ratios else None
        ),
        "dominant_mode_median_m": (
            float(np.median(dominant_modes)) if dominant_modes else None
        ),
        "layered_bins": bin_details,
        "histogram": histogram,
        "along_edges": along_edges,
        "height_edges": height_edges,
    }


def analyze_case(
    case: Tuple[str, Path, Path],
    reference_start: int,
    reference_end: int,
    corridor_radius_m: float,
    z_min_m: float,
    z_max_m: float,
    along_bin_m: float,
    height_bin_m: float,
) -> Tuple[Dict[str, Any], Dict[str, np.ndarray]]:
    label, pcd_path, trajectory_path = case
    if not pcd_path.is_file() or not trajectory_path.is_file():
        raise FileNotFoundError(f"missing inputs for {label}: {pcd_path}, {trajectory_path}")
    trajectory = load_tum(trajectory_path)
    end = min(reference_end, trajectory.shape[0])
    if reference_start < 0 or end - reference_start < 2:
        raise ValueError(f"{label}: invalid reference trajectory slice")
    reference_xyz = trajectory[reference_start:end, 1:4]
    reference_step = np.linalg.norm(np.diff(reference_xyz[:, :2], axis=0), axis=1)
    reference_s = np.concatenate(([0.0], np.cumsum(reference_step)))
    cloud, point_count = read_pcd_layout(pcd_path)
    along, height, lateral_distance, xy = collect_corridor_points(
        cloud, reference_xyz, reference_s, corridor_radius_m, z_min_m, z_max_m
    )
    layer_metrics = detect_layered_path_bins(
        along, height, float(reference_s[-1]), along_bin_m, height_bin_m,
        z_min_m, z_max_m
    )
    histogram = layer_metrics.pop("histogram")
    along_edges = layer_metrics.pop("along_edges")
    height_edges = layer_metrics.pop("height_edges")
    result = {
        "label": label,
        "pcd_path": str(pcd_path.resolve()),
        "trajectory_path": str(trajectory_path.resolve()),
        "pcd_point_count": point_count,
        "trajectory_pose_count": int(trajectory.shape[0]),
        "reference_pose_range": [reference_start, end],
        "reference_path_length_m": float(reference_s[-1]),
        "corridor_radius_m": corridor_radius_m,
        "corridor_point_count": int(along.size),
        "relative_height_range_m": [z_min_m, z_max_m],
        "relative_height_percentiles_m": (
            [float(value) for value in np.percentile(height, [1, 10, 50, 90, 99])]
            if height.size else []
        ),
        **layer_metrics,
    }
    arrays = {
        "along": along,
        "height": height,
        "lateral_distance": lateral_distance,
        "xy": xy,
        "reference_xyz": reference_xyz,
        "histogram": histogram,
        "along_edges": along_edges,
        "height_edges": height_edges,
    }
    return result, arrays


def plot_cases(
    analyses: list[Tuple[Dict[str, Any], Dict[str, np.ndarray]]],
    output: Path,
) -> None:
    columns = len(analyses)
    figure, axes = plt.subplots(2, columns, figsize=(7.0 * columns, 9.0),
                                squeeze=False, constrained_layout=True)
    for column, (result, arrays) in enumerate(analyses):
        histogram = arrays["histogram"].T
        density = np.log1p(histogram)
        axes[0, column].imshow(
            density,
            origin="lower",
            aspect="auto",
            extent=(arrays["along_edges"][0], arrays["along_edges"][-1],
                    arrays["height_edges"][0], arrays["height_edges"][-1]),
            cmap="magma",
            interpolation="nearest",
        )
        axes[0, column].set_title(
            f"{result['label']}\nlayered bins "
            f"{result['layered_path_bins']}/{result['evaluated_path_bins']} "
            f"({100.0 * result['layered_path_fraction']:.1f}%)"
        )
        axes[0, column].set_xlabel("early-loop corridor arc length [m]")
        axes[0, column].set_ylabel("map point Z - early trajectory Z [m]")
        axes[0, column].grid(alpha=0.15)

        xy = arrays["xy"]
        height = arrays["height"]
        if xy.shape[0] > 400_000:
            selection = np.random.default_rng(7).choice(
                xy.shape[0], 400_000, replace=False
            )
            xy = xy[selection]
            height = height[selection]
        scatter = axes[1, column].scatter(
            xy[:, 0], xy[:, 1], c=height, s=0.15, alpha=0.35,
            cmap="turbo", vmin=result["relative_height_range_m"][0],
            vmax=result["relative_height_range_m"][1], rasterized=True,
        )
        reference = arrays["reference_xyz"]
        axes[1, column].plot(reference[:, 0], reference[:, 1], color="black",
                             linewidth=1.0, label="early reference path")
        axes[1, column].set_aspect("equal", adjustable="box")
        axes[1, column].set_title(
            f"{result['label']} corridor, color = relative Z"
        )
        axes[1, column].set_xlabel("map X [m]")
        axes[1, column].set_ylabel("map Y [m]")
        axes[1, column].legend(loc="best")
        figure.colorbar(scatter, ax=axes[1, column], label="relative Z [m]")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", type=parse_case, required=True)
    parser.add_argument("--reference-start", type=int, default=50)
    parser.add_argument("--reference-end", type=int, default=450)
    parser.add_argument("--corridor-radius-m", type=float, default=10.0)
    parser.add_argument("--z-min-m", type=float, default=-15.0)
    parser.add_argument("--z-max-m", type=float, default=3.0)
    parser.add_argument("--along-bin-m", type=float, default=5.0)
    parser.add_argument("--height-bin-m", type=float, default=0.25)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-png", type=Path, required=True)
    args = parser.parse_args()
    if args.reference_start < 0 or args.reference_end <= args.reference_start:
        parser.error("invalid reference pose range")
    positive = (args.corridor_radius_m, args.along_bin_m, args.height_bin_m)
    if not all(math.isfinite(value) and value > 0.0 for value in positive):
        parser.error("radius and bin sizes must be finite and positive")
    if not args.z_min_m < args.z_max_m:
        parser.error("z-min-m must be less than z-max-m")

    analyses = [
        analyze_case(
            case, args.reference_start, args.reference_end,
            args.corridor_radius_m, args.z_min_m, args.z_max_m,
            args.along_bin_m, args.height_bin_m,
        )
        for case in args.case
    ]
    serializable = {
        "schema_version": 1,
        "method": (
            "PCD points within the XY corridor around backend poses "
            f"[{args.reference_start}, {args.reference_end}) are expressed "
            "relative to the nearest pose height; separated dense histogram "
            "modes are counted per arc-length bin."
        ),
        "cases": [analysis[0] for analysis in analyses],
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(serializable, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    plot_cases(analyses, args.output_png)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
