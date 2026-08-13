#!/usr/bin/env python3
"""Compare two TUM trajectories at the reference timestamps.

The ROS recorder and the ROS-free DatasetRunner do not necessarily emit a
pose at the same scan cadence.  This tool therefore interpolates the
candidate translation and quaternion at every reference timestamp instead of
comparing line numbers.  It intentionally uses only the Python standard
library so it can also run in a clean validation environment.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


Pose = Tuple[float, float, float, float, float, float, float, float]
Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]  # x, y, z, w


def _parse(path: Path) -> List[Pose]:
    poses: List[Pose] = []
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
            values = tuple(float(field) for field in fields)
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{path}:{line_number}: non-finite value")
            if poses and values[0] <= poses[-1][0]:
                raise ValueError(
                    f"{path}:{line_number}: timestamps are not strictly increasing"
                )
            quaternion_norm = math.sqrt(sum(value * value for value in values[4:8]))
            if quaternion_norm < 1.0e-12:
                raise ValueError(f"{path}:{line_number}: zero-length quaternion")
            poses.append(values)  # type: ignore[arg-type]
    if not poses:
        raise ValueError(f"{path}: trajectory contains no poses")
    return poses


def _normalize(quaternion: Sequence[float]) -> Quaternion:
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(value / norm for value in quaternion)  # type: ignore[return-value]


def _slerp(first: Quaternion, second: Quaternion, fraction: float) -> Quaternion:
    first = _normalize(first)
    second = _normalize(second)
    dot = sum(a * b for a, b in zip(first, second))
    if dot < 0.0:
        second = tuple(-value for value in second)  # type: ignore[assignment]
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 1.0 - 1.0e-10:
        return _normalize(
            tuple(a + fraction * (b - a) for a, b in zip(first, second))
        )
    angle = math.acos(dot)
    sine = math.sin(angle)
    first_weight = math.sin((1.0 - fraction) * angle) / sine
    second_weight = math.sin(fraction * angle) / sine
    return _normalize(
        tuple(
            first_weight * a + second_weight * b
            for a, b in zip(first, second)
        )
    )


def _interpolate(
    candidate: Sequence[Pose], timestamp: float, endpoint_tolerance: float
) -> Optional[Tuple[Vector3, Quaternion, float]]:
    times = [pose[0] for pose in candidate]
    index = bisect.bisect_left(times, timestamp)
    if index == 0:
        if abs(timestamp - times[0]) > endpoint_tolerance:
            return None
        pose = candidate[0]
        return (tuple(pose[1:4]), _normalize(pose[4:8]), abs(timestamp - times[0]))
    if index == len(candidate):
        if abs(timestamp - times[-1]) > endpoint_tolerance:
            return None
        pose = candidate[-1]
        return (tuple(pose[1:4]), _normalize(pose[4:8]), abs(timestamp - times[-1]))

    before = candidate[index - 1]
    after = candidate[index]
    interval = after[0] - before[0]
    if interval <= 0.0:
        raise ValueError("candidate timestamps are not strictly increasing")
    fraction = (timestamp - before[0]) / interval
    position = tuple(
        before[axis] + fraction * (after[axis] - before[axis])
        for axis in range(1, 4)
    )
    rotation = _slerp(
        _normalize(before[4:8]), _normalize(after[4:8]), fraction
    )
    nearest_time_error = min(timestamp - before[0], after[0] - timestamp)
    return position, rotation, nearest_time_error


def _rotation_error(first: Quaternion, second: Quaternion) -> float:
    dot = abs(sum(a * b for a, b in zip(_normalize(first), _normalize(second))))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def _path_length(poses: Sequence[Pose]) -> float:
    return sum(
        math.dist(previous[1:4], current[1:4])
        for previous, current in zip(poses, poses[1:])
    )


def _error_stats(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {}
    return {
        "mean": sum(values) / len(values),
        "rmse": math.sqrt(sum(value * value for value in values) / len(values)),
        "p95": _percentile(values, 0.95),
        "max": max(values),
    }


def compare(
    reference_path: Path,
    candidate_path: Path,
    endpoint_tolerance: float,
) -> Dict[str, object]:
    reference = _parse(reference_path)
    candidate = _parse(candidate_path)
    translations: List[float] = []
    rotations: List[float] = []
    nearest_time_errors: List[float] = []
    reference_timestamps: List[float] = []
    missing_timestamps: List[float] = []

    for pose in reference:
        interpolated = _interpolate(candidate, pose[0], endpoint_tolerance)
        if interpolated is None:
            missing_timestamps.append(pose[0])
            continue
        position, rotation, nearest_time_error = interpolated
        translations.append(math.dist(pose[1:4], position))
        rotations.append(_rotation_error(_normalize(pose[4:8]), rotation))
        nearest_time_errors.append(nearest_time_error)
        reference_timestamps.append(pose[0])

    result: Dict[str, object] = {
        "schema_version": 1,
        "reference": {
            "path": str(reference_path.resolve()),
            "pose_count": len(reference),
            "timestamp_first": reference[0][0],
            "timestamp_last": reference[-1][0],
            "path_length_m": _path_length(reference),
        },
        "candidate": {
            "path": str(candidate_path.resolve()),
            "pose_count": len(candidate),
            "timestamp_first": candidate[0][0],
            "timestamp_last": candidate[-1][0],
            "path_length_m": _path_length(candidate),
        },
        "endpoint_tolerance_seconds": endpoint_tolerance,
        "matched_reference_pose_count": len(translations),
        "missing_reference_pose_count": len(missing_timestamps),
        "coverage": len(translations) / len(reference),
        "matched_timestamp_first": reference_timestamps[0]
        if reference_timestamps
        else None,
        "matched_timestamp_last": reference_timestamps[-1]
        if reference_timestamps
        else None,
        "maximum_nearest_candidate_time_error_seconds": max(nearest_time_errors)
        if nearest_time_errors
        else None,
        "translation_error_m": _error_stats(translations),
        "rotation_error_rad": _error_stats(rotations),
        "missing_reference_timestamps": missing_timestamps,
    }
    return result


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--endpoint-tolerance",
        type=float,
        default=1.0e-6,
        help="allow endpoint timestamp clamping within this many seconds",
    )
    parser.add_argument("--minimum-coverage", type=float)
    parser.add_argument("--maximum-translation-rmse", type=float)
    parser.add_argument("--maximum-translation-error", type=float)
    parser.add_argument("--maximum-rotation-error", type=float)
    return parser.parse_args()


def main() -> None:
    args = _args()
    if args.endpoint_tolerance < 0.0 or not math.isfinite(args.endpoint_tolerance):
        raise ValueError("--endpoint-tolerance must be finite and non-negative")
    result = compare(args.reference, args.candidate, args.endpoint_tolerance)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    temporary.replace(args.output)
    translation = result["translation_error_m"]
    rotation = result["rotation_error_rad"]
    print(
        "matched="
        f"{result['matched_reference_pose_count']}/"
        f"{result['reference']['pose_count']} "
        f"translation_rmse_m={translation.get('rmse', float('nan')):.9f} "
        f"translation_max_m={translation.get('max', float('nan')):.9f} "
        f"rotation_max_rad={rotation.get('max', float('nan')):.9f}"
    )

    gates = (
        ("coverage", args.minimum_coverage, result["coverage"]),
        (
            "translation_rmse_m",
            args.maximum_translation_rmse,
            translation.get("rmse", math.inf),
        ),
        (
            "translation_max_m",
            args.maximum_translation_error,
            translation.get("max", math.inf),
        ),
        (
            "rotation_max_rad",
            args.maximum_rotation_error,
            rotation.get("max", math.inf),
        ),
    )
    failures = []
    for name, limit, value in gates:
        if limit is None:
            continue
        if not math.isfinite(limit) or limit < 0.0:
            raise ValueError(f"{name} gate must be finite and non-negative")
        if name == "coverage":
            passed = value >= limit
        else:
            passed = value <= limit
        if not passed:
            failures.append(f"{name}={value} violates gate {limit}")
    if failures:
        raise SystemExit("trajectory parity gate failed: " + "; ".join(failures))


if __name__ == "__main__":
    main()
