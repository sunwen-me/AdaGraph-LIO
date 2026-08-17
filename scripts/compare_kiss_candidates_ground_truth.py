#!/usr/bin/env python3
"""Compare logged KISS coarse poses with a ground-truth trajectory.

The map frame is estimated from the first pass by aligning the normal Point-LIO
trajectory to ground truth. Candidate timestamps from later bag passes are
wrapped by the ground-truth duration before lookup.
"""

import argparse
import bisect
import math
import re
from pathlib import Path

import numpy as np


POSE_RE = re.compile(
    r"(?P<source>KISS (?:diagnostic )?coarse pose|prior fallback pose)"
    r".*?(?:submap_id=(?P<submap>\d+))?"
    r".*?timestamp_ns=(?P<timestamp_ns>[-+0-9]+)"
    r".*?retrieval_score=(?P<score>[-+0-9.eE]+)"
    r".*?tx=(?P<tx>[-+0-9.eE]+) ty=(?P<ty>[-+0-9.eE]+)"
    r" tz=(?P<tz>[-+0-9.eE]+)"
    r" qx=(?P<qx>[-+0-9.eE]+) qy=(?P<qy>[-+0-9.eE]+)"
    r" qz=(?P<qz>[-+0-9.eE]+) qw=(?P<qw>[-+0-9.eE]+)"
)


def read_tum(path: Path, quaternion_order="xyzw"):
    poses = []
    for raw in path.read_text().splitlines():
        fields = raw.split()
        if len(fields) != 8:
            continue
        values = np.asarray([float(value) for value in fields], dtype=float)
        if not np.isfinite(values).all():
            continue
        q = values[4:8]
        if quaternion_order == "wxyz":
            q = q[[1, 2, 3, 0]]
        q /= np.linalg.norm(q)
        poses.append((values[0], values[1:4], q))
    if len(poses) < 3:
        raise ValueError(f"not enough poses in {path}")
    return poses


def nearest_pose(poses, times, timestamp, tolerance):
    index = bisect.bisect_left(times, timestamp)
    candidates = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    if not candidates:
        return None
    pose = min(candidates, key=lambda item: abs(item[0] - timestamp))
    return pose if abs(pose[0] - timestamp) <= tolerance else None


def rotation_from_quaternion(q):
    x, y, z, w = q
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quaternion_from_rotation(rotation):
    trace = np.trace(rotation)
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(max(1e-15, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif index == 1:
            scale = math.sqrt(max(1e-15, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = math.sqrt(max(1e-15, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale
    result = np.asarray([x, y, z, w], dtype=float)
    return result / np.linalg.norm(result)


def quaternion_angle(first, second):
    dot = abs(float(np.dot(first, second)))
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def fit_map_to_ground_truth(estimated, ground_truth, tolerance):
    gt_times = [pose[0] for pose in ground_truth]
    estimated_points = []
    ground_truth_points = []
    for timestamp, position, _ in estimated:
        match = nearest_pose(ground_truth, gt_times, timestamp, tolerance)
        if match is not None:
            estimated_points.append(position)
            ground_truth_points.append(match[1])
    if len(estimated_points) < 3:
        raise ValueError("cannot align first-pass trajectory to ground truth")
    source = np.asarray(ground_truth_points)
    target = np.asarray(estimated_points)
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = target_mean - rotation @ source_mean
    return rotation, translation, len(estimated_points)


def alignment_error_summary(estimated, ground_truth, rotation, translation,
                            tolerance):
    gt_times = [pose[0] for pose in ground_truth]
    errors = []
    for timestamp, position, _ in estimated:
        match = nearest_pose(ground_truth, gt_times, timestamp, tolerance)
        if match is None:
            continue
        errors.append(np.linalg.norm(position -
                                     (rotation @ match[1] + translation)))
    if not errors:
        return {}
    values = np.asarray(errors)
    return {
        "min": float(values.min()),
        "median": float(np.median(values)),
        "max": float(values.max()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--estimated", type=Path, required=True)
    parser.add_argument(
        "--ground-truth-quaternion-order",
        choices=("xyzw", "wxyz"),
        default="wxyz",
        help="M2DGR ground-truth files store qw qx qy qz; estimated TUM stays xyzw.",
    )
    parser.add_argument("--match-tolerance", type=float, default=0.05)
    parser.add_argument(
        "--fit-before-seconds",
        type=float,
        help=(
            "Fit the map-to-ground-truth transform only with poses at most "
            "this many seconds after the ground-truth start. Use a time "
            "strictly before an injected fault so post-fault odometry cannot "
            "bias candidate error analysis."
        ),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    ground_truth = read_tum(
        args.ground_truth, args.ground_truth_quaternion_order
    )
    estimated_all = read_tum(args.estimated)
    gt_start = ground_truth[0][0]
    gt_end = ground_truth[-1][0]
    loop_period = gt_end - gt_start
    if args.fit_before_seconds is not None:
        if not math.isfinite(args.fit_before_seconds) or args.fit_before_seconds <= 0.0:
            raise ValueError("--fit-before-seconds must be finite and positive")
        fit_end = min(gt_end, gt_start + args.fit_before_seconds)
    else:
        fit_end = gt_end
    first_pass = [
        pose for pose in estimated_all
        if gt_start <= pose[0] <= fit_end
    ]
    rotation, translation, aligned_count = fit_map_to_ground_truth(
        first_pass, ground_truth, args.match_tolerance
    )

    times = [pose[0] for pose in ground_truth]
    rows = []
    for line in args.log.read_text().splitlines():
        match = POSE_RE.search(line)
        if not match:
            continue
        values = match.groupdict()
        timestamp = int(values["timestamp_ns"]) * 1e-9
        source_timestamp = timestamp
        while source_timestamp > gt_end + args.match_tolerance:
            source_timestamp -= loop_period
        truth = nearest_pose(ground_truth, times, source_timestamp, args.match_tolerance)
        if truth is None:
            continue
        truth_position = rotation @ truth[1] + translation
        kiss_position = np.asarray([float(values[name]) for name in ("tx", "ty", "tz")])
        truth_rotation = rotation @ rotation_from_quaternion(truth[2])
        kiss_rotation = rotation_from_quaternion(
            np.asarray([float(values[name]) for name in ("qx", "qy", "qz", "qw")])
        )
        rows.append(
            {
                "timestamp": timestamp,
                "source_timestamp": source_timestamp,
                "submap_id": (
                    int(values["submap"]) if values["submap"] else None
                ),
                "source": values["source"],
                "retrieval_score": float(values["score"]),
                "translation_error_m": float(np.linalg.norm(kiss_position - truth_position)),
                "rotation_error_deg": math.degrees(
                    quaternion_angle(
                        quaternion_from_rotation(kiss_rotation),
                        quaternion_from_rotation(truth_rotation),
                    )
                ),
            }
        )

    result = {
        "aligned_first_pass_poses": aligned_count,
        "fit_end_timestamp": fit_end,
        "fit_before_seconds": args.fit_before_seconds,
        "loop_period_seconds": loop_period,
        "candidate_count": len(rows),
        "candidates": rows,
        "first_pass_alignment_translation_error_m": alignment_error_summary(
            first_pass, ground_truth, rotation, translation,
            args.match_tolerance),
    }
    if rows:
        errors = np.asarray([row["translation_error_m"] for row in rows])
        angles = np.asarray([row["rotation_error_deg"] for row in rows])
        result["translation_error_m"] = {
            "min": float(errors.min()),
            "median": float(np.median(errors)),
            "max": float(errors.max()),
        }
        result["rotation_error_deg"] = {
            "min": float(angles.min()),
            "median": float(np.median(angles)),
            "max": float(angles.max()),
        }
    output = args.output or Path("-")
    payload = __import__("json").dumps(result, indent=2)
    if str(output) == "-":
        print(payload)
    else:
        output.write_text(payload + "\n")
        print(output)


if __name__ == "__main__":
    main()
