#!/usr/bin/env python3

"""Compare loop-relative poses with GT after fitting the fixed body-frame axes."""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def load_tum(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 8 or values.shape[0] < 2:
        raise ValueError(f"{path}: expected a TUM trajectory")
    return values


def interpolate_gt(gt: np.ndarray, timestamps: np.ndarray) -> tuple[np.ndarray, Rotation]:
    indices = np.searchsorted(gt[:, 0], timestamps)
    indices = np.clip(indices, 1, gt.shape[0] - 1)
    left = gt[indices - 1]
    right = gt[indices]
    alpha = (timestamps - left[:, 0]) / (right[:, 0] - left[:, 0])
    positions = ((1.0 - alpha[:, None]) * left[:, 1:4]
                 + alpha[:, None] * right[:, 1:4])
    rotations = []
    for timestamp, first, second in zip(timestamps, left, right):
        rotations.append(Slerp(
            [first[0], second[0]],
            Rotation.from_quat(np.stack((first[4:8], second[4:8]))),
        )([timestamp]).as_quat()[0])
    return positions, Rotation.from_quat(np.asarray(rotations))


def fit_body_axes(estimated: np.ndarray, gt_positions: np.ndarray,
                  gt_rotations: Rotation, start: int, end: int,
                  stride: int, delta: int) -> tuple[Rotation, np.ndarray]:
    estimated_rotations = Rotation.from_quat(estimated[:, 4:8])
    gt_vectors = []
    estimated_vectors = []
    for match in range(start, min(end, estimated.shape[0] - delta), stride):
        query = match + delta
        gt_vectors.append(
            gt_rotations[match].inv().apply(
                gt_positions[query] - gt_positions[match]
            )
        )
        estimated_vectors.append(
            estimated_rotations[match].inv().apply(
                estimated[query, 1:4] - estimated[match, 1:4]
            )
        )
    gt_matrix = np.asarray(gt_vectors)
    estimated_matrix = np.asarray(estimated_vectors)
    left, _, right_transpose = np.linalg.svd(gt_matrix.T @ estimated_matrix)
    matrix = right_transpose.T @ left.T
    if np.linalg.det(matrix) < 0.0:
        right_transpose[-1] *= -1.0
        matrix = right_transpose.T @ left.T
    residual = np.linalg.norm((matrix @ gt_matrix.T).T - estimated_matrix,
                              axis=1)
    return Rotation.from_matrix(matrix), residual


def parse_candidate(text: str) -> tuple[int, int, np.ndarray, Rotation]:
    values = text.split(",")
    if len(values) != 8:
        raise argparse.ArgumentTypeError(
            "candidate must be query,match,x,y,z,roll,pitch,yaw"
        )
    query = int(values[0])
    match = int(values[1])
    numbers = np.asarray([float(value) for value in values[2:]], dtype=np.float64)
    return query, match, numbers[:3], Rotation.from_euler("xyz", numbers[3:])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimated", required=True, type=Path)
    parser.add_argument("--ground-truth", required=True, type=Path)
    parser.add_argument("--candidate", action="append", required=True,
                        type=parse_candidate)
    parser.add_argument("--calibration-start", type=int, default=10)
    parser.add_argument("--calibration-end", type=int, default=900)
    parser.add_argument("--calibration-stride", type=int, default=25)
    parser.add_argument("--calibration-delta", type=int, default=25)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    estimated = load_tum(args.estimated)
    ground_truth = load_tum(args.ground_truth)
    gt_positions, gt_rotations = interpolate_gt(ground_truth, estimated[:, 0])
    body_axes, calibration_residual = fit_body_axes(
        estimated, gt_positions, gt_rotations,
        args.calibration_start, args.calibration_end,
        args.calibration_stride, args.calibration_delta,
    )
    comparisons = []
    for query, match, measured_translation, measured_rotation in args.candidate:
        if query < 0 or match < 0 or query >= estimated.shape[0] or match >= estimated.shape[0]:
            raise ValueError(f"candidate node index out of range: {query}, {match}")
        gt_translation = gt_rotations[match].inv().apply(
            gt_positions[query] - gt_positions[match]
        )
        gt_rotation = gt_rotations[match].inv() * gt_rotations[query]
        expected_translation = body_axes.apply(gt_translation)
        expected_rotation = body_axes * gt_rotation * body_axes.inv()
        comparisons.append({
            "query_node": query,
            "match_node": match,
            "measured_translation_m": measured_translation.tolist(),
            "gt_translation_in_slam_base_m": expected_translation.tolist(),
            "translation_error_vector_m": (
                measured_translation - expected_translation
            ).tolist(),
            "translation_error_norm_m": float(np.linalg.norm(
                measured_translation - expected_translation
            )),
            "measured_rpy_rad": measured_rotation.as_euler("xyz").tolist(),
            "gt_rpy_in_slam_base_rad": expected_rotation.as_euler("xyz").tolist(),
            "rotation_error_rad": float(
                (expected_rotation.inv() * measured_rotation).magnitude()
            ),
            "query_timestamp": float(estimated[query, 0]),
            "match_timestamp": float(estimated[match, 0]),
        })
    result = {
        "schema_version": 1,
        "estimated_path": str(args.estimated.resolve()),
        "ground_truth_path": str(args.ground_truth.resolve()),
        "node_id_is_trajectory_row_index": True,
        "gt_body_to_slam_base_rpy_rad": body_axes.as_euler("xyz").tolist(),
        "body_axis_fit_translation_residual_median_m": float(
            np.median(calibration_residual)
        ),
        "body_axis_fit_translation_residual_p95_m": float(
            np.percentile(calibration_residual, 95)
        ),
        "comparisons": comparisons,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
