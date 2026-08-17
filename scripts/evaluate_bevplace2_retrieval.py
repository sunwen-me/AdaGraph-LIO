#!/usr/bin/env python3
"""Evaluate BEVPlace++ causal place retrieval against TUM ground truth."""

from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.models as torchvision_models


def load_tum_positions(path: Path) -> tuple[np.ndarray, np.ndarray]:
    tum = np.loadtxt(path, usecols=(0, 1, 2, 3), dtype=np.float64)
    return tum[:, 0], tum[:, 1:]


def associate_positions(timestamps: np.ndarray, gt_times: np.ndarray, gt_positions: np.ndarray) -> np.ndarray:
    right = np.searchsorted(gt_times, timestamps, side="left")
    right = np.clip(right, 0, len(gt_times) - 1)
    left = np.clip(right - 1, 0, len(gt_times) - 1)
    choose_left = np.abs(timestamps - gt_times[left]) <= np.abs(gt_times[right] - timestamps)
    return gt_positions[np.where(choose_left, left, right)]


def load_model(source: Path, checkpoint: Path):
    # The upstream constructor asks torchvision to fetch ImageNet weights even
    # though the BEVPlace++ checkpoint replaces every parameter.
    original_resnet34 = torchvision_models.resnet34
    def no_download_resnet34(*args, **kwargs):
        kwargs.pop("pretrained", None)
        kwargs["weights"] = None
        return original_resnet34(*args, **kwargs)
    torchvision_models.resnet34 = no_download_resnet34
    sys.path.insert(0, str(source))
    from REIN import REIN  # pylint: disable=import-outside-toplevel
    model = REIN().cuda().eval()
    state = torch.load(checkpoint, map_location="cuda", weights_only=False)
    missing, unexpected = model.load_state_dict(state["state_dict"], strict=False)
    if missing or unexpected:
        raise RuntimeError(f"checkpoint mismatch: missing={missing}, unexpected={unexpected}")
    return model


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("frames", type=Path, help="frames.csv from extract_bevplace2_bev.py")
    parser.add_argument("ground_truth", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--source", type=Path, required=True, help="BEVPlace++ checkout")
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--minimum-separation-s", type=float, default=30.0)
    parser.add_argument("--positive-radius-m", type=float, default=10.0)
    parser.add_argument("--top-k", type=int, default=5)
    args = parser.parse_args()

    rows = list(csv.DictReader(args.frames.open()))
    timestamps = np.asarray([float(row["timestamp_s"]) for row in rows])
    root = args.frames.parent
    gt_times, gt_positions = load_tum_positions(args.ground_truth)
    positions = associate_positions(timestamps, gt_times, gt_positions)
    model = load_model(args.source, args.checkpoint)

    descriptors = []
    with torch.inference_mode():
        for start in range(0, len(rows), args.batch_size):
            batch = []
            for row in rows[start:start + args.batch_size]:
                image = cv2.imread(str(root / row["image"]), cv2.IMREAD_GRAYSCALE)
                if image is None:
                    raise RuntimeError(f"failed to read {row['image']}")
                batch.append(np.repeat((image.astype(np.float32) / 256.0)[None], 3, axis=0))
            tensor = torch.from_numpy(np.stack(batch)).cuda(non_blocking=True)
            _, _, descriptor = model(tensor)
            descriptors.append(descriptor.cpu().numpy())
    descriptors = np.concatenate(descriptors)

    result_rows = []
    evaluable = 0
    top1_hits = 0
    topk_hits = 0
    for query in range(len(rows)):
        candidates = np.flatnonzero(timestamps <= timestamps[query] - args.minimum_separation_s)
        if not len(candidates):
            continue
        distances_to_gt = np.linalg.norm(positions[candidates] - positions[query], axis=1)
        positives = candidates[distances_to_gt <= args.positive_radius_m]
        if not len(positives):
            continue
        evaluable += 1
        scores = descriptors[candidates] @ descriptors[query]
        ranked = candidates[np.argsort(-scores)[:args.top_k]]
        hit1 = bool(np.linalg.norm(positions[ranked[0]] - positions[query]) <= args.positive_radius_m)
        hitk = bool(np.any(np.linalg.norm(positions[ranked] - positions[query], axis=1) <= args.positive_radius_m))
        top1_hits += hit1
        topk_hits += hitk
        result_rows.append({
            "query_index": int(query),
            "query_time_s": float(timestamps[query]),
            "top_candidates": [int(item) for item in ranked],
            "top_candidate_gt_distance_m": float(np.linalg.norm(positions[ranked[0]] - positions[query])),
            "top1_hit": hit1,
            "topk_hit": hitk,
        })
    summary = {
        "frames": len(rows),
        "evaluable_queries": evaluable,
        "minimum_separation_s": args.minimum_separation_s,
        "positive_radius_m": args.positive_radius_m,
        "top_k": args.top_k,
        "recall_at_1": top1_hits / evaluable if evaluable else None,
        "recall_at_k": topk_hits / evaluable if evaluable else None,
        "queries": result_rows,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({key: summary[key] for key in summary if key != "queries"}, indent=2))


if __name__ == "__main__":
    main()
