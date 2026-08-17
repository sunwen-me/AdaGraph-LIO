#!/usr/bin/env python3
"""Local GPU inference sidecar for the BEVPlace++ loop detector."""

from __future__ import annotations

import argparse
import os
import signal
import socket
import struct
import sys
from pathlib import Path

import cv2
import numpy as np
import torch
import torchvision.models as torchvision_models


REQUEST = struct.Struct("<4sI")
IMAGE_BYTES = 201 * 201
POSE_VALUES = 8
RANSAC_REPROJECTION_THRESHOLD_M = 0.5
RANSAC_ITERATIONS = 1000


def read_exact(connection: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def load_model(source: Path, checkpoint: Path):
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
    model.load_state_dict(state["state_dict"])
    return model


def fit_rigid_transform(source: np.ndarray,
                        target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Returns the 2D rigid transform mapping source coordinates to target."""
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    left, _, right_transpose = np.linalg.svd(covariance)
    correction = np.eye(2, dtype=np.float32)
    correction[1, 1] = np.linalg.det(right_transpose.T @ left.T)
    rotation = right_transpose.T @ correction @ left.T
    translation = target_center - rotation @ source_center
    return rotation.astype(np.float32), translation.astype(np.float32)


def rigid_ransac(source: np.ndarray,
                 target: np.ndarray) -> tuple[np.ndarray, np.ndarray, int, float] | None:
    """BEVPlace++-style two-point RANSAC with a metric rigid model."""
    if len(source) < 3:
        return None
    generator = np.random.default_rng(0)
    best_mask = None
    best_count = 0
    best_error = np.inf
    for _ in range(RANSAC_ITERATIONS):
        sample = generator.choice(len(source), size=2, replace=False)
        if np.linalg.norm(source[sample[0]] - source[sample[1]]) < 1e-3:
            continue
        if np.linalg.norm(target[sample[0]] - target[sample[1]]) < 1e-3:
            continue
        rotation, translation = fit_rigid_transform(source[sample], target[sample])
        errors = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
        mask = errors < RANSAC_REPROJECTION_THRESHOLD_M
        count = int(mask.sum())
        mean_error = float(errors[mask].mean()) if count else np.inf
        if count > best_count or (count == best_count and mean_error < best_error):
            best_mask = mask
            best_count = count
            best_error = mean_error
    if best_mask is None or best_count < 3:
        return None
    rotation, translation = fit_rigid_transform(source[best_mask], target[best_mask])
    errors = np.linalg.norm(source @ rotation.T + translation - target, axis=1)
    inliers = errors < RANSAC_REPROJECTION_THRESHOLD_M
    if int(inliers.sum()) < 3:
        return None
    rotation, translation = fit_rigid_transform(source[inliers], target[inliers])
    residual = float(np.linalg.norm(
        source[inliers] @ rotation.T + translation - target[inliers], axis=1).mean())
    return rotation, translation, int(inliers.sum()), residual


def estimate_planar_pose(model, query_image: np.ndarray,
                         match_image: np.ndarray) -> tuple[np.ndarray | None, str]:
    """Returns match<-query [R(2x2), translation, inliers, residual]."""
    images = np.stack((query_image, match_image)).astype(np.float32) / 256.0
    tensor = torch.from_numpy(np.repeat(images[:, None], 3, axis=1)).cuda(
        non_blocking=True)
    with torch.inference_mode():
        _, local_features, _ = model(tensor)
    local_features = local_features.permute(0, 2, 3, 1).float().cpu().numpy()

    detector = cv2.FastFeatureDetector_create(threshold=10, nonmaxSuppression=True)
    query_keypoints = detector.detect(query_image, None)
    match_keypoints = detector.detect(match_image, None)
    if len(query_keypoints) < 3 or len(match_keypoints) < 3:
        return None, (f"insufficient_fast_keypoints query={len(query_keypoints)} "
                      f"match={len(match_keypoints)}")

    def descriptors(keypoints, features):
        selected = []
        vectors = []
        for keypoint in keypoints:
            col = int(round(keypoint.pt[0]))
            row = int(round(keypoint.pt[1]))
            if 0 <= row < IMAGE_BYTES // 201 and 0 <= col < 201:
                selected.append(keypoint)
                vectors.append(features[row, col])
        return selected, np.asarray(vectors, dtype=np.float32)

    query_keypoints, query_descriptors = descriptors(query_keypoints, local_features[0])
    match_keypoints, match_descriptors = descriptors(match_keypoints, local_features[1])
    if len(query_descriptors) < 3 or len(match_descriptors) < 3:
        return None, (f"insufficient_feature_vectors query={len(query_descriptors)} "
                      f"match={len(match_descriptors)}")
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    # BEVPlace++ retains every nearest neighbor and lets rigid RANSAC reject
    # the large outlier set. A Lowe ratio gate is not part of that method.
    matches = [pair[0] for pair in matcher.knnMatch(
        query_descriptors, match_descriptors, k=1) if pair]
    if len(matches) < 3:
        return None, (f"insufficient_nearest_matches count={len(matches)} "
                      f"query_kp={len(query_keypoints)} match_kp={len(match_keypoints)}")

    def metric(keypoint):
        # BuildBevImage uses row=100-x/0.4 and column=100-y/0.4.
        return ((100.0 - keypoint.pt[1]) * 0.4,
                (100.0 - keypoint.pt[0]) * 0.4)

    query_points = np.float32([metric(query_keypoints[item.queryIdx])
                               for item in matches])
    match_points = np.float32([metric(match_keypoints[item.trainIdx])
                               for item in matches])
    pose = rigid_ransac(query_points, match_points)
    if pose is None:
        return None, f"rigid_ransac_failed nearest_matches={len(matches)}"
    rotation, translation, inliers, residual = pose
    return (np.asarray((rotation[0, 0], rotation[0, 1], rotation[1, 0],
                        rotation[1, 1], translation[0], translation[1],
                        inliers, residual),
                       dtype="<f4"),
            f"ok query_kp={len(query_keypoints)} match_kp={len(match_keypoints)} "
            f"nearest_matches={len(matches)} inliers={inliers} residual_m={residual:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    args = parser.parse_args()
    if args.socket.exists() and not args.socket.is_socket():
        raise RuntimeError(f"refusing to replace non-socket path: {args.socket}")
    if args.socket.exists():
        args.socket.unlink()
    model = load_model(args.source, args.checkpoint)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(str(args.socket))
    server.listen()
    print(f"ready socket={args.socket}", flush=True)

    def shutdown(_signal, _frame):
        server.close()
        if args.socket.exists() and args.socket.is_socket():
            args.socket.unlink()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    while True:
        connection, _ = server.accept()
        with connection:
            header = read_exact(connection, REQUEST.size)
            if header is None:
                continue
            magic, size = REQUEST.unpack(header)
            payload = read_exact(connection, size)
            if payload is None:
                connection.sendall(REQUEST.pack(b"ERR!", 0))
                continue
            if magic == b"BVP2" and size == IMAGE_BYTES:
                image = np.frombuffer(payload, dtype=np.uint8).reshape(201, 201)
                tensor = torch.from_numpy(
                    np.repeat((image.astype(np.float32) / 256.0)[None], 3, axis=0)[None]
                ).cuda(non_blocking=True)
                with torch.inference_mode():
                    _, _, descriptor = model(tensor)
                result = descriptor.cpu().numpy().astype("<f4", copy=False).ravel()
                connection.sendall(REQUEST.pack(b"BVR2", result.size))
                connection.sendall(result.tobytes())
                continue
            if magic == b"BVP3" and size == 2 * IMAGE_BYTES:
                images = np.frombuffer(payload, dtype=np.uint8).reshape(2, 201, 201)
                pose, detail = estimate_planar_pose(model, images[0], images[1])
                print(f"BVP3 {detail}", flush=True)
                if pose is not None:
                    connection.sendall(REQUEST.pack(b"BVR3", POSE_VALUES))
                    connection.sendall(pose.tobytes())
                    continue
            connection.sendall(REQUEST.pack(b"ERR!", 0))


if __name__ == "__main__":
    main()
