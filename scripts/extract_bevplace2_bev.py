#!/usr/bin/env python3
"""Extract BEVPlace++-compatible BEV images from a ROS 2 bag.

The projection intentionally follows the upstream BEVPlace++ generator:
80 m square, 0.4 m cells, voxel de-duplication, then capped point counts.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8


def pointcloud_xyz(message) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    required = [fields.get(name) for name in ("x", "y", "z")]
    if any(field is None for field in required):
        raise ValueError("PointCloud2 has no x/y/z fields")
    formats = []
    for field in required:
        if field.datatype == POINT_FIELD_FLOAT32:
            formats.append("<f4")
        elif field.datatype == POINT_FIELD_FLOAT64:
            formats.append("<f8")
        else:
            raise ValueError(f"unsupported point field type for {field.name}")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": formats,
        "offsets": [field.offset for field in required],
        "itemsize": message.point_step,
    })
    points = np.frombuffer(message.data, dtype=dtype, count=message.width * message.height)
    return np.column_stack((points["x"], points["y"], points["z"])).astype(np.float32, copy=False)


def bev_image(points: np.ndarray) -> np.ndarray:
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(np.abs(points[:, 0]) < 40.0) &
                    (np.abs(points[:, 1]) < 40.0) &
                    (np.abs(points[:, 2]) < 40.0)]
    image = np.zeros((201, 201), dtype=np.uint8)
    if points.size == 0:
        return image

    voxels = np.floor(points / 0.4).astype(np.int32)
    _, unique_indices = np.unique(voxels, axis=0, return_index=True)
    points = points[np.sort(unique_indices)]
    x_index = 100 - np.floor(points[:, 1] / 0.4).astype(np.int32)
    y_index = 100 - np.floor(points[:, 0] / 0.4).astype(np.int32)
    valid = ((x_index >= 0) & (x_index < 201) &
             (y_index >= 0) & (y_index < 201))
    np.add.at(image, (y_index[valid], x_index[valid]), 1)
    image = np.minimum(image, 10).astype(np.uint8) * 10
    maximum = int(image.max())
    if maximum:
        image = (image.astype(np.float32) / maximum * 255.0).astype(np.uint8)
    return image


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", required=True)
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args()
    if args.stride < 1:
        raise ValueError("stride must be positive")

    args.output.mkdir(parents=True, exist_ok=True)
    image_dir = args.output / "bev"
    image_dir.mkdir(exist_ok=True)
    manifest_path = args.output / "frames.csv"

    with AnyReader([args.bag]) as reader, manifest_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("index", "timestamp_s", "image"))
        connections = [connection for connection in reader.connections
                       if connection.topic == args.topic]
        if not connections:
            raise RuntimeError(f"topic not found: {args.topic}")
        sampled = 0
        for frame_index, (_, timestamp_ns, rawdata) in enumerate(reader.messages(connections=connections)):
            if frame_index % args.stride:
                continue
            message = reader.deserialize(rawdata, connections[0].msgtype)
            image_name = f"{sampled:06d}.png"
            if not cv2.imwrite(str(image_dir / image_name), bev_image(pointcloud_xyz(message))):
                raise RuntimeError(f"failed to write {image_name}")
            writer.writerow((sampled, f"{timestamp_ns / 1e9:.9f}", f"bev/{image_name}"))
            sampled += 1
    print(f"frames={sampled}")
    print(f"manifest={manifest_path}")


if __name__ == "__main__":
    main()
