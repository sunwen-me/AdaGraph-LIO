#!/usr/bin/env python3
"""Align UrbanV2X NovAtel ground truth to a ROS1 bag and write TUM poses."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path


def _bag_time_bounds_ns(path: Path) -> tuple[int, int]:
    """Read only ROS1 chunk metadata, without decoding any message payloads."""
    try:
        from rosbags.rosbag1.reader import Header, RecordType, read_uint32
    except ImportError as exc:
        raise RuntimeError("install the Python 'rosbags' package to read ROS1 bags") from exc

    with path.open("rb") as bag:
        if bag.readline() != b"#ROSBAG V2.0\n":
            raise ValueError(f"{path} is not a ROS1 v2 bag")
        header = Header.read(bag, RecordType.BAGHEADER)
        index_pos = header.get_uint64("index_pos")
        connection_count = header.get_uint32("conn_count")
        chunk_count = header.get_uint32("chunk_count")
        if index_pos == 0 or chunk_count == 0:
            raise ValueError(f"{path} has no usable ROS1 index")

        bag.seek(index_pos)
        for _ in range(connection_count):
            Header.read(bag, RecordType.CONNECTION)
            Header.read(bag)

        starts: list[int] = []
        ends: list[int] = []
        for _ in range(chunk_count):
            chunk = Header.read(bag, RecordType.CHUNK_INFO)
            count = chunk.get_uint32("count")
            if count:
                starts.append(chunk.get_time("start_time"))
                ends.append(chunk.get_time("end_time"))
            bag.seek(4 + count * 8, os.SEEK_CUR)

    if not starts:
        raise ValueError(f"{path} has no timestamped chunks")
    return min(starts), max(ends)


def _rotation_to_quaternion(matrix: list[list[float]]) -> tuple[float, float, float, float]:
    """Convert a proper rotation matrix to xyzw quaternion order."""
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((m21 - m12) / scale, (m02 - m20) / scale,
                (m10 - m01) / scale, 0.25 * scale)
    if m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (0.25 * scale, (m01 + m10) / scale,
                (m02 + m20) / scale, (m21 - m12) / scale)
    if m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return ((m01 + m10) / scale, 0.25 * scale,
                (m12 + m21) / scale, (m02 - m20) / scale)
    scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return ((m02 + m20) / scale, (m12 + m21) / scale,
            0.25 * scale, (m10 - m01) / scale)


def _orientation_xyzw(roll_deg: float, pitch_deg: float,
                      heading_deg: float) -> tuple[float, float, float, float]:
    """NovAtel roll/pitch/heading to ROS ENU map-from-base orientation."""
    yaw = math.radians(90.0 - heading_deg)
    pitch = -math.radians(pitch_deg)
    roll = math.radians(roll_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    return _rotation_to_quaternion([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _enu_from_ecef(dx: float, dy: float, dz: float, latitude_deg: float,
                   longitude_deg: float) -> tuple[float, float, float]:
    latitude = math.radians(latitude_deg)
    longitude = math.radians(longitude_deg)
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    return (
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    )


def _read_rows(path: Path) -> list[dict[str, float]]:
    with path.open(encoding="utf-8") as source:
        names = source.readline().split()
        source.readline()  # Units row.
        required = {"UTCTime", "Latitude", "Longitude", "X-ECEF", "Y-ECEF", "Z-ECEF",
                    "Roll", "Pitch", "Heading"}
        missing = required - set(names)
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")
        indexes = {name: names.index(name) for name in required}
        rows: list[dict[str, float]] = []
        for line_number, line in enumerate(source, start=3):
            fields = line.split()
            if len(fields) != len(names):
                raise ValueError(f"{path}:{line_number}: expected {len(names)} columns")
            rows.append({name: float(fields[index]) for name, index in indexes.items()})
    if not rows:
        raise ValueError(f"{path} contains no ground-truth rows")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path)
    parser.add_argument(
        "--padding-seconds", type=float, default=0.02,
        help="GT retained before/after the bag range for endpoint interpolation (default: 0.02).",
    )
    args = parser.parse_args()
    if args.padding_seconds < 0.0:
        raise ValueError("--padding-seconds must be non-negative")

    bag_start_ns, bag_end_ns = _bag_time_bounds_ns(args.bag)
    bag_start = bag_start_ns * 1e-9
    bag_end = bag_end_ns * 1e-9
    rows = _read_rows(args.ground_truth)
    lower = bag_start - args.padding_seconds
    upper = bag_end + args.padding_seconds
    clipped = [row for row in rows if lower <= row["UTCTime"] <= upper]
    if len(clipped) < 2:
        raise ValueError("bag and ground truth have fewer than two overlapping samples")

    origin = clipped[0]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii") as output:
        for row in clipped:
            east, north, up = _enu_from_ecef(
                row["X-ECEF"] - origin["X-ECEF"],
                row["Y-ECEF"] - origin["Y-ECEF"],
                row["Z-ECEF"] - origin["Z-ECEF"],
                origin["Latitude"], origin["Longitude"],
            )
            qx, qy, qz, qw = _orientation_xyzw(
                row["Roll"], row["Pitch"], row["Heading"])
            output.write(
                f'{row["UTCTime"]:.9f} {east:.9f} {north:.9f} {up:.9f} '
                f"{qx:.12f} {qy:.12f} {qz:.12f} {qw:.12f}\n")

    summary = {
        "bag": str(args.bag),
        "ground_truth": str(args.ground_truth),
        "output": str(args.output),
        "bag_start_seconds": bag_start,
        "bag_end_seconds": bag_end,
        "bag_duration_seconds": (bag_end_ns - bag_start_ns) * 1e-9,
        "ground_truth_start_seconds": clipped[0]["UTCTime"],
        "ground_truth_end_seconds": clipped[-1]["UTCTime"],
        "sample_count": len(clipped),
        "padding_seconds": args.padding_seconds,
        "origin": {
            "latitude_deg": origin["Latitude"],
            "longitude_deg": origin["Longitude"],
            "ecef_m": [origin["X-ECEF"], origin["Y-ECEF"], origin["Z-ECEF"]],
        },
    }
    summary_path = args.summary or args.output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="ascii")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"align_urbanv2x_ground_truth: error: {exc}", file=sys.stderr)
        raise SystemExit(1)
