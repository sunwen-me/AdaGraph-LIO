#!/usr/bin/env python3
"""Convert an Oxford Spires sequence to a compact ROS 2 sensor bag.

The Oxford Spires raw LiDAR archive contains binary PCD files with the Hesai
point layout used by the dataset's ROS 2 bags.  This converter streams those
files from the ZIP archive, so extracting the 8.8 GB of PCD data is not
required.  Point-level ``timestamp`` and ``ring`` fields are preserved.
"""

from __future__ import annotations

import argparse
import csv
import math
import shutil
import struct
import sys
import zipfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


@dataclass(frozen=True)
class CloudEntry:
    timestamp_ns: int
    name: str


@dataclass(frozen=True)
class ImuEntry:
    timestamp_ns: int
    acc: tuple[float, float, float]
    gyro: tuple[float, float, float]


def parse_timestamp(text: str) -> int:
    whole, fraction = text.strip().split(".", 1)
    if not whole.isdigit() or not fraction.isdigit() or len(fraction) > 9:
        raise ValueError(f"invalid nanosecond timestamp: {text!r}")
    return int(whole) * 1_000_000_000 + int(fraction.ljust(9, "0"))


def format_timestamp(timestamp_ns: int) -> str:
    return f"{timestamp_ns // 1_000_000_000}.{timestamp_ns % 1_000_000_000:09d}"


def parse_cloud_entries(archive: zipfile.ZipFile) -> list[CloudEntry]:
    entries: list[CloudEntry] = []
    for info in archive.infolist():
        path = Path(info.filename)
        if path.suffix != ".pcd":
            continue
        entries.append(CloudEntry(parse_timestamp(path.stem), info.filename))
    entries.sort(key=lambda entry: entry.timestamp_ns)
    if not entries:
        raise ValueError(f"no PCD files found in {archive.filename}")
    return entries


def read_imu(path: Path) -> list[ImuEntry]:
    entries: list[ImuEntry] = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "secs",
            "nsecs",
            "acc_x",
            "acc_y",
            "acc_z",
            "ang_vel_x",
            "ang_vel_y",
            "ang_vel_z",
        }
        if not required.issubset(reader.fieldnames or ()):
            raise ValueError(
                "IMU CSV is missing fields: "
                + ", ".join(sorted(required - set(reader.fieldnames or ())))
            )
        for row in reader:
            timestamp_ns = int(row["secs"]) * 1_000_000_000 + int(row["nsecs"])
            values = tuple(
                float(row[name])
                for name in (
                    "acc_x",
                    "acc_y",
                    "acc_z",
                    "ang_vel_x",
                    "ang_vel_y",
                    "ang_vel_z",
                )
            )
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite IMU value at {timestamp_ns}")
            entries.append(ImuEntry(timestamp_ns, values[:3], values[3:]))
    entries.sort(key=lambda entry: entry.timestamp_ns)
    if not entries:
        raise ValueError(f"no IMU rows found in {path}")
    return entries


def read_binary_pcd(
    archive: zipfile.ZipFile, name: str
) -> tuple[int, bytes, dict[str, int]]:
    """Return width, binary point bytes, and field offsets from a PCD entry."""

    header: dict[str, list[str]] = {}
    with archive.open(name) as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"truncated PCD header: {name}")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            parts = decoded.split()
            key = parts[0].upper()
            header[key] = parts[1:]
            if key == "DATA":
                if parts[1].lower() != "binary":
                    raise ValueError(f"only binary PCD is supported: {name}")
                break

        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        counts = [int(value) for value in header.get("COUNT", [])]
        if not counts:
            counts = [1] * len(fields)
        if len(fields) != len(sizes) or len(fields) != len(counts):
            raise ValueError(f"inconsistent PCD field header: {name}")
        offsets: dict[str, int] = {}
        point_step = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets.setdefault(field, point_step)
            point_step += size * count

        required = {"x", "y", "z", "intensity", "timestamp", "ring"}
        if not required.issubset(offsets):
            raise ValueError(
                f"PCD {name} is missing fields: "
                + ", ".join(sorted(required - set(offsets)))
            )
        width = int(header.get("WIDTH", ["0"])[0])
        points = stream.read(width * point_step)
        if len(points) != width * point_step:
            raise ValueError(f"truncated PCD payload: {name}")
        return width, points, offsets | {"_point_step": point_step}


def make_point_cloud_message(
    timestamp_ns: int,
    width: int,
    data: bytes,
    offsets: dict[str, int],
    frame_id: str,
):
    from sensor_msgs.msg import PointCloud2, PointField

    message = PointCloud2()
    message.header.stamp.sec = timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = timestamp_ns % 1_000_000_000
    message.header.frame_id = frame_id
    message.height = 1
    message.width = width
    message.fields = [
        PointField(name="x", offset=offsets["x"], datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=offsets["y"], datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=offsets["z"], datatype=PointField.FLOAT32, count=1),
        PointField(
            name="intensity",
            offset=offsets["intensity"],
            datatype=PointField.FLOAT32,
            count=1,
        ),
        PointField(
            name="timestamp",
            offset=offsets["timestamp"],
            datatype=PointField.FLOAT64,
            count=1,
        ),
        PointField(name="ring", offset=offsets["ring"], datatype=PointField.UINT16, count=1),
    ]
    message.is_bigendian = False
    message.point_step = offsets["_point_step"]
    message.row_step = message.point_step * width
    message.data = data
    message.is_dense = False
    return message


def make_imu_message(entry: ImuEntry):
    from sensor_msgs.msg import Imu

    message = Imu()
    message.header.stamp.sec = entry.timestamp_ns // 1_000_000_000
    message.header.stamp.nanosec = entry.timestamp_ns % 1_000_000_000
    message.header.frame_id = "imu"
    message.linear_acceleration.x, message.linear_acceleration.y, message.linear_acceleration.z = entry.acc
    message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = entry.gyro
    message.orientation_covariance[0] = -1.0
    return message


def iter_selected_imu(
    entries: list[ImuEntry], start_ns: int, end_ns: int
) -> Iterator[ImuEntry]:
    for entry in entries:
        if start_ns <= entry.timestamp_ns <= end_ns:
            yield entry


def convert(args: argparse.Namespace) -> None:
    try:
        import rosbag2_py
        from rclpy.serialization import serialize_message
    except ImportError as exc:  # pragma: no cover - environment diagnostic
        raise RuntimeError("source ROS 2 before running this converter") from exc

    output = args.output.resolve()
    if output.exists():
        if not args.overwrite:
            raise FileExistsError(f"output already exists: {output}")
        if output.is_dir():
            shutil.rmtree(output)
        else:
            output.unlink()

    imu_entries = read_imu(args.imu_csv)
    with zipfile.ZipFile(args.lidar_zip) as archive:
        clouds = parse_cloud_entries(archive)
        if args.max_lidar_frames is not None:
            if args.max_lidar_frames <= 0:
                raise ValueError("--max-lidar-frames must be positive")
            clouds = clouds[: args.max_lidar_frames]
        if args.duration_seconds is not None:
            if args.duration_seconds <= 0:
                raise ValueError("--duration-seconds must be positive")
            cutoff = clouds[0].timestamp_ns + round(args.duration_seconds * 1e9)
            clouds = [entry for entry in clouds if entry.timestamp_ns <= cutoff]
        if not clouds:
            raise ValueError("selection contains no LiDAR frames")

        start_ns = clouds[0].timestamp_ns
        end_ns = clouds[-1].timestamp_ns
        selected_imu = iter_selected_imu(imu_entries, start_ns, end_ns + 200_000_000)
        imu_iter = iter(selected_imu)
        next_imu = next(imu_iter, None)

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(output), storage_id=args.storage_id),
            rosbag2_py.ConverterOptions("", ""),
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=args.imu_topic,
                type="sensor_msgs/msg/Imu",
                serialization_format="cdr",
                offered_qos_profiles=[],
                type_description_hash="",
            )
        )
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=1,
                name=args.lidar_topic,
                type="sensor_msgs/msg/PointCloud2",
                serialization_format="cdr",
                offered_qos_profiles=[],
                type_description_hash="",
            )
        )

        imu_count = 0
        lidar_count = 0
        point_count = 0
        first_point_delta = None
        last_point_delta = None
        for cloud in clouds:
            while next_imu is not None and next_imu.timestamp_ns <= cloud.timestamp_ns:
                message = make_imu_message(next_imu)
                writer.write(args.imu_topic, serialize_message(message), next_imu.timestamp_ns)
                imu_count += 1
                next_imu = next(imu_iter, None)

            width, data, offsets = read_binary_pcd(archive, cloud.name)
            if first_point_delta is None:
                timestamp_offset = offsets["timestamp"]
                point_step = offsets["_point_step"]
                first_point_time = struct.unpack_from("<d", data, timestamp_offset)[0]
                first_point_delta = first_point_time - cloud.timestamp_ns / 1e9
                last_point_time = struct.unpack_from(
                    "<d", data, (width - 1) * point_step + timestamp_offset
                )[0]
                last_point_delta = last_point_time - cloud.timestamp_ns / 1e9
            message = make_point_cloud_message(
                cloud.timestamp_ns, width, data, offsets, args.lidar_frame
            )
            writer.write(args.lidar_topic, serialize_message(message), cloud.timestamp_ns)
            lidar_count += 1
            point_count += width

        while next_imu is not None:
            message = make_imu_message(next_imu)
            writer.write(args.imu_topic, serialize_message(message), next_imu.timestamp_ns)
            imu_count += 1
            next_imu = next(imu_iter, None)
        del writer

    print(
        "converted "
        f"lidar_frames={lidar_count} lidar_points={point_count} imu={imu_count} "
        f"start={format_timestamp(start_ns)} end={format_timestamp(end_ns)} "
        f"first_point_offset_s={first_point_delta:.9f} "
        f"last_point_offset_s={last_point_delta:.9f} output={output}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lidar-zip", type=Path, required=True)
    parser.add_argument("--imu-csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--imu-topic", default="/alphasense_driver_ros/imu")
    parser.add_argument("--lidar-topic", default="/hesai/pandar")
    parser.add_argument("--lidar-frame", default="hesai")
    parser.add_argument(
        "--storage-id",
        choices=("mcap", "sqlite3"),
        default="mcap",
        help="rosbag2 storage backend (MCAP is more deterministic for large replay)",
    )
    parser.add_argument("--max-lidar-frames", type=int)
    parser.add_argument("--duration-seconds", type=float)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main() -> int:
    try:
        convert(build_parser().parse_args())
    except (FileExistsError, OSError, ValueError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
