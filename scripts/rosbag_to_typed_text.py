#!/usr/bin/env python3
"""Convert a ROS 2 bag into AdaGraph-LIO's typed text interchange format.

The converter is intentionally read-only with respect to the bag.  It
deserializes only the selected IMU and PointCloud2 topics, applies the same
Velodyne filtering/time-offset normalization as SensorBridge, and performs a
deterministic external sort by the packet timestamp used by DatasetRunner.

The bag receive order is not always sensor-time order: a Velodyne packet can
be recorded before IMU samples whose header timestamps are later than that
scan's end time.  Sorting the typed packets makes the offline player feed the
same ordered values into the frontend queues without changing the bag.
"""

from __future__ import annotations

import argparse
import heapq
import json
import math
import os
import shutil
import struct
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional


@dataclass(frozen=True)
class Packet:
    timestamp_ns: int
    sequence: int
    line: str


def _import_ros() -> tuple[object, object, object, object, object]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:  # pragma: no cover - environment diagnostic
        raise RuntimeError(
            "ROS 2 Python modules are required; source the configured ROS "
            "environment before running this converter"
        ) from exc
    return rosbag2_py, deserialize_message, get_message, point_cloud2, None


def _stamp_seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _seconds_to_ns(seconds: float) -> int:
    if not math.isfinite(seconds):
        raise ValueError("non-finite timestamp")
    value = round(seconds * 1.0e9)
    if value < -(1 << 63) or value > (1 << 63) - 1:
        raise ValueError("timestamp is outside int64 nanosecond range")
    return int(value)


def _float32(value: object) -> float:
    """Round a Python/numpy scalar to the float32 value seen by C++."""

    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def _finite_float32(value: object) -> bool:
    return math.isfinite(_float32(value))


def _field_names(msg: object) -> set[str]:
    return {str(field.name) for field in msg.fields}


def _read_velodyne_points(
    msg: object,
    point_cloud2: object,
    blind_squared: float,
    max_range_squared: float,
    filter_rate: int,
) -> tuple[list[tuple[float, float, float, float, float]], float, float]:
    fields = _field_names(msg)
    required = {"x", "y", "z", "intensity", "time"}
    missing = required - fields
    if missing:
        raise ValueError(
            "Velodyne PointCloud2 is missing fields: " + ", ".join(sorted(missing))
        )

    rows = point_cloud2.read_points(
        msg,
        field_names=["x", "y", "z", "intensity", "time"],
        skip_nans=False,
    )
    points: list[tuple[float, float, float, float, float]] = []
    minimum_offset = math.inf
    maximum_offset = -math.inf
    for index, row in enumerate(rows):
        if index % filter_rate != 0:
            continue
        x = _float32(row[0])
        y = _float32(row[1])
        z = _float32(row[2])
        intensity = _float32(row[3])
        offset = _float32(row[4])
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        distance_squared = x * x + y * y + z * z
        # SensorBridge uses strict blind/range comparisons.
        if distance_squared <= blind_squared or distance_squared >= max_range_squared:
            continue
        if not math.isfinite(offset):
            continue
        points.append((x, y, z, intensity, offset))
        minimum_offset = min(minimum_offset, offset)
        maximum_offset = max(maximum_offset, offset)

    return points, minimum_offset, maximum_offset


def _packet_line_for_imu(msg: object) -> Packet:
    timestamp = _stamp_seconds(msg.header.stamp)
    values = (
        timestamp,
        float(msg.linear_acceleration.x),
        float(msg.linear_acceleration.y),
        float(msg.linear_acceleration.z),
        float(msg.angular_velocity.x),
        float(msg.angular_velocity.y),
        float(msg.angular_velocity.z),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU message contains a non-finite value")
    line = "imu " + " ".join(format(value, ".17g") for value in values)
    return Packet(_seconds_to_ns(timestamp), 0, line)


def _packet_line_for_cloud(
    msg: object,
    point_cloud2: object,
    blind_squared: float,
    max_range_squared: float,
    filter_rate: int,
) -> Optional[Packet]:
    header_time = _stamp_seconds(msg.header.stamp)
    points, minimum_offset, maximum_offset = _read_velodyne_points(
        msg, point_cloud2, blind_squared, max_range_squared, filter_rate
    )
    if not points:
        return None
    start = header_time + minimum_offset
    end = header_time + maximum_offset
    start_ns = _seconds_to_ns(start)
    end_ns = _seconds_to_ns(end)
    normalized = [
        (x, y, z, intensity, offset - minimum_offset)
        for x, y, z, intensity, offset in points
    ]
    pieces = [
        "cloud",
        format(start, ".17g"),
        format(end, ".17g"),
        str(len(normalized)),
    ]
    for x, y, z, intensity, offset in normalized:
        pieces.extend(
            (
                format(x, ".9g"),
                format(y, ".9g"),
                format(z, ".9g"),
                format(intensity, ".9g"),
                format(offset, ".17g"),
            )
        )
    return Packet(end_ns, 0, " ".join(pieces))


def _flush_chunk(records: list[Packet], directory: Path, index: int) -> Path:
    records.sort(key=lambda packet: (packet.timestamp_ns, packet.sequence))
    path = directory / f"chunk_{index:06d}.txt"
    with path.open("w", encoding="utf-8", newline="\n") as output:
        for packet in records:
            output.write(f"{packet.timestamp_ns}\t{packet.sequence}\t{packet.line}\n")
    records.clear()
    return path


def _iter_chunk(path: Path) -> Iterator[Packet]:
    with path.open("r", encoding="utf-8") as source:
        for raw in source:
            timestamp, sequence, line = raw.rstrip("\n").split("\t", 2)
            yield Packet(int(timestamp), int(sequence), line)


def _merge_chunks(chunks: list[Path], output_path: Path, metadata: dict[str, object]) -> int:
    iterators = [iter(_iter_chunk(path)) for path in chunks]
    heap: list[tuple[int, int, int, str]] = []
    for index, iterator in enumerate(iterators):
        try:
            packet = next(iterator)
        except StopIteration:
            continue
        heapq.heappush(heap, (packet.timestamp_ns, packet.sequence, index, packet.line))

    output_path.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    last_timestamp: Optional[int] = None
    with output_path.open("w", encoding="utf-8", newline="\n") as output:
        output.write("# adagraph typed dataset v1\n")
        output.write("# generated by scripts/rosbag_to_typed_text.py\n")
        output.write("# metadata " + json.dumps(metadata, sort_keys=True) + "\n")
        while heap:
            timestamp, sequence, index, line = heapq.heappop(heap)
            if last_timestamp is not None and timestamp < last_timestamp:
                raise RuntimeError("external merge produced non-monotonic packets")
            output.write(line + "\n")
            last_timestamp = timestamp
            count += 1
            try:
                packet = next(iterators[index])
            except StopIteration:
                continue
            heapq.heappush(
                heap,
                (packet.timestamp_ns, packet.sequence, index, packet.line),
            )
    return count


def convert(args: argparse.Namespace) -> dict[str, object]:
    rosbag2_py, deserialize_message, get_message, point_cloud2, _ = _import_ros()
    input_path = Path(args.bag).resolve()
    output_path = Path(args.output).resolve()
    if not input_path.exists():
        raise FileNotFoundError(f"bag path does not exist: {input_path}")
    if output_path.exists() and not args.force:
        raise FileExistsError(
            f"refusing to overwrite existing output; use --force: {output_path}"
        )

    if args.frontend == "super_lio":
        blind = float(args.blind)
        maximum_range = float(args.max_range)
        filter_rate = int(args.filter_rate)
    else:
        # Match point_lio_node: SensorBridge is lossless and the vendor owns
        # range/sampling policy.
        blind = 0.0
        maximum_range = 1.0e6
        filter_rate = 1
    if not math.isfinite(blind) or blind < 0.0:
        raise ValueError("--blind must be finite and non-negative")
    if not math.isfinite(maximum_range) or maximum_range <= blind:
        raise ValueError("--max-range must exceed --blind")
    if filter_rate < 1:
        raise ValueError("--filter-rate must be positive")

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        metadata.name: get_message(metadata.type)
        for metadata in reader.get_all_topics_and_types()
        if metadata.name in (args.imu_topic, args.lidar_topic)
    }
    if args.imu_topic not in topic_types:
        raise ValueError(f"IMU topic not present in bag: {args.imu_topic}")
    if args.lidar_topic not in topic_types:
        raise ValueError(f"lidar topic not present in bag: {args.lidar_topic}")

    chunk_directory = Path(
        tempfile.mkdtemp(prefix="adagraph-typed-chunks-", dir=str(output_path.parent))
    )
    chunks: list[Path] = []
    records: list[Packet] = []
    record_bytes = 0
    sequence = 0
    message_count = 0
    imu_count = 0
    cloud_count = 0
    skipped_cloud_count = 0
    last_bag_timestamp = -1
    try:
        while reader.has_next():
            item = reader.read_next()
            topic, serialized, bag_timestamp = item[0], item[1], item[2]
            if topic not in topic_types:
                continue
            if bag_timestamp < last_bag_timestamp:
                raise RuntimeError("rosbag reader returned non-monotonic bag timestamps")
            last_bag_timestamp = bag_timestamp
            message = deserialize_message(serialized, topic_types[topic])
            packet: Optional[Packet]
            if topic == args.imu_topic:
                packet = _packet_line_for_imu(message)
                imu_count += 1
            else:
                packet = _packet_line_for_cloud(
                    message,
                    point_cloud2,
                    blind * blind,
                    maximum_range * maximum_range,
                    filter_rate,
                )
                if packet is None:
                    skipped_cloud_count += 1
                else:
                    cloud_count += 1
            message_count += 1
            if packet is None:
                continue
            packet = Packet(packet.timestamp_ns, sequence, packet.line)
            sequence += 1
            records.append(packet)
            record_bytes += len(packet.line.encode("utf-8")) + 32
            if record_bytes >= args.chunk_bytes:
                chunks.append(_flush_chunk(records, chunk_directory, len(chunks)))
                record_bytes = 0

        if records:
            chunks.append(_flush_chunk(records, chunk_directory, len(chunks)))

        metadata = {
            "schema_version": 1,
            "bag": str(input_path),
            "frontend": args.frontend,
            "imu_topic": args.imu_topic,
            "lidar_topic": args.lidar_topic,
            "blind_m": blind,
            "max_range_m": maximum_range,
            "filter_rate": filter_rate,
            "imu_messages": imu_count,
            "cloud_messages": cloud_count,
            "clouds_skipped_after_filtering": skipped_cloud_count,
            "packets": imu_count + cloud_count,
        }
        packet_count = _merge_chunks(chunks, output_path, metadata)
        return {**metadata, "output": str(output_path), "packets_written": packet_count}
    finally:
        shutil.rmtree(chunk_directory, ignore_errors=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="ROS 2 bag directory")
    parser.add_argument("output", help="typed text output path")
    parser.add_argument(
        "--frontend", choices=("super_lio", "point_lio"), default="super_lio"
    )
    parser.add_argument("--imu-topic", default="/handsfree/imu")
    parser.add_argument("--lidar-topic", default="/velodyne_points")
    parser.add_argument("--storage-id", default="sqlite3")
    parser.add_argument("--blind", type=float, default=2.0)
    parser.add_argument("--max-range", type=float, default=150.0)
    parser.add_argument("--filter-rate", type=int, default=3)
    parser.add_argument(
        "--chunk-bytes",
        type=int,
        default=256 * 1024 * 1024,
        help="maximum encoded packet bytes held before an external-sort chunk is written",
    )
    parser.add_argument("--force", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.chunk_bytes <= 0:
        print("--chunk-bytes must be positive", file=sys.stderr)
        return 2
    try:
        summary = convert(args)
    except Exception as exc:  # pragma: no cover - CLI error path
        print(f"rosbag_to_typed_text: error: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
