#!/usr/bin/env python3

"""Record the latest nav_msgs/OccupancyGrid message as compact JSON.

The recorder intentionally stores metadata and cell semantics rather than the
full ROS message.  The publisher uses transient-local durability, so a
subscriber started after the node is ready still receives the latest grid.
"""

import argparse
import collections
import hashlib
import json
import signal
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def _stamp_to_nanoseconds(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _summary(topic: str, message: Optional[OccupancyGrid]) -> Dict[str, Any]:
    if message is None:
        return {
            "schema_version": 1,
            "status": "no_message",
            "topic": topic,
        }

    values = [int(value) for value in message.data]
    histogram = collections.Counter(values)
    occupied = sum(value > 0 for value in values)
    free = sum(value == 0 for value in values)
    unknown = sum(value < 0 for value in values)
    encoded = bytes((value + 256) % 256 for value in values)
    classification = bytes(
        2 if value > 0 else 1 if value == 0 else 0 for value in values
    )
    origin = message.info.origin
    return {
        "schema_version": 1,
        "status": "received",
        "topic": topic,
        "frame_id": message.header.frame_id,
        "stamp_nanoseconds": _stamp_to_nanoseconds(message.header.stamp),
        "resolution": float(message.info.resolution),
        "width": int(message.info.width),
        "height": int(message.info.height),
        "origin": {
            "position": {
                "x": float(origin.position.x),
                "y": float(origin.position.y),
                "z": float(origin.position.z),
            },
            "orientation": {
                "x": float(origin.orientation.x),
                "y": float(origin.orientation.y),
                "z": float(origin.orientation.z),
                "w": float(origin.orientation.w),
            },
        },
        "data_length": len(values),
        "cell_semantics": {
            "occupied_positive": occupied,
            "free_zero": free,
            "unknown_negative": unknown,
            "histogram": {
                str(value): count for value, count in sorted(histogram.items())
            },
        },
        "data_sha256": hashlib.sha256(encoded).hexdigest(),
        "classification_sha256": hashlib.sha256(classification).hexdigest(),
    }


class OccupancyRecorder(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("adagraph_occupancy_summary_recorder")
        self.topic = topic
        self.latest: Optional[OccupancyGrid] = None
        self.received_count = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            OccupancyGrid, topic, self._on_grid, qos
        )

    def _on_grid(self, message: OccupancyGrid) -> None:
        self.latest = message
        self.received_count += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record the latest OccupancyGrid message as JSON."
    )
    parser.add_argument("output", type=Path)
    parser.add_argument("topic", nargs="?", default="/map")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OccupancyRecorder(args.topic)
    stop_requested = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        result = _summary(args.topic, node.latest)
        result["received_count"] = node.received_count
        args.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = args.output.with_suffix(args.output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(args.output)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
