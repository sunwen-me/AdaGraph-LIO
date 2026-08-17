#!/usr/bin/env python3

"""Record the latest nav_msgs/Path snapshot in TUM trajectory format."""

import argparse
from pathlib import Path

import rclpy
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node


class PathTumRecorder(Node):
    def __init__(self, topic: str, output_path: Path) -> None:
        super().__init__("path_tum_recorder")
        self._topic = topic
        self._output_path = output_path
        self._latest: RosPath | None = None
        self._message_count = 0
        self.create_subscription(RosPath, topic, self._on_path, 10)
        self.get_logger().info(f"Recording latest {topic} to {output_path}")

    def _on_path(self, message: RosPath) -> None:
        self._latest = message
        self._message_count += 1

    def close(self) -> None:
        poses = self._latest.poses if self._latest is not None else []
        temporary = self._output_path.with_suffix(self._output_path.suffix + ".tmp")
        self._output_path.parent.mkdir(parents=True, exist_ok=True)
        with temporary.open("w", encoding="ascii") as output:
            previous_stamp = None
            for stamped in poses:
                stamp = stamped.header.stamp.sec + stamped.header.stamp.nanosec * 1e-9
                if previous_stamp is not None and stamp <= previous_stamp:
                    raise ValueError("Path timestamps are not strictly increasing")
                previous_stamp = stamp
                position = stamped.pose.position
                orientation = stamped.pose.orientation
                output.write(
                    f"{stamp:.9f} "
                    f"{position.x:.9f} {position.y:.9f} {position.z:.9f} "
                    f"{orientation.x:.9f} {orientation.y:.9f} "
                    f"{orientation.z:.9f} {orientation.w:.9f}\n"
                )
        temporary.replace(self._output_path)
        print(
            f"Recorded {len(poses)} poses from {self._message_count} path snapshots",
            flush=True,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Output TUM trajectory path")
    parser.add_argument("--topic", default="/lio/backend/optimized_path")
    args = parser.parse_args()

    rclpy.init()
    node = PathTumRecorder(args.topic, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
