#!/usr/bin/env python3

import argparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomTumRecorder(Node):
    def __init__(self, topic: str, output_path: str) -> None:
        super().__init__("odom_tum_recorder")
        self._output = open(output_path, "w", encoding="ascii", buffering=1)
        self._count = 0
        self.create_subscription(Odometry, topic, self._on_odom, 100)
        self.get_logger().info(
            f"Recording {topic} to {output_path}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self._output.write(
            f"{stamp:.9f} "
            f"{position.x:.9f} {position.y:.9f} {position.z:.9f} "
            f"{orientation.x:.9f} {orientation.y:.9f} "
            f"{orientation.z:.9f} {orientation.w:.9f}\n"
        )
        self._count += 1

    def close(self) -> None:
        print(f"Recorded {self._count} poses", flush=True)
        self._output.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record nav_msgs/Odometry poses in TUM trajectory format."
    )
    parser.add_argument("output", help="Output TUM trajectory path")
    parser.add_argument("--topic", default="/lio/odom")
    args = parser.parse_args()

    rclpy.init()
    node = OdomTumRecorder(args.topic, args.output)
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
