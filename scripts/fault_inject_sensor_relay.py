#!/usr/bin/env python3
"""Replay a ROS2 sensor stream with deterministic, wall-clock fault windows.

The relay is intentionally opt-in and does not modify a rosbag. It is used by
run_m2dgr_eval.sh to exercise recovery after a map has been built during an
earlier loop of the same bag.
"""

import argparse
import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, PointCloud2

from fault_inject_time_normalizer import ReplayTimeNormalizer


def in_window(elapsed, start, end):
    return start >= 0.0 and start <= elapsed < end


def shifted_stamp(stamp, offset_seconds):
    total = stamp.sec + stamp.nanosec * 1e-9 + offset_seconds
    sec = math.floor(total)
    nanosec = int(round((total - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    if nanosec < 0:
        sec -= 1
        nanosec += 1_000_000_000
    stamp.sec = int(sec)
    stamp.nanosec = nanosec


class FaultRelay(Node):
    def __init__(self, args):
        super().__init__("adagraph_lio_fault_relay")
        self.args = args
        self.started = None
        self.stream_start_stamp = None
        self.dropped_lidar = 0
        self.relayed_lidar = 0
        self.relayed_imu = 0
        self.time_normalizer = ReplayTimeNormalizer(
            args.normalize_loop_time, args.loop_reset_threshold_seconds
        )
        output_qos = QoSProfile(depth=100)
        output_qos.reliability = ReliabilityPolicy.RELIABLE
        output_qos.durability = DurabilityPolicy.VOLATILE
        self.lidar_pub = self.create_publisher(
            PointCloud2, args.output_lidar, output_qos
        )
        self.imu_pub = self.create_publisher(Imu, args.output_imu, output_qos)
        self.create_subscription(
            PointCloud2,
            args.input_lidar,
            self.on_lidar,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            args.input_imu,
            self.on_imu,
            qos_profile_sensor_data,
        )

    def elapsed(self, stream_stamp=None):
        if self.args.fault_clock == "stamp":
            if stream_stamp is None:
                return 0.0
            if self.stream_start_stamp is None:
                self.stream_start_stamp = stream_stamp
            return max(0.0, stream_stamp - self.stream_start_stamp)
        if self.started is None:
            self.started = time.monotonic()
        return time.monotonic() - self.started

    def fault_active(self, elapsed):
        return in_window(elapsed, self.args.start, self.args.end)

    @staticmethod
    def stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_lidar(self, message):
        source = self.stamp_seconds(message.header.stamp)
        offset = self.time_normalizer.lidar_offset(source)
        elapsed = self.elapsed(source + offset)
        if self.fault_active(elapsed) and self.args.drop_lidar:
            self.dropped_lidar += 1
            return
        output = message
        if offset != 0.0 or (
            self.fault_active(elapsed) and self.args.timestamp_jump != 0.0
        ):
            output = copy.deepcopy(message)
        if offset != 0.0:
            shifted_stamp(output.header.stamp, offset)
        if self.fault_active(elapsed) and self.args.timestamp_jump != 0.0:
            shifted_stamp(output.header.stamp, self.args.timestamp_jump)
        self.lidar_pub.publish(output)
        self.time_normalizer.observe_output(source, offset)
        self.relayed_lidar += 1

    def on_imu(self, message):
        # Keep IMU timing unchanged. The LiDAR header jump/drop is enough to
        # exercise frontend timestamp health without fabricating IMU samples.
        source = self.stamp_seconds(message.header.stamp)
        offset = self.time_normalizer.imu_offset()
        output = message
        if offset != 0.0:
            output = copy.deepcopy(message)
            shifted_stamp(output.header.stamp, offset)
        self.imu_pub.publish(output)
        self.time_normalizer.observe_output(source, offset)
        self.relayed_imu += 1

    def report(self):
        print(
            f"fault relay summary: relayed_lidar={self.relayed_lidar} "
            f"dropped_lidar={self.dropped_lidar} relayed_imu={self.relayed_imu} "
            f"normalized_loops={self.time_normalizer.normalized_loops} "
            f"fault_clock={self.args.fault_clock}",
            flush=True,
        )


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-lidar", default="/velodyne_points")
    parser.add_argument("--input-imu", default="/handsfree/imu")
    parser.add_argument("--output-lidar", required=True)
    parser.add_argument("--output-imu", required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--drop-lidar", action="store_true")
    parser.add_argument("--timestamp-jump", type=float, default=0.0)
    parser.add_argument(
        "--fault-clock", choices=("stamp", "wall"), default="stamp",
        help="Use sensor timestamps (default) or relay wall time for the fault window.",
    )
    parser.add_argument("--normalize-loop-time", action="store_true")
    parser.add_argument("--loop-reset-threshold-seconds", type=float, default=1.0)
    args = parser.parse_args()
    if args.start < 0.0 or args.end <= args.start:
        parser.error("--end must be greater than --start, and --start non-negative")
    if args.loop_reset_threshold_seconds <= 0.0:
        parser.error("--loop-reset-threshold-seconds must be positive")
    return args


def main():
    args = parse_args()
    rclpy.init()
    node = FaultRelay(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 bag shutdown can invalidate the context before spin() returns.
        # Preserve unexpected exceptions while keeping normal teardown quiet.
        if rclpy.ok():
            raise
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
