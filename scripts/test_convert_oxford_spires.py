#!/usr/bin/env python3

import csv
import struct
import tempfile
import unittest
import zipfile
from pathlib import Path

import convert_oxford_spires as converter


def make_pcd(points: list[tuple[float, float, float, float, float, int]]) -> bytes:
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z _ intensity _ timestamp ring _\n"
        "SIZE 4 4 4 1 4 1 8 2 1\n"
        "TYPE F F F U F U F U U\n"
        "COUNT 1 1 1 4 1 4 1 1 14\n"
        f"WIDTH {len(points)}\n"
        "HEIGHT 1\n"
        f"POINTS {len(points)}\n"
        "DATA binary\n"
    ).encode("ascii")
    payload = bytearray(48 * len(points))
    for index, (x, y, z, intensity, timestamp, ring) in enumerate(points):
        offset = index * 48
        struct.pack_into("<fff", payload, offset, x, y, z)
        struct.pack_into("<f", payload, offset + 16, intensity)
        struct.pack_into("<d", payload, offset + 24, timestamp)
        struct.pack_into("<H", payload, offset + 32, ring)
    return header + payload


class TimestampTest(unittest.TestCase):
    def test_preserves_nanoseconds(self) -> None:
        value = converter.parse_timestamp("1710255656.000000123")
        self.assertEqual(value, 1_710_255_656_000_000_123)
        self.assertEqual(converter.format_timestamp(value), "1710255656.000000123")


class PcdTest(unittest.TestCase):
    def test_reads_hesai_binary_layout(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            archive_path = Path(temporary) / "clouds.zip"
            with zipfile.ZipFile(archive_path, "w") as archive:
                archive.writestr(
                    "lidar-clouds/1710255656.973875000.pcd",
                    make_pcd(
                        [
                            (1.0, 2.0, 3.0, 4.0, 1710255656.973876, 5),
                            (6.0, 7.0, 8.0, 9.0, 1710255657.073876, 6),
                        ]
                    ),
                )
            with zipfile.ZipFile(archive_path) as archive:
                entries = converter.parse_cloud_entries(archive)
                width, data, offsets = converter.read_binary_pcd(
                    archive, entries[0].name
                )

        self.assertEqual(width, 2)
        self.assertEqual(offsets["timestamp"], 24)
        self.assertEqual(offsets["ring"], 32)
        self.assertEqual(offsets["_point_step"], 48)
        self.assertAlmostEqual(struct.unpack_from("<d", data, 24)[0], 1710255656.973876)
        self.assertEqual(struct.unpack_from("<H", data, 32)[0], 5)


class ImuTest(unittest.TestCase):
    def test_reads_and_sorts_csv(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "imu.csv"
            with path.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.writer(stream)
                writer.writerow(
                    [
                        "secs",
                        "nsecs",
                        "acc_x",
                        "acc_y",
                        "acc_z",
                        "ang_vel_x",
                        "ang_vel_y",
                        "ang_vel_z",
                    ]
                )
                writer.writerow([2, 3, 1, 2, 3, 4, 5, 6])
                writer.writerow([1, 2, 7, 8, 9, 10, 11, 12])
            entries = converter.read_imu(path)

        self.assertEqual([entry.timestamp_ns for entry in entries], [1_000_000_002, 2_000_000_003])
        self.assertEqual(entries[0].acc, (7.0, 8.0, 9.0))


if __name__ == "__main__":
    unittest.main()
