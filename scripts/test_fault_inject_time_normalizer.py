#!/usr/bin/env python3
"""Regression tests for replay timestamp normalization without ROS imports."""

import unittest

from fault_inject_time_normalizer import ReplayTimeNormalizer


class ReplayTimeNormalizerTest(unittest.TestCase):
    def test_ignores_cross_stream_jitter_and_uses_one_shared_offset(self):
        normalizer = ReplayTimeNormalizer(True, reset_threshold_seconds=1.0)
        self.assertEqual(normalizer.lidar_offset(50.0), 0.0)
        normalizer.observe_output(50.0, normalizer.imu_offset())
        normalizer.observe_output(49.8, normalizer.imu_offset())
        self.assertEqual(normalizer.normalized_loops, 0)
        self.assertEqual(normalizer.imu_offset(), 0.0)

    def test_only_large_lidar_rewind_starts_a_shared_new_epoch(self):
        normalizer = ReplayTimeNormalizer(True, reset_threshold_seconds=1.0)
        self.assertEqual(normalizer.lidar_offset(73.0), 0.0)
        normalizer.observe_output(73.0, normalizer.imu_offset())
        self.assertAlmostEqual(normalizer.lidar_offset(0.1), 72.900001)
        self.assertAlmostEqual(normalizer.imu_offset(), 72.900001)
        self.assertEqual(normalizer.normalized_loops, 1)

    def test_disabled_normalization_leaves_timestamps_unchanged(self):
        normalizer = ReplayTimeNormalizer(False)
        self.assertEqual(normalizer.lidar_offset(73.0), 0.0)
        self.assertEqual(normalizer.lidar_offset(0.1), 0.0)
        self.assertEqual(normalizer.normalized_loops, 0)


if __name__ == "__main__":
    unittest.main()
