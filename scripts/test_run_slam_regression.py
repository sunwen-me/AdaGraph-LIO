#!/usr/bin/env python3

"""Unit coverage for recorded SLAM regression acceptance gates."""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_slam_regression  # noqa: E402


class RegressionAcceptanceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.case = {
            "acceptance": {
                "minimum_pose_count": 1,
                "maximum_ape_rmse_m": 0.1,
            }
        }
        self.summary = {
            "trajectory": {"pose_count": 1, "path_length_m": 0.0},
            "ape_translation_m": {"rmse": 0.1},
        }

    def test_accepts_recorded_quality_at_limit(self) -> None:
        self.assertEqual(
            run_slam_regression.evaluate_acceptance(self.case, self.summary), []
        )

    def test_rejects_missing_or_excess_quality_values(self) -> None:
        failures = run_slam_regression.evaluate_acceptance(
            self.case, {"trajectory": {}, "ape_translation_m": {"rmse": 0.2}}
        )
        self.assertTrue(any("pose_count" in failure for failure in failures))
        self.assertTrue(any("ape_rmse_m" in failure for failure in failures))


if __name__ == "__main__":
    unittest.main()
