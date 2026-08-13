#!/usr/bin/env python3

"""Unit coverage for recorded SLAM regression acceptance gates."""

import sys
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_slam_regression  # noqa: E402


class MemoryAcceptanceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.case = {
            "acceptance": {
                "memory": {
                    "maximum_rss_kib": 100,
                    "maximum_local_history_backing_bytes": 200,
                }
            }
        }
        self.summary = {
            "trajectory": {"pose_count": 1, "path_length_m": 0.0},
            "ape_translation_m": {},
            "resources": {"maximum_resident_set_kib": 100},
            "diagnostics": {
                "prior_map_memory": {"local_frame_backing_bytes": 200}
            },
        }

    def test_accepts_memory_at_budget(self) -> None:
        self.assertEqual(
            run_slam_regression.evaluate_acceptance(self.case, self.summary), []
        )

    def test_rejects_excess_or_missing_memory_diagnostics(self) -> None:
        over_budget = {
            **self.summary,
            "resources": {"maximum_resident_set_kib": 101},
            "diagnostics": {
                "prior_map_memory": {"local_frame_backing_bytes": 201}
            },
        }
        failures = run_slam_regression.evaluate_acceptance(self.case, over_budget)
        self.assertTrue(any("maximum RSS" in failure for failure in failures))
        self.assertTrue(any("local-history backing" in failure for failure in failures))

        missing = {**self.summary, "resources": {}, "diagnostics": {}}
        failures = run_slam_regression.evaluate_acceptance(self.case, missing)
        self.assertTrue(any("resident-set diagnostic" in failure for failure in failures))
        self.assertTrue(any("local-history memory diagnostic" in failure for failure in failures))


if __name__ == "__main__":
    unittest.main()
