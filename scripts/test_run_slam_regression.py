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


class SavedMapAcceptanceTest(unittest.TestCase):
    """The saved map is the one artifact no other gate inspects.

    Keyframe clouds are retained for a whole session with nothing evicting
    them, so any future bound on that growth changes what the map is built
    from. These cases pin down that the check can actually fail, because a
    coverage gate that only ever passes is worse than none.
    """

    CASE = {
        "acceptance": {
            "pcd": {
                "minimum_points": 2000,
                "require_trajectory_coverage": True,
                "coverage_margin_m": 2.0,
            }
        }
    }

    @staticmethod
    def summary(minimum, maximum, point_count=7415):
        return {
            "trajectory": {
                "pose_count": 6800,
                "path_length_m": 28.4,
                "start_position_m": [0.0, 0.0, 0.0],
                "end_position_m": [-0.35, 0.64, -0.17],
            },
            "ape_translation_m": {"rmse": 0.40},
            "pcd": {
                "files": [
                    {
                        "path": "/tmp/map.pcd",
                        "finite": True,
                        "point_count": point_count,
                        "minimum": minimum,
                        "maximum": maximum,
                    }
                ]
            },
        }

    def test_accepts_a_map_spanning_the_trajectory(self) -> None:
        summary = self.summary([-14.71, -3.99, -2.05], [10.87, 22.30, 2.94])
        self.assertEqual(
            run_slam_regression.evaluate_acceptance(self.CASE, summary), []
        )

    def test_rejects_a_map_saved_away_from_the_trajectory(self) -> None:
        # What an identity map->base transform or a lost early submap looks
        # like: plenty of finite points, none of them near the path.
        summary = self.summary([100.0, 100.0, 100.0], [120.0, 120.0, 120.0])
        failures = run_slam_regression.evaluate_acceptance(self.CASE, summary)
        self.assertTrue(
            any("does not cover the trajectory" in failure for failure in failures),
            failures,
        )

    def test_rejects_a_collapsed_map(self) -> None:
        summary = self.summary(
            [-14.71, -3.99, -2.05], [10.87, 22.30, 2.94], point_count=10
        )
        failures = run_slam_regression.evaluate_acceptance(self.CASE, summary)
        self.assertTrue(
            any("point count" in failure for failure in failures), failures
        )

    def test_reports_missing_endpoints_instead_of_passing(self) -> None:
        summary = self.summary([-14.71, -3.99, -2.05], [10.87, 22.30, 2.94])
        summary["trajectory"].pop("start_position_m")
        summary["trajectory"].pop("end_position_m")
        failures = run_slam_regression.evaluate_acceptance(self.CASE, summary)
        self.assertTrue(
            any("endpoint positions" in failure for failure in failures), failures
        )


if __name__ == "__main__":
    unittest.main()
