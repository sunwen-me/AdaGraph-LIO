#!/usr/bin/env python3

"""Unit coverage for structured diagnostics extracted from node logs."""

import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
import summarize_tum  # noqa: E402


class ParseNodeDiagnosticsTest(unittest.TestCase):
    def test_accepts_position_only_ground_truth(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "ground_truth.tum"
            path.write_text(
                "1.0 0 0 0 0 0 0 0\n2.0 1 0 0 0 0 0 0\n",
                encoding="ascii",
            )
            _, summary = summarize_tum.parse_tum(
                path, allow_position_only=True
            )

        self.assertFalse(summary["orientation_available"])
        self.assertEqual(summary["pose_count"], 2)
        self.assertAlmostEqual(summary["path_length_m"], 1.0)

    def test_parses_loop_closure_scheduler(self) -> None:
        log = (
            "[LoopClosureScheduler]: submitted/replaced/cancelled/"
            "completed/published/callback_failures=120/8/9/112/11/0"
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "node.log"
            path.write_text(log, encoding="ascii")
            diagnostics = summarize_tum.parse_node_diagnostics(path)

        self.assertEqual(diagnostics["loop_closure_scheduler"]["submitted"], 120)
        self.assertEqual(
            diagnostics["loop_closure_scheduler"]["constraints_published"], 11
        )
        self.assertEqual(
            diagnostics["loop_closure_scheduler"]["callback_failures"], 0
        )

    def test_parses_point_prior_map_and_keyframe_memory(self) -> None:
        log = "\n".join(
            [
                "[PointLIOMemory]: ivox(entries/buckets/capacity/estimated)="
                "123/257/1000000/8192, "
                "preprocess(queued_points/queued_imus/vectors)=4/5/4096, "
                "frame_vectors=2048 bytes",
                "[PipelineRuntimeMemory]: rss/allocated/retained="
                "14000/28000/42000",
                "[BackendScheduler]: enqueued/processed/failed=8/8/0, "
                "keyframe_clouds(unique/points/logical/backing)=6/700/22400/32768",
                "[PriorMapLocalizationMemory]: prior(points/backing)=200/6400, "
                "crop_index(cells/indices/payload)=5/200/800, "
                "local_frames(count/points/backing)=8/400/12800, "
                "peak_local_submap(points/backing)=300/9600, "
                "peak_crop(points/backing)=250/8000, "
                "peak_pcl(source_odom/aligned)=9600/8000, "
                "peak_runtime(kiss_rss/allocated/retained)=12000/24000/36000, "
                "registration_rss/allocated/retained=13000/25000/37000, "
                "peak_explicit_working_set=45200 bytes",
            ]
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "node.log"
            path.write_text(log, encoding="ascii")
            diagnostics = summarize_tum.parse_node_diagnostics(path)

        self.assertEqual(diagnostics["point_lio_memory"]["ivox_entries"], 123)
        self.assertEqual(
            diagnostics["point_lio_memory"]["preprocess_vector_backing_bytes"],
            4096,
        )
        self.assertEqual(
            diagnostics["pipeline_runtime_memory"]["glibc_retained_bytes"],
            42000,
        )
        self.assertEqual(
            diagnostics["map_keyframe_memory"]["unique_cloud_backing_bytes"],
            32768,
        )
        self.assertEqual(
            diagnostics["prior_map_memory"]["peak_explicit_working_set_bytes"],
            45200,
        )
        self.assertEqual(
            diagnostics["prior_map_memory"]["peak_kiss_glibc_retained_bytes"],
            36000,
        )

    def test_parses_recovery_reconnect_and_tf_evidence(self) -> None:
        log = "\n".join(
            [
                "Submap recovery candidate: target_session=1, "
                "target_trajectory=1, target_submap=3, new_trajectory=2, "
                "query_timestamp=45.250000000",
                " ---> [PointLIORecovery]: committed old_trajectory=1, "
                "new_trajectory=2, session=1, timestamp=45.250000000, "
                "first_durable_keyframe_pending=true",
                " ---> [PointLIORecovery]: first_durable_keyframe=78, "
                "trajectory=2, session=1, timestamp=45.350000000",
                " ---> [PointLIORecovery]: reconnect_constraint_enqueued "
                "trajectory=2, session=1, timestamp=45.350000000",
                " ---> [PointLIORecovery]: verification_committed "
                "trajectory=2, session=1, timestamp=45.550000000",
                " ---> [PointLIORecovery]: final_snapshot old_trajectory=1, "
                "new_trajectory=2, old_present=1, new_present=1, "
                "relocalization_constraints=1",
                " ---> [PointLIOTf]: odom_base_published=100, "
                "map_odom_published=100, nonmonotonic_odom_base_stamps=0, "
                "nonmonotonic_map_odom_stamps=0",
                " ---> [BackendScheduler]: enqueued/processed/failed=80/80/0, "
                "nodes/constraints=78/77, max_queue=4, backpressure_waits=0, "
                "optimizer updates/failures=80/0, optimizer max pose delta="
                "0.1 m/0.01 rad, global_constraints_dropped=0, "
                "relocalization_constraints enqueued/processed/dropped=1/1/0, "
                "map_snapshot_version=80, submaps/finished=5/3, "
                "accepted_keyframes=78, snapshot_observer_failures=0, "
                "evaluation_drift constraints/distance/yaw=77/42.5 m/0.034 rad",
            ]
        )
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "node.log"
            path.write_text(log, encoding="ascii")
            diagnostics = summarize_tum.parse_node_diagnostics(path)

        recovery = diagnostics["submap_recovery"]
        self.assertEqual(recovery["candidate"]["target_submap"], 3)
        self.assertEqual(recovery["transaction"]["new_trajectory"], 2)
        self.assertEqual(recovery["first_durable_keyframe"]["keyframe"], 78)
        self.assertEqual(recovery["final_snapshot"]["relocalization_constraints"], 1)
        self.assertEqual(recovery["verification_committed"], 1)
        self.assertEqual(diagnostics["tf_publication"]["map_odom_published"], 100)
        self.assertEqual(
            diagnostics["backend_scheduler"][
                "relocalization_constraints_processed"
            ],
            1,
        )
        self.assertEqual(
            diagnostics["backend_scheduler"][
                "evaluation_drifted_odometry_constraints"
            ],
            77,
        )
        self.assertAlmostEqual(
            diagnostics["backend_scheduler"]["evaluation_drifted_yaw_rad"],
            0.034,
        )


if __name__ == "__main__":
    unittest.main()
