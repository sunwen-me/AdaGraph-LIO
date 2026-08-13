#!/usr/bin/env python3

"""Unit coverage for structured diagnostics extracted from node logs."""

import sys
import tempfile
import unittest
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parent))
import summarize_tum  # noqa: E402


class ParseNodeDiagnosticsTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
