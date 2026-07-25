from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from scripts.dump_costmap_odom import CostmapFrame, OdomFrame, write_dataset


class DumpCostmapOdomTest(unittest.TestCase):
    def test_writes_stacked_costmaps_and_explicit_odom_columns(self) -> None:
        costmaps = [
            CostmapFrame(
                ts=10.0 + index,
                frame_id="world",
                resolution=0.05,
                origin=(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                grid=np.full((2, 3), index, dtype=np.int8),
            )
            for index in range(2)
        ]
        odometry = [
            OdomFrame(
                ts=10.0,
                frame_id="world",
                pose=(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0),
            )
        ]

        with tempfile.TemporaryDirectory() as directory:
            manifest_path, npz_path = write_dataset(
                Path(directory),
                costmaps=costmaps,
                odometry=odometry,
                source="replay",
                transport="lcm",
                costmap_topic="global_costmap",
                odom_topic="odom",
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            with np.load(npz_path) as archive:
                self.assertEqual(archive["costmaps"].shape, (2, 2, 3))
                self.assertEqual(archive["costmaps"].dtype, np.int8)
                self.assertEqual(archive["odom"].shape, (1, 8))

        self.assertEqual(manifest["source"], "replay")
        self.assertEqual(manifest["costmap"]["frame_count"], 2)
        self.assertEqual(
            manifest["odom"]["columns"],
            ["ts", "x", "y", "z", "qx", "qy", "qz", "qw"],
        )

    def test_writes_variable_size_costmaps_as_individual_arrays(self) -> None:
        costmaps = [
            CostmapFrame(
                ts=10.0 + index,
                frame_id="world",
                resolution=0.05,
                origin=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
                grid=np.zeros(shape, dtype=np.int8),
            )
            for index, shape in enumerate(((2, 3), (3, 4)))
        ]

        with tempfile.TemporaryDirectory() as directory:
            manifest_path, npz_path = write_dataset(
                Path(directory),
                costmaps=costmaps,
                odometry=[],
                source="replay",
                transport="zenoh",
                costmap_topic="global_costmap",
                odom_topic="odom",
            )
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            with np.load(npz_path) as archive:
                self.assertNotIn("costmaps", archive.files)
                self.assertEqual(archive["costmap_000"].shape, (2, 3))
                self.assertEqual(archive["costmap_001"].shape, (3, 4))
                self.assertEqual(archive["odom"].shape, (0, 8))

        self.assertEqual(
            manifest["costmap"]["array_layout"],
            "costmap_NNN[H,W] (variable-size frames)",
        )


if __name__ == "__main__":
    unittest.main()
