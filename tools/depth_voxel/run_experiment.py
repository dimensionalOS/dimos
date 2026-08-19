# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build the lidar reference map, then score depth filters against it by voxel overlap."""

from __future__ import annotations

import json
import sys
import time

import numpy as np

from dimos.memory.cli.dataset import open_store
from dimos.memory.tf import StreamTF
from tools.depth_voxel import maps, recording as recordings
from tools.depth_voxel.filters import DepthFilter
from tools.depth_voxel.maps import VOXEL_SIZE_METERS, build_depth_map, build_lidar_map, compare
from tools.depth_voxel.pipeline import DepthProjector, PoseTrack

FRAME_STRIDE = 3
"""Use every 3rd depth frame (~10 Hz), matching the lidar sweep rate."""

CONFIGS: list[DepthFilter] = [
    DepthFilter(name="00-baseline-unfiltered"),
    DepthFilter(name="01-range-6m", max_range_m=6.0),
    DepthFilter(name="02-range-4m", max_range_m=4.0),
    DepthFilter(name="03-range-8m", max_range_m=8.0),
    DepthFilter(name="04-range6-median3", max_range_m=6.0, median_kernel=3),
    DepthFilter(name="05-range6-median5", max_range_m=6.0, median_kernel=5),
    DepthFilter(name="06-range6-speckle", max_range_m=6.0, speckle_max_deviation_m=0.05),
    DepthFilter(name="07-range6-gradient", max_range_m=6.0, gradient_max_step_m=0.15),
    DepthFilter(name="08-range6-blob", max_range_m=6.0, min_component_pixels=500),
    DepthFilter(name="09-range6-erode2", max_range_m=6.0, erode_border_pixels=2),
    DepthFilter(
        name="10-range6-temporal", max_range_m=6.0, temporal_max_deviation_m=0.05, temporal_window=3
    ),
    DepthFilter(
        name="11-combo",
        min_range_m=0.3,
        max_range_m=6.0,
        median_kernel=3,
        gradient_max_step_m=0.15,
        min_component_pixels=500,
        erode_border_pixels=2,
    ),
    # Round two: stereo depth error grows with range^2, so tighten the range gate and
    # re-test the pixel-space stages inside the range where depth is actually accurate.
    DepthFilter(name="12-range-3m", max_range_m=3.0),
    DepthFilter(name="13-range-2m", max_range_m=2.0),
    DepthFilter(name="14-range3-median3", max_range_m=3.0, median_kernel=3),
    DepthFilter(
        name="15-range3-temporal", max_range_m=3.0, temporal_max_deviation_m=0.03, temporal_window=5
    ),
    DepthFilter(name="16-range3-gradient", max_range_m=3.0, gradient_max_step_m=0.08),
    DepthFilter(
        name="17-range3-blob-erode",
        max_range_m=3.0,
        min_component_pixels=1000,
        erode_border_pixels=3,
    ),
    DepthFilter(
        name="18-range3-combo",
        min_range_m=0.3,
        max_range_m=3.0,
        median_kernel=3,
        gradient_max_step_m=0.08,
        min_component_pixels=1000,
        erode_border_pixels=3,
    ),
    # Round three: overlap peaks near a 4 m range gate, and the discontinuity-gradient
    # and temporal-median stages were the only pixel-space stages that raised recall
    # as well as precision. Stack them on the 4 m gate.
    DepthFilter(name="19-range-5m", max_range_m=5.0),
    DepthFilter(name="20-range4-gradient15", max_range_m=4.0, gradient_max_step_m=0.15),
    DepthFilter(name="21-range4-gradient08", max_range_m=4.0, gradient_max_step_m=0.08),
    DepthFilter(
        name="22-range4-temporal", max_range_m=4.0, temporal_max_deviation_m=0.05, temporal_window=3
    ),
    DepthFilter(
        name="23-range4-gradient-temporal",
        max_range_m=4.0,
        gradient_max_step_m=0.15,
        temporal_max_deviation_m=0.05,
        temporal_window=3,
    ),
    DepthFilter(
        name="24-range4-combo",
        min_range_m=0.3,
        max_range_m=4.0,
        gradient_max_step_m=0.15,
        temporal_max_deviation_m=0.05,
        temporal_window=3,
        min_component_pixels=500,
        erode_border_pixels=1,
    ),
    DepthFilter(name="25-range5-gradient15", max_range_m=5.0, gradient_max_step_m=0.15),
]


def main(recording_name: str, only: str | None = None) -> None:
    recording = recordings.get(recording_name)
    results_dir = recording.results_dir
    results_dir.mkdir(parents=True, exist_ok=True)
    store = open_store(recording.db_path)
    with store:
        tf = StreamTF.from_store(store, recording.tf_stream)
        assert tf is not None
        poses = PoseTrack.from_store(store, recording)
        projector = DepthProjector.from_store(store, tf, poses, recording)
        start, end = maps.shared_window(store, recording)
        print(f"window {start:.3f}..{end:.3f} ({end - start:.1f} s) voxel {VOXEL_SIZE_METERS} m")

        lidar_key_path = results_dir / "lidar_voxel_keys.npy"
        if lidar_key_path.exists():
            lidar_keys = np.load(lidar_key_path)
            print(f"lidar map: {len(lidar_keys)} voxels (cached)")
        else:
            began = time.time()
            lidar_grid = build_lidar_map(store, poses, recording, start, end)
            lidar_keys = maps.voxel_keys(lidar_grid)
            np.save(lidar_key_path, lidar_keys)
            print(f"lidar map: {len(lidar_keys)} voxels in {time.time() - began:.0f} s")

        for config in CONFIGS:
            if only and only not in config.name:
                continue
            out_path = results_dir / f"{config.name}.json"
            if out_path.exists():
                print(f"{config.name}: cached")
                continue
            began = time.time()
            grid = build_depth_map(store, projector, recording, start, end, config, FRAME_STRIDE)
            depth_keys = maps.voxel_keys(grid)
            np.save(results_dir / f"{config.name}_keys.npy", depth_keys)
            stats = compare(depth_keys, lidar_keys)
            elapsed = time.time() - began
            out_path.write_text(
                json.dumps(
                    {
                        "name": config.name,
                        "params": config.describe(),
                        "seconds": round(elapsed, 1),
                        **stats.as_dict(),
                    },
                    indent=2,
                )
            )
            print(f"{config.name:26s} [{config.describe()}] {stats.summary()}  ({elapsed:.0f}s)")
            grid.dispose()


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
