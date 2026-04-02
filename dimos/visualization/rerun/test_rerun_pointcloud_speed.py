# Copyright 2025-2026 Dimensional Inc.
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

"""Test: replay lidar frames through VoxelGridMapper → Rerun as colored boxes.

Iterates lidar frames one by one, builds the map incrementally,
and logs each update to Rerun to measure real-world throughput.
"""

import time

import numpy as np
import rerun as rr

from dimos.mapping.voxels import VoxelGridMapper
from dimos.utils.testing.replay import TimedSensorReplay

VOXEL_SIZE = 0.05


# Build turbo LUT once at import from matplotlib (one-time ~2ms cost)
def _build_turbo_lut():
    from matplotlib import colormaps

    cmap = colormaps["turbo"]
    t = np.linspace(0, 1, 256)
    return (cmap(t)[:, :3] * 255).astype(np.uint8)


_LUT = _build_turbo_lut()


def _turbo_colors(points):
    z = points[:, 2]
    idx = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)
    return _LUT[idx]


def test_rerun_pointcloud_viz():
    """Replay lidar → VoxelGridMapper → Rerun, frame by frame."""
    mapper = VoxelGridMapper(publish_interval=-1, voxel_size=VOXEL_SIZE)

    rr.init("pointcloud_replay", spawn=True)

    half = VOXEL_SIZE / 2
    frame_times = []

    for i, frame in enumerate(TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate()):
        mapper.add_frame(frame)

        pcd = mapper.get_global_pointcloud2()
        points, _ = pcd.as_numpy()
        if len(points) == 0:
            continue

        start = time.perf_counter()
        colors = _turbo_colors(points)
        archetype = rr.Boxes3D(
            centers=points,
            half_sizes=[half, half, half],
            colors=colors,
            fill_mode="solid",
        )
        rr.log("world/global_map", archetype)
        elapsed = time.perf_counter() - start

        frame_times.append((len(points), elapsed * 1000))

        if i % 50 == 0:
            print(f"frame {i}: {len(points):,} pts → {elapsed * 1000:.1f} ms")

    mapper.stop()

    # Summary
    pts, ms = zip(*frame_times, strict=False)
    print("\n--- Summary ---")
    print(f"Frames: {len(frame_times)}")
    print(f"Max points: {max(pts):,}")
    print(f"Avg convert+log: {np.mean(ms):.1f} ms")
    print(f"Max convert+log: {max(ms):.1f} ms")
    print(f"Theoretical max Hz: {1000 / max(ms):.1f}")

    input("\nPress Enter to close Rerun viewer...")
