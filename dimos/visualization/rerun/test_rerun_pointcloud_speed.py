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

Compares two approaches:
  - LUT colors: compute 400k x 3 uint8 color array, pass as colors=
  - class_ids: compute 400k uint16 class IDs, viewer resolves colors via AnnotationContext
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


def _turbo_class_ids(points):
    z = points[:, 2]
    return ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint16)


def test_rerun_pointcloud_viz():
    """Replay lidar → VoxelGridMapper → Rerun, frame by frame."""
    mapper = VoxelGridMapper(publish_interval=-1, voxel_size=VOXEL_SIZE)

    rr.init("pointcloud_replay", spawn=True)

    # Register turbo colormap as AnnotationContext (viewer-side color resolution)
    rr.log(
        "/",
        rr.AnnotationContext(
            [
                rr.datatypes.ClassDescription(
                    info=rr.datatypes.AnnotationInfo(id=i, color=_LUT[i].tolist())
                )
                for i in range(256)
            ]
        ),
        static=True,
    )

    half = VOXEL_SIZE / 2
    lut_times = []
    cid_times = []

    for i, frame in enumerate(TimedSensorReplay("unitree_go2_bigoffice/lidar").iterate()):
        mapper.add_frame(frame)

        pcd = mapper.get_global_pointcloud2()
        points, _ = pcd.as_numpy()
        if len(points) == 0:
            continue

        # Benchmark LUT colors (don't log, just time)
        t0 = time.perf_counter()
        colors = _turbo_colors(points)
        rr.Boxes3D(
            centers=points,
            half_sizes=[half, half, half],
            colors=colors,
            fill_mode="solid",
        )
        lut_ms = (time.perf_counter() - t0) * 1000

        # Benchmark class_ids (log this one to viewer)
        t0 = time.perf_counter()
        class_ids = _turbo_class_ids(points)
        archetype = rr.Boxes3D(
            centers=points,
            half_sizes=[half, half, half],
            class_ids=class_ids,
            fill_mode="solid",
        )
        rr.log("world/global_map", archetype)
        cid_ms = (time.perf_counter() - t0) * 1000

        lut_times.append((len(points), lut_ms))
        cid_times.append((len(points), cid_ms))

        if i % 50 == 0:
            print(f"frame {i}: {len(points):,} pts → lut={lut_ms:.1f}ms  class_ids={cid_ms:.1f}ms")

    mapper.stop()

    # Summary
    _, lut_ms_all = zip(*lut_times, strict=False)
    pts, cid_ms_all = zip(*cid_times, strict=False)
    print("\n--- Summary ---")
    print(f"Frames: {len(cid_times)}")
    print(f"Max points: {max(pts):,}")
    print(f"LUT colors   — avg: {np.mean(lut_ms_all):.1f}ms  max: {max(lut_ms_all):.1f}ms")
    print(f"class_ids    — avg: {np.mean(cid_ms_all):.1f}ms  max: {max(cid_ms_all):.1f}ms")
    print(f"Speedup: {np.mean(lut_ms_all) / np.mean(cid_ms_all):.1f}x")

    input("\nPress Enter to close Rerun viewer...")
