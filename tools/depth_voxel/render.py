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

"""Render saved voxel-key maps to an .rrd with height-gradient coloring."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import rerun as rr

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2, register_colormap_annotation
from tools.depth_voxel.maps import VOXEL_SIZE_METERS

RENDER_POINT_SCALE = 0.8
"""Draw points slightly smaller than the voxel so gaps read as transparency."""

RENDER_CLIP_METERS = ((-15.0, 15.0), (-15.0, 15.0), (-2.0, 6.0))
"""Display-only crop. The lidar map has stray returns out to +-80 m that would otherwise
own the auto-fit camera and flatten the height colormap; both maps use the same box so
the screenshots are visually comparable. Statistics are computed on the uncropped maps."""


def load_cloud(path: Path) -> PointCloud2:
    keys = np.load(path)
    centers = (keys.astype(np.float32) + np.float32(0.5)) * np.float32(VOXEL_SIZE_METERS)
    inside = np.ones(len(centers), dtype=bool)
    for axis, (low, high) in enumerate(RENDER_CLIP_METERS):
        inside &= (centers[:, axis] >= low) & (centers[:, axis] <= high)
    return PointCloud2.from_numpy(centers[inside], frame_id="world")


def main(out_path: str, *key_paths: str) -> None:
    rr.init("dimos depth_voxel", recording_id="depth_voxel_compare")
    rr.save(out_path)
    register_colormap_annotation("turbo")
    for key_path in key_paths:
        path = Path(key_path)
        cloud = load_cloud(path)
        entity = f"world/{path.stem.replace('_keys', '')}"
        rr.log(
            entity,
            cloud.to_rerun(voxel_size=VOXEL_SIZE_METERS * RENDER_POINT_SCALE, mode="boxes"),
            static=True,
        )
        print(f"{entity}: {len(cloud.points_f32())} voxels")
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main(sys.argv[1], *sys.argv[2:])
