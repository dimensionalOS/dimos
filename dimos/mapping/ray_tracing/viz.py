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

"""Rendering for voxel maps and the loaded premap."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.visualization.rerun.bridge import RerunEntry, keyed_by_seq, region_entity

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    from dimos.visualization.rerun.bridge import RerunMulti

# rerun is imported inside the renderers: it is heavy and only viewers load it.

MAP_REGIONS_ENTITY = "world/map_regions"
LOADED_MAP_COLOR = (130, 130, 130)
PREMAP_POINT_RADIUS = 0.008


def voxel_map_points(
    pts: NDArray[np.float32], voxel_size: float, height_range: tuple[float, float] | None = None
) -> Archetype:
    """Voxel centers colored by height on the turbo ramp.

    The ramp spans height_range, or the points' own span when none is given.
    """
    import rerun as rr

    if len(pts) == 0:
        return rr.Points3D([])
    z = pts[:, 2]
    lo, hi = height_range if height_range is not None else (float(z.min()), float(z.max()))
    class_ids = (np.clip((z - lo) / (hi - lo + 1e-8), 0.0, 1.0) * 255).astype(np.uint8)
    return rr.Points3D(pts, class_ids=class_ids, radii=voxel_size / 3)


def render_voxel_map(
    msg: PointCloud2, voxel_size: float, height_range: tuple[float, float]
) -> Archetype:
    return voxel_map_points(msg.points_f32(), voxel_size, height_range)


@keyed_by_seq
def render_map_region(
    msg: PointCloud2, voxel_size: float, height_range: tuple[float, float]
) -> RerunMulti:
    """One region of the voxel map on its own static entity, empty when the region emptied."""
    cell = voxel_map_points(msg.points_f32(), voxel_size, height_range)
    return [RerunEntry(region_entity(MAP_REGIONS_ENTITY, msg.seq), cell, static=True)]


def log_loaded_map(points: NDArray[np.float32]) -> None:
    import rerun as rr

    rr.log(
        "world/loaded_map",
        rr.Points3D(points, colors=[LOADED_MAP_COLOR], radii=PREMAP_POINT_RADIUS),
    )
