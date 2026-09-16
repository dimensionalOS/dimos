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

if TYPE_CHECKING:
    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype

LOADED_MAP_COLOR = (130, 130, 130)
PREMAP_POINT_RADIUS = 0.008


def voxel_map_points(pts: NDArray[np.float32], voxel_size: float) -> Archetype:
    """Voxel centers colored by height on the turbo ramp."""
    import rerun as rr

    if len(pts) == 0:
        return rr.Points3D([])
    z = pts[:, 2]
    class_ids = ((z - z.min()) / (z.max() - z.min() + 1e-8) * 255).astype(np.uint8)
    return rr.Points3D(pts, class_ids=class_ids, radii=voxel_size / 3)


def log_loaded_map(points: NDArray[np.float32]) -> None:
    import rerun as rr

    rr.log(
        "world/loaded_map",
        rr.Points3D(points, colors=[LOADED_MAP_COLOR], radii=PREMAP_POINT_RADIUS),
    )
