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

"""unitree_go::msg::HeightMap — local elevation grid (rt/utlidar/height_map_array).

A ``height`` x ``width`` grid of terrain heights (metres) with the cell at
``[row, col]`` sitting at world XY ``origin + (col, row) * resolution``. Unknown
cells carry a large sentinel (~1e9), dropped by :meth:`to_rerun`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dimos.robot.unitree.go2.dds.msgs.base import PrettyMsg

# utlidar leaves unmeasured cells at ~1e9; anything this large is "no data".
_INVALID_ABOVE = 1e6


@dataclass(repr=False)
class HeightMap(PrettyMsg):
    resolution: float  # metres per cell
    origin: np.ndarray  # f32[2], world XY of cell [0, 0]
    data: np.ndarray  # f32[height, width], terrain height in metres
    frame_id: str
    ts: float  # source stamp (seconds)

    def to_rerun(self) -> Any:
        """Height grid as world-placed ``Points3D`` (one point per known cell)."""
        import rerun as rr

        rows, cols = np.indices(self.data.shape)
        z = self.data
        valid = np.abs(z) < _INVALID_ABOVE
        ox, oy = float(self.origin[0]), float(self.origin[1])
        xs = ox + cols[valid] * self.resolution
        ys = oy + rows[valid] * self.resolution
        zs = z[valid]
        return rr.Points3D(np.column_stack([xs, ys, zs]))
