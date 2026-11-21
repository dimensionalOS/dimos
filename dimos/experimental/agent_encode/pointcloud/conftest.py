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

from __future__ import annotations

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@pytest.fixture
def room() -> PointCloud2:
    """Floor at z=0 over 6 x 4 m, a wall at x=3, a wall at y=2, a 0.4 m box at (1, -1)."""
    rng = np.random.default_rng(0)
    floor = np.column_stack([rng.uniform(-3, 3, 4000), rng.uniform(-2, 2, 4000), np.zeros(4000)])
    east = np.column_stack(
        [np.full(3000, 3.0), rng.uniform(-2, 2, 3000), rng.uniform(0, 2.5, 3000)]
    )
    north = np.column_stack(
        [rng.uniform(-3, 3, 3000), np.full(3000, 2.0), rng.uniform(0, 2.5, 3000)]
    )
    box = np.column_stack(
        [rng.uniform(0.8, 1.2, 500), rng.uniform(-1.2, -0.8, 500), rng.uniform(0, 0.8, 500)]
    )
    pts = np.vstack([floor, east, north, box]).astype(np.float32)
    return PointCloud2.from_numpy(pts, frame_id="map", timestamp=7.0)
