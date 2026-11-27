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

from dimos.experimental.agent_encode.pointcloud.queries.nearest import Nearest
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_nearest_finds_the_wall(room: PointCloud2) -> None:
    hit = Nearest(to=(2.0, 0.0)).run(Select(z=(0.15, 1.0)).run(room))
    assert hit is not None
    assert hit.distance_m == pytest.approx(1.0, abs=0.05), "east wall is 1 m away, floor excluded"
    assert hit.point_m[0] == pytest.approx(3.0, abs=0.02)
    assert hit.from_m == (2.0, 0.0, hit.point_m[2])
    empty = PointCloud2.from_numpy(
        np.zeros((0, 3), dtype=np.float32), frame_id="map", timestamp=1.0
    )
    assert Nearest(to=(0.0, 0.0, 0.0)).run(empty) is None
