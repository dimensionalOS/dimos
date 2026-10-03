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

from dimos.experimental.agent_encode.pointcloud.queries.bounds import Bounds
from dimos.experimental.agent_encode.pointcloud.queries.select import Select
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_select_inside_a_shape_and_its_bounds(room: PointCloud2) -> None:
    hit = Select(inside=Box(center=(1.0, -1.0, 0.4), size=(0.6, 0.6, 0.8))).run(room)
    miss = Select(inside=Box(center=(-2.0, 0.0, 0.5), size=(0.6, 0.6, 0.8))).run(room)
    assert len(hit) > 400
    bounds = Bounds().run(hit)
    assert bounds is not None
    assert bounds[0][0] >= 0.69 and bounds[1][0] <= 1.31, "box plus floor under it"
    assert len(miss) == 0 and Bounds().run(miss) is None
    assert hit.frame_id == "map" and hit.ts == 7.0


def test_unbounded_cylinder_and_inclusive_z() -> None:
    points = np.array([[0.25, 0.25, 0.0], [0.25, 0.25, 1.0], [0.0, 0.0, 0.5]], dtype=np.float32)
    cloud = PointCloud2.from_numpy(points, frame_id="map")
    above = Select(inside=Cylinder(center=(0.25, 0.25), radius=0.1, z=(0.5, None))).run(cloud)
    assert len(above) == 1
    assert len(Select(z=(0.0, 1.0)).run(cloud)) == 3, "both ends included"
    rest = Select(outside=Cylinder(center=(0.25, 0.25), radius=0.1)).run(cloud)
    assert rest.points_f32().tolist() == [[0.0, 0.0, 0.5]]
