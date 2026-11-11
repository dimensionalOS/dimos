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

from dimos.experimental.agent_encode.pointcloud.queries.sweep import Sweep
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def test_sweep_stops_at_first_contact(room: PointCloud2) -> None:
    body = Cylinder(center=(0.0, 0.0), radius=0.3, z=(0.15, 1.0))
    east = Sweep(body, heading_deg=0.0, max_m=5.0).run(room)
    assert east.distance_m == pytest.approx(2.7, abs=0.1), "3 m wall minus 0.3 m radius"
    north = Sweep(body, heading_deg=90.0, max_m=5.0).run(room)
    assert north.distance_m == pytest.approx(1.7, abs=0.1)
    west = Sweep(body, heading_deg=180.0, max_m=2.0).run(room)
    assert west.distance_m is None and west.point_m is None and west.start_inside == 0
    inside_box = Sweep(Cylinder((1.0, -1.0), 0.3, (0.0, 0.8)), heading_deg=0.0, max_m=1.0).run(room)
    assert inside_box.start_inside > 0 and inside_box.distance_m == 0.0
    into_box = Sweep(Cylinder((1.0, 0.5), 0.1, (0.15, 1.0)), heading_deg=270.0, max_m=3.0).run(room)
    assert into_box.distance_m == pytest.approx(1.2, abs=0.15)


def test_three_dimensional_sweep_checks_nonmultiple_endpoint() -> None:
    cloud = PointCloud2.from_numpy(np.array([[0, 0, 0.13]], dtype=np.float32), frame_id="map")
    up = Sweep(Box((0, 0, 0), (0.05, 0.05, 0.05)), direction=(0, 0, 1), max_m=0.13).run(cloud)
    assert up.distance_m == 0.13 and up.direction == (0.0, 0.0, 1.0)


def test_sweep_rejects_thin_shapes_and_non_finite_headings() -> None:
    with pytest.raises(ValueError, match="body-sized"):
        Sweep(Cylinder((0, 0.25), 0.001, (0, 2)), heading_deg=0, max_m=2)
    with pytest.raises(ValueError, match="heading_deg must be finite"):
        Sweep(Box((0, 0, 0), (1, 1, 1)), heading_deg=float("nan"))
