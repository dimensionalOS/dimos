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

import numpy as np

from dimos.mapping.ray_tracing.viz import render_map_region, voxel_map_points
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

HEIGHT_RANGE = (-1.0, 3.0)


def _at_heights(heights: list[float]) -> np.ndarray:
    return np.array([[0.0, 0.0, z] for z in heights], dtype=np.float32)


def test_the_height_ramp_spans_the_range_and_clamps_outside_it() -> None:
    arch = voxel_map_points(_at_heights([-1.0, 1.0, 3.0, -5.0, 10.0]), 0.1, HEIGHT_RANGE)
    assert arch.class_ids.as_arrow_array().to_pylist() == [0, 127, 255, 0, 255]


def test_map_regions_land_static_on_their_own_cell_entities() -> None:
    region = PointCloud2.from_numpy(_at_heights([-1.0, 3.0]))
    region.seq = (1 << 16) | (-2 & 0xFFFF)
    (path, arch, static) = render_map_region(region, 0.1, HEIGHT_RANGE)[0]
    assert path == "world/map_regions/1_-2" and static
    assert len(arch.positions.as_arrow_array()) == 2
    assert arch.class_ids.as_arrow_array().to_pylist() == [0, 255]

    emptied = PointCloud2.from_numpy(np.zeros((0, 3), dtype=np.float32))
    emptied.seq = region.seq
    (path, arch, static) = render_map_region(emptied, 0.1, HEIGHT_RANGE)[0]
    assert path == "world/map_regions/1_-2" and static
    assert len(arch.positions.as_arrow_array()) == 0
