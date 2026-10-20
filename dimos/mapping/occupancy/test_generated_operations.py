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

from copy import deepcopy

from dimos_cdrgen.registry import decode, encode
import numpy as np
import pytest

from dimos.mapping.occupancy.extrude_occupancy import generate_mujoco_scene, identify_convex_shapes
from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.occupancy.operations import overlay_occupied, smooth_occupied
from dimos.messages.builtin_interfaces.msg import Time
from dimos.messages.nav_msgs.msg import OccupancyGrid
from dimos.messages.std_msgs.msg import Header
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid as LegacyGrid
from dimos.msgs.occupancy import occupancy_from_array, occupancy_view


@pytest.fixture(params=[(0, 0), (-1, 999999999), (1700000000, 123456789)])
def grids(request):
    cells = np.zeros((7, 7), dtype=np.int8)
    cells[0, 0] = -1
    cells[3, 3] = cells[4, 4] = 100
    generated = occupancy_from_array(
        cells,
        header=Header(frame_id="map", stamp=Time(sec=request.param[0], nanosec=request.param[1])),
        resolution=0.25,
    )
    generated.info.map_load_time = Time(sec=7, nanosec=9)
    return generated, LegacyGrid(grid=cells, resolution=0.25, frame_id="map", ts=0)


@pytest.mark.parametrize(
    "operation,kwargs",
    [
        (gradient, {"max_distance": 0.5}),
        (voronoi_gradient, {"max_distance": 0.5}),
        (smooth_occupied, {}),
        (simple_inflate, {"radius": 0.25}),
    ],
)
def test_existing_algorithms_preserve_input_identity_and_exact_generated_metadata(
    grids, operation, kwargs
):
    generated, legacy = grids
    original = deepcopy(generated)

    actual = operation(generated, **kwargs)
    old = operation(legacy, **kwargs)
    received = decode(encode(actual), OccupancyGrid)

    assert type(actual) is OccupancyGrid
    assert type(old) is LegacyGrid
    np.testing.assert_array_equal(occupancy_view(received), old.grid)
    assert received.header == original.header
    assert received.info.map_load_time == Time(sec=7, nanosec=9)
    assert received.info.origin == original.info.origin
    assert occupancy_view(received)[0, 0] == -1
    np.testing.assert_array_equal(generated.data, original.data)
    assert generated.header == original.header
    assert generated.info == original.info
    actual.header.stamp.nanosec = 0
    actual.info.origin.position.x = 99
    actual.data[0] = 42
    np.testing.assert_array_equal(generated.data, original.data)
    assert generated.header == original.header
    assert generated.info == original.info
    assert legacy.ts == 0


def test_gradient_retains_unknown_obstacles_and_exact_distance_cost(grids):
    generated, _ = grids

    actual = gradient(generated, max_distance=0.5)

    assert occupancy_view(actual)[0, 0] == -1
    assert occupancy_view(actual)[3, 3] == 100
    assert occupancy_view(actual)[3, 2] == 50
    assert occupancy_view(actual)[1, 1] == 0


def test_overlay_preserves_base_metadata_and_rejects_different_dimensions(grids):
    base, _ = grids
    cells = np.zeros((7, 7), dtype=np.int8)
    cells[2, 5] = 100
    overlay = occupancy_from_array(
        cells, header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)), resolution=0.25
    )

    actual = overlay_occupied(base, overlay)

    assert occupancy_view(actual)[2, 5] == 100
    assert occupancy_view(actual)[3, 3] == 100
    assert actual.header == base.header
    assert actual.info.map_load_time == base.info.map_load_time
    other = occupancy_from_array(
        np.zeros((2, 2), dtype=np.int8), header=overlay.header, resolution=0.25
    )
    with pytest.raises(ValueError, match="Grid shapes must match"):
        overlay_occupied(base, other)


def test_generated_and_legacy_extrusion_produce_identical_offline_scene(grids):
    generated, legacy = grids

    actual = generate_mujoco_scene(generated)

    assert actual == generate_mujoco_scene(legacy)
    assert identify_convex_shapes(generated) == identify_convex_shapes(legacy)
    assert '<mujoco model="scene_occupancy">' in actual
    assert 'name="wall_0"' in actual
