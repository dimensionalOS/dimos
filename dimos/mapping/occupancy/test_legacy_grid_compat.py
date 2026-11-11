# Copyright 2025-2026 Dimensional Inc.
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

"""The shared grid algorithms keep a caller's grid family while consumers migrate.

Mapping and navigation consumers still pass legacy ``dimos.msgs`` values at this
layer. A migrated viewer caller passes generated CDR messages. Both must receive
their own family back, with identical cells, until the owning layer migrates.
"""

import pickle

from dimos_messages.nav_msgs.msg import OccupancyGrid as GeneratedOccupancyGrid
from dimos_messages.std_msgs.msg import Header
import numpy as np

from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.occupancy.operations import smooth_occupied
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.message_utils import occupancy as occupancy_utils, time as message_time
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def _cells() -> np.ndarray:
    cells = np.zeros((12, 12), dtype=np.int8)
    cells[4:6, 4:6] = 100
    cells[0, 0] = -1
    return cells


def _algorithms():
    return (gradient, lambda grid: simple_inflate(grid, 0.2), smooth_occupied)


def test_grid_algorithms_keep_the_callers_grid_family() -> None:
    cells = _cells()
    legacy = OccupancyGrid(grid=cells.copy(), resolution=0.1)
    generated = occupancy_utils.from_array(
        cells.copy(),
        header=Header(stamp=message_time.time_from_seconds(0.0), frame_id=""),
        resolution=0.1,
    )

    for algorithm in _algorithms():
        legacy_result = algorithm(legacy)
        generated_result = algorithm(generated)
        assert isinstance(legacy_result, OccupancyGrid)
        assert not isinstance(legacy_result, GeneratedOccupancyGrid)
        assert isinstance(generated_result, GeneratedOccupancyGrid)
        assert legacy_result.resolution == legacy.resolution
        np.testing.assert_array_equal(legacy_result.grid, occupancy_utils.view(generated_result))


def test_legacy_pointcloud_still_yields_a_legacy_grid() -> None:
    path = get_data("lcm_msgs") / "sensor_msgs/PointCloud2.pickle"
    with open(path, "rb") as handle:
        cloud = PointCloud2.lcm_decode(pickle.loads(handle.read()))

    grid = general_occupancy(cloud, resolution=0.05, min_height=0.1, max_height=2.0)

    assert isinstance(grid, OccupancyGrid)
    assert not isinstance(grid, GeneratedOccupancyGrid)
    assert grid.width > 0 and grid.height > 0
    assert grid.frame_id == cloud.frame_id


def test_legacy_grid_renders_to_a_legacy_image() -> None:
    image = visualize_occupancy_grid(OccupancyGrid(grid=_cells(), resolution=0.1), "rainbow")

    assert isinstance(image, Image)
    assert image.data.shape[:2] == (12, 12)
