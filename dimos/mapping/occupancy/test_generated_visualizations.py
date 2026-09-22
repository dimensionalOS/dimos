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

# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.sensor_msgs.msg import Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.mapping.occupancy.visualizations import Palette, visualize_occupancy_grid
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.image import image_view


@pytest.mark.parametrize("palette", ["rainbow", "turbo"])
def test_generated_image_preserves_header_and_colors(palette: Palette) -> None:
    grid = OccupancyGrid(
        header=Header(stamp=Time(sec=1, nanosec=999), frame_id="map"),
        info=MapMetaData(width=3, height=1, resolution=1),
        data=[-1, 0, 100],
    )
    image = Image.decode(visualize_occupancy_grid(grid, palette).encode())
    assert image.header == grid.header
    assert image.encoding == "bgr8"
    assert image_view(image).shape == (1, 3, 3)
    np.testing.assert_array_equal(
        image_view(image)[0, 0], [0, 0, 0] if palette == "rainbow" else [28, 24, 34]
    )
    assert not np.array_equal(image_view(image)[0, 1], image_view(image)[0, 2])


@pytest.mark.parametrize("footprint", [False, True])
def test_path_rendering_is_relative_to_rotated_grid(footprint: bool) -> None:
    grid = OccupancyGrid(
        info=MapMetaData(
            width=10, height=10, resolution=1, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=[0] * 100,
    )
    path = Path(
        poses=[
            PoseStamped(pose=Pose(position=Point(x=x, y=3), orientation=Quaternion(w=1)))
            for x in [2, 4, 6]
        ]
    )
    expected = (
        visualize_path(grid, path, 1, 2)
        if footprint
        else visualize_occupancy_grid(grid, "rainbow", path)
    )
    grid.info.origin = Pose(
        position=Point(x=20, y=10), orientation=quaternion_from_euler(0, 0, math.pi / 2)
    )
    path.poses = [
        PoseStamped(
            pose=Pose(
                position=Point(x=17, y=10 + x), orientation=quaternion_from_euler(0, 0, math.pi / 2)
            )
        )
        for x in [2, 4, 6]
    ]
    actual = (
        visualize_path(grid, path, 1, 2)
        if footprint
        else visualize_occupancy_grid(grid, "rainbow", path)
    )
    np.testing.assert_array_equal(image_view(actual), image_view(expected))
