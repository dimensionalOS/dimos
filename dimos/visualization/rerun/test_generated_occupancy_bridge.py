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

import math
from types import SimpleNamespace
from unittest.mock import patch

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np
import pytest

from dimos.visualization.rerun.bridge import RerunBridgeModule
from dimos.visualization.rerun.message_helpers import occupancy_mesh

pytestmark = pytest.mark.filterwarnings("error::rerun.error_utils.RerunWarning")


def test_rotated_occupancy_cdr_mesh_and_texture() -> None:
    grid = OccupancyGrid(
        header=Header(frame_id="map", stamp=Time(sec=0, nanosec=0)),
        info=MapMetaData(
            width=2,
            height=2,
            resolution=0.5,
            origin=Pose(
                position=Point(x=10, y=20, z=3),
                orientation=Quaternion(z=math.sqrt(0.5), w=math.sqrt(0.5), x=0.0, y=0.0),
            ),
            map_load_time=Time(sec=0, nanosec=0),
        ),
        data=np.array([0, 100, -1, 50], dtype=np.int8),
    )
    decoded = cdr_decode(cdr_encode(grid), OccupancyGrid)
    bridge = RerunBridgeModule()
    bridge._min_intervals = {}
    try:
        with patch("rerun.log") as log:
            bridge._on_message(decoded, SimpleNamespace(name="/map"))
            mesh = log.call_args_list[0].args[1]
            np.testing.assert_allclose(
                mesh.vertex_positions.as_arrow_array().to_pylist(),
                [[10, 20, 3], [10, 21, 3], [9, 21, 3], [9, 20, 3]],
            )
            texture = mesh.albedo_texture_buffer.as_arrow_array().to_pylist()[0]
            assert list(texture) == [0, 0, 0, 255, 36, 36, 64, 255, 72, 73, 129, 255, 0, 0, 0, 255]
            assert log.call_args_list[1].args[1].parent_frame.as_arrow_array().to_pylist() == [
                "tf#/map"
            ]
    finally:
        bridge.stop()
    assert cdr_encode(decoded) == cdr_encode(grid)


def test_empty_and_invalid_occupancy_data() -> None:
    assert (
        occupancy_mesh(
            OccupancyGrid(
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                info=MapMetaData(
                    map_load_time=Time(sec=0, nanosec=0),
                    resolution=0.0,
                    width=0,
                    height=0,
                    origin=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                data=np.array([], dtype=np.int8),
            )
        )
        .vertex_positions.as_arrow_array()
        .to_pylist()
        == []
    )
    with pytest.raises(ValueError, match="dimensions"):
        occupancy_mesh(
            OccupancyGrid(
                info=MapMetaData(
                    width=2,
                    height=2,
                    map_load_time=Time(sec=0, nanosec=0),
                    resolution=0.0,
                    origin=Pose(
                        position=Point(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    ),
                ),
                data=np.array([0], dtype=np.int8),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
