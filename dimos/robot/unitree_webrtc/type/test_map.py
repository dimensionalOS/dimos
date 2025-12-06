# Copyright 2025 Dimensional Inc.
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

import pytest

from dimos.robot.unitree_webrtc.testing.helpers import show3d, show3d_stream
from dimos.robot.unitree_webrtc.testing.mock import Mock
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map, splice_sphere
from dimos.utils.reactive import backpressure
from dimos.utils.testing import SensorReplay


@pytest.mark.vis
def test_costmap_vis():
    map = Map()
    for frame in Mock("office").iterate():
        print(frame)
        map.add_frame(frame)
    costmap = map.costmap
    print(costmap)
    show3d(costmap.smudge().pointcloud, title="Costmap").run()


@pytest.mark.vis
def test_reconstruction_with_realtime_vis():
    show3d_stream(Map().consume(Mock("office").stream(rate_hz=60.0)), clearframe=True).run()


@pytest.mark.vis
def test_splice_vis():
    mock = Mock("test")
    target = mock.load("a")
    insert = mock.load("b")
    show3d(splice_sphere(target.pointcloud, insert.pointcloud, shrink=0.7)).run()


@pytest.mark.vis
def test_robot_vis():
    show3d_stream(
        Map().consume(backpressure(Mock("office").stream())),
        clearframe=True,
        title="gloal dynamic map test",
    )


def test_robot_mapping():
    lidar_stream = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
    map = Map(voxel_size=0.5)

    # this will block until map has consumed the whole stream
    map.consume(lidar_stream.stream()).run()

    # we investigate built map
    costmap = map.costmap()

    assert costmap.grid.shape == (404, 276)

    assert 70 <= costmap.unknown_percent <= 80, (
        f"Unknown percent {costmap.unknown_percent} is not within the range 70-80"
    )

    assert 5 < costmap.free_percent < 10, (
        f"Free percent {costmap.free_percent} is not within the range 5-10"
    )

    assert 8 < costmap.occupied_percent < 15, (
        f"Occupied percent {costmap.occupied_percent} is not within the range 8-15"
    )
