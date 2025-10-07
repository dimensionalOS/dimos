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

from dimos.robot.unitree_webrtc.testing.helpers import show3d
from dimos.robot.unitree_webrtc.testing.mock import Mock
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.map import Map, splice_sphere
from dimos.utils.testing import SensorReplay


@pytest.mark.vis
def test_costmap_vis():
    map = Map()
    map.start()
    mock = Mock("office")
    frames = list(mock.iterate())

    for frame in frames:
        print(frame)
        map.add_frame(frame)

    # Get global map and costmap
    global_map = map.to_lidar_message()
    print(f"Global map has {len(global_map.pointcloud.points)} points")
    show3d(global_map.pointcloud, title="Global Map").run()


@pytest.mark.vis
def test_reconstruction_with_realtime_vis():
    map = Map()
    map.start()
    mock = Mock("office")

    # Process frames and visualize final map
    for frame in mock.iterate():
        map.add_frame(frame)

    show3d(map.pointcloud, title="Reconstructed Map").run()


@pytest.mark.vis
def test_splice_vis():
    mock = Mock("test")
    target = mock.load("a")
    insert = mock.load("b")
    show3d(splice_sphere(target.pointcloud, insert.pointcloud, shrink=0.7)).run()


@pytest.mark.vis
def test_robot_vis():
    map = Map()
    map.start()
    mock = Mock("office")

    # Process all frames
    for frame in mock.iterate():
        map.add_frame(frame)

    show3d(map.pointcloud, title="global dynamic map test").run()


def test_robot_mapping():
    lidar_replay = SensorReplay("office_lidar", autocast=LidarMessage.from_msg)
    map = Map(voxel_size=0.5)

    # Mock the output streams to avoid publishing errors
    class MockStream:
        def publish(self, msg):
            pass  # Do nothing

    map.local_costmap = MockStream()
    map.global_costmap = MockStream()
    map.global_map = MockStream()

    # Process all frames from replay
    for frame in lidar_replay.iterate():
        map.add_frame(frame)

    # Check the built map
    global_map = map.to_lidar_message()
    pointcloud = global_map.pointcloud

    # Verify map has points
    assert len(pointcloud.points) > 0
    print(f"Map contains {len(pointcloud.points)} points")

    map._close_module()
