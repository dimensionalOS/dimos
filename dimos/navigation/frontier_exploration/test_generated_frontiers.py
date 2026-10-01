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

import math

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.occupancy import grid_to_world
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)


@pytest.mark.parametrize("yaw", [0, math.pi / 2])
def test_generated_frontier_centroid_respects_map_origin(request, yaw):
    explorer = WavefrontFrontierExplorer(min_frontier_perimeter=0.1)
    request.addfinalizer(explorer.stop)
    cells = np.full((20, 20), -1, dtype=np.int8)
    cells[5:15, 5:15] = 0
    grid = OccupancyGrid(
        header=Header(frame_id="map"),
        info=MapMetaData(
            width=20,
            height=20,
            resolution=0.5,
            origin=Pose(
                position=Point(x=2, y=3, z=1), orientation=quaternion_from_euler(0, 0, yaw)
            ),
        ),
        data=cells.ravel(),
    )
    received = OccupancyGrid.decode(grid.encode())
    centroids = explorer.detect_frontiers(grid_to_world(received, (10, 10)), received)
    assert len(centroids) == 1
    expected = grid_to_world(received, (9.5, 9.5))
    assert (centroids[0].x, centroids[0].y, centroids[0].z) == pytest.approx(
        (expected.x, expected.y, expected.z)
    )


def test_stopping_exploration_retains_current_pose_header(request):
    explorer = WavefrontFrontierExplorer()
    request.addfinalizer(explorer.stop)
    source = PoseStamped(
        header=Header(frame_id="odom", stamp=Time(sec=1700000000, nanosec=123456789)),
        pose=Pose(position=Point(x=2, y=3), orientation=Quaternion(w=1)),
    )
    explorer.latest_odometry = source
    explorer.exploration_active = True
    goals = []
    explorer.goal_request.subscribe(lambda value: goals.append(PoseStamped.decode(value.encode())))
    assert explorer.stop_exploration()
    assert goals == [source]
    source.pose.position.x = 100
    assert goals[0].pose.position.x == 2


def test_exploration_loop_publishes_generated_goal_with_map_header(request):
    explorer = WavefrontFrontierExplorer(goal_timeout=0.001)
    request.addfinalizer(explorer.stop)
    cells = np.full((20, 20), -1, dtype=np.int8)
    cells[:, :10] = 0
    header = Header(frame_id="odom", stamp=Time(sec=1700000000, nanosec=123456789))
    explorer.latest_costmap = OccupancyGrid(
        header=header,
        info=MapMetaData(
            width=20, height=20, resolution=0.5, origin=Pose(orientation=Quaternion(w=1))
        ),
        data=cells.ravel(),
    )
    explorer.latest_odometry = PoseStamped(
        header=header, pose=Pose(position=Point(x=2, y=5), orientation=Quaternion(w=1))
    )
    goals = []

    def receive(value):
        goals.append(PoseStamped.decode(value.encode()))
        explorer.exploration_active = False

    explorer.goal_request.subscribe(receive)
    explorer.exploration_active = True
    explorer._run_exploration_loop()
    assert len(goals) == 1
    assert goals[0].header == header
    assert goals[0].pose.position.x == 5
