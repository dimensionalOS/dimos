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
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.msgs.occupancy import grid_to_world, occupancy_view, world_to_grid
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.utilities import point_to_pose_stamped


@pytest.mark.parametrize("router_name", ["random", "coverage", "frontier"])
@pytest.mark.parametrize("yaw", [0, math.pi / 2])
def test_generated_patrol_goals(router_name, yaw):
    cells = np.zeros((24, 24), dtype=np.int8)
    cells[[0, -1], :] = 100
    cells[:, [0, -1]] = 100
    grid = OccupancyGrid(
        header=Header(frame_id="map", stamp=Time(sec=1700000000, nanosec=123456789)),
        info=MapMetaData(
            width=24,
            height=24,
            resolution=0.5,
            origin=Pose(
                position=Point(x=2, y=3, z=1), orientation=quaternion_from_euler(0, 0, yaw)
            ),
        ),
        data=cells.ravel(),
    )
    router = create_patrol_router(router_name, 0.5)
    assert router.next_goal() is None
    received = OccupancyGrid.decode(grid.encode())
    router.handle_occupancy_grid(received)
    router.handle_odom(point_to_pose_stamped(grid_to_world(received, (12, 12)), received.header))
    initial = router.get_saturation()
    goal = router.next_goal()
    assert goal is not None
    decoded = PoseStamped.decode(goal.encode())
    assert decoded.header == grid.header
    assert decoded.pose.position.z == pytest.approx(1)
    x, y = world_to_grid(received, decoded.pose.position)
    assert occupancy_view(received)[round(y), round(x)] == 0
    router.handle_odom(decoded)
    assert router.get_saturation() > initial
    router.reset()
    assert router.next_goal() is None
    assert router.get_saturation() == 0
