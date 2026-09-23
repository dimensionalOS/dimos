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

"""Generated message boundaries for the command center."""

import asyncio
import base64
import math
import zlib

from dimos_generated.geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Twist,
    TwistStamped,
)
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
import pytest

from dimos.msgs.geometry import quaternion_from_euler
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule


@pytest.fixture
def module():
    value = WebsocketVisModule()
    value._create_server()
    try:
        yield value
    finally:
        value.stop()


def test_click_and_velocity_generate_cdr_messages(module):
    points, velocities, stamped = [], [], []
    subscriptions = [
        module.goal_request.subscribe(lambda msg: points.append(PoseStamped.decode(msg.encode()))),
        module.tele_cmd_vel.subscribe(lambda msg: velocities.append(Twist.decode(msg.encode()))),
        module.movecmd_stamped.subscribe(
            lambda msg: stamped.append(TwistStamped.decode(msg.encode()))
        ),
    ]
    try:
        handlers = module.sio.handlers["/"]
        asyncio.run(handlers["click"]("client", [2.5, -1.0]))
        asyncio.run(
            handlers["move_command"](
                "client",
                {
                    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.8},
                },
            )
        )
        assert points[0].header.frame_id == "world"
        assert points[0].pose == Pose(position=Point(x=2.5, y=-1), orientation=Quaternion(w=1))
        assert velocities[0].linear.x == 0.5
        assert velocities[0].angular.z == 0.8
        assert stamped[0].twist == velocities[0]
        assert stamped[0].header.frame_id == "base_link"
    finally:
        for unsubscribe in subscriptions:
            unsubscribe()


def test_generated_pose_path_and_costmap_produce_browser_state(module):
    pose = PoseStamped(pose=Pose(position=Point(x=2, y=3, z=4)))
    module._on_robot_pose(PoseStamped.decode(pose.encode()))
    module._on_path(Path(poses=[pose]))
    grid = OccupancyGrid(
        info=MapMetaData(
            width=2,
            height=2,
            resolution=1,
            origin=Pose(
                position=Point(x=5, y=-2), orientation=quaternion_from_euler(0, 0, math.pi / 2)
            ),
        ),
        data=[100, 0, 0, -1],
    )
    wire = grid.encode()
    module._on_global_costmap(OccupancyGrid.decode(wire))
    assert module.vis_state["robot_pose"] == {"type": "vector", "c": [2, 3, 4]}
    assert module.vis_state["path"] == {"type": "path", "points": [[2, 3]]}
    costmap = module.vis_state["costmap"]
    assert costmap["origin"] == {"type": "vector", "c": [5, -2, 0]}
    assert costmap["origin_theta"] == pytest.approx(math.pi / 2)
    assert costmap["resolution"] == 1
    assert costmap["grid"]["shape"] == [2, 2]
    assert list(zlib.decompress(base64.b64decode(costmap["grid"]["data"]))) == [100, 100, 100, 255]
    assert grid.encode() == wire
