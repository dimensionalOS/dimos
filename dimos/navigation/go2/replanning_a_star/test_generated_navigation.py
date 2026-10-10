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
from threading import Event

from dimos_generated.builtin_interfaces.msg import Time
from dimos_generated.geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
from dimos_generated.nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from dimos_generated.std_msgs.msg import Bool, Header
from dimos_message_build.registry import decode as cdr_decode, encode as cdr_encode
import numpy as np

from dimos.core.global_config import GlobalConfig
from dimos.navigation.go2.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.go2.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.go2.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.go2.replanning_a_star.path_clearance import PathClearance


def test_global_planner_reaches_goal_with_generated_cdr_feedback():
    header = Header(stamp=Time(sec=1700000000, nanosec=123456789), frame_id="map")
    grid = OccupancyGrid(
        header=header,
        info=MapMetaData(
            width=24,
            height=24,
            resolution=0.25,
            origin=Pose(
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                position=Point(x=0.0, y=0.0, z=0.0),
            ),
            map_load_time=Time(sec=0, nanosec=0),
        ),
        data=np.zeros(24 * 24, dtype=np.int8),
    )
    planner = GlobalPlanner(GlobalConfig())
    planner._local_planner._control_frequency = 100
    arrived = Event()
    commands: list[Twist] = []
    paths: list[Path] = []
    position = [1.0, 1.0, 0.0]

    def update_odom() -> None:
        message = PoseStamped(
            header=header,
            pose=Pose(
                position=Point(x=position[0], y=position[1], z=0.0),
                orientation=Quaternion(
                    z=math.sin(position[2] / 2), w=math.cos(position[2] / 2), x=0.0, y=0.0
                ),
            ),
        )
        planner.handle_odom(cdr_decode(cdr_encode(message), PoseStamped))

    def command(message: Twist) -> None:
        decoded = cdr_decode(cdr_encode(message), Twist)
        commands.append(decoded)
        position[0] += decoded.linear.x * math.cos(position[2]) * 0.1
        position[1] += decoded.linear.x * math.sin(position[2]) * 0.1
        position[2] += decoded.angular.z * 0.1
        update_odom()

    def goal_reached(message: Bool) -> None:
        if cdr_decode(cdr_encode(message), Bool).data:
            arrived.set()

    subscriptions = [
        planner.cmd_vel.subscribe(command),
        planner.goal_reached.subscribe(goal_reached),
        planner.path.subscribe(lambda value: paths.append(cdr_decode(cdr_encode(value), Path))),
    ]
    try:
        planner.handle_global_costmap(cdr_decode(cdr_encode(grid), OccupancyGrid))
        update_odom()
        planner.start()
        planner.handle_goal_request(
            PoseStamped(
                header=header,
                pose=Pose(
                    position=Point(x=3, y=1, z=0.0),
                    orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                ),
            )
        )
        assert arrived.wait(5), "planner did not report arrival"
        assert abs(position[0] - 3) < 0.25
        assert abs(position[1] - 1) < 0.1
        assert any(command.linear.x > 0 for command in commands)
        published = next(path for path in paths if len(path.poses))
        assert published.header == header
        assert all(pose.header == header for pose in published.poses)
    finally:
        planner.stop()
        for subscription in subscriptions:
            subscription.dispose()
    assert commands[-1] == Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
    )


def test_clearance_mask_tracks_generated_map_geometry():
    path = Path(
        poses=[
            PoseStamped(
                pose=Pose(
                    position=Point(x=x, y=2, z=0.0),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                ),
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
            for x in [2, 3, 4]
        ],
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    clearance = PathClearance(GlobalConfig(robot_width=0.1), path)
    grid = OccupancyGrid(
        info=MapMetaData(
            width=10,
            height=10,
            resolution=1,
            origin=Pose(
                orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                position=Point(x=0.0, y=0.0, z=0.0),
            ),
            map_load_time=Time(sec=0, nanosec=0),
        ),
        data=np.asarray([0] * 100, dtype=np.int8),
        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
    )
    clearance.update_costmap(grid)
    original = clearance.mask.copy()
    assert original[2, 2]
    grid.info.origin.position.x = 1
    shifted = clearance.mask.copy()
    assert shifted[2, 1] and not original[2, 1]
    grid.info.resolution = 0.5
    scaled = clearance.mask
    assert scaled[4, 2] and not shifted[4, 2]


def test_cancel_joins_control_loop_and_ends_with_generated_stop():
    config = GlobalConfig()
    maps = NavigationMap(config, "gradient")
    maps.update(
        OccupancyGrid(
            info=MapMetaData(
                width=10,
                height=10,
                resolution=1,
                origin=Pose(
                    orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                    position=Point(x=0.0, y=0.0, z=0.0),
                ),
                map_load_time=Time(sec=0, nanosec=0),
            ),
            data=np.asarray([0] * 100, dtype=np.int8),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
    planner = LocalPlanner(config, maps, 0.2)
    planner.handle_odom(
        PoseStamped(
            pose=Pose(
                position=Point(x=1, y=1, z=0.0), orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0)
            ),
            header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
        )
    )
    moving = Event()
    commands = []

    def command(message: Twist) -> None:
        commands.append(cdr_decode(cdr_encode(message), Twist))
        if message.linear.x > 0:
            moving.set()

    subscription = planner.cmd_vel.subscribe(command)
    try:
        planner.start_planning(
            Path(
                poses=[
                    PoseStamped(
                        pose=Pose(
                            position=Point(x=x, y=1, z=0.0),
                            orientation=Quaternion(w=1, x=0.0, y=0.0, z=0.0),
                        ),
                        header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
                    )
                    for x in [1, 2, 3]
                ],
                header=Header(stamp=Time(sec=0, nanosec=0), frame_id=""),
            )
        )
        thread = planner._thread
        assert thread is not None
        assert moving.wait(2)
        planner.stop()
        assert not thread.is_alive()
        assert commands[-1] == Twist(
            linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)
        )
    finally:
        planner.stop()
        subscription.dispose()
