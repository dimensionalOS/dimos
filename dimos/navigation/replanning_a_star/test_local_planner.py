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

from threading import Event

import numpy as np
import pytest

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap


@pytest.fixture
def local_planner():
    config = GlobalConfig()
    navigation_map = NavigationMap(config, "gradient")
    navigation_map.update(
        OccupancyGrid(grid=np.zeros((60, 60), dtype=np.int8), resolution=0.1, origin=Pose())
    )
    planner = LocalPlanner(config, navigation_map, goal_tolerance=0.2)
    try:
        yield planner
    finally:
        thread = planner._thread
        planner.stop()
        if thread is not None:
            thread.join(timeout=2.0)
            assert not thread.is_alive()


def test_aligned_robot_at_path_start_moves_toward_distant_endpoint(local_planner):
    planner = local_planner
    planner.handle_odom(PoseStamped(position=[1.0, 2.0, 0.0]))
    path = Path(poses=[PoseStamped(position=[x, 2.0, 0.0]) for x in np.linspace(1.0, 3.0, 21)])
    progress = Event()
    forward_commands = []
    outcomes = []

    def on_command(command: Twist) -> None:
        if command.linear.x > 0:
            forward_commands.append(command)
            progress.set()

    def on_stop(outcome: str) -> None:
        outcomes.append(outcome)
        progress.set()

    with planner.cmd_vel.subscribe(on_command), planner.stopped_navigating.subscribe(on_stop):
        planner.start_planning(path)
        assert progress.wait(timeout=2.0)

    assert forward_commands
    assert outcomes == []


@pytest.mark.parametrize("positions", [[1.0], [0.0, 1.0]])
def test_already_at_endpoint_arrives_without_translation(local_planner, positions):
    planner = local_planner
    planner.handle_odom(PoseStamped(position=[1.0, 2.0, 0.0]))
    path = Path(poses=[PoseStamped(position=[x, 2.0, 0.0]) for x in positions])
    finished = Event()
    commands = []
    outcomes = []

    def on_stop(outcome: str) -> None:
        outcomes.append(outcome)
        finished.set()

    with planner.cmd_vel.subscribe(commands.append), planner.stopped_navigating.subscribe(on_stop):
        planner.start_planning(path)
        assert finished.wait(timeout=2.0)

    assert outcomes == ["arrived"]
    assert all(command.linear.x == 0 and command.linear.y == 0 for command in commands)


def test_single_pose_turn_does_not_translate_or_arrive_before_rotation(local_planner):
    planner = local_planner
    planner.handle_odom(PoseStamped(position=[1.0, 2.0, 0.0]))
    goal = PoseStamped(
        position=[1.0, 2.0, 0.0], orientation=Quaternion.from_euler(Vector3(0.0, 0.0, np.pi / 2))
    )
    rotating = Event()
    commands = []
    outcomes = []

    def on_command(command: Twist) -> None:
        commands.append(command)
        if command.angular.z > 0:
            rotating.set()

    with (
        planner.cmd_vel.subscribe(on_command),
        planner.stopped_navigating.subscribe(outcomes.append),
    ):
        planner.start_planning(Path(poses=[goal]))
        assert rotating.wait(timeout=2.0)

    assert outcomes == []
    assert all(command.linear.x == 0 and command.linear.y == 0 for command in commands)
