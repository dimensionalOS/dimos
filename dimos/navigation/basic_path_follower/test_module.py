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

from __future__ import annotations

from collections.abc import Generator
import time

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.basic_path_follower.module import BasicPathFollower


def _odom(x: float = 0.0, y: float = 0.0, yaw_quat: tuple = (0, 0, 0, 1)) -> Odometry:
    qx, qy, qz, qw = yaw_quat
    return Odometry(ts=time.time(), frame_id="odom", pose=Pose(x, y, 0.0, qx, qy, qz, qw))


def _path(*xy: tuple) -> Path:
    poses = [PoseStamped(ts=time.time(), frame_id="odom", position=(x, y, 0.0)) for x, y in xy]
    return Path(ts=time.time(), frame_id="odom", poses=poses)


def _wait_for(predicate, timeout: float = 1.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


@pytest.fixture()
def follower() -> Generator[tuple[BasicPathFollower, list, list], None, None]:
    module = BasicPathFollower(control_frequency=100.0)
    cmds: list = []
    reached: list = []
    unsubs = [
        module.nav_cmd_vel.subscribe(cmds.append),
        module.goal_reached.subscribe(reached.append),
    ]
    try:
        yield module, cmds, reached
    finally:
        module._cancel()
        for unsub in unsubs:
            unsub()
        module._close_module()


def test_drives_forward_along_path(follower):
    module, cmds, _ = follower
    module._on_odometry(_odom(0.0, 0.0))
    module._on_path(_path((1.0, 0.0), (2.0, 0.0), (3.0, 0.0)))

    assert _wait_for(lambda: any(c.linear.x > 0 for c in cmds))
    moving = [c for c in cmds if c.linear.x > 0]
    assert abs(moving[0].angular.z) < 0.1


def test_rotates_in_place_when_facing_away(follower):
    module, cmds, _ = follower
    module._on_odometry(_odom(0.0, 0.0, yaw_quat=(0, 0, 1, 0)))
    module._on_path(_path((1.0, 0.0), (2.0, 0.0)))

    assert _wait_for(lambda: len(cmds) > 0 and cmds[-1].linear.x == 0)
    assert abs(cmds[-1].angular.z) > 0


def test_goal_reached_when_near_last_waypoint(follower):
    module, cmds, reached = follower
    module._on_odometry(_odom(2.9, 0.0))
    module._on_path(_path((1.0, 0.0), (2.0, 0.0), (3.0, 0.0)))

    assert _wait_for(lambda: len(reached) == 1)
    assert reached[0].data
    assert cmds[-1].linear.x == 0
    assert cmds[-1].angular.z == 0


def test_stop_movement_cancels(follower):
    module, cmds, _ = follower
    module._on_odometry(_odom(0.0, 0.0))
    module._on_path(_path((1.0, 0.0), (2.0, 0.0)))
    assert _wait_for(lambda: any(c.linear.x > 0 for c in cmds))

    module._on_stop(Bool(True))
    time.sleep(0.05)
    count = len(cmds)
    time.sleep(0.1)
    assert len(cmds) == count
    assert cmds[-1].linear.x == 0


def test_empty_path_cancels(follower):
    module, cmds, _ = follower
    module._on_odometry(_odom(0.0, 0.0))
    module._on_path(_path((1.0, 0.0), (2.0, 0.0)))
    assert _wait_for(lambda: any(c.linear.x > 0 for c in cmds))

    module._on_path(_path())
    time.sleep(0.05)
    count = len(cmds)
    time.sleep(0.1)
    assert len(cmds) == count
