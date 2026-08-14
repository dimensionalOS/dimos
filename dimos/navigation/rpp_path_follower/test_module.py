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

from collections.abc import Iterator
import math
import time
from typing import Any

import pytest
from pytest_mock import MockerFixture

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.rpp_path_follower.module import RPPPathFollower
from dimos.protocol.rpc.spec import RPCSpec


class _NoopRPC(RPCSpec):
    def __init__(self, **_kwargs: Any) -> None:
        pass

    def serve_rpc(self, _function: Any, _name: str) -> Any:
        return lambda: None

    def call(self, _name: str, _arguments: Any, _callback: Any) -> Any:
        return lambda: None

    def call_nowait(self, _name: str, _arguments: Any) -> None:
        pass


def _pose(x: float, y: float, yaw: float) -> PoseStamped:
    return PoseStamped(
        ts=1.0,
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
    )


def _path(goal_x: float, goal_y: float, goal_yaw: float = 0.0) -> Path:
    path_yaw = math.atan2(goal_y, goal_x)
    return Path(
        ts=1.0,
        frame_id="world",
        poses=[_pose(0.0, 0.0, path_yaw), _pose(goal_x, goal_y, goal_yaw)],
    )


@pytest.fixture
def follower(mocker: MockerFixture) -> Iterator[RPPPathFollower]:
    mocker.patch("dimos.core.module.get_loop", return_value=(None, None))
    module = RPPPathFollower(
        rpc_transport=_NoopRPC,
        speed=0.25,
        min_linear_speed=0.15,
        slowdown_distance=0.4,
        goal_tolerance=0.06,
        orientation_tolerance=math.radians(5.0),
        max_yaw_rate=0.25,
        min_angular_speed=0.12,
        rotation_threshold=math.radians(20.0),
        lookahead_dist=0.3,
        lookahead_min=0.2,
        lookahead_max=0.4,
        lookahead_speed_scale=1.0,
        synthesize_tangent_headings=True,
    )
    yield module
    module.stop()


def test_explicit_manipulation_stance_yaw_is_preserved(follower, mocker) -> None:
    mocker.patch.object(follower.nav_cmd_vel, "publish")
    follower._on_base_pose(_pose(0.0, 0.0, 0.0))
    follower._on_path(_path(0.0, 1.0, goal_yaw=0.0))

    follower._control_once(time.monotonic())

    assert follower._controller._path is not None
    assert follower._controller._path.poses[-1].orientation.euler[2] == pytest.approx(0.0)


def test_large_heading_error_rotates_within_g1_yaw_limit(follower, mocker) -> None:
    publish = mocker.patch.object(follower.nav_cmd_vel, "publish")
    follower._on_base_pose(_pose(0.0, 0.0, 0.0))
    follower._on_path(_path(0.0, 1.0))

    follower._control_once(time.monotonic())

    command = publish.call_args.args[0]
    assert command.linear.x == 0.0
    assert command.angular.z == pytest.approx(0.25)


def test_near_goal_regulation_keeps_usable_linear_floor(follower, mocker) -> None:
    publish = mocker.patch.object(follower.nav_cmd_vel, "publish")
    follower._on_base_pose(_pose(0.0, 0.0, 0.0))
    follower._on_path(_path(0.2, 0.0))

    follower._control_once(time.monotonic())

    command = publish.call_args.args[0]
    assert command.linear.x == pytest.approx(0.15)
    assert command.angular.z == 0.0


def test_arrival_publishes_zero_and_goal_reached(follower, mocker) -> None:
    command_publish = mocker.patch.object(follower.nav_cmd_vel, "publish")
    reached_publish = mocker.patch.object(follower.goal_reached, "publish")
    follower._on_base_pose(_pose(0.0, 0.0, 0.0))
    follower._on_path(_path(1.0, 0.0))
    follower._control_once(time.monotonic())

    follower._on_base_pose(_pose(1.0, 0.0, 0.0))
    follower._control_once(time.monotonic())

    command = command_publish.call_args.args[0]
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
    assert reached_publish.call_args.args[0].data is True


def test_stale_pose_stops_an_active_controller(follower, mocker) -> None:
    publish = mocker.patch.object(follower.nav_cmd_vel, "publish")
    follower._on_base_pose(_pose(0.0, 0.0, 0.0))
    follower._on_path(_path(1.0, 0.0))
    follower._control_once(time.monotonic())
    publish.reset_mock()

    assert follower._pose_received_at is not None
    follower._control_once(follower._pose_received_at + follower.config.pose_timeout + 0.1)

    command = publish.call_args.args[0]
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0
