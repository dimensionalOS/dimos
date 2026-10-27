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

"""AlfredHighLevel's wheel odometry, polled off a fake controller."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
import math
from typing import Any
from unittest.mock import MagicMock

import pytest

from dimos.core.module import Module
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.diy.alfred import effector_high_level
from dimos.robot.diy.alfred.effector_high_level import (
    AlfredHighLevel,
    AlfredHighLevelConfig,
    _rebase,
)

DT = 0.02
# Where the controller's count stood when the run started: 36 m and a turn into its day.
START = (36.0, -4.0, 1.0)
FORWARD = (36.0 + 0.1 * math.cos(1.0), -4.0 + 0.1 * math.sin(1.0), 1.0)  # 0.1 m ahead
TURNED = (FORWARD[0], FORWARD[1], 1.2)  # then 0.2 rad on the spot


class _Clock:
    """``time.time`` for the poll: one tick of ``dt`` per reading."""

    def __init__(self, dt: float) -> None:
        self.now, self.dt = 100.0, dt

    def time(self) -> float:
        self.now += self.dt
        return self.now


class _Answered:
    """A portal.Future that has already come back."""

    def __init__(self, value: Any) -> None:
        self._value = value

    def done(self) -> bool:
        return True

    def result(self) -> Any:
        return self._value


class _Controller:
    """The FlowBase's odometry RPC: cumulative poses in its inverted-Y frame, one per call,
    stopping the poll with the last."""

    def __init__(self, poses: list[tuple[float, float, float]], stop: asyncio.Event) -> None:
        self._poses = [(x, -y, -yaw) for x, y, yaw in poses]  # ROS frame in, driver frame out
        self._stop = stop

    def get_odometry(self, _request: dict[str, Any]) -> _Answered:
        x, y, yaw = self._poses.pop(0)
        if not self._poses:
            self._stop.set()
        return _Answered({"translation": [x, y], "rotation": yaw})


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> Iterator[AlfredHighLevel]:
    """An AlfredHighLevel with the framework's Module.__init__ skipped, the odometry port
    mocked and the clock ticking once per reading."""
    monkeypatch.setattr(Module, "__init__", lambda self, **kwargs: None)
    monkeypatch.setattr(effector_high_level, "time", _Clock(DT))
    module = AlfredHighLevel()
    module.config = AlfredHighLevelConfig(wheel_odometry_hz=1000.0)
    module.wheel_odometry = MagicMock()
    yield module


def _published(module: AlfredHighLevel, poses: list[tuple[float, float, float]]) -> list[Odometry]:
    """Run the poll over ``poses`` (ROS-frame x, y, yaw, ``DT`` apart) and collect its output."""
    module.wheel_odometry.reset_mock()

    async def run() -> None:
        module._odometry_stop = asyncio.Event()
        await module._poll_wheel_odometry(_Controller(poses, module._odometry_stop))

    asyncio.run(run())
    return [call.args[0] for call in module.wheel_odometry.publish.call_args_list]


def _pose(odometry: Odometry) -> tuple[float, float, float]:
    pose = odometry.pose.pose
    return (pose.position.x, pose.position.y, pose.orientation.to_euler().z)


def _twist(odometry: Odometry) -> tuple[float, float, float]:
    twist = odometry.twist.twist
    return (twist.linear.x, twist.linear.y, twist.angular.z)


def test_wheel_odometry_starts_where_the_run_started(module: AlfredHighLevel) -> None:
    """The controller counts from when its own driver booted; the run's first reading is
    the origin, so the Desktop view and a stack without a lidar start the robot at zero."""
    assert AlfredHighLevelConfig().rebase_odometry is True
    first, second, third = _published(module, [START, FORWARD, TURNED])
    assert _pose(first) == pytest.approx((0.0, 0.0, 0.0))
    assert _pose(second) == pytest.approx((0.1, 0.0, 0.0)), "0.1 m along the start heading"
    assert _pose(third) == pytest.approx((0.1, 0.0, 0.2))
    assert (first.frame_id, first.child_frame_id) == ("wheel_odom", "base_link")


def test_the_twist_is_the_same_body_motion_rebased_or_not(module: AlfredHighLevel) -> None:
    rebased = _published(module, [START, FORWARD, TURNED])
    module.config = AlfredHighLevelConfig(wheel_odometry_hz=1000.0, rebase_odometry=False)
    raw = _published(module, [START, FORWARD, TURNED])

    assert _pose(raw[0]) == pytest.approx(START), "off, the controller's own frame"
    assert _twist(rebased[0]) == _twist(raw[0]) == (0.0, 0.0, 0.0)
    assert _twist(rebased[1]) == pytest.approx((0.1 / DT, 0.0, 0.0))
    assert _twist(rebased[2]) == pytest.approx((0.0, 0.0, 0.2 / DT))
    for with_origin, without in zip(rebased, raw, strict=True):
        assert _twist(with_origin) == pytest.approx(_twist(without))


def test_rebase_is_a_change_of_origin_not_a_subtraction() -> None:
    """From an origin facing +y, a metre of world +y is a metre ahead and world +x is to
    the right; the yaw is wrapped against the origin's."""
    facing_left = (1.0, 2.0, math.pi / 2)
    assert _rebase(facing_left, 1.0, 3.0, math.pi / 2 + 0.1) == pytest.approx((1.0, 0.0, 0.1))
    assert _rebase(facing_left, 2.0, 2.0, math.pi / 2) == pytest.approx((0.0, -1.0, 0.0))
    assert _rebase((0.0, 0.0, 3.0), 0.0, 0.0, -3.0) == pytest.approx((0.0, 0.0, 2 * math.pi - 6.0))
