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

"""The navigation skills on the MLS planner wrapper, without starting the binary."""

from __future__ import annotations

from collections.abc import Awaitable, Callable, Generator
import math
from typing import NamedTuple, cast
from unittest.mock import Mock

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
import pytest
from pytest_mock import MockerFixture

from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    PREEMPT_CANCELED,
    PREEMPT_REPLACED,
    MLSPlannerNative,
    goal_id,
)

_GOAL = PoseStamped(ts=1000.25, position=(1.0, 0.0, 0.0))


class _GoalTransport(Transport[PointStamped]):
    """Keeps every goal message the wrapper sends."""

    def __init__(self) -> None:
        self.sent: list[PointStamped] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Stream[PointStamped] | None, value: PointStamped) -> None:
        self.sent.append(value)

    def subscribe(
        self,
        callback: Callable[[PointStamped], object],
        selfstream: Stream[PointStamped] | None = None,
    ) -> Callable[[], None]:
        return lambda: None


class _Rig(NamedTuple):
    module: MLSPlannerNative
    sent: list[PointStamped]
    pose: Mock
    tool_update: Mock

    def updates(self) -> list[str]:
        return [call.args[1] for call in self.tool_update.call_args_list]


def _result(called: Awaitable[str]) -> str:
    """The return of an async skill, which has already run when called off the module loop."""
    return cast("str", called)


def _report(goal: PoseStamped, status: int, text: str = "") -> GoalStatus:
    return GoalStatus(goal_id=GoalID(id=goal_id(goal)), status=status, text=text)


@pytest.fixture()
def rig(mocker: MockerFixture) -> Generator[_Rig, None, None]:
    module = MLSPlannerNative(start_z_offset_m=0.3)
    transport = _GoalTransport()
    module.goal.transport = transport
    pose = mocker.patch.object(module, "_pose", return_value=None)
    tool_update = mocker.patch.object(module, "tool_update")
    mocker.patch.object(module, "start_tool")
    mocker.patch.object(module, "stop_tool")
    try:
        yield _Rig(module, transport.sent, pose, tool_update)
    finally:
        module._goal = None
        module._close_module()


def test_the_planner_exposes_the_navigation_skills(rig: _Rig) -> None:
    assert {s.func_name for s in rig.module.get_skills()} == {"go_to", "stop_navigation"}


def test_goal_id_is_the_stamp_as_the_planner_renders_it() -> None:
    assert goal_id(_GOAL) == "1000.250000000"


@pytest.mark.parametrize(
    ("status", "text", "outcome"),
    [
        (GoalStatus.SUCCEEDED, "reached", "Reached the target"),
        (GoalStatus.PREEMPTED, PREEMPT_REPLACED, "Replaced by another goal"),
        (GoalStatus.PREEMPTED, PREEMPT_CANCELED, "Canceled"),
        (GoalStatus.REJECTED, "bad goal", "Failed: bad goal"),
        (GoalStatus.ABORTED, "no safe path", "Blocked: no safe path"),
    ],
)
def test_a_report_that_ends_the_goal_ends_the_go_to(
    rig: _Rig, status: int, text: str, outcome: str
) -> None:
    rig.module._goal = _GOAL
    rig.module._on_nav_status(_report(_GOAL, status, text))

    assert rig.updates() == [f"{outcome}; robot position unknown."]
    assert rig.module._goal is None
    # The planner ended the goal itself, so there is nothing to cancel.
    assert rig.sent == []


@pytest.mark.parametrize(
    "report",
    [
        pytest.param(_report(_GOAL, GoalStatus.ACTIVE, "following the path"), id="held"),
        pytest.param(_report(PoseStamped(ts=900.0), GoalStatus.SUCCEEDED), id="other goal"),
    ],
)
def test_a_report_that_does_not_end_the_goal_is_ignored(rig: _Rig, report: GoalStatus) -> None:
    rig.module._goal = _GOAL
    rig.module._on_nav_status(report)

    assert rig.updates() == []
    assert rig.module._goal is _GOAL


def test_go_to_sends_the_point_at_the_height_of_the_floor_under_the_robot(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 1.5))

    _result(rig.module.go_to(4.0, 2.0))

    (point,) = rig.sent
    goal = rig.module._goal
    assert goal is not None
    assert (point.x, point.y, point.z) == pytest.approx((4.0, 2.0, 1.2))
    assert (point.ts, point.frame_id) == (goal.ts, "odom")


def test_go_to_sends_nothing_without_a_pose(rig: _Rig) -> None:
    assert _result(rig.module.go_to(4.0, 2.0)).startswith("No goal sent")
    assert rig.sent == []


def test_a_second_go_to_is_refused_while_one_is_under_way(rig: _Rig) -> None:
    rig.module._goal = _GOAL

    assert _result(rig.module.go_to(4.0, 2.0)).startswith("Already navigating")
    assert rig.sent == []


def test_stop_navigation_cancels_the_goal_and_reports_how_far_it_was(rig: _Rig) -> None:
    rig.module._goal = _GOAL
    rig.pose.return_value = PoseStamped(position=(1.0, 3.0, 0.0))

    assert _result(rig.module.stop_navigation()) == "Stopped."

    assert rig.updates() == ["Canceled; robot at (1.00, 3.00), 3.00 m from the target."]
    (cancel,) = rig.sent
    assert all(math.isnan(v) for v in (cancel.x, cancel.y, cancel.z))


def test_stop_navigation_with_no_goal_sends_nothing(rig: _Rig) -> None:
    assert _result(rig.module.stop_navigation()) == "Stopped."
    assert rig.sent == []
    assert rig.updates() == []
