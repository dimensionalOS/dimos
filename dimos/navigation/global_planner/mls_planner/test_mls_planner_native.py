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
from typing import Generic, NamedTuple, TypeVar, cast
from unittest.mock import Mock

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time
import pytest
from pytest_mock import MockerFixture

from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.global_planner.mls_planner.mls_planner_native import (
    MLSPlannerNative,
    reports_goal,
)
from dimos.navigation.spec import position_goal

T = TypeVar("T")


def _route(*points: tuple[float, float], ts: float = 1000.25) -> Path:
    return Path(ts=ts, frame_id="odom", poses=[position_goal(x, y, 0.0, "odom") for x, y in points])


_GOAL = _route((1.0, 0.0))


def _at(*points: tuple[float, float]) -> list[dict[str, float]]:
    return [{"x": x, "y": y} for x, y in points]


class _Recorder(Transport[T], Generic[T]):
    """Keeps every message the wrapper sends on a port."""

    def __init__(self) -> None:
        self.sent: list[T] = []

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def broadcast(self, selfstream: Stream[T] | None, value: T) -> None:
        self.sent.append(value)

    def subscribe(
        self, callback: Callable[[T], object], selfstream: Stream[T] | None = None
    ) -> Callable[[], None]:
        return lambda: None


class _Rig(NamedTuple):
    module: MLSPlannerNative
    sent: list[PoseStamped]
    """Messages on the goal port, which only carries cancels now."""
    routes: list[Path]
    """Messages on the waypoints port."""
    pose: Mock
    tool_update: Mock
    tools: Mock
    """Records start_tool and stop_tool calls in order."""

    def updates(self) -> list[str]:
        return [call.args[1] for call in self.tool_update.call_args_list]

    def tool_calls(self) -> list[str]:
        return [name for name, _, _ in self.tools.mock_calls]


def _result(called: Awaitable[str]) -> str:
    """The return of an async skill, which has already run when called off the module loop."""
    return cast("str", called)


def _report(route: Path, status: int, text: str = "") -> GoalStatus:
    sec, nsec = route.ros_timestamp()
    return GoalStatus(goal_id=GoalID(stamp=Time(sec, nsec)), status=status, text=text)


@pytest.fixture()
def rig(mocker: MockerFixture) -> Generator[_Rig, None, None]:
    module = MLSPlannerNative(start_z_offset_m=0.3)
    goals: _Recorder[PoseStamped] = _Recorder()
    routes: _Recorder[Path] = _Recorder()
    module.goal.transport = goals
    module.waypoints.transport = routes
    pose = mocker.patch.object(module, "_pose", return_value=None)
    tool_update = mocker.patch.object(module, "tool_update")
    tools = Mock()
    mocker.patch.object(module, "start_tool", tools.start_tool)
    mocker.patch.object(module, "stop_tool", tools.stop_tool)
    yield _Rig(module, goals.sent, routes.sent, pose, tool_update, tools)
    module._goal = None
    module._close_module()


def test_the_planner_exposes_the_navigation_skills(rig: _Rig) -> None:
    assert {s.func_name for s in rig.module.get_skills()} == {"go_to", "stop_navigation"}


@pytest.mark.parametrize(
    ("status", "text", "outcome"),
    [
        (GoalStatus.SUCCEEDED, "reached", "Reached the target"),
        (GoalStatus.PREEMPTED, "replaced by another goal", "Replaced by another goal"),
        (GoalStatus.PREEMPTED, "canceled", "Canceled"),
        (GoalStatus.ABORTED, "no safe path", "Failed: no safe path"),
        (
            GoalStatus.ABORTED,
            "no progress, waypoint 2 of 3",
            "Failed: no progress, waypoint 2 of 3",
        ),
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
        pytest.param(_report(Path(ts=900.0), GoalStatus.SUCCEEDED), id="other goal"),
    ],
)
def test_a_report_that_does_not_end_the_goal_is_ignored(rig: _Rig, report: GoalStatus) -> None:
    rig.module._goal = _GOAL
    rig.module._on_nav_status(report)

    assert rig.updates() == []
    assert rig.module._goal is _GOAL


def test_a_report_for_a_go_to_already_replaced_leaves_the_new_one_alone(rig: _Rig) -> None:
    newer = _route((2.0, 0.0), ts=1001.0)
    rig.module._goal = newer

    rig.module._finish("Reached the target", cancel=False, ended=_GOAL)

    assert rig.updates() == []
    assert rig.module._goal is newer


def test_go_to_sends_the_point_at_the_height_of_the_floor_under_the_robot(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 1.5))

    _result(rig.module.go_to(_at((4.0, 2.0))))

    (route,) = rig.routes
    assert rig.module._goal is route
    (point,) = route.poses
    assert (point.x, point.y, point.z) == pytest.approx((4.0, 2.0, 1.2))
    assert route.frame_id == "odom"
    assert point.orientation.is_zero()


@pytest.mark.parametrize("degrees", [90.0, 0.0])
def test_go_to_with_degrees_sends_the_heading(rig: _Rig, degrees: float) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 1.5))

    _result(rig.module.go_to(_at((4.0, 2.0)), degrees=degrees))

    (route,) = rig.routes
    (goal,) = route.poses
    assert not goal.orientation.is_zero()
    assert goal.yaw == pytest.approx(math.radians(degrees))


def test_a_heading_goal_reports_how_far_off_the_heading_it_stopped(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(1.0, 3.0, 0.0))
    _result(rig.module.go_to(_at((1.0, 3.0)), degrees=90.0))

    assert _result(rig.module.stop_navigation()) == "Stopped."

    assert rig.updates() == [
        "Canceled; robot at (1.00, 3.00), 0.00 m from the target, 90 degrees off its heading."
    ]


def test_go_to_sends_nothing_without_a_pose(rig: _Rig) -> None:
    assert _result(rig.module.go_to(_at((4.0, 2.0)))).startswith("No goal sent")
    assert rig.routes == []


def test_a_second_go_to_replaces_the_one_under_way(rig: _Rig) -> None:
    rig.module._goal = _GOAL
    rig.pose.return_value = PoseStamped(position=(1.0, 0.0, 0.3))

    _result(rig.module.go_to(_at((4.0, 2.0))))

    assert rig.updates() == [
        "Replaced by another goal; robot at (1.00, 0.00), 0.00 m from the target."
    ]
    # The old stream closes before the new one opens, so each go_to has its own.
    assert rig.tool_calls() == ["stop_tool", "start_tool"]
    (route,) = rig.routes
    (point,) = route.poses
    assert (point.x, point.y) == (4.0, 2.0)
    assert rig.module._goal is not _GOAL


def test_a_go_to_without_a_pose_cancels_the_one_under_way(rig: _Rig) -> None:
    rig.module._goal = _GOAL

    assert _result(rig.module.go_to(_at((4.0, 2.0)))).startswith("No goal sent")
    (cancel,) = rig.sent
    assert math.isnan(cancel.x)
    assert rig.module._goal is None


def test_stop_navigation_cancels_the_goal_and_reports_how_far_it_was(rig: _Rig) -> None:
    rig.module._goal = _GOAL
    rig.pose.return_value = PoseStamped(position=(1.0, 3.0, 0.0))

    assert _result(rig.module.stop_navigation()) == "Stopped."

    assert rig.updates() == ["Canceled; robot at (1.00, 3.00), 3.00 m from the target."]
    (cancel,) = rig.sent
    assert all(math.isnan(v) for v in (cancel.x, cancel.y, cancel.z))


def test_stop_navigation_cancels_a_goal_that_did_not_come_from_go_to(rig: _Rig) -> None:
    assert _result(rig.module.stop_navigation()) == "Stopped."
    (cancel,) = rig.sent
    assert math.isnan(cancel.x)
    assert rig.updates() == []


_SQUARE = _at((2.0, 0.0), (4.0, 0.0), (4.0, 2.0))


def test_go_to_sends_every_waypoint_at_the_floor_with_the_heading_on_the_last(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 1.5))

    _result(rig.module.go_to(_SQUARE, degrees=90.0))

    (route,) = rig.routes
    assert rig.module._goal is route
    assert [(p.x, p.y, p.z) for p in route.poses] == pytest.approx(
        [(2.0, 0.0, 1.2), (4.0, 0.0, 1.2), (4.0, 2.0, 1.2)]
    )
    assert [p.orientation.is_zero() for p in route.poses] == [True, True, False]
    assert route.poses[-1].yaw == pytest.approx(math.radians(90.0))


def test_a_report_on_the_route_stamp_ends_the_go_to_against_its_last_waypoint(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 0.3))
    _result(rig.module.go_to(_SQUARE))
    (route,) = rig.routes
    rig.pose.return_value = PoseStamped(position=(4.0, 1.0, 0.3))

    rig.module._on_nav_status(_report(route, GoalStatus.ABORTED, "no progress, waypoint 3 of 3"))

    assert rig.updates() == [
        "Failed: no progress, waypoint 3 of 3; robot at (4.00, 1.00), 1.00 m from the target."
    ]
    assert rig.tool_calls() == ["start_tool", "stop_tool"]
    assert rig.module._goal is None


def test_stop_navigation_reports_the_distance_to_the_last_waypoint(rig: _Rig) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 0.3))
    _result(rig.module.go_to(_SQUARE))

    assert _result(rig.module.stop_navigation()) == "Stopped."

    assert rig.updates() == ["Canceled; robot at (0.00, 0.00), 4.47 m from the target."]
    (cancel,) = rig.sent
    assert math.isnan(cancel.x)


@pytest.mark.parametrize(
    "waypoints",
    [
        pytest.param([], id="empty"),
        pytest.param([{"x": 1.0}], id="missing y"),
        pytest.param([{"x": 1.0, "y": math.nan}], id="nan"),
        pytest.param([{"x": 1.0, "y": "two"}], id="not a number"),
    ],
)
def test_go_to_rejects_bad_waypoints_before_sending_anything(
    rig: _Rig, waypoints: list[dict[str, float]]
) -> None:
    rig.pose.return_value = PoseStamped(position=(0.0, 0.0, 0.3))

    with pytest.raises(ValueError):
        _result(rig.module.go_to(waypoints))

    assert rig.routes == []
    assert rig.tool_calls() == []


def test_a_route_keeps_its_stamp_across_the_wire() -> None:
    route = _route((1.0, 0.0), (2.0, 0.0), ts=1762300000.123456789)
    back = Path.lcm_decode(route.lcm_encode())
    assert reports_goal(_report(route, GoalStatus.SUCCEEDED), back)
    assert reports_goal(_report(back, GoalStatus.SUCCEEDED), route)
