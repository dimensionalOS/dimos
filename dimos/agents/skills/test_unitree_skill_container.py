# Copyright 2025-2026 Dimensional Inc.
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

import difflib
from typing import Any

from langchain_core.messages import HumanMessage

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.navigation.base import NavigationState
from dimos.robot.unitree.unitree_skill_container import _UNITREE_COMMANDS, UnitreeSkillContainer


class StubNavigation(Module):
    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        return True

    @rpc
    def get_state(self) -> NavigationState:
        return NavigationState.IDLE

    @rpc
    def is_goal_reached(self) -> bool:
        return False

    @rpc
    def cancel_goal(self) -> bool:
        return True


class StubGO2Connection(Module):
    last_twist: Twist | None = None
    last_duration: float | None = None

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        self.last_twist = twist
        self.last_duration = duration
        return True

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return {}


class PlainGO2Connection:
    last_twist: Twist | None = None
    last_duration: float | None = None

    def __init__(self) -> None:
        self.calls: list[tuple[Twist, float]] = []

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        self.last_twist = twist
        self.last_duration = duration
        self.calls.append((twist, duration))
        return True


class FakeTransform:
    def __init__(self, pose: PoseStamped) -> None:
        self._pose = pose
        self.ts = pose.ts

    def to_pose(self) -> PoseStamped:
        return self._pose


class FakeTF:
    def __init__(self, poses: list[PoseStamped]) -> None:
        self._poses = poses
        self._index = 0

    def get(self, _source: str, _target: str) -> FakeTransform:
        pose = self._poses[min(self._index, len(self._poses) - 1)]
        self._index += 1
        return FakeTransform(pose)


class MockedUnitreeSkill(UnitreeSkillContainer):
    pass


def _pose(x: float, y: float = 0.0, ts: float = 1.0) -> PoseStamped:
    return PoseStamped(ts=ts, position=[x, y, 0.0])


def test_pounce(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            MockedUnitreeSkill.blueprint(),
            StubNavigation.blueprint(),
            StubGO2Connection.blueprint(),
        ],
        messages=[HumanMessage("Pounce! Use the execute_sport_command tool.")],
    )

    response = history[-1].content.lower()
    assert "pounce" in response


def test_did_you_mean() -> None:
    suggestions = difflib.get_close_matches("Pounce", _UNITREE_COMMANDS.keys(), n=3, cutoff=0.6)
    assert "FrontPounce" in suggestions
    assert "Pose" in suggestions


def test_move_clamps_direct_velocity_without_tf_monitoring() -> None:
    skill = object.__new__(MockedUnitreeSkill)
    connection = PlainGO2Connection()
    skill._connection = connection  # type: ignore[assignment]

    result = skill.move(x=2.0, y=-2.0, yaw=3.0, duration=30.0)

    assert "velocity=(0.50, -0.40, 1.00)" in result
    assert len(connection.calls) == 11
    first_twist, first_duration = connection.calls[0]
    assert first_duration == 1.0
    assert first_twist.linear.x == 0.5
    assert first_twist.linear.y == -0.4
    assert first_twist.angular.z == 1.0
    assert connection.calls[-1][0].is_zero()
    assert connection.calls[-1][1] == 0.0


def test_move_stops_and_recovers_when_tf_progress_stalls() -> None:
    skill = object.__new__(MockedUnitreeSkill)
    connection = PlainGO2Connection()
    skill._connection = connection  # type: ignore[assignment]
    skill._tf = FakeTF([_pose(0.0, ts=1.0), _pose(0.0, ts=2.0), _pose(0.0, ts=3.0)])  # type: ignore[assignment]

    result = skill.move(x=0.5, duration=5.0)

    assert "appears blocked" in result
    assert "reverse recovery" in result
    assert len(connection.calls) == 5
    assert connection.calls[0][0].linear.x == 0.5
    assert connection.calls[0][1] == 1.0
    assert connection.calls[1][0].linear.x == 0.5
    assert connection.calls[1][1] == 1.0
    assert connection.calls[2][0].is_zero()
    assert connection.calls[3][0].linear.x == -0.2
    assert connection.calls[3][1] == 0.8
    assert connection.calls[4][0].is_zero()


def test_move_does_not_report_blocked_when_tf_timestamp_is_stale() -> None:
    skill = object.__new__(MockedUnitreeSkill)
    connection = PlainGO2Connection()
    skill._connection = connection  # type: ignore[assignment]
    skill._tf = FakeTF([_pose(0.0, ts=1.0)] * 5)  # type: ignore[assignment]

    result = skill.move(x=0.5, duration=3.0)

    assert "Completed direct movement" in result
    assert "appears blocked" not in result
    assert len(connection.calls) == 4
    assert all(call[0].linear.x == 0.5 for call in connection.calls[:-1])
    assert connection.calls[-1][0].is_zero()
