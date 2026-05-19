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

import math
from typing import Any

from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.types.robot_location import RobotLocation


class FakeCamera(Module):
    color_image: Out[Image]


class FakeOdom(Module):
    odom: Out[PoseStamped]


class StubSpatialMemory(Module):
    @rpc
    def tag_location(self, robot_location: RobotLocation) -> bool:
        return True

    @rpc
    def query_tagged_location(self, query: str) -> RobotLocation | None:
        return None

    @rpc
    def query_by_text(self, text: str, limit: int = 5) -> list[dict[str, Any]]:
        return []


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


class StubObjectTracking(Module):
    @rpc
    def track(self, bbox: list[float]) -> dict[str, Any]:
        return {}

    @rpc
    def stop_track(self) -> bool:
        return True

    @rpc
    def is_tracking(self) -> bool:
        return False


_STUB_BLUEPRINTS = [
    StubSpatialMemory.blueprint(),
    StubNavigation.blueprint(),
    StubObjectTracking.blueprint(),
]


class MockedStopNavSkill(NavigationSkillContainer):
    _skill_started = True

    def _cancel_goal_and_stop(self):
        pass


class MockedExploreNavSkill(NavigationSkillContainer):
    _skill_started = True

    def _start_exploration(self, timeout):
        return "Exploration completed successfuly"

    def _cancel_goal_and_stop(self):
        pass


class MockedSemanticNavSkill(NavigationSkillContainer):
    _skill_started = True

    def _navigate_by_tagged_location(self, query):
        return None

    def _navigate_to_object(self, query):
        return None

    def _navigate_using_semantic_map(self, query):
        return f"Successfuly arrived at '{query}'"


class MockedPositionNavSkill(NavigationSkillContainer):
    """Direct-instantiation harness for navigate_to_position /
    rotate_toward_position / current_pose tests. Skips the heavy parent
    __init__ and records goals at the _navigate_to boundary."""

    def __init__(self, latest_odom: PoseStamped | None = None) -> None:
        self._skill_started = True
        self._latest_odom = latest_odom
        self.recorded_goals: list[PoseStamped] = []

    def _navigate_to(self, pose: PoseStamped, message: str) -> str:
        self.recorded_goals.append(pose)
        return message


def test_stop_movement(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedStopNavSkill.blueprint(),
            *_STUB_BLUEPRINTS,
        ],
        messages=[HumanMessage("Stop moving. Use the stop_movement tool.")],
    )

    assert "stopped" in history[-1].content.lower()


def test_start_exploration(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedExploreNavSkill.blueprint(),
            *_STUB_BLUEPRINTS,
        ],
        messages=[
            HumanMessage("Take a look around for 10 seconds. Use the start_exploration tool.")
        ],
    )

    assert "explor" in history[-1].content.lower()


def test_go_to_semantic_location(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedSemanticNavSkill.blueprint(),
            *_STUB_BLUEPRINTS,
        ],
        messages=[HumanMessage("Go to the bookshelf. Use the navigate_with_text tool.")],
    )

    assert "success" in history[-1].content.lower()


def test_navigate_to_position_sets_goal_with_yaw() -> None:
    skill = MockedPositionNavSkill()
    skill.navigate_to_position(x=3.0, y=4.0, yaw_deg=90.0)

    assert len(skill.recorded_goals) == 1
    goal = skill.recorded_goals[0]
    assert goal.frame_id == "map"
    assert goal.position.x == pytest.approx(3.0)
    assert goal.position.y == pytest.approx(4.0)
    assert math.degrees(goal.yaw) == pytest.approx(90.0, abs=0.1)


def test_rotate_toward_position_computes_yaw_from_odom() -> None:
    skill = MockedPositionNavSkill(
        latest_odom=PoseStamped(
            position=(0.0, 0.0, 0.0),
            orientation=(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
        )
    )
    skill.rotate_toward_position(x=0.0, y=1.0)

    assert len(skill.recorded_goals) == 1
    goal = skill.recorded_goals[0]
    assert goal.position.x == pytest.approx(0.0)
    assert goal.position.y == pytest.approx(0.0)
    assert math.degrees(goal.yaw) == pytest.approx(90.0, abs=0.1)


def test_rotate_toward_position_without_odom_returns_message() -> None:
    skill = MockedPositionNavSkill()

    result = skill.rotate_toward_position(x=1.0, y=0.0)

    assert "no odometry" in result.lower()
    assert skill.recorded_goals == []


def test_current_pose_returns_pose_stamped() -> None:
    odom = PoseStamped(
        position=(1.0, 2.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        frame_id="map",
    )
    skill = MockedPositionNavSkill(latest_odom=odom)

    result = skill.current_pose()

    assert isinstance(result, PoseStamped)
    assert result is odom
    assert result.position.x == pytest.approx(1.0)
    assert result.position.y == pytest.approx(2.0)
    assert result.frame_id == "map"


def test_current_pose_without_odom_raises() -> None:
    skill = MockedPositionNavSkill()

    with pytest.raises(RuntimeError, match="No odometry"):
        skill.current_pose()
