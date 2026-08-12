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

from typing import Any

from langchain_core.messages import HumanMessage
import pytest
from pytest_mock import MockerFixture

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


@pytest.fixture
def navigation_container(mocker: MockerFixture):
    mocker.patch("dimos.models.vl.qwen.QwenVlModel")
    container = NavigationSkillContainer()
    yield container
    container.stop()


def test_navigate_to_reports_synchronous_planner_failure(
    mocker: MockerFixture,
    navigation_container: NavigationSkillContainer,
) -> None:
    navigation = mocker.Mock()
    navigation.set_goal.return_value = True
    navigation.get_state.return_value = NavigationState.IDLE
    navigation.is_goal_reached.return_value = False
    navigation_container._navigation = navigation
    goal = PoseStamped()

    result = navigation_container._navigate_to(goal, "Found a semantic match.")

    assert result == (
        "Found a semantic match. No navigable path was found; the robot did not start moving."
    )
    navigation.set_goal.assert_called_once_with(goal)


def test_navigate_to_position_builds_world_frame_goal(
    mocker: MockerFixture,
    navigation_container: NavigationSkillContainer,
) -> None:
    navigation = mocker.Mock()
    navigation.set_goal.return_value = True
    navigation.get_state.return_value = NavigationState.FOLLOWING_PATH
    navigation_container._navigation = navigation
    navigation_container._skill_started = True
    navigation_container._latest_odom = PoseStamped(position=[0.25, -2.5, 0.5])

    result = navigation_container.navigate_to_position(1.25, -2.5)

    goal = navigation.set_goal.call_args.args[0]
    assert goal.frame_id == "map"
    assert (goal.position.x, goal.position.y, goal.position.z) == (1.25, -2.5, 0.5)
    assert goal.yaw == pytest.approx(0.0)
    assert "Started navigating" in result


@pytest.mark.parametrize(
    ("state", "reached", "expected"),
    [
        (NavigationState.FOLLOWING_PATH, False, "FOLLOWING_PATH"),
        (NavigationState.IDLE, True, "was reached"),
        (NavigationState.IDLE, False, "without reaching"),
    ],
)
def test_navigation_status_reports_goal_outcome(
    mocker: MockerFixture,
    navigation_container: NavigationSkillContainer,
    state: NavigationState,
    reached: bool,
    expected: str,
) -> None:
    navigation = mocker.Mock()
    navigation.get_state.return_value = state
    navigation.is_goal_reached.return_value = reached
    navigation_container._navigation = navigation
    navigation_container._skill_started = True

    assert expected in navigation_container.navigation_status()
