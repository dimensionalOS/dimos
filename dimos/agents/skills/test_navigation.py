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

import threading
from typing import Any

from dimos_lcm.std_msgs import Bool
from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.go2.replanning_a_star.spec import NavigationInterfaceSpec
from dimos.perception.experimental.spatial_memory_spec import SpatialMemorySpec
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
def container(mocker):
    """A started container on a mocked planner, spatial memory and tool-stream
    helpers, with a tagged location 'bed' 1 m from the robot."""
    module = NavigationSkillContainer()
    try:
        module._skill_started = True
        mocker.patch.object(
            module, "_navigation", mocker.Mock(spec=NavigationInterfaceSpec), create=True
        )
        mocker.patch.object(
            module, "_spatial_memory", mocker.Mock(spec=SpatialMemorySpec), create=True
        )
        module._spatial_memory.query_tagged_location.return_value = RobotLocation(
            name="bed", position=(1.0, 2.0, 0.0), rotation=(0.0, 0.0, 0.0)
        )
        module._latest_odom = PoseStamped(position=make_vector3(0.0, 2.0, 0.0))
        for name in ("start_tool", "tool_update", "stop_tool"):
            mocker.patch.object(module, name)
        yield module
    finally:
        module.stop()


def test_navigate_with_text_keeps_the_tool_open_while_the_goal_is_under_way(container) -> None:
    message = container.navigate_with_text("bed")

    assert "A tool update reports when the robot stops" in message
    goal = container._navigation.set_goal.call_args.args[0]
    assert (goal.position.x, goal.position.y) == (1.0, 2.0)
    container.start_tool.assert_called_once_with("navigate_with_text")
    container.stop_tool.assert_not_called()


def test_goal_reached_reports_where_the_robot_stopped_and_closes_the_tool(container) -> None:
    container.navigate_with_text("bed")

    container._on_goal_reached(Bool(False))

    container.tool_update.assert_called_once_with(
        "navigate_with_text",
        "Navigation to 'bed' stopped short of the target: robot at (0.00, 2.00), 1.00 m from it.",
    )
    container.stop_tool.assert_called_once_with("navigate_with_text")


def test_arrival_is_reported(container) -> None:
    container.navigate_with_text("bed")
    container._latest_odom = PoseStamped(position=make_vector3(1.0, 2.0, 0.0))

    container._on_goal_reached(Bool(True))

    container.tool_update.assert_called_once_with(
        "navigate_with_text", "Arrived at 'bed': robot at (1.00, 2.00), 0.00 m from it."
    )
    container.stop_tool.assert_called_once_with("navigate_with_text")


def test_goal_reached_is_reported_once(container) -> None:
    container.navigate_with_text("bed")

    container._on_goal_reached(Bool(False))
    container._on_goal_reached(Bool(True))

    container.tool_update.assert_called_once()
    container.stop_tool.assert_called_once()


def test_goal_reached_without_a_navigation_under_way_is_ignored(container) -> None:
    container._on_goal_reached(Bool(True))

    container.tool_update.assert_not_called()
    container.stop_tool.assert_not_called()


def test_a_planner_giving_up_inside_set_goal_still_ends_the_navigation(container) -> None:
    # The planner finds no path inside set_goal and publishes goal_reached before
    # the call returns. The frame arrives on the transport's thread.
    frame = threading.Thread(target=container._on_goal_reached, args=(Bool(False),))
    container._navigation.set_goal.side_effect = lambda _goal: frame.start()

    container.navigate_with_text("bed")
    frame.join()

    container.tool_update.assert_called_once()
    assert "stopped short" in container.tool_update.call_args.args[1]
    # The call's own stop_tool can run as well, a no-op on the closed stream.
    container.stop_tool.assert_called_with("navigate_with_text")
    assert container._active is None


def test_a_second_call_while_a_goal_is_under_way_is_refused(container) -> None:
    container.navigate_with_text("bed")

    message = container.navigate_with_text("kitchen")

    assert message == "Already navigating to 'bed'. Call stop_navigation first."
    container._navigation.set_goal.assert_called_once()
    # The open stream is re-stamped with the second call's movement hold.
    assert container.start_tool.call_count == 2
    container.stop_tool.assert_not_called()

    container._on_goal_reached(Bool(True))

    assert "Arrived at 'bed'" in container.tool_update.call_args.args[1]
    container.stop_tool.assert_called_once_with("navigate_with_text")


def test_an_exception_during_the_lookup_closes_the_tool(container) -> None:
    container._spatial_memory.query_tagged_location.side_effect = RuntimeError("rpc failed")

    with pytest.raises(RuntimeError):
        container.navigate_with_text("bed")

    container.stop_tool.assert_called_once_with("navigate_with_text")
    # The next call is not refused.
    container._spatial_memory.query_tagged_location.side_effect = None
    assert "Started navigating" in container.navigate_with_text("bed")


def test_an_exception_from_set_goal_closes_the_tool(container) -> None:
    container._navigation.set_goal.side_effect = RuntimeError("planner down")

    with pytest.raises(RuntimeError):
        container.navigate_with_text("bed")

    container.stop_tool.assert_called_once_with("navigate_with_text")
    assert container._active is None
    container._on_goal_reached(Bool(False))
    container.tool_update.assert_not_called()


def test_stop_navigation_during_a_lookup_leaves_the_stream_to_that_call(container) -> None:
    # stop_navigation and navigate_with_text sent in one agent turn run in parallel.
    # Here the stop lands while the lookup is under way.
    def stop_meanwhile(_query):
        container.stop_navigation()
        return RobotLocation(name="bed", position=(1.0, 2.0, 0.0), rotation=(0.0, 0.0, 0.0))

    container._spatial_memory.query_tagged_location.side_effect = stop_meanwhile

    message = container.navigate_with_text("bed")

    assert "Started navigating" in message
    container._navigation.cancel_goal.assert_called_once_with()
    container._navigation.set_goal.assert_called_once()
    # The stop left the stream to the call, and the goal it sent keeps it open.
    container.stop_tool.assert_not_called()

    container._on_goal_reached(Bool(True))

    container.stop_tool.assert_called_once_with("navigate_with_text")


def test_no_match_releases_the_tool_at_once(container) -> None:
    container._spatial_memory.query_tagged_location.return_value = None
    container._spatial_memory.query_by_text.return_value = []

    message = container.navigate_with_text("sofa")

    assert "No matching location" in message
    container._navigation.set_goal.assert_not_called()
    container.stop_tool.assert_called_once_with("navigate_with_text")


def test_stop_navigation_cancels_the_goal_and_closes_the_tool(container) -> None:
    container.navigate_with_text("bed")

    assert container.stop_navigation() == "Stopped"

    container._navigation.cancel_goal.assert_called_once_with()
    container.stop_tool.assert_called_once_with("navigate_with_text")
    # The cancel's own goal_reached frame is not reported as a stop short of the target.
    container._on_goal_reached(Bool(False))
    container.tool_update.assert_not_called()


def test_stop_navigation_with_nothing_under_way(container) -> None:
    assert container.stop_navigation() == "Stopped"

    container._navigation.cancel_goal.assert_called_once_with()
    container.stop_tool.assert_called_once_with("navigate_with_text")
