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

from langchain_core.messages import HumanMessage
import pytest

from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image


class FakeCamera(Module):
    color_image: Out[Image]


class FakeOdom(Module):
    odom: Out[PoseStamped]


class MockedStopNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _cancel_goal_and_stop(self):
        pass


class MockedExploreNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _start_exploration(self, timeout):
        return "Exploration completed successfuly"

    def _cancel_goal_and_stop(self):
        pass


class MockedSemanticNavSkill(NavigationSkillContainer):
    rpc_calls: list[str] = []

    def __init__(self):
        Module.__init__(self)
        self._skill_started = True

    def _navigate_by_tagged_location(self, query):
        return None

    def _navigate_to_object(self, query):
        return None

    def _navigate_using_semantic_map(self, query):
        return f"Successfuly arrived at '{query}'"


@pytest.mark.integration
def test_stop_movement(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedStopNavSkill.blueprint(),
        ],
        messages=[HumanMessage("Stop moving. Use the stop_movement tool.")],
    )

    assert "stopped" in history[-1].content.lower()


@pytest.mark.integration
def test_start_exploration(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedExploreNavSkill.blueprint(),
        ],
        messages=[
            HumanMessage("Take a look around for 10 seconds. Use the start_exploration tool.")
        ],
    )

    assert "explor" in history[-1].content.lower()


@pytest.mark.integration
def test_go_to_semantic_location(agent_setup) -> None:
    history = agent_setup(
        blueprints=[
            FakeCamera.blueprint(),
            FakeOdom.blueprint(),
            MockedSemanticNavSkill.blueprint(),
        ],
        messages=[HumanMessage("Go to the bookshelf. Use the navigate_with_text tool.")],
    )

    assert "success" in history[-1].content.lower()
