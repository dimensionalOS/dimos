# Copyright 2025 Dimensional Inc.
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

from pathlib import Path

import pytest

from dimos.agents.agent import Agent
from dimos.agents.testing import MockModel
from dimos.protocol.skill.test_coordinator import SkillContainerTest


@pytest.fixture
def fixture_dir():
    return Path(__file__).parent / "fixtures"


@pytest.fixture
def potato_system_prompt() -> str:
    return "Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"


@pytest.fixture
def skill_container():
    container = SkillContainerTest()
    try:
        yield container
    finally:
        container.stop()


@pytest.fixture
def create_fake_agent(fixture_dir):
    agent = None

    def _agent_factory(*, system_prompt, skill_containers, fixture):
        mock_model = MockModel(json_path=fixture_dir / fixture)

        nonlocal agent
        agent = Agent(system_prompt=system_prompt, model_instance=mock_model)

        for skill_container in skill_containers:
            agent.register_skills(skill_container)

        agent.start()

        return agent

    try:
        yield _agent_factory
    finally:
        if agent:
            agent.stop()


@pytest.fixture
def create_potato_agent(potato_system_prompt, skill_container, fixture_dir):
    agent = None

    def _agent_factory(*, fixture):
        mock_model = MockModel(json_path=fixture_dir / fixture)

        nonlocal agent
        agent = Agent(system_prompt=potato_system_prompt, model_instance=mock_model)
        agent.register_skills(skill_container)
        agent.start()

        return agent

    try:
        yield _agent_factory
    finally:
        if agent:
            agent.stop()
