#!/usr/bin/env python3

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

from contextlib import contextmanager

from dimos.agents.agent import Agent
from dimos.core import start
from dimos.protocol.skill.test_coordinator import SkillContainerTest

system_prompt = (
    "Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"
)


@contextmanager
def dimos_cluster():
    dimos = start(2)
    try:
        yield dimos
    finally:
        dimos.close_all()


@contextmanager
def local():
    """Local context: both agent and testcontainer run locally"""
    testcontainer = SkillContainerTest()
    agent = Agent(system_prompt=system_prompt)
    try:
        yield agent, testcontainer
    except Exception as e:
        print(f"Error: {e}")
        import traceback

        traceback.print_exc()
        raise e
    finally:
        # Ensure cleanup happens while event loop is still active
        agent.stop()
        testcontainer.stop()


@contextmanager
def partial():
    """Dask context: testcontainer on dimos, agent local"""
    with dimos_cluster() as dimos:
        testcontainer = dimos.deploy(SkillContainerTest)
        agent = Agent(system_prompt=system_prompt)
        try:
            yield agent, testcontainer
        finally:
            agent.stop()
            testcontainer.stop()


@contextmanager
def full():
    """Dask context: both agent and testcontainer deployed on dimos"""
    with dimos_cluster() as dimos:
        testcontainer = dimos.deploy(SkillContainerTest)
        agent = dimos.deploy(Agent, system_prompt=system_prompt)
        try:
            yield agent, testcontainer
        finally:
            agent.stop()
            testcontainer.stop()


def check_agent(agent_context) -> None:
    """Test agent initialization and basic functionality across different configurations"""
    with agent_context() as [agent, testcontainer]:
        agent.register_skills(testcontainer)
        agent.start()

        print("query agent")

        agent.query(
            "hi there, please tell me what's your name and current date, and how much is 124181112 + 124124?"
        )

        print("Agent loop finished, asking about camera")

        agent.query("tell me what you see on the camera?")

        print("=" * 150)
        print("End of test", agent.get_agent_id())
        print("=" * 150)

        # you can run skillspy and agentspy in parallel with this test for a better observation of what's happening


if __name__ == "__main__":
    list(map(check_agent, [local, partial, full]))
