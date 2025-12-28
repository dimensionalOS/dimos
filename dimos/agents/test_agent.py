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

import pytest
import pytest_asyncio

from dimos.agents.agent import Agent
from dimos.core import start
from dimos.protocol.skill.test_coordinator import SkillContainerTest

system_prompt = (
    "Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"
)


@pytest.fixture(scope="session")
def dimos_cluster():
    """Session-scoped fixture to initialize dimos cluster once."""
    dimos = start(2)
    try:
        yield dimos
    finally:
        dimos.shutdown()


@pytest_asyncio.fixture
async def local():
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
        try:
            agent.stop()
        except Exception:
            pass
        try:
            testcontainer.stop()
        except Exception:
            pass


@pytest_asyncio.fixture
async def dask_mixed(dimos_cluster):
    """Dask context: testcontainer on dimos, agent local"""
    testcontainer = dimos_cluster.deploy(SkillContainerTest)
    agent = Agent(system_prompt=system_prompt)
    try:
        yield agent, testcontainer
    finally:
        try:
            agent.stop()
        except Exception:
            pass
        try:
            testcontainer.stop()
        except Exception:
            pass


@pytest_asyncio.fixture
async def dask_full(dimos_cluster):
    """Dask context: both agent and testcontainer deployed on dimos"""
    testcontainer = dimos_cluster.deploy(SkillContainerTest)
    agent = dimos_cluster.deploy(Agent, system_prompt=system_prompt)
    try:
        yield agent, testcontainer
    finally:
        try:
            agent.stop()
        except Exception:
            pass
        try:
            testcontainer.stop()
        except Exception:
            pass


@pytest_asyncio.fixture(params=["local", "dask_mixed", "dask_full"])
async def agent_context(request):
    """Parametrized fixture that runs tests with different agent configurations"""
    param = request.param

    if param == "local":
        testcontainer = SkillContainerTest()
        agent = Agent(system_prompt=system_prompt)
        try:
            yield agent, testcontainer
        finally:
            try:
                agent.stop()
            except Exception:
                pass
            try:
                testcontainer.stop()
            except Exception:
                pass
    elif param == "dask_mixed":
        dimos_cluster = request.getfixturevalue("dimos_cluster")
        testcontainer = dimos_cluster.deploy(SkillContainerTest)
        agent = Agent(system_prompt=system_prompt)
        try:
            yield agent, testcontainer
        finally:
            try:
                agent.stop()
            except Exception:
                pass
            try:
                testcontainer.stop()
            except Exception:
                pass
    elif param == "dask_full":
        dimos_cluster = request.getfixturevalue("dimos_cluster")
        testcontainer = dimos_cluster.deploy(SkillContainerTest)
        agent = dimos_cluster.deploy(Agent, system_prompt=system_prompt)
        try:
            yield agent, testcontainer
        finally:
            try:
                agent.stop()
            except Exception:
                pass
            try:
                testcontainer.stop()
            except Exception:
                pass


# @pytest.mark.timeout(40)
@pytest.mark.tool
@pytest.mark.asyncio
async def test_agent_init(agent_context) -> None:
    """Test agent initialization and basic functionality across different configurations"""
    agent, testcontainer = agent_context

    agent.register_skills(testcontainer)
    agent.start()

    # agent.run_implicit_skill("uptime_seconds")

    print("query agent")
    # When running locally, call the async method directly
    agent.query(
        "hi there, please tell me what's your name and current date, and how much is 124181112 + 124124?"
    )
    print("Agent loop finished, asking about camera")
    agent.query("tell me what you see on the camera?")

    # you can run skillspy and agentspy in parallel with this test for a better observation of what's happening
