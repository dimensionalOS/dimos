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

from dimos.agents.agent import Agent
from dimos.protocol.skill.test_coordinator import SkillContainerTest


@pytest.mark.tool
@pytest.mark.asyncio
async def test_agent_init() -> None:
    system_prompt = (
        "Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"
    )

    # # Uncomment the following lines to use a dimos module system
    # dimos = start(2)
    # testcontainer = dimos.deploy(SkillContainerTest)
    # agent = Agent(system_prompt=system_prompt)

    ## uncomment the following lines to run agents in a main loop without a module system
    testcontainer = SkillContainerTest()
    agent = Agent(system_prompt=system_prompt)

    agent.register_skills(testcontainer)
    agent.start()

    agent.run_implicit_skill("uptime_seconds")

    await agent.query_async(
        "hi there, please tell me what's your name and current date, and how much is 124181112 + 124124?"
    )

    # agent loop is considered finished once no active skills remain,
    # agent will stop it's loop if passive streams are active
    print("Agent loop finished, asking about camera")

    # we query again (this shows subsequent querying, but we could have asked for camera image in the original query,
    # it all runs in parallel, and agent might get called once or twice depending on timing of skill responses)
    # await agent.query_async("tell me what you see on the camera?")

    # you can run skillspy and agentspy in parallel with this test for a better observation of what's happening
    await agent.query_async("tell me exactly everything we've talked about until now")

    print("Agent loop finished")

    agent.stop()
    testcontainer.stop()
    dimos.stop()
