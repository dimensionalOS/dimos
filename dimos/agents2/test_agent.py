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

import asyncio

import pytest

from dimos.agents2.agent import Agent
from dimos.protocol.skill.test_coordinator import TestContainer


@pytest.mark.asyncio
async def test_agent_init():
    system_prompt = (
        "Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"
    )

    ## Uncomment the following lines to use a real module system
    # from dimos.core import start
    # dimos = start(2)
    # testcontainer = dimos.deploy(TestContainer)
    # agent = dimos.deploy(Agent, system_prompt=system_prompt)

    testcontainer = TestContainer()
    agent = Agent(system_prompt=system_prompt)

    agent.register_skills(testcontainer)

    agent.start()

    print(
        agent.query_async(
            "hi there, please tell me what's your name and current date, and how much is 124181112 + 124124?"
        )
    )

    await asyncio.sleep(20)
    print(agent)
