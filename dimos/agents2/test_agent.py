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
import time

import pytest

from dimos.agents2.agent import Agent
from dimos.protocol.skill import SkillContainer, skill


class TestContainer(SkillContainer):
    @skill()
    def add(self, x: int, y: int) -> int:
        """Adds two integers."""
        time.sleep(0.3)
        return x + y

    @skill()
    def sub(self, x: int, y: int) -> int:
        """Subs two integers."""
        time.sleep(0.3)
        return x - y


@pytest.mark.asyncio
async def test_agent_init():
    from dimos.core import start

    dimos = start(2)
    agent = dimos.deploy(
        Agent,
        system_prompt="Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate",
    )
    # agent = Agent(
    #    system_prompt="Your name is Mr. Potato, potatoes are bad at math. Use a tools if asked to calculate"
    # )
    agent.register_skills(TestContainer())
    agent.start()

    print(
        agent.query_async(
            "hi there, please tell me what's your name, and use add tool to add 124181112 and 124124."
        )
    )

    await asyncio.sleep(10)
