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

from dimos.agents2.main import Agent
from dimos.core import start
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
    # dimos = start(2)
    # agent = dimos.deploy(Agent)
    agent = Agent()
    agent.register_skills(TestContainer())
    agent.start()

    agent.query(
        "hi there, use add tool to add 124181112 and 124124. don't sum yourself, use a tool I provided"
    )

    await asyncio.sleep(5)
