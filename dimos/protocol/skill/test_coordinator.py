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
from typing import Generator, Optional

import pytest

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.protocol.skill.type import Reducer, Return, Stream


class TestContainer(SkillContainer):
    @skill()
    def add(self, x: int, y: int) -> int:
        return x + y

    @skill()
    def delayadd(self, x: int, y: int) -> int:
        time.sleep(0.3)
        return x + y

    @skill(stream=Stream.call_agent, reducer=Reducer.all)
    def counter(self, count_to: int, delay: Optional[float] = 0.1) -> Generator[int, None, None]:
        """Counts from 1 to count_to, with an optional delay between counts."""
        for i in range(1, count_to + 1):
            if delay > 0:
                time.sleep(delay)
            yield i

    @skill(stream=Stream.passive, reducer=Reducer.sum)
    def counter_passive(
        self, count_to: int, delay: Optional[float] = 0.1
    ) -> Generator[int, None, None]:
        """Counts from 1 to count_to, with an optional delay between counts."""
        for i in range(1, count_to + 1):
            if delay > 0:
                time.sleep(delay)
            yield i


@pytest.mark.asyncio
async def test_coordinator_parallel_calls():
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(TestContainer())

    skillCoordinator.start()
    skillCoordinator.call_skill("test-call-0", "delayadd", {"args": [1, 2]})

    time.sleep(0.1)

    cnt = 0
    while await skillCoordinator.wait_for_updates(1):
        print(skillCoordinator)

        skillstates = skillCoordinator.generate_snapshot()

        tool_msg = skillstates[f"test-call-{cnt}"].agent_encode()
        tool_msg.content == cnt + 1

        cnt += 1
        if cnt < 5:
            skillCoordinator.call_skill(
                f"test-call-{cnt}-delay",
                "delayadd",
                {"args": [cnt, 2]},
            )
            skillCoordinator.call_skill(
                f"test-call-{cnt}",
                "add",
                {"args": [cnt, 2]},
            )

        time.sleep(0.1 * cnt)


@pytest.mark.asyncio
async def test_coordinator_generator():
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(TestContainer())

    skillCoordinator.start()

    # here we call a skill that generates a sequence of messages
    skillCoordinator.call_skill("test-gen-0", "counter", {"args": [10]})

    skillstate = None
    # periodically agent is stopping it's thinking cycle and asks for updates
    while await skillCoordinator.wait_for_updates(1):
        skillstate = skillCoordinator.generate_snapshot(clear=True)

        # reducer is generating a summary
        print("Skill State:", skillstate)
        print("Agent update:", skillstate["test-gen-0"].agent_encode())
        # we simulate agent thinking
        await asyncio.sleep(0.25)

    print("Skill lifecycle finished")
