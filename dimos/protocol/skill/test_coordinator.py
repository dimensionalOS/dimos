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
from pprint import pprint

import pytest

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.testing_utils import TestContainer


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
    skillCoordinator.call_skill("test-call-0", "counter", {"args": [10]})

    skillstate = None
    while await skillCoordinator.wait_for_updates(1):
        skillstate = skillCoordinator.generate_snapshot(clear=True)
        print("Skill State:", skillstate)
        print("Agent update:", skillstate["test-call-0"].agent_encode())
        # we simulate agent thinking
        await asyncio.sleep(0.25)

    print("Skill lifecycle finished")
    print(
        "All messages:"
        + "".join(
            map(lambda x: f"\n  {x}", skillstate["test-call-0"].messages),
        ),
    )
