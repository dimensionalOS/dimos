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
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.protocol.skill.testing_utils import TestContainer

# def test_coordinator_skill_export():
#     skillCoordinator = SkillCoordinator()
#     skillCoordinator.register_skills(TestContainer())

#     assert skillCoordinator.get_tools() == [
#         {
#             "function": {
#                 "description": "",
#                 "name": "add",
#                 "parameters": {
#                     "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
#                     "required": ["x", "y"],
#                     "type": "object",
#                 },
#             },
#             "type": "function",
#         },
#         {
#             "function": {
#                 "description": "",
#                 "name": "delayadd",
#                 "parameters": {
#                     "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
#                     "required": ["x", "y"],
#                     "type": "object",
#                 },
#             },
#             "type": "function",
#         },
#     ]

#     print(pprint(skillCoordinator.get_tools()))


class TestContainer2(SkillContainer):
    @skill()
    def add(self, x: int, y: int) -> int:
        #        time.sleep(0.25)
        return x + y

    @skill()
    def delayadd(self, x: int, y: int) -> int:
        time.sleep(0.5)
        return x + y


@pytest.mark.asyncio
async def test_coordinator_generator():
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(TestContainer())

    skillCoordinator.start()

    skillCoordinator.call("test-call-0", "delayadd", {"args": [1, 2]})

    time.sleep(0.1)

    cnt = 0
    while await skillCoordinator.wait_for_updates(1):
        print(skillCoordinator)

        skillstates = skillCoordinator.generate_snapshot()

        tool_msg = skillstates[f"test-call-{cnt}"].agent_encode()
        tool_msg.content == cnt + 1

        cnt += 1
        if cnt < 5:
            skillCoordinator.call(
                f"test-call-{cnt}-delay",
                "delayadd",
                {"args": [cnt, 2]},
            )
            skillCoordinator.call(
                f"test-call-{cnt}",
                "add",
                {"args": [cnt, 2]},
            )

        time.sleep(0.1 * cnt)

    print("All updates processed successfully.")
