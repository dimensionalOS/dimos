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

from pprint import pprint

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.testing_utils import TestContainer


def test_coordinator_skill_export():
    skillCoordinator = SkillCoordinator()
    skillCoordinator.register_skills(TestContainer())

    assert skillCoordinator.get_tools() == [
        {
            "function": {
                "description": "",
                "name": "add",
                "parameters": {
                    "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
                    "required": ["x", "y"],
                    "type": "object",
                },
            },
            "type": "function",
        },
        {
            "function": {
                "description": "",
                "name": "delayadd",
                "parameters": {
                    "properties": {"x": {"type": "integer"}, "y": {"type": "integer"}},
                    "required": ["x", "y"],
                    "type": "object",
                },
            },
            "type": "function",
        },
    ]

    print(pprint(skillCoordinator.get_tools()))
