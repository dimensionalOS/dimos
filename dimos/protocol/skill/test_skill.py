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

import time

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.skill import SkillContainer, skill
from dimos.protocol.skill.testing_utils import TestContainer


def test_introspect_skill():
    testContainer = TestContainer()
    print(testContainer.skills())


def test_internals():
    agentInterface = SkillCoordinator()
    agentInterface.start()

    testContainer = TestContainer()

    agentInterface.register_skills(testContainer)

    # skillcall=True makes the skill function exit early,
    # it doesn't behave like a blocking function,
    #
    # return is passed as SkillMsg to the agent topic
    testContainer.delayadd(2, 4, skillcall=True)
    testContainer.add(1, 2, skillcall=True)

    time.sleep(0.25)
    print(agentInterface)

    time.sleep(0.75)
    print(agentInterface)

    print(agentInterface.state_snapshot())

    print(agentInterface.skills())

    print(agentInterface)

    agentInterface.call("test-call-1", "delayadd", 1, 2)

    time.sleep(0.25)
    print(agentInterface)
    time.sleep(0.75)

    print(agentInterface)


def test_standard_usage():
    agentInterface = SkillCoordinator()
    agentInterface.start()

    testContainer = TestContainer()

    agentInterface.register_skills(testContainer)

    # we can investigate skills
    print(agentInterface.skills())

    # we can execute a skill
    agentInterface.call("test-call-2", "delayadd", 1, 2)

    # while skill is executing, we can introspect the state
    # (we see that the skill is running)
    time.sleep(0.25)
    print(agentInterface)
    time.sleep(0.75)

    # after the skill has finished, we can see the result
    # and the skill state
    print(agentInterface)


def test_module():
    from dimos.core import Module, start

    class MockModule(Module, SkillContainer):
        def __init__(self):
            super().__init__()
            SkillContainer.__init__(self)

        @skill()
        def add(self, x: int, y: int) -> int:
            time.sleep(0.5)
            return x * y

    agentInterface = SkillCoordinator()
    agentInterface.start()

    dimos = start(1)
    mock_module = dimos.deploy(MockModule)

    agentInterface.register_skills(mock_module)

    # we can execute a skill
    agentInterface.call("test-call-3", "add", 1, 2)

    # while skill is executing, we can introspect the state
    # (we see that the skill is running)
    time.sleep(0.25)
    print(agentInterface)
    time.sleep(0.75)

    # after the skill has finished, we can see the result
    # and the skill state
    print(agentInterface)
