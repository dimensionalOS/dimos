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

from dimos.protocol.tool.agent_listener import AgentInput
from dimos.protocol.tool.tool import ToolContainer, tool


class TestContainer(ToolContainer):
    @tool()
    def add(self, x: int, y: int) -> int:
        return x + y

    @tool()
    def delayadd(self, x: int, y: int) -> int:
        time.sleep(1)
        return x + y


def test_introspect_tool():
    testContainer = TestContainer()
    print(testContainer.tools)


def test_comms():
    agentInput = AgentInput()
    agentInput.start()

    testContainer = TestContainer()

    agentInput.register_tools(testContainer)

    print(testContainer.delayadd(2, 4, toolcall=True))
    print(testContainer.add(1, 2))

    time.sleep(1.3)
