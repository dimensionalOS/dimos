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
from dimos.protocol.tool.types import Return, Stream


class TestContainer(ToolContainer):
    @tool()
    def add(self, x: int, y: int) -> int:
        return x + y

    @tool()
    def delayadd(self, x: int, y: int) -> int:
        time.sleep(0.5)
        return x + y


def test_introspect_tool():
    testContainer = TestContainer()
    print(testContainer.tools)


def test_internals():
    agentInput = AgentInput()
    agentInput.start()

    testContainer = TestContainer()

    agentInput.register_tools(testContainer)

    # toolcall=True makes the tool function exit early,
    # it doesn't behave like a blocking function,
    #
    # return is passed as AgentMsg to the agent topic
    testContainer.delayadd(2, 4, toolcall=True)
    testContainer.add(1, 2, toolcall=True)

    time.sleep(0.25)
    print(agentInput)

    time.sleep(0.75)
    print(agentInput)

    print(agentInput.state_snapshot())

    print(agentInput.tools())

    print(agentInput)

    agentInput.execute_tool("delayadd", 1, 2)

    time.sleep(0.25)
    print(agentInput)
    time.sleep(0.75)

    print(agentInput)


def test_standard_usage():
    agentInput = AgentInput(agent_callback=print)
    agentInput.start()

    testContainer = TestContainer()

    agentInput.register_tools(testContainer)

    # we can investigate tools
    print(agentInput.tools())

    # we can execute a tool
    agentInput.execute_tool("delayadd", 1, 2)

    # while tool is executing, we can introspect the state
    # (we see that the tool is running)
    time.sleep(0.25)
    print(agentInput)
    time.sleep(0.75)

    # after the tool has finished, we can see the result
    # and the tool state
    print(agentInput)
