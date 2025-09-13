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

print("Starting human CLI...")
import json
import queue
from typing import Any, List, Optional, TypedDict

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall, ToolMessage

from dimos.agents2 import Output, Reducer, Stream, skill
from dimos.core import In, Module, Out, pLCMTransport, rpc
from dimos.protocol.pubsub.lcmpubsub import PickleLCM


def run_cli():
    human_transport = pLCMTransport("/human_input")
    agent_transport = pLCMTransport("/agent")

    def tool_call_logger(tool: ToolCall):
        f = tool.get("function")
        arguments = json.loads(f.get("arguments", "{}"))
        args = arguments.get("args", [])
        kwargs = arguments.get("kwargs", {})
        name = f.get("name")

        return f"<toolcall> {name}({args}, {kwargs})"

    def receive_msg(msg):
        if isinstance(msg, SystemMessage):
            print(f"<system> {msg.content}")
        if isinstance(msg, AIMessage):
            print(
                f"<agent> {msg.content}"
                + "  ".join(
                    list(map(tool_call_logger, msg.additional_kwargs.get("tool_calls", []))),
                )
            )
        if isinstance(msg, ToolMessage):
            print(f"<tool> {msg.content}")

    agent_transport.subscribe(receive_msg)

    while True:
        # read cli input
        text_line = input("> ")
        if text_line.lower() in ["exit", "quit"]:
            break
        human_transport.publish(None, text_line)


if __name__ == "__main__":
    run_cli()
