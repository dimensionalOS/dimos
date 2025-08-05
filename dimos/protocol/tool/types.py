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
from enum import Enum
from typing import Optional

from dimos.types.timestamped import Timestamped


class Call(Enum):
    Implicit = 0
    Explicit = 1


class Reducer(Enum):
    all = lambda data: data
    latest = lambda data: data[-1] if data else None
    average = lambda data: sum(data) / len(data) if data else None


class Stream(Enum):
    # no streaming
    none = 0
    # passive stream, doesn't schedule an agent call, but returns the value to the agent
    passive = 1
    # calls the agent with every value emitted, schedules an agent call
    call_agent = 2


class Return(Enum):
    # doesn't return anything to an agent
    none = 0
    # returns the value to the agent, but doesn't schedule an agent call
    passive = 1
    # calls the agent with the value, scheduling an agent call
    call_agent = 2


class ToolConfig:
    def __init__(self, name: str, reducer: Reducer, stream: Stream, ret: Return):
        self.name = name
        self.reducer = reducer
        self.stream = stream
        self.ret = ret

    def __str__(self):
        parts = [f"name={self.name}"]

        # Only show reducer if stream is not none (streaming is happening)
        if self.stream != Stream.none:
            reducer_name = "unknown"
            if self.reducer == Reducer.latest:
                reducer_name = "latest"
            elif self.reducer == Reducer.all:
                reducer_name = "all"
            elif self.reducer == Reducer.average:
                reducer_name = "average"
            parts.append(f"reducer={reducer_name}")
            parts.append(f"stream={self.stream.name}")

        # Always show return mode
        parts.append(f"ret={self.ret.name}")
        return f"Tool({', '.join(parts)})"


class MsgType(Enum):
    start = 0
    stream = 1
    ret = 2
    error = 3


class AgentMsg(Timestamped):
    ts: float
    type: MsgType

    def __init__(
        self,
        tool_name: str,
        content: str | int | float | dict | list,
        type: Optional[MsgType] = MsgType.ret,
    ) -> None:
        self.ts = time.time()
        self.tool_name = tool_name
        self.content = content
        self.type = type

    def __repr__(self):
        return f"AgentMsg(tool={self.tool_name}, content={self.content}, type={self.type})"
