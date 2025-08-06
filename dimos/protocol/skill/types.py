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
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Generic, Optional, TypeVar

from dimos.types.timestamped import Timestamped


class Call(Enum):
    Implicit = 0
    Explicit = 1


class Reducer(Enum):
    none = 0
    all = 1
    latest = 2
    average = 3


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


@dataclass
class SkillConfig:
    name: str
    reducer: Reducer
    stream: Stream
    ret: Return
    f: Callable | None = None
    autostart: bool = False

    def bind(self, f: Callable) -> "SkillConfig":
        self.f = f
        return self

    def call(self, *args, **kwargs) -> Any:
        if self.f is None:
            raise ValueError(
                "Function is not bound to the SkillConfig. This should be called only within AgentListener."
            )

        return self.f(*args, **kwargs, skillcall=True)

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
        return f"Skill({', '.join(parts)})"


class MsgType(Enum):
    pending = 0
    start = 1
    stream = 2
    ret = 3
    error = 4


class AgentMsg(Timestamped):
    ts: float
    type: MsgType

    def __init__(
        self,
        skill_name: str,
        content: str | int | float | dict | list,
        type: MsgType = MsgType.ret,
    ) -> None:
        self.ts = time.time()
        self.skill_name = skill_name
        self.content = content
        self.type = type

    def __repr__(self):
        return self.__str__()

    @property
    def end(self) -> bool:
        return self.type == MsgType.ret or self.type == MsgType.error

    @property
    def start(self) -> bool:
        return self.type == MsgType.start

    def __str__(self):
        time_ago = time.time() - self.ts

        if self.type == MsgType.start:
            return f"Start({time_ago:.1f}s ago)"
        if self.type == MsgType.ret:
            return f"Ret({time_ago:.1f}s ago, val={self.content})"
        if self.type == MsgType.error:
            return f"Error({time_ago:.1f}s ago, val={self.content})"
        if self.type == MsgType.pending:
            return f"Pending({time_ago:.1f}s ago)"
        if self.type == MsgType.stream:
            return f"Stream({time_ago:.1f}s ago, val={self.content})"
