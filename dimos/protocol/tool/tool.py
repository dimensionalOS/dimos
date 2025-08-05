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

import inspect
import threading
from enum import Enum
from typing import Any, Callable, Generic, Optional, TypedDict, TypeVar, cast

from dimos.core import colors
from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, MsgType, ToolCommsSpec


class Call(Enum):
    Implicit = 0
    Explicit = 1


class Reducer(Enum):
    latest = lambda data: data[-1] if data else None
    all = lambda data: data
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
        parts = [f"name={colors.yellow(self.name)}"]

        # Only show reducer if stream is not none (streaming is happening)
        if self.stream != Stream.none:
            reducer_name = "unknown"
            if self.reducer == Reducer.latest:
                reducer_name = "latest"
            elif self.reducer == Reducer.all:
                reducer_name = "all"
            elif self.reducer == Reducer.average:
                reducer_name = "average"
            parts.append(f"reducer={colors.green(reducer_name)}")
            parts.append(f"stream={colors.red(self.stream.name)}")

        # Always show return mode
        parts.append(f"ret={colors.blue(self.ret.name)}")

        return f"Tool({', '.join(parts)})"


def tool(reducer=Reducer.latest, stream=Stream.none, ret=Return.call_agent):
    def decorator(f: Callable[..., Any]) -> Any:
        def wrapper(self, *args, **kwargs):
            tool = f"{f.__name__}"

            def run_function():
                self.agent_comms.publish(AgentMsg(tool, None, type=MsgType.start))
                val = f(self, *args, **kwargs)
                self.agent_comms.publish(AgentMsg(tool, val, type=MsgType.ret))

            if kwargs.get("toolcall"):
                del kwargs["toolcall"]
                thread = threading.Thread(target=run_function)
                thread.start()
                return None

            return run_function()

        wrapper._tool = ToolConfig(name=f.__name__, reducer=reducer, stream=stream, ret=ret)  # type: ignore[attr-defined]
        wrapper.__name__ = f.__name__  # Preserve original function name
        wrapper.__doc__ = f.__doc__  # Preserve original docstring
        return wrapper

    return decorator


class CommsSpec:
    agent_comms_class: type[ToolCommsSpec]


class LCMComms(CommsSpec):
    agent_comms_class: type[ToolCommsSpec] = LCMToolComms


class ToolContainer:
    comms: CommsSpec = LCMComms()
    _agent_comms: Optional[ToolCommsSpec] = None

    @property
    def tools(self) -> dict[str, ToolConfig]:
        # Avoid recursion by excluding this property itself
        return {
            name: getattr(self, name)._tool
            for name in dir(self)
            if not name.startswith("_")
            and name != "tools"
            and hasattr(getattr(self, name), "_tool")
        }

    @property
    def agent_comms(self) -> ToolCommsSpec:
        if self._agent_comms is None:
            self._agent_comms = self.comms.agent_comms_class()
        return self._agent_comms
