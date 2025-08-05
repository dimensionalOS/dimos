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
from enum import Enum
from typing import Any, Callable, Generic, Optional, TypeVar

from dimos.protocol.tool.comms import AgentMsg, LCMToolComms, ToolCommsSpec


class Call(Enum):
    Implicit = "implicit"
    Explicit = "explicit"


class Reducer(Enum):
    latest = lambda data: data[-1] if data else None
    all = lambda data: data
    average = lambda data: sum(data) / len(data) if data else None


class Stream(Enum):
    none = "none"
    passive = "passive"
    call_agent = "call_agent"


class Return(Enum):
    none = "none"
    passive = "passive"
    call_agent = "call_agent"


def tool(reducer=Reducer.latest, stream=Stream.none, ret=Return.call_agent):
    def decorator(f: Callable[..., Any]) -> Any:
        def wrapper(self, *args, **kwargs):
            val = f(self, *args, **kwargs)
            tool = f"{self.__class__.__name__}.{f.__name__}"
            self.agent.publish(AgentMsg(tool, val))
            return val

        wrapper._tool = {reducer: reducer, stream: stream, ret: ret}
        return wrapper

    return decorator


class CommsSpec:
    agent: ToolCommsSpec


class LCMComms(CommsSpec):
    agent: ToolCommsSpec = LCMToolComms


class ToolContainer:
    comms: CommsSpec = LCMComms()
    _agent: ToolCommsSpec = None

    @property
    def tools(self):
        # Avoid recursion by excluding this property itself
        return {
            name: getattr(self, name)
            for name in dir(self)
            if not name.startswith("_")
            and name != "tools"
            and hasattr(getattr(self, name), "_tool")
        }

    @property
    def agent(self):
        if self._agent is None:
            self._agent = self.comms.agent()
        return self._agent
