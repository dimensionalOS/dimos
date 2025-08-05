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

from dimos.core import colors, rpc
from dimos.protocol.tool.comms import LCMToolComms, ToolCommsSpec
from dimos.protocol.tool.types import (
    AgentMsg,
    MsgType,
    Reducer,
    Return,
    Stream,
    ToolConfig,
)


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

        tool_config = ToolConfig(name=f.__name__, reducer=reducer, stream=stream, ret=ret)

        wrapper._tool = tool_config  # type: ignore[attr-defined]
        wrapper.__name__ = f.__name__  # Preserve original function name
        wrapper.__doc__ = f.__doc__  # Preserve original docstring
        return wrapper

    return decorator


class CommsSpec:
    agent_comms_class: type[ToolCommsSpec]


class LCMComms(CommsSpec):
    agent_comms_class: type[ToolCommsSpec] = LCMToolComms


# here we can have also dynamic tools potentially
# agent can check .tools each time when introspecting
class ToolContainer:
    comms: CommsSpec = LCMComms()
    _agent_comms: Optional[ToolCommsSpec] = None
    dynamic_tools = False

    def __str__(self) -> str:
        return f"ToolContainer({self.__class__.__name__})"

    @rpc
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
