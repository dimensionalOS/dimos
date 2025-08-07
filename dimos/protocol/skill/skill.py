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
from typing import Any, Callable, Optional

from dimos.core import rpc
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.types import (
    AgentMsg,
    MsgType,
    Reducer,
    Return,
    SkillConfig,
    Stream,
)


def function_to_schema(func) -> dict:
    type_map = {
        str: "string",
        int: "integer",
        float: "number",
        bool: "boolean",
        list: "array",
        dict: "object",
        type(None): "null",
    }

    try:
        signature = inspect.signature(func)
    except ValueError as e:
        raise ValueError(f"Failed to get signature for function {func.__name__}: {str(e)}")

    parameters = {}
    for param in signature.parameters.values():
        try:
            param_type = type_map.get(param.annotation, "string")
        except KeyError as e:
            raise KeyError(
                f"Unknown type annotation {param.annotation} for parameter {param.name}: {str(e)}"
            )
        parameters[param.name] = {"type": param_type}

    required = [
        param.name for param in signature.parameters.values() if param.default == inspect._empty
    ]

    return {
        "type": "function",
        "function": {
            "name": func.__name__,
            "description": (func.__doc__ or "").strip(),
            "parameters": {
                "type": "object",
                "properties": parameters,
                "required": required,
            },
        },
    }


def skill(reducer=Reducer.latest, stream=Stream.none, ret=Return.call_agent):
    def decorator(f: Callable[..., Any]) -> Any:
        def wrapper(self, *args, **kwargs):
            skill = f"{f.__name__}"

            if kwargs.get("skillcall"):
                del kwargs["skillcall"]

                def run_function():
                    self.agent_comms.publish(AgentMsg(skill, None, type=MsgType.start))
                    try:
                        val = f(self, *args, **kwargs)
                        self.agent_comms.publish(AgentMsg(skill, val, type=MsgType.ret))
                    except Exception as e:
                        self.agent_comms.publish(AgentMsg(skill, str(e), type=MsgType.error))

                thread = threading.Thread(target=run_function)
                thread.start()
                return None

            return f(self, *args, **kwargs)

        skill_config = SkillConfig(
            name=f.__name__, reducer=reducer, stream=stream, ret=ret, schema=function_to_schema(f)
        )

        # implicit RPC call as well
        wrapper.__rpc__ = True  # type: ignore[attr-defined]
        wrapper._skill = skill_config  # type: ignore[attr-defined]
        wrapper.__name__ = f.__name__  # Preserve original function name
        wrapper.__doc__ = f.__doc__  # Preserve original docstring
        return wrapper

    return decorator


class CommsSpec:
    agent: type[SkillCommsSpec]


class LCMComms(CommsSpec):
    agent: type[SkillCommsSpec] = LCMSkillComms


# here we can have also dynamic skills potentially
# agent can check .skills each time when introspecting
class SkillContainer:
    comms: CommsSpec = LCMComms
    _agent_comms: Optional[SkillCommsSpec] = None
    dynamic_skills = False

    def __str__(self) -> str:
        return f"SkillContainer({self.__class__.__name__})"

    @rpc
    def skills(self) -> dict[str, SkillConfig]:
        # Avoid recursion by excluding this property itself
        return {
            name: getattr(self, name)._skill
            for name in dir(self)
            if not name.startswith("_")
            and name != "skills"
            and hasattr(getattr(self, name), "_skill")
        }

    @property
    def agent_comms(self) -> SkillCommsSpec:
        if self._agent_comms is None:
            self._agent_comms = self.comms.agent()
        return self._agent_comms
