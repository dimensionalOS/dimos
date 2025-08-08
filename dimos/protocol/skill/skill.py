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
from typing import Any, Callable, Optional, get_origin, get_args, Union, List, Dict

from dimos.core import rpc
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.types import (
    SkillMsg,
    MsgType,
    Reducer,
    Return,
    SkillConfig,
    Stream,
)


def python_type_to_json_schema(python_type) -> dict:
    """Convert Python type annotations to JSON Schema format."""
    # Handle None/NoneType
    if python_type is type(None) or python_type is None:
        return {"type": "null"}

    # Handle Union types (including Optional)
    origin = get_origin(python_type)
    if origin is Union:
        args = get_args(python_type)
        # Handle Optional[T] which is Union[T, None]
        if len(args) == 2 and type(None) in args:
            non_none_type = args[0] if args[1] is type(None) else args[1]
            schema = python_type_to_json_schema(non_none_type)
            # For OpenAI function calling, we don't use anyOf for optional params
            return schema
        else:
            # For other Union types, use anyOf
            return {"anyOf": [python_type_to_json_schema(arg) for arg in args]}

    # Handle List/list types
    if origin in (list, List):
        args = get_args(python_type)
        if args:
            return {"type": "array", "items": python_type_to_json_schema(args[0])}
        return {"type": "array"}

    # Handle Dict/dict types
    if origin in (dict, Dict):
        return {"type": "object"}

    # Handle basic types
    type_map = {
        str: {"type": "string"},
        int: {"type": "integer"},
        float: {"type": "number"},
        bool: {"type": "boolean"},
        list: {"type": "array"},
        dict: {"type": "object"},
    }

    return type_map.get(python_type, {"type": "string"})


def function_to_schema(func) -> dict:
    """Convert a function to OpenAI function schema format."""
    try:
        signature = inspect.signature(func)
    except ValueError as e:
        raise ValueError(f"Failed to get signature for function {func.__name__}: {str(e)}")

    properties = {}
    required = []

    for param_name, param in signature.parameters.items():
        # Skip 'self' parameter for methods
        if param_name == "self":
            continue

        # Get the type annotation
        if param.annotation != inspect.Parameter.empty:
            param_schema = python_type_to_json_schema(param.annotation)
        else:
            # Default to string if no type annotation
            param_schema = {"type": "string"}

        # Add description from docstring if available (would need more sophisticated parsing)
        properties[param_name] = param_schema

        # Add to required list if no default value
        if param.default == inspect.Parameter.empty:
            required.append(param_name)

    return {
        "type": "function",
        "function": {
            "name": func.__name__,
            "description": (func.__doc__ or "").strip(),
            "parameters": {
                "type": "object",
                "properties": properties,
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
                call_id = kwargs.pop("call_id", "unknown")

                def run_function():
                    self.agent_comms.publish(SkillMsg(call_id, skill, None, type=MsgType.start))
                    try:
                        val = f(self, *args, **kwargs)
                        self.agent_comms.publish(SkillMsg(call_id, skill, val, type=MsgType.ret))
                    except Exception as e:
                        self.agent_comms.publish(
                            SkillMsg(call_id, skill, str(e), type=MsgType.error)
                        )

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
