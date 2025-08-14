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

import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional

from dimos.core import rpc
from dimos.protocol.service import Configurable
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.schema import function_to_schema
from dimos.protocol.skill.type import (
    MsgType,
    Reducer,
    Return,
    SkillConfig,
    SkillMsg,
    Stream,
)

# skill is a decorator that allows us to specify a skill behaviour for a function.
#
# there are several parameters that can be specified:
# - ret: how to return the value from the skill, can be one of:
#
#        Return.none: doesn't return anything to an agent
#        Return.passive: doesn't schedule an agent call but
#                        returns the value to the agent when agent is called
#        Return.call_agent: calls the agent with the value, scheduling an agent call
#
# - stream: if the skill streams values, it can behave in several ways:
#
#        Stream.none: no streaming, skill doesn't emit any values
#        Stream.passive: doesn't schedule an agent call upon emitting a value,
#                        returns the streamed value to the agent when agent is called
#        Stream.call_agent: calls the agent with every value emitted, scheduling an agent call
#
# - reducer: defines an optional strategy for passive streams and how we collapse potential
#            multiple values into something meaningful for the agent
#
#        Reducer.none: no reduction, every emitted value is returned to the agent
#        Reducer.latest: only the latest value is returned to the agent
#        Reducer.average: assumes the skill emits a number,
#                         the average of all values is returned to the agent


def skill(reducer=Reducer.latest, stream=Stream.none, ret=Return.call_agent):
    def decorator(f: Callable[..., Any]) -> Any:
        def wrapper(self, *args, **kwargs):
            skill = f"{f.__name__}"

            call_id = kwargs.get("call_id", None)
            if call_id:
                del kwargs["call_id"]

                def run_function():
                    return self.call_skill(call_id, skill, args, kwargs)

                thread = threading.Thread(target=run_function)
                thread.start()
                return None

            return f(self, *args, **kwargs)

        # sig = inspect.signature(f)
        # params = list(sig.parameters.values())
        # if params and params[0].name == "self":
        #     params = params[1:]  # Remove first parameter 'self'

        # wrapper.__signature__ = sig.replace(parameters=params)

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


@dataclass
class SkillContainerConfig:
    skill_transport: type[SkillCommsSpec] = LCMSkillComms


# Inherited by any class that wants to provide skills
# (This component works standalone but commonly used by DimOS modules)
#
# - It allows us to specify a communication layer for skills (LCM for now by default)
# - introspection of available skills via the `skills` RPC method
# - ability to provide dynamic context dependant skills with dynamic_skills flag
#   for this you'll need to override the `skills` method to return a dynamic set of skills
#   SkillCoordinator will call this method to get the skills available upon every request to
#   the agent
#
#
# Hosts the function execution and handles correct publishing of skill messages
# according to the skill decorator configuration
class SkillContainer(Configurable[SkillContainerConfig]):
    default_config = SkillContainerConfig
    _skill_transport: Optional[SkillCommsSpec] = None

    dynamic_skills = False

    def __str__(self) -> str:
        return f"SkillContainer({self.__class__.__name__})"

    def call_skill(
        self, call_id: str, skill_name: str, args: tuple[Any, ...], kwargs: dict[str, Any]
    ) -> None:
        f = getattr(self, skill_name, None)

        if f is None:
            raise ValueError(f"Skill '{skill_name}' not found in {self.__class__.__name__}")

        self.skill_transport.publish(SkillMsg(call_id, skill, None, type=MsgType.start))
        try:
            val = f(*args, **kwargs)
            self.skill_transport.publish(SkillMsg(call_id, skill, val, type=MsgType.ret))
        except Exception as e:
            import traceback

            formatted_traceback = "".join(traceback.TracebackException.from_exception(e).format())

            self.skill_transport.publish(
                SkillMsg(
                    call_id,
                    skill,
                    {"msg": str(e), "traceback": formatted_traceback},
                    type=MsgType.error,
                )
            )

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
    def skill_transport(self) -> SkillCommsSpec:
        if self._skill_transport is None:
            self._skill_transport = self.config.skill_transport()
        return self._skill_transport
