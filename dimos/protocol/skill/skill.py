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

"""Decorator and runtime for exposing robot capabilities as LLM tool calls.

This module provides the core abstractions for defining and executing *skills*: wrappers around
robot capabilities that allow LLMs to invoke them as tool calls.
Skills transform high-level intentions (e.g., "navigate to the kitchen") into concrete actions.

Core components
---------------
`@skill` decorator
    Transforms `Module` methods into agent-callable tools with configurable execution
    semantics (streaming, passive/active behavior, thread pooling). Auto-generates JSON
    schemas for LLM tool calling.

`SkillContainer` class
    Base infrastructure inherited by all Modules, providing skill execution, threading,
    and transport. Available automatically via `Module` inheritance.

`SkillContainerConfig` dataclass
    Configuration for skill transport layer (defaults to LCM-based communication).

Architecture
------------
Skills execute in a distributed, asynchronous environment where every `Module` inherits
from `SkillContainer`. The `@skill` decorator wraps methods with execution routing and
schema generation.

See also
--------
`dimos.core.module.Module` : Base class that inherits `SkillContainer`
`dimos.protocol.skill.coordinator.SkillCoordinator` : Manages skill execution state for agents
`dimos.protocol.skill.type` : Enums and types for skill configuration
`dimos.agents2.agent` : LLM agents that discover and invoke skills

Related docs
------------
- Build your first skill tutorial: `docs/tutorials/skill_basics/tutorial.md`
- Wire a skill to an agent tutorial: `docs/tutorials/skill_with_agent/tutorial.md`
- Explainer on the Skill concept: `docs/concepts/skills.md`

Examples
--------
Basic skill returning a result:

>>> from dimos.core.module import Module
>>> from dimos.protocol.skill.skill import skill
>>>
>>> class RobotSkills(Module):
...     @skill()
...     def greet(self, name: str) -> str:
...         '''Greet someone by name.'''
...         return f"Hello, {name}!"

Notes
-----
**Thread pool execution:**
Skills execute in a `ThreadPoolExecutor` when called via coordinator (with `call_id`),
preventing blocking during long-running operations.

**Passive skill requirements:**
Skills with `stream=Stream.passive` never wake the agent except on errors. Their data
is only delivered when an active skill keeps the agent loop running; by design,
passive skills are for auxiliary data like telemetry during primary tasks.
"""

import asyncio
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Annotated, Any

from annotated_doc import Doc

# from dimos.core.core import rpc
from dimos.protocol.skill.comms import LCMSkillComms, SkillCommsSpec
from dimos.protocol.skill.schema import function_to_schema
from dimos.protocol.skill.type import (
    MsgType,
    Output,
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


def rpc(fn: Callable[..., Any]) -> Callable[..., Any]:
    fn.__rpc__ = True  # type: ignore[attr-defined]
    return fn


def skill(
    reducer: Annotated[
        Reducer,
        Doc(
            """Aggregation strategy for *passive* streams when multiple values are emitted.

            Only relevant when `stream=Stream.passive`.
            """
        ),
    ] = Reducer.latest,
    stream: Annotated[
        Stream,
        Doc(
            """
            Controls how generator/iterator returns are handled.

            Use `Stream.none` for non-streaming skills, `Stream.passive` for streaming without
            triggering agent calls (values aggregated by reducer), or `Stream.call_agent` to
            trigger an agent call for each yielded value.
            """
        ),
    ] = Stream.none,
    ret: Annotated[
        Return,
        Doc(
            """
            Controls how the final return value is delivered to the agent.

            Use `Return.none` to suppress return value, `Return.passive` to make value available
            when agent queries, or `Return.call_agent` to actively schedule an agent call with
            the result.

            Note: forced to `Return.passive` when `stream=Stream.passive` to maintain
            consistent passive behavior.
            """
        ),
    ] = Return.call_agent,
    output: Annotated[
        Output,
        Doc(
            """Presentation hint for how the agent should interpret the output.

            Use `Output.standard` for normal text, `Output.human` for human-readable formatted
            output, or `Output.image` for visual content.
            """
        ),
    ] = Output.standard,
    hide_skill: Annotated[
        bool,
        Doc(
            """If True, prevents the skill from appearing in the agent's available skills list.

            Hidden skills can still be called programmatically but won't be offered to LLMs during
            tool selection. Useful for internal or administrative skills.
            """
        ),
    ] = False,
) -> Callable:
    """Decorator that transforms `Module` methods into agent-callable skills.

    The `@skill` decorator is what allows methods on a `Module` to be invoked as tool calls by LLM agents.
    It does this by wrapping methods with execution routing, message protocol handling,
    and automatic schema generation.

    When an agent calls a skill through the SkillCoordinator, the skill executes in
    a background thread pool and publishes messages tracking its execution state
    (start → [stream]* → ret/error). This enables non-blocking execution, progress
    monitoring, and distributed deployment across machines.

    Examples:

        >>> from dimos.core.module import Module
        >>> from dimos.protocol.skill.type import Stream, Reducer, Return

        Basic skill returning a string result:

        >>> class NavigationSkills(Module):
        ...     def __init__(self):
        ...         super().__init__()
        ...         self.goal = None
        ...
        ...     def _set_goal(self, location: str) -> None:
        ...         self.goal = location
        ...
        ...     @skill()
        ...     def navigate_to(self, location: str) -> str:
        ...         '''Navigate to a named location.'''
        ...         self._set_goal(location)
        ...         return f"Navigating to {location}"

        Streaming skill with progress updates:

        >>> class MonitorSkills(Module):
        ...     @skill(stream=Stream.call_agent, ret=Return.call_agent)
        ...     def monitor_task(self, count: int) -> str:
        ...         '''Monitor a long-running operation.'''
        ...         for i in range(count):
        ...             yield f"Progress: {i+1}/{count}"
        ...         return "Task completed"

        Passive skill with reducer aggregation:

        >>> class RobotSkills(Module):
        ...     def _get_frames(self):
        ...         for i in range(5):
        ...             yield f"frame_{i}"
        ...
        ...     @skill(stream=Stream.passive, reducer=Reducer.latest)
        ...     def stream_camera(self) -> str:
        ...         '''Stream camera frames in background.'''
        ...         for frame in self._get_frames():
        ...             yield frame
        ...         return "Camera stopped"
        ...
        ...     @skill(ret=Return.call_agent)  # Active companion keeps loop alive
        ...     def navigate_to(self, location: str) -> str:
        ...         '''Navigate while camera streams.'''
        ...         return f"Arrived at {location}"

        Hidden administrative skill:

        >>> class SystemSkills(Module):
        ...     def _calibrate(self) -> None:
        ...         pass  # Internal calibration logic
        ...
        ...     @skill(hide_skill=True)
        ...     def internal_calibration(self) -> str:
        ...         '''Internal calibration routine.'''
        ...         self._calibrate()
        ...         return "Calibration complete"

        See also the tutorials and other examples of skills in the library.

    Notes:

        **Key Contracts:**

        - Return strings for LLM compatibility (non-strings with `agent_encode()`
          method are auto-encoded)
        - Methods must be on subclasses of Module
        - Parameters must be JSON-serializable for schema generation

        **Passive Skill Warning:**

        When using `stream=Stream.passive`:

        - `ret` is forced to `Return.passive` automatically
        - Passive skills NEVER wake the agent (except on errors)
        - Data is only delivered when an active skill keeps the loop running
        - If only passive skills are running, the loop exits and data from passive skills is lost
        - So, pair passive skills with an active companion skill

        See `Stream.passive` docstring for full semantics.

        **Message Flow:**

        When called via coordinator, skills follow this state machine:

        Pending → Started (publish MsgType.start)
               → [Streaming (publish MsgType.stream)]*
               → Completed (publish MsgType.ret) OR Error (publish MsgType.error)

        **RPC Discovery:**

        This decorator automatically makes skills discoverable
        via the RPC system without requiring a separate `@rpc` decorator.
        Skills can be queried by agents through the SkillCoordinator.

        **Best Practices:**

        - Write clear docstrings - they become the skill descriptions LLMs see
        - Return meaningful strings that help agents understand outcomes
        - Handle errors gracefully with contextual messages for agent recovery
    """

    def decorator(f: Callable[..., Any]) -> Any:
        def wrapper(self, *args, **kwargs):  # type: ignore[no-untyped-def]
            skill = f"{f.__name__}"

            call_id = kwargs.get("call_id", None)
            if call_id:
                del kwargs["call_id"]

                return self.call_skill(call_id, skill, args, kwargs)
                # def run_function():
                #    return self.call_skill(call_id, skill, args, kwargs)
                #
                # thread = threading.Thread(target=run_function)
                # thread.start()
                # return None

            return f(self, *args, **kwargs)

        # sig = inspect.signature(f)
        # params = list(sig.parameters.values())
        # if params and params[0].name == "self":
        #     params = params[1:]  # Remove first parameter 'self'
        # wrapper.__signature__ = sig.replace(parameters=params)

        skill_config = SkillConfig(
            name=f.__name__,
            reducer=reducer,  # type: ignore[arg-type]
            stream=stream,
            # if stream is passive, ret must be passive too
            ret=ret.passive if stream == Stream.passive else ret,
            output=output,
            schema=function_to_schema(f),
            hide_skill=hide_skill,
        )

        wrapper.__rpc__ = True  # type: ignore[attr-defined]
        wrapper._skill_config = skill_config  # type: ignore[attr-defined]
        wrapper.__name__ = f.__name__  # Preserve original function name
        wrapper.__doc__ = f.__doc__  # Preserve original docstring
        return wrapper

    return decorator


@dataclass
class SkillContainerConfig:
    skill_transport: type[SkillCommsSpec] = LCMSkillComms


def threaded(f: Callable[..., Any]) -> Callable[..., None]:
    """Decorator to run a function in a thread pool."""

    def wrapper(self, *args, **kwargs):  # type: ignore[no-untyped-def]
        if self._skill_thread_pool is None:
            self._skill_thread_pool = ThreadPoolExecutor(
                max_workers=50, thread_name_prefix="skill_worker"
            )
        self._skill_thread_pool.submit(f, self, *args, **kwargs)
        return None

    return wrapper


# Inherited by any class that wants to provide skills
# (This component works standalone but commonly used by DimOS modules)
#
# Hosts the function execution and handles correct publishing of skill messages
# according to the individual skill decorator configuration
#
# - It allows us to specify a communication layer for skills (LCM for now by default)
# - introspection of available skills via the `skills` RPC method
# - ability to provide dynamic context dependant skills with dynamic_skills flag
#   for this you'll need to override the `skills` method to return a dynamic set of skills
#   SkillCoordinator will call this method to get the skills available upon every request to
#   the agent


class SkillContainer:
    skill_transport_class: type[SkillCommsSpec] = LCMSkillComms
    _skill_thread_pool: ThreadPoolExecutor | None = None
    _skill_transport: SkillCommsSpec | None = None

    @rpc
    def dynamic_skills(self) -> bool:
        return False

    def __str__(self) -> str:
        return f"SkillContainer({self.__class__.__name__})"

    @rpc
    def stop(self) -> None:
        if self._skill_transport:
            self._skill_transport.stop()
            self._skill_transport = None

        if self._skill_thread_pool:
            self._skill_thread_pool.shutdown(wait=True)
            self._skill_thread_pool = None

        # Continue the MRO chain if there's a parent stop() method
        if hasattr(super(), "stop"):
            super().stop()  # type: ignore[misc]

    # TODO: figure out standard args/kwargs passing format,
    # use same interface as skill coordinator call_skill method
    @threaded
    def call_skill(
        self, call_id: str, skill_name: str, args: tuple[Any, ...], kwargs: dict[str, Any]
    ) -> None:
        f = getattr(self, skill_name, None)

        if f is None:
            raise ValueError(f"Function '{skill_name}' not found in {self.__class__.__name__}")

        config = getattr(f, "_skill_config", None)
        if config is None:
            raise ValueError(f"Function '{skill_name}' in {self.__class__.__name__} is not a skill")

        # we notify the skill transport about the start of the skill call
        self.skill_transport.publish(SkillMsg(call_id, skill_name, None, type=MsgType.start))

        try:
            val = f(*args, **kwargs)

            # check if the skill returned a coroutine, if it is, block until it resolves
            if isinstance(val, asyncio.Future):
                val = asyncio.run(val)  # type: ignore[arg-type]

            # check if the skill is a generator, if it is, we need to iterate over it
            if hasattr(val, "__iter__") and not isinstance(val, str):
                last_value = None
                for v in val:
                    last_value = v
                    self.skill_transport.publish(
                        SkillMsg(call_id, skill_name, v, type=MsgType.stream)
                    )
                self.skill_transport.publish(
                    SkillMsg(call_id, skill_name, last_value, type=MsgType.ret)
                )

            else:
                self.skill_transport.publish(SkillMsg(call_id, skill_name, val, type=MsgType.ret))

        except Exception as e:
            import traceback

            formatted_traceback = "".join(traceback.TracebackException.from_exception(e).format())

            self.skill_transport.publish(
                SkillMsg(
                    call_id,
                    skill_name,
                    {"msg": str(e), "traceback": formatted_traceback},
                    type=MsgType.error,
                )
            )

    @rpc
    def skills(self) -> dict[str, SkillConfig]:
        # Avoid recursion by excluding this property itself
        # Also exclude known properties that shouldn't be accessed
        excluded = {"skills", "tf", "rpc", "skill_transport"}
        return {
            name: getattr(self, name)._skill_config
            for name in dir(self)
            if not name.startswith("_")
            and name not in excluded
            and hasattr(getattr(self, name), "_skill_config")
        }

    @property
    def skill_transport(self) -> SkillCommsSpec:
        if self._skill_transport is None:
            self._skill_transport = self.skill_transport_class()
        return self._skill_transport
