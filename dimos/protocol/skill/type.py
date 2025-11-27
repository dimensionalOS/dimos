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
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import time
from typing import Annotated, Any, Generic, Literal, TypeVar

from annotated_doc import Doc

from dimos.types.timestamped import Timestamped
from dimos.utils.generic import truncate_display_string

# This file defines protocol messages used for communication between skills and agents


class Output(Enum):
    standard = 0
    human = 1
    image = 2  # this is same as separate_message, but maybe clearer for users


class Stream(Enum):
    """Controls how streaming skill outputs are handled.

    Streaming skills (generators/iterators) emit multiple values during execution.
    This enum determines whether each emitted value should wake the agent or just
    accumulate silently in the background.
    """

    none = 0
    """No streaming. Skill returns a single value, not a generator."""

    passive = 1
    """Passive streaming that accumulates values without waking the agent.

    Values accumulate via the configured reducer but do not trigger agent calls.
    Passive skill data is only delivered when an active skill keeps the agent
    loop running long enough for a snapshot to be generated.

    Behavior:
    - Each `yield` applies the reducer to accumulate state
    - Never wakes the agent (except on errors)
    - Forces `ret=Return.passive` regardless of user setting

    How delivery works:
    - The agent loop checks for active skills before generating snapshots
      (snapshots include info about all skills)
    - If active skills exist, the loop continues.

    Note:
    - If *only* passive skills are running, the loop exits immediately at the
      termination check without generating a snapshot.
      Remaining passive data will not be delivered to the agent.
    - That is, passive skills *require* an active companion skill (like `human_input` or
      any `stream=Stream.call_agent` skill) to ensure data reaches the agent.

    Examples of use cases:
    - Video streaming during navigation (with active navigation skill)
    - Sensor telemetry alongside task execution

    Anti-patterns:
    - Using passive skills without active skills (data never delivered)
    - Starting passive skills with short-lived active skills (data may be lost)
    """

    call_agent = 2
    """Active streaming that wakes the agent on every value.

    Each `yield` triggers an immediate agent call. Use for progress updates or
    incremental results that the agent should process immediately.
    """


class Return(Enum):
    """Controls how skill return values are delivered and whether they wake the agent.

    While Stream controls behavior during execution (for generators), Return controls
    what happens when a skill completes. This determines whether the agent is directly notified
    of completion and whether the return value is included in snapshots.

    Note: Errors always wake the agent regardless of Return setting.

    Constraint: `stream=Stream.passive` forces `ret=Return.passive` automatically.
    """

    none = 0
    """Return value discarded, agent not notified.

    Use for fire-and-forget operations where the agent doesn't need
    to know about completion.

    Examples of use cases:
    - Background logging or telemetry
    - Fire-and-forget actuator commands
    - Cleanup operations
    """

    passive = 1
    """Return value stored but agent not woken.

    The skill completes silently, but the return value is stored and appears in
    snapshots when the agent wakes for other reasons.

    Critical: If no active skills are running when this skill completes, the
    agent loop exits and this return value is never delivered.

    Note: When `stream=Stream.passive`, `ret` is forced to this value.

    Use cases:
    - Status checks collected alongside active tasks
    - Sensor readings that don't justify waking agent
    """

    call_agent = 2
    """Return value triggers immediate agent notification.

    Skill completion wakes the agent and delivers the return value immediately.
    This is the default and most common behavior.

    Use cases:
    - Task results that require agent decision-making
    - Navigation/manipulation completions
    - Any operation where agent needs the result
    """

    callback = 3
    """Not implemented. Reserved for future callback pattern."""


@dataclass
class SkillConfig:
    """Configuration for a skill, created by the @skill decorator.

    Attached to decorated methods as `_skill_config`. Used by SkillCoordinator
    to control execution behavior.
    """

    name: Annotated[str, Doc("Skill name (from decorated function name).")]
    reducer: Annotated[ReducerF, Doc("Aggregation function for streaming values.")]
    stream: Annotated[Stream, Doc("Streaming behavior (none/passive/call_agent).")]
    ret: Annotated[
        Return,
        Doc(
            "Return value delivery (none/passive/call_agent). "
            "Note: Forced to `passive` when `stream=Stream.passive`."
        ),
    ]
    output: Annotated[Output, Doc("Presentation hint for agent (standard/human/image).")]
    schema: Annotated[dict[str, Any], Doc("OpenAI function-calling schema for LLM invocation.")]
    f: Annotated[Callable | None, Doc("Bound method reference (set via `bind()`)")] = None  # type: ignore[type-arg]
    autostart: Annotated[bool, Doc("Reserved for future use (currently unused).")] = False
    hide_skill: Annotated[bool, Doc("If True, skill hidden from LLM tool selection.")] = False

    def bind(self, f: Callable) -> SkillConfig:  # type: ignore[type-arg]
        self.f = f
        return self

    def call(self, call_id, *args, **kwargs) -> Any:  # type: ignore[no-untyped-def]
        if self.f is None:
            raise ValueError(
                "Function is not bound to the SkillConfig. This should be called only within AgentListener."
            )

        return self.f(*args, **kwargs, call_id=call_id)

    def __str__(self) -> str:
        parts = [f"name={self.name}"]

        # Only show reducer if stream is not none (streaming is happening)
        if self.stream != Stream.none:
            parts.append(f"stream={self.stream.name}")

        # Always show return mode
        parts.append(f"ret={self.ret.name}")
        return f"Skill({', '.join(parts)})"


class MsgType(Enum):
    pending = 0
    start = 1
    stream = 2
    reduced_stream = 3
    ret = 4
    error = 5


M = TypeVar("M", bound="MsgType")


def maybe_encode(something: Any) -> str:
    if hasattr(something, "agent_encode"):
        return something.agent_encode()  # type: ignore[no-any-return]
    return something  # type: ignore[no-any-return]


class SkillMsg(Timestamped, Generic[M]):
    ts: float
    type: M
    call_id: str
    skill_name: str
    content: str | int | float | dict | list  # type: ignore[type-arg]

    def __init__(
        self,
        call_id: str,
        skill_name: str,
        content: Any,
        type: M,
    ) -> None:
        self.ts = time.time()
        self.call_id = call_id
        self.skill_name = skill_name
        # any tool output can be a custom type that knows how to encode itself
        # like a costmap, path, transform etc could be translatable into strings

        self.content = maybe_encode(content)
        self.type = type

    @property
    def end(self) -> bool:
        return self.type == MsgType.ret or self.type == MsgType.error

    @property
    def start(self) -> bool:
        return self.type == MsgType.start

    def __str__(self) -> str:  # type: ignore[return]
        time_ago = time.time() - self.ts

        if self.type == MsgType.start:
            return f"Start({time_ago:.1f}s ago)"
        if self.type == MsgType.ret:
            return f"Ret({time_ago:.1f}s ago, val={truncate_display_string(self.content)})"
        if self.type == MsgType.error:
            return f"Error({time_ago:.1f}s ago, val={truncate_display_string(self.content)})"
        if self.type == MsgType.pending:
            return f"Pending({time_ago:.1f}s ago)"
        if self.type == MsgType.stream:
            return f"Stream({time_ago:.1f}s ago, val={truncate_display_string(self.content)})"
        if self.type == MsgType.reduced_stream:
            return f"Stream({time_ago:.1f}s ago, val={truncate_display_string(self.content)})"


# typing looks complex but it's a standard reducer function signature, using SkillMsgs
# (Optional[accumulator], msg) -> accumulator
ReducerF = Callable[
    [SkillMsg[Literal[MsgType.reduced_stream]] | None, SkillMsg[Literal[MsgType.stream]]],
    SkillMsg[Literal[MsgType.reduced_stream]],
]


C = TypeVar("C")  # content type
A = TypeVar("A")  # accumulator type
# define a naive reducer function type that's generic in terms of the accumulator type
SimpleReducerF = Callable[[A | None, C], A]


def make_reducer(simple_reducer: SimpleReducerF) -> ReducerF:  # type: ignore[type-arg]
    """
    Converts a naive reducer function into a standard reducer function.
    The naive reducer function should accept an accumulator and a message,
    and return the updated accumulator.
    """

    def reducer(
        accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
        msg: SkillMsg[Literal[MsgType.stream]],
    ) -> SkillMsg[Literal[MsgType.reduced_stream]]:
        # Extract the content from the accumulator if it exists
        acc_value = accumulator.content if accumulator else None

        # Apply the simple reducer to get the new accumulated value
        new_value = simple_reducer(acc_value, msg.content)

        # Wrap the result in a SkillMsg with reduced_stream type
        return SkillMsg(
            call_id=msg.call_id,
            skill_name=msg.skill_name,
            content=new_value,
            type=MsgType.reduced_stream,
        )

    return reducer


# just a convinience class to hold reducer functions
def _make_skill_msg(
    msg: SkillMsg[Literal[MsgType.stream]], content: Any
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """Helper to create a reduced stream message with new content."""
    return SkillMsg(
        call_id=msg.call_id,
        skill_name=msg.skill_name,
        content=content,
        type=MsgType.reduced_stream,
    )


def sum_reducer(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """Sum reducer that adds values together."""
    acc_value = accumulator.content if accumulator else None
    new_value = acc_value + msg.content if acc_value else msg.content  # type: ignore[operator]
    return _make_skill_msg(msg, new_value)


def latest_reducer(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """Latest reducer that keeps only the most recent value."""
    return _make_skill_msg(msg, msg.content)


def all_reducer(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """All reducer that collects all values into a list."""
    acc_value = accumulator.content if accumulator else None
    new_value = [*acc_value, msg.content] if acc_value else [msg.content]  # type: ignore[misc]
    return _make_skill_msg(msg, new_value)


def accumulate_list(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """List concatenation reducer: extends accumulator list with message content list."""
    acc_value = accumulator.content if accumulator else []
    return _make_skill_msg(msg, acc_value + msg.content)  # type: ignore[operator]


def accumulate_dict(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """Dict merge reducer: merges message content dict into accumulator dict."""
    acc_value = accumulator.content if accumulator else {}
    return _make_skill_msg(msg, {**acc_value, **msg.content})  # type: ignore[dict-item]


def accumulate_string(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """String concatenation reducer: joins values with newlines.

    Examples:
        >>> m = lambda s: SkillMsg('id', 'x', s, MsgType.stream)
        >>> accumulate_string(None, m('A')).content  # no leading newline
        'A'
        >>> accumulate_string(accumulate_string(None, m('A')), m('B')).content
        'A\\nB'
        >>> # Edge case: empty string as first yield doesn't cause leading newline
        >>> accumulate_string(accumulate_string(None, m('')), m('X')).content
        'X'
    """
    prefix = f"{accumulator.content}\n" if accumulator and accumulator.content else ""
    new_value = prefix + msg.content
    return _make_skill_msg(msg, new_value)


class Reducer:
    r"""Namespace for reducer functions that aggregate streaming skill outputs.

    Reducers apply only to streaming skills (`stream=Stream.passive` or
    `stream=Stream.call_agent`). Each time a skill yields a value, the
    SkillCoordinator applies the configured reducer to update accumulated state:

    ```python
    self.reduced_stream_msg = self.skill_config.reducer(self.reduced_stream_msg, msg)
    ```

    **Reducer signature:**

    ```python
    (accumulator: SkillMsg | None, msg: SkillMsg) -> SkillMsg
    ```

    **Custom reducers:**

    Create custom reducers using `make_reducer()` to wrap simple reduction functions.

    Examples:
        Basic counter with sum reduction:

        >>> from dimos.protocol.skill import skill
        >>> from dimos.protocol.skill.type import Stream, Reducer, Output
        >>> from dimos.core.module import Module
        >>>
        >>> class CounterSkills(Module):
        ...     @skill(stream=Stream.passive, reducer=Reducer.sum)
        ...     def count_events(self, n: int):
        ...         for i in range(1, n + 1):
        ...             yield i
        ...         # Coordinator accumulates: 1, 1+2=3, 3+3=6, 6+4=10, 10+5=15

        Video stream with latest frame:

        >>> class CameraSkills(Module):
        ...     @skill(stream=Stream.passive, output=Output.image, reducer=Reducer.latest)
        ...     def video_stream(self):
        ...         while True:
        ...             frame = self.camera.read()
        ...             yield frame
        ...             # Only latest frame kept (memory efficient for high-frequency data)

        Navigation progress log:

        >>> class NavSkills(Module):
        ...     @skill(stream=Stream.call_agent, reducer=Reducer.string)
        ...     def goto_location(self, x: float, y: float):
        ...         yield f"Moving to ({x:.2f}, {y:.2f})..."
        ...         # ... navigation logic ...
        ...         yield f"Arrived at ({x:.2f}, {y:.2f})"
        ...         # Agent sees: "Moving to (5.00, 3.00)...\nArrived at (5.00, 3.00)"

    Note:
        Memory considerations: `Reducer.latest` and `Reducer.sum` use constant memory,
        while `Reducer.all`, `Reducer.accumulate_list`, `Reducer.accumulate_dict`, and
        `Reducer.string` grow linearly with the number of yielded values. Use `Reducer.latest`
        for high-frequency sensor data to avoid memory growth.

        Reducers have no effect on non-streaming skills (`stream=Stream.none`).
    """

    sum: Annotated[
        ReducerF,
        Doc(
            """Adds numeric values together. O(1) memory. Use for counting events
            or computing running totals."""
        ),
    ] = sum_reducer

    latest: Annotated[
        ReducerF,
        Doc(
            """Keeps only the most recent value, discarding previous state. O(1) memory.
            Ideal for high-frequency data (sensor readings, video frames)."""
        ),
    ] = latest_reducer

    all: Annotated[
        ReducerF,
        Doc(
            """Collects all streamed values into a list. O(n) memory. Use for event
            logs or history where you need the complete sequence."""
        ),
    ] = all_reducer

    accumulate_list: Annotated[
        ReducerF,
        Doc(
            """Concatenates yielded lists into one. O(n) memory. Unlike `all`,
            expects values to already be lists."""
        ),
    ] = accumulate_list

    accumulate_dict: Annotated[
        ReducerF,
        Doc(
            """Merges yielded dicts into one; later values overwrite earlier for
            duplicate keys. O(n) memory in unique keys."""
        ),
    ] = accumulate_dict

    string: Annotated[
        ReducerF,
        Doc(
            """Joins string values with newlines. O(n) memory. Use for progress logs
            or text reports."""
        ),
    ] = accumulate_string
