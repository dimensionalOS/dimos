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
from typing import Any, Generic, Literal, TypeVar

from dimos.types.timestamped import Timestamped
from dimos.utils.generic import truncate_display_string

# This file defines protocol messages used for communication between skills and agents


class Output(Enum):
    standard = 0
    human = 1
    image = 2  # this is same as separate_message, but maybe clearer for users


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
    # calls the function to get a value, when the agent is being called
    callback = 3  # TODO: this is a work in progress, not implemented yet


@dataclass
class SkillConfig:
    name: str
    reducer: ReducerF
    stream: Stream
    ret: Return
    output: Output
    schema: dict[str, Any]
    f: Callable | None = None  # type: ignore[type-arg]
    autostart: bool = False
    hide_skill: bool = False

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
    """All reducer that collects all values into a list."""
    acc_value = accumulator.content if accumulator else []
    return _make_skill_msg(msg, acc_value + msg.content)  # type: ignore[operator]


def accumulate_dict(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """All reducer that collects all values into a list."""
    acc_value = accumulator.content if accumulator else {}
    return _make_skill_msg(msg, {**acc_value, **msg.content})  # type: ignore[dict-item]


def accumulate_string(
    accumulator: SkillMsg[Literal[MsgType.reduced_stream]] | None,
    msg: SkillMsg[Literal[MsgType.stream]],
) -> SkillMsg[Literal[MsgType.reduced_stream]]:
    """All reducer that collects all values into a list."""
    acc_value = accumulator.content if accumulator else ""
    return _make_skill_msg(msg, acc_value + "\n" + msg.content)  # type: ignore[operator]


class Reducer:
    sum = sum_reducer
    latest = latest_reducer
    all = all_reducer
    accumulate_list = accumulate_list
    accumulate_dict = accumulate_dict
    string = accumulate_string
