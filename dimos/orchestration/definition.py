# Copyright 2025-2026 Dimensional Inc.
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

"""The primitives an orchestrator definition file is parsed into.

Static checks then run over them to confirm the orchestrator is valid.
"""

import json
from typing import Annotated, Literal

from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, JsonValue, model_validator


def _source(value: JsonValue) -> str:
    """A string is an expression already, and any other value is written as a literal."""
    return value if isinstance(value, str) else json.dumps(value)


Source = Annotated[str, BeforeValidator(_source)]


class _Struct(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)


class Signal(_Struct):
    name: str
    stream: str
    field: str | None = None


class Event(_Struct):
    name: str
    when: str


class Activity(_Struct):
    name: str
    start: str
    cancel: str | None = None
    uses: tuple[str, ...] = ()

    @property
    def resources(self) -> frozenset[str]:
        """What it uses. Its cancel RPC counts as a resource."""
        return frozenset(self.uses if self.cancel is None else (*self.uses, self.cancel))


class Check(_Struct):
    expect: tuple[str, ...]
    fallback: str | None = None


class Transition(_Struct):
    goto: str
    if_: str | None = Field(default=None, validation_alias="if")
    mode: Literal["graceful", "immediate"] = "graceful"
    set: dict[str, Source] = {}
    trigger: str | None = None
    after: Source | None = None

    @model_validator(mode="after")
    def _at_most_one_cause(self) -> "Transition":
        if self.trigger is not None and self.after is not None:
            raise ValueError("a transition has at most one of `trigger` and `after`")
        return self


class Node(_Struct):
    """One node. The nodes inside it are named, not nested."""

    name: str
    transitions: tuple[Transition, ...] = ()
    requires: tuple[Check, ...] = ()
    ensures: tuple[Check, ...] = ()
    claims: tuple[str, ...] = ()
    activity: str | None = None
    args: dict[str, Source] = {}
    result: str | None = None
    children: tuple[str, ...] = ()
    parallel: tuple[str, ...] = ()
    initial: str | None = None
    outcomes: tuple[Literal["done", "failed"], ...] = ()

    @model_validator(mode="after")
    def _at_most_one_kind(self) -> "Node":
        if self.children and self.parallel:
            raise ValueError("a node has at most one of `children` and `parallel`")
        return self

    @property
    def inside(self) -> tuple[str, ...]:
        return (*self.children, *self.parallel)

    @property
    def is_switch(self) -> bool:
        return any(t.trigger is None and t.after is None for t in self.transitions)


class OrchestratorDefinition(_Struct):
    """A whole orchestrator file. `root` names one of `nodes`."""

    root: str
    constants: dict[str, JsonValue] = {}
    signals: tuple[Signal, ...] = ()
    events: tuple[Event, ...] = ()
    vars: dict[str, JsonValue] = {}
    activities: tuple[Activity, ...] = ()
    resources: tuple[str, ...] = ()
    nodes: tuple[Node, ...] = ()
