# Copyright 2026 Dimensional Inc.
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

"""State machine definitions loaded from TOML."""

from __future__ import annotations

from pathlib import Path
import re
import sys
from typing import Annotated

from pydantic import (
    AfterValidator,
    BaseModel,
    BeforeValidator,
    ConfigDict,
    Discriminator,
    Field,
    JsonValue,
    Tag,
    ValidationError,
    model_validator,
)

from dimos.orchestration.expressions import Environment, ExpressionError, kind_of

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib

DO_EVENTS = ("do.succeeded", "do.failed", "do.cancelled")

_DURATION = re.compile(r"(\d+(?:\.\d+)?)([smh])")
_UNIT_SECONDS = {"s": 1.0, "m": 60.0, "h": 3600.0}
_TARGET = re.compile(r"[A-Za-z_]\w*\.[A-Za-z_]\w*")


class DefinitionError(ValueError):
    """A state machine file that fails to parse or check."""


def _parse_duration(value: JsonValue) -> float:
    match = _DURATION.fullmatch(value) if isinstance(value, str) else None
    if match is None:
        raise ValueError(f"expected a duration like '5s', '3m' or '2h', got {value!r}")
    return float(match.group(1)) * _UNIT_SECONDS[match.group(2)]


def _check_target(value: str) -> str:
    if _TARGET.fullmatch(value) is None:
        raise ValueError(f"expected role.method, got {value!r}")
    return value


Duration = Annotated[float, BeforeValidator(_parse_duration)]
Target = Annotated[str, AfterValidator(_check_target)]


def _kind(value: JsonValue | BaseModel, kinds: tuple[str, ...]) -> str | None:
    if isinstance(value, BaseModel):
        present = [kind for kind in kinds if kind in type(value).model_fields]
    elif isinstance(value, dict):
        present = [kind for kind in kinds if kind in value]
    else:
        return None
    return present[0] if len(present) == 1 else None


def _step_kind(value: JsonValue | BaseModel) -> str | None:
    return _kind(value, ("set", "publish", "call"))


def _activity_kind(value: JsonValue | BaseModel) -> str | None:
    return _kind(value, ("call", "action"))


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class StateMachineInfo(_Strict):
    """The state machine's name and its top-level initial state."""

    name: str
    initial: str


class EventDeclaration(_Strict):
    """An external event that send_event accepts."""


class SetStep(_Strict):
    """Assign vars inside the engine."""

    set: dict[str, JsonValue]


class PublishStep(_Strict):
    """Publish a value on a declared output."""

    publish: str
    value: JsonValue


class CallStep(_Strict):
    """Call a role's method without waiting for the reply."""

    call: Target
    args: dict[str, JsonValue] = Field(default_factory=dict)


Step = Annotated[
    Annotated[SetStep, Tag("set")]
    | Annotated[PublishStep, Tag("publish")]
    | Annotated[CallStep, Tag("call")],
    Discriminator(
        _step_kind,
        custom_error_type="step_kind",
        custom_error_message="a step needs exactly one of set, publish or call",
    ),
]


class CallActivity(_Strict):
    """One RPC whose reply is the activity's outcome."""

    call: Target
    args: dict[str, JsonValue] = Field(default_factory=dict)
    timeout: Duration | None = None


class ActionActivity(_Strict):
    """A long-running goal started, reported and cancelled through the action contract."""

    action: Target
    args: dict[str, JsonValue] = Field(default_factory=dict)
    timeout: Duration | None = None


Activity = Annotated[
    Annotated[CallActivity, Tag("call")] | Annotated[ActionActivity, Tag("action")],
    Discriminator(
        _activity_kind,
        custom_error_type="activity_kind",
        custom_error_message="an activity needs exactly one of call or action",
    ),
]


class Transition(_Strict):
    """A way out of a state, with exactly one trigger."""

    event: str | None = None
    when: str | None = None
    after: Duration | None = None
    guard: str | None = Field(default=None, alias="if")
    to: str | None = None
    set: dict[str, JsonValue] = Field(default_factory=dict)
    run: list[Step] = Field(default_factory=list)

    @model_validator(mode="after")
    def _one_trigger(self) -> Transition:
        triggers = [self.event, self.when, self.after]
        count = sum(trigger is not None for trigger in triggers)
        if count != 1:
            raise ValueError(f"needs exactly one of event, when or after, got {count}")
        return self


class State(_Strict):
    """One state block, either a leaf or a parent of other states."""

    name: str
    parent: str | None = None
    initial: str | None = None
    entry: list[Step] = Field(default_factory=list)
    exit: list[Step] = Field(default_factory=list)
    do: Activity | None = None
    transitions: list[Transition] = Field(default_factory=list, alias="transition")


class StateMachineDefinition(_Strict):
    """A whole state machine file."""

    state_machine: StateMachineInfo = Field(alias="machine")
    params: dict[str, JsonValue] = Field(default_factory=dict)
    vars: dict[str, JsonValue] = Field(default_factory=dict)
    inputs: dict[str, str] = Field(default_factory=dict)
    outputs: dict[str, str] = Field(default_factory=dict)
    events: dict[str, EventDeclaration] = Field(default_factory=dict)
    states: list[State] = Field(alias="state", min_length=1)

    @model_validator(mode="after")
    def _consistent(self) -> StateMachineDefinition:
        problems = _problems(self)
        if problems:
            raise ValueError("\n".join(problems))
        return self


def _ancestry(name: str, by_name: dict[str, State]) -> list[State]:
    chain: list[State] = []
    seen: set[str] = set()
    current = by_name.get(name)
    while current is not None and current.name not in seen:
        seen.add(current.name)
        chain.append(current)
        current = by_name.get(current.parent) if current.parent is not None else None
    return chain


def _expression_problems(
    at: str, source: str, env: Environment, kind: str | None = None
) -> list[str]:
    try:
        expression = env.compile(source)
    except ExpressionError as error:
        return [f"{at}: {error}"]
    if kind is not None and expression.kind not in (kind, "dyn"):
        return [f"{at}: gives {expression.kind}, expected {kind}"]
    return []


def _value_problems(
    at: str, value: JsonValue, env: Environment, kind: str | None = None
) -> list[str]:
    if isinstance(value, str):
        return _expression_problems(at, value, env, kind)
    if kind is not None and kind_of(value) != kind:
        return [f"{at}: {value!r} is {kind_of(value)}, expected {kind}"]
    return []


def _args_problems(at: str, args: dict[str, JsonValue], env: Environment) -> list[str]:
    return [
        problem
        for name, value in args.items()
        for problem in _value_problems(f"{at}: {name}", value, env)
    ]


def _set_problems(
    at: str,
    assignments: dict[str, JsonValue],
    definition: StateMachineDefinition,
    env: Environment,
) -> list[str]:
    problems: list[str] = []
    for var, value in assignments.items():
        if var not in definition.vars:
            problems.append(f"{at}: undeclared var {var!r}")
        else:
            problems += _value_problems(f"{at}: {var}", value, env, kind_of(definition.vars[var]))
    return problems


def _step_problems(
    where: str, steps: list[Step], definition: StateMachineDefinition, env: Environment
) -> list[str]:
    problems: list[str] = []
    for index, step in enumerate(steps):
        at = f"{where} {index}"
        if isinstance(step, SetStep):
            problems += _set_problems(f"{at}: set", step.set, definition, env)
        elif isinstance(step, PublishStep):
            if step.publish not in definition.outputs:
                problems.append(f"{at}: publish: undeclared output {step.publish!r}")
            problems += _value_problems(f"{at}: value", step.value, env)
        else:
            problems += _args_problems(f"{at}: args", step.args, env)
    return problems


def _problems(definition: StateMachineDefinition) -> list[str]:
    problems: list[str] = []
    by_name: dict[str, State] = {}
    for state in definition.states:
        if state.name in by_name:
            problems.append(f"state {state.name!r}: name: duplicate state name")
        by_name.setdefault(state.name, state)

    children: dict[str, list[str]] = {}
    for state in definition.states:
        if state.parent is None:
            continue
        if state.parent not in by_name:
            problems.append(f"state {state.name!r}: parent: unknown state {state.parent!r}")
        else:
            children.setdefault(state.parent, []).append(state.name)

    for name in by_name:
        chain = _ancestry(name, by_name)
        last = chain[-1]
        if last.parent == name:
            cycle = " -> ".join([state.name for state in chain] + [name])
            problems.append(f"state {name!r}: parent: cycle {cycle}")

    initial = definition.state_machine.initial
    if initial not in by_name:
        problems.append(f"machine: initial: unknown state {initial!r}")
    elif by_name[initial].parent in by_name:
        problems.append(f"machine: initial: {initial!r} is not a top-level state")

    allowed_events = {*DO_EVENTS, *definition.inputs, *definition.events}
    env = Environment(definition.params, definition.vars, definition.inputs)
    for state in definition.states:
        where = f"state {state.name!r}"
        kids = children.get(state.name, [])
        if kids and state.initial is None:
            problems.append(f"{where}: initial: a parent needs an initial child")
        if state.initial is not None:
            if state.initial not in by_name:
                problems.append(f"{where}: initial: unknown state {state.initial!r}")
            elif state.initial not in kids:
                problems.append(
                    f"{where}: initial: {state.initial!r} is not a child of {state.name!r}"
                )
        if state.do is not None and not any(
            transition.event == "do.failed"
            for ancestor in _ancestry(state.name, by_name)
            for transition in ancestor.transitions
        ):
            problems.append(f"{where}: do: no do.failed transition on this state or a parent")
        if state.do is not None:
            problems += _args_problems(f"{where}: do: args", state.do.args, env)
        problems += _step_problems(f"{where}: entry", state.entry, definition, env)
        problems += _step_problems(f"{where}: exit", state.exit, definition, env)
        for index, transition in enumerate(state.transitions):
            at = f"{where}: transition {index}"
            if transition.to is not None and transition.to not in by_name:
                problems.append(f"{at}: to: unknown state {transition.to!r}")
            if transition.event is not None and transition.event not in allowed_events:
                problems.append(
                    f"{at}: event: {transition.event!r} is not a do event, an input or a declared event"
                )
            if transition.guard is not None:
                problems += _expression_problems(f"{at}: if", transition.guard, env, "bool")
            if transition.when is not None:
                problems += _expression_problems(f"{at}: when", transition.when, env, "bool")
            problems += _set_problems(f"{at}: set", transition.set, definition, env)
            problems += _step_problems(f"{at}: run", transition.run, definition, env)
    return problems


def _location(loc: tuple[int | str, ...], raw: dict[str, JsonValue]) -> str:
    parts: list[str] = []
    rest = list(loc)
    states = raw.get("state")
    if len(rest) >= 2 and rest[0] == "state" and isinstance(rest[1], int):
        index = rest[1]
        block = states[index] if isinstance(states, list) and index < len(states) else None
        name = block.get("name") if isinstance(block, dict) else None
        parts.append(f"state {name!r}" if isinstance(name, str) else f"state {index}")
        rest = rest[2:]
    for part in rest:
        if isinstance(part, int) and parts:
            parts[-1] = f"{parts[-1]} {part}"
        elif not parts or parts[-1] != part:
            parts.append(str(part))
    return ": ".join(parts)


def _messages(error: ValidationError, raw: dict[str, JsonValue]) -> list[str]:
    messages: list[str] = []
    for detail in error.errors():
        ctx = detail.get("ctx", {})
        message = str(ctx["error"]) if detail["type"] == "value_error" else detail["msg"]
        where = _location(detail["loc"], raw)
        messages += [f"{where}: {line}" if where else line for line in message.splitlines()]
    return messages


def load_definition(path: Path | str) -> StateMachineDefinition:
    """Load and check a state machine file, raising DefinitionError on any problem."""
    with open(path, "rb") as file:
        try:
            raw = tomllib.load(file)
        except tomllib.TOMLDecodeError as error:
            raise DefinitionError(f"{path}: {error}") from error
    try:
        return StateMachineDefinition.model_validate(raw)
    except ValidationError as error:
        lines = _messages(error, raw)
        raise DefinitionError("\n".join(f"{path}: {line}" for line in lines)) from error
