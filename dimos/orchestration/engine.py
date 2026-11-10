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

"""The pure state machine core, which turns events into effects."""

from __future__ import annotations

from collections.abc import Callable
import copy
from dataclasses import dataclass, field
import logging

from pydantic import JsonValue

from dimos.orchestration.definition import (
    CallStep,
    PublishStep,
    SetStep,
    State,
    StateMachineDefinition,
    Step,
    Transition,
)
from dimos.orchestration.expressions import Environment, ExpressionError, Scope

logger = logging.getLogger(__name__)

MAX_WHEN_CHAIN = 100

_Trigger = Callable[[Transition], bool]


@dataclass(frozen=True)
class InputMessage:
    """A message on one of the state machine's inputs."""

    name: str
    msg: dict[str, JsonValue]


@dataclass(frozen=True)
class ExternalEvent:
    """An event sent from outside the state machine."""

    name: str
    data: dict[str, JsonValue] = field(default_factory=dict)


Event = InputMessage | ExternalEvent


@dataclass(frozen=True)
class Publish:
    """Publish a value on an output."""

    output: str
    value: JsonValue


@dataclass(frozen=True)
class StartCall:
    """Call a role's method."""

    goal_id: str
    target: str
    args: dict[str, JsonValue]
    nowait: bool


Effect = Publish | StartCall


class StateMachineError(Exception):
    """The state machine stopped because its definition misbehaved at runtime."""


class Engine:
    """Runs one state machine definition, one event at a time."""

    def __init__(self, definition: StateMachineDefinition, id_prefix: str) -> None:
        self._definition = definition
        self._states = {state.name: state for state in definition.states}
        self._env = Environment(definition.params, definition.vars, definition.inputs)
        self._vars: dict[str, JsonValue] = copy.deepcopy(definition.vars)
        self._inputs: dict[str, JsonValue] = {}
        self._active: list[str] = []
        self._id_prefix = id_prefix
        self._next_id = 0
        self._effects: list[Effect] = []
        self._error: str | None = None

    @property
    def active(self) -> tuple[str, ...]:
        """The active states, from the top state down to the leaf."""
        return tuple(self._active)

    @property
    def vars(self) -> dict[str, JsonValue]:
        """A copy of the current vars."""
        return copy.deepcopy(self._vars)

    def start(self, now: float) -> list[Effect]:
        """Enter the initial state and return the effects of doing so."""
        self._enter(None, self._definition.state_machine.initial, {})
        self._settle({})
        return self._drain()

    def handle(self, event: Event, now: float) -> list[Effect]:
        """Process one event and return the effects it caused."""
        if self._error is not None:
            raise StateMachineError(self._error)
        name, payload = self._accept(event)
        match = self._match(lambda transition: transition.event == name, payload)
        if match is not None:
            self._take(*match, payload)
        self._settle(payload)
        return self._drain()

    def _accept(self, event: Event) -> tuple[str, dict[str, JsonValue]]:
        if isinstance(event, InputMessage):
            if event.name not in self._definition.inputs:
                raise ValueError(f"unknown input {event.name!r}")
            self._inputs[event.name] = event.msg
            return event.name, {"msg": event.msg}
        if event.name not in self._definition.events:
            raise ValueError(f"unknown event {event.name!r}")
        return event.name, {"data": event.data}

    def _drain(self) -> list[Effect]:
        effects, self._effects = self._effects, []
        return effects

    def _scope(self, payload: dict[str, JsonValue]) -> Scope:
        return self._env.scope(self._vars, self._inputs, payload)

    def _match(
        self, trigger: _Trigger, payload: dict[str, JsonValue]
    ) -> tuple[State, Transition] | None:
        for name in reversed(self._active):
            state = self._states[name]
            for transition in state.transitions:
                if trigger(transition) and self._allows(transition, payload):
                    return state, transition
        return None

    def _allows(self, transition: Transition, payload: dict[str, JsonValue]) -> bool:
        conditions = [transition.when, transition.guard]
        scope = self._scope(payload)
        return all(
            self._env.compile(condition).test(scope)
            for condition in conditions
            if condition is not None
        )

    def _settle(self, payload: dict[str, JsonValue]) -> None:
        for _ in range(MAX_WHEN_CHAIN + 1):
            match = self._match(lambda transition: transition.when is not None, payload)
            if match is None:
                return
            self._take(*match, payload)
        self._error = f"more than {MAX_WHEN_CHAIN} when transitions in a row in {self.active}"
        raise StateMachineError(self._error)

    def _take(self, source: State, transition: Transition, payload: dict[str, JsonValue]) -> None:
        assignments = self._evaluate_set(transition.set, payload)
        if transition.to is None:
            self._vars |= assignments
            self._run(transition.run, payload)
            return
        domain = self._domain(source.name, transition.to)
        while self._active and self._active[-1] != domain:
            self._run(self._states[self._active.pop()].exit, payload)
        self._vars |= assignments
        self._run(transition.run, payload)
        self._enter(domain, transition.to, payload)

    def _ancestry(self, name: str) -> list[str]:
        chain = [name]
        parent = self._states[name].parent
        while parent is not None:
            chain.append(parent)
            parent = self._states[parent].parent
        return chain

    def _domain(self, source: str, target: str) -> str | None:
        target_ancestors = self._ancestry(target)[1:]
        for ancestor in self._ancestry(source)[1:]:
            if ancestor in target_ancestors:
                return ancestor
        return None

    def _enter(self, domain: str | None, target: str, payload: dict[str, JsonValue]) -> None:
        path = self._ancestry(target)
        if domain is not None:
            path = path[: path.index(domain)]
        state = self._states[target]
        for name in reversed(path):
            self._active.append(name)
            self._run(self._states[name].entry, payload)
        while state.initial is not None:
            state = self._states[state.initial]
            self._active.append(state.name)
            self._run(state.entry, payload)

    def _evaluate_set(
        self, assignments: dict[str, JsonValue], payload: dict[str, JsonValue]
    ) -> dict[str, JsonValue]:
        scope = self._scope(payload)
        return {
            var: self._env.compile(value).assign(scope, self._vars[var])
            if isinstance(value, str)
            else value
            for var, value in assignments.items()
        }

    def _run(self, steps: list[Step], payload: dict[str, JsonValue]) -> None:
        for step in steps:
            if isinstance(step, SetStep):
                self._vars |= self._evaluate_set(step.set, payload)
                continue
            try:
                self._effects.append(self._effect(step, self._scope(payload)))
            except ExpressionError as error:
                logger.warning("Skipping a step: %s", error)

    def _effect(self, step: PublishStep | CallStep, scope: Scope) -> Effect:
        if isinstance(step, PublishStep):
            return Publish(step.publish, self._resolve(step.value, scope))
        args = {name: self._resolve(value, scope) for name, value in step.args.items()}
        return StartCall(self._new_id(), step.call, args, nowait=True)

    def _resolve(self, value: JsonValue, scope: Scope) -> JsonValue:
        return self._env.compile(value).evaluate(scope) if isinstance(value, str) else value

    def _new_id(self) -> str:
        self._next_id += 1
        return f"{self._id_prefix}-{self._next_id}"
