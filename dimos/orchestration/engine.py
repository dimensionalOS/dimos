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
from typing import Literal

from pydantic import JsonValue

from dimos.orchestration.definition import (
    ActionActivity,
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

MAX_CHAIN = 100

_Trigger = Callable[[Transition], bool]
_Payload = dict[str, JsonValue]


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


@dataclass(frozen=True)
class ActionResult:
    """The final status of an action the engine started."""

    goal_id: str
    status: Literal["succeeded", "failed", "cancelled"]
    reason: str = ""


@dataclass(frozen=True)
class CallReply:
    """The reply to a call the engine started as an activity."""

    goal_id: str
    ok: bool
    result: JsonValue = None
    reason: str = ""


@dataclass(frozen=True)
class TimerFired:
    """A timer the engine armed has reached its deadline."""

    timer_id: str


Event = InputMessage | ExternalEvent | ActionResult | CallReply | TimerFired


@dataclass(frozen=True)
class Publish:
    """Publish a value on an output."""

    output: str
    value: JsonValue


@dataclass(frozen=True)
class StartCall:
    """Call a role's method, reporting the reply unless nowait is set."""

    goal_id: str
    target: str
    args: dict[str, JsonValue]
    nowait: bool


@dataclass(frozen=True)
class StartAction:
    """Start an action under the engine's goal id."""

    goal_id: str
    target: str
    args: dict[str, JsonValue]


@dataclass(frozen=True)
class CancelAction:
    """Cancel an action the engine started."""

    goal_id: str
    target: str


@dataclass(frozen=True)
class ArmTimer:
    """Fire a TimerFired event at the deadline."""

    timer_id: str
    deadline: float


@dataclass(frozen=True)
class ClearTimer:
    """Forget a timer that has not fired yet."""

    timer_id: str


Effect = Publish | StartCall | StartAction | CancelAction | ArmTimer | ClearTimer


class StateMachineError(Exception):
    """The state machine stopped because its definition misbehaved at runtime."""


@dataclass(frozen=True)
class _Activity:
    goal_id: str
    state: str
    target: str
    is_action: bool
    timer_id: str | None


@dataclass(frozen=True)
class _Timer:
    state: str
    transition: int | None
    seconds: float


class Engine:
    """Runs one state machine definition, one event at a time."""

    def __init__(self, definition: StateMachineDefinition, id_prefix: str) -> None:
        self._definition = definition
        self._states = {state.name: state for state in definition.states}
        self._env = Environment(definition.params, definition.vars, definition.inputs)
        self._vars: dict[str, JsonValue] = copy.deepcopy(definition.vars)
        self._inputs: dict[str, JsonValue] = {}
        self._active: list[str] = []
        self._activities: dict[str, _Activity] = {}
        self._timers: dict[str, _Timer] = {}
        self._failed_starts: list[tuple[str, str]] = []
        self._id_prefix = id_prefix
        self._next_id = 0
        self._now = 0.0
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
        self._now = now
        self._enter(None, self._definition.state_machine.initial, {})
        self._settle({})
        return self._drain()

    def handle(self, event: Event, now: float) -> list[Effect]:
        """Process one event and return the effects it caused."""
        if self._error is not None:
            raise StateMachineError(self._error)
        self._now = now
        payload = self._dispatch(event)
        if payload is not None:
            self._settle(payload)
        return self._drain()

    def _dispatch(self, event: Event) -> _Payload | None:
        if isinstance(event, InputMessage | ExternalEvent):
            name, payload = self._accept(event)
            match = self._match(lambda transition: transition.event == name, payload)
            if match is not None:
                self._take(*match, payload)
            return payload
        if isinstance(event, TimerFired):
            return self._fire(event.timer_id)
        activity = self._activity(event.goal_id)
        if activity is None:
            logger.debug("Dropping a stale result for goal %s", event.goal_id)
            return None
        self._finish(activity)
        if isinstance(event, ActionResult):
            return self._outcome(activity.state, f"do.{event.status}", {"reason": event.reason})
        if event.ok:
            return self._outcome(
                activity.state, "do.succeeded", {"reason": event.reason, "result": event.result}
            )
        return self._outcome(activity.state, "do.failed", {"reason": event.reason})

    def _accept(self, event: InputMessage | ExternalEvent) -> tuple[str, _Payload]:
        if isinstance(event, InputMessage):
            if event.name not in self._definition.inputs:
                raise ValueError(f"unknown input {event.name!r}")
            self._inputs[event.name] = event.msg
            return event.name, {"msg": event.msg}
        if event.name not in self._definition.events:
            raise ValueError(f"unknown event {event.name!r}")
        return event.name, {"data": event.data}

    def _fire(self, timer_id: str) -> _Payload | None:
        timer = self._timers.pop(timer_id, None)
        if timer is None:
            logger.debug("Dropping timer %s, which is no longer armed", timer_id)
            return None
        state = self._states[timer.state]
        if timer.transition is not None:
            transition = state.transitions[timer.transition]
            if self._allows(transition, {}):
                self._take(state, transition, {})
            return {}
        activity = self._activities.pop(state.name)
        if activity.is_action:
            self._effects.append(CancelAction(activity.goal_id, activity.target))
        reason = f"timed out after {timer.seconds:g} s"
        return self._outcome(state.name, "do.failed", {"reason": reason})

    def _outcome(self, state: str, name: str, payload: _Payload) -> _Payload:
        match = self._match(lambda transition: transition.event == name, payload, state)
        if match is None and name == "do.cancelled":
            match = self._match(lambda transition: transition.event == "do.failed", payload, state)
        if match is not None:
            self._take(*match, payload)
        return payload

    def _activity(self, goal_id: str) -> _Activity | None:
        return next(
            (activity for activity in self._activities.values() if activity.goal_id == goal_id),
            None,
        )

    def _finish(self, activity: _Activity) -> None:
        del self._activities[activity.state]
        if activity.timer_id is not None:
            del self._timers[activity.timer_id]
            self._effects.append(ClearTimer(activity.timer_id))

    def _drain(self) -> list[Effect]:
        effects, self._effects = self._effects, []
        return effects

    def _scope(self, payload: _Payload) -> Scope:
        return self._env.scope(self._vars, self._inputs, payload)

    def _match(
        self, trigger: _Trigger, payload: _Payload, start: str | None = None
    ) -> tuple[State, Transition] | None:
        names = self._ancestry(start) if start is not None else reversed(self._active)
        for name in names:
            state = self._states[name]
            for transition in state.transitions:
                if trigger(transition) and self._allows(transition, payload):
                    return state, transition
        return None

    def _allows(self, transition: Transition, payload: _Payload) -> bool:
        conditions = [transition.when, transition.guard]
        scope = self._scope(payload)
        return all(
            self._env.compile(condition).test(scope)
            for condition in conditions
            if condition is not None
        )

    def _settle(self, payload: _Payload) -> None:
        for _ in range(MAX_CHAIN + 1):
            if self._failed_starts:
                goal_id, reason = self._failed_starts.pop(0)
                activity = self._activity(goal_id)
                if activity is not None:
                    self._finish(activity)
                    self._outcome(activity.state, "do.failed", {"reason": reason})
                continue
            match = self._match(lambda transition: transition.when is not None, payload)
            if match is None:
                return
            self._take(*match, payload)
        self._error = f"more than {MAX_CHAIN} transitions in a row in {self.active}"
        raise StateMachineError(self._error)

    def _take(self, source: State, transition: Transition, payload: _Payload) -> None:
        assignments = self._evaluate_set(transition.set, payload)
        if transition.to is None:
            self._vars |= assignments
            self._run(transition.run, payload)
            return
        domain = self._domain(source.name, transition.to)
        while self._active and self._active[-1] != domain:
            self._exit(self._active.pop(), payload)
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

    def _exit(self, name: str, payload: _Payload) -> None:
        activity = self._activities.pop(name, None)
        if activity is not None and activity.is_action:
            self._effects.append(CancelAction(activity.goal_id, activity.target))
        for timer_id in [
            timer_id for timer_id, timer in self._timers.items() if timer.state == name
        ]:
            del self._timers[timer_id]
            self._effects.append(ClearTimer(timer_id))
        self._run(self._states[name].exit, payload)

    def _enter(self, domain: str | None, target: str, payload: _Payload) -> None:
        path = self._ancestry(target)
        if domain is not None:
            path = path[: path.index(domain)]
        names = list(reversed(path))
        state = self._states[target]
        while state.initial is not None:
            state = self._states[state.initial]
            names.append(state.name)
        for name in names:
            self._active.append(name)
            self._run(self._states[name].entry, payload)
            self._start_activity(self._states[name], payload)
            for index, transition in enumerate(self._states[name].transitions):
                if transition.after is not None:
                    self._arm(name, index, transition.after)

    def _start_activity(self, state: State, payload: _Payload) -> None:
        if state.do is None:
            return
        goal_id = self._new_id()
        is_action = isinstance(state.do, ActionActivity)
        target = state.do.action if isinstance(state.do, ActionActivity) else state.do.call
        try:
            scope = self._scope(payload)
            args = {name: self._resolve(value, scope) for name, value in state.do.args.items()}
        except ExpressionError as error:
            self._failed_starts.append((goal_id, str(error)))
            args = None
        timer_id = None
        if args is not None:
            start: Effect = (
                StartAction(goal_id, target, args)
                if is_action
                else StartCall(goal_id, target, args, nowait=False)
            )
            self._effects.append(start)
            if state.do.timeout is not None:
                timer_id = self._arm(state.name, None, state.do.timeout)
        self._activities[state.name] = _Activity(goal_id, state.name, target, is_action, timer_id)

    def _arm(self, state: str, transition: int | None, seconds: float) -> str:
        timer_id = self._new_id()
        self._timers[timer_id] = _Timer(state, transition, seconds)
        self._effects.append(ArmTimer(timer_id, self._now + seconds))
        return timer_id

    def _evaluate_set(self, assignments: dict[str, JsonValue], payload: _Payload) -> _Payload:
        scope = self._scope(payload)
        return {
            var: self._env.compile(value).assign(scope, self._vars[var])
            if isinstance(value, str)
            else value
            for var, value in assignments.items()
        }

    def _run(self, steps: list[Step], payload: _Payload) -> None:
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
