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

"""A dimos module that runs a state machine file."""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Iterable
from concurrent.futures import ThreadPoolExecutor
from functools import partial
import importlib
import json
import os
from pathlib import Path
import re
import time
from typing import TypeVar
import uuid

from dimos_lcm.actionlib_msgs import GoalStatus
import numpy as np
from pydantic import Field, JsonValue, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.agents.annotation import skill
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import Blueprint
from dimos.core.core import rpc
from dimos.core.dynamic_module import (
    MethodSpec,
    PortSpec,
    RefSpec,
    make_module_class,
    make_spec_class,
)
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.std_msgs.String import String
from dimos.orchestration.definition import (
    ActionActivity,
    CallActivity,
    CallStep,
    DefinitionError,
    StateMachineDefinition,
    load_definition,
)
from dimos.orchestration.engine import (
    ActionResult,
    ArmTimer,
    CallReply,
    CancelAction,
    ClearTimer,
    Effect,
    Engine,
    Event,
    ExternalEvent,
    InputMessage,
    Publish,
    Snapshot,
    StartAction,
    StartCall,
    StateMachineError,
    TimerFired,
)
from dimos.orchestration.expressions import kind_of
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_T = TypeVar("_T")

MACHINES_DIR = STATE_DIR / "machines"

_MESSAGE_TYPE = re.compile(r"([A-Za-z_]\w*)\.([A-Za-z_]\w*)")
_FINAL_STATUS = {
    GoalStatus.SUCCEEDED: "succeeded",
    GoalStatus.ABORTED: "failed",
    GoalStatus.REJECTED: "failed",
    GoalStatus.PREEMPTED: "cancelled",
    GoalStatus.RECALLED: "cancelled",
}


class StateMachineConfig(ModuleConfig):
    definition: StateMachineDefinition
    state_file: Path | None = None
    params: dict[str, JsonValue] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _check_params(self) -> Self:
        defaults = self.definition.params
        for name, value in self.params.items():
            if name not in defaults:
                raise ValueError(f"params: unknown param {name!r}")
            if kind_of(value) != kind_of(defaults[name]):
                raise ValueError(
                    f"params: {name} is {kind_of(value)}, expected {kind_of(defaults[name])}"
                )
        return self


class StateMachineModule(Module):
    """Runs one state machine definition, with ports and role refs generated from its file."""

    config: StateMachineConfig

    action_result: In[GoalStatus]
    machine_state: Out[String]

    _engine: Engine
    _executor: ThreadPoolExecutor | None = None
    _timers: dict[str, asyncio.TimerHandle]
    _snapshot: Snapshot | None
    _entered: float
    _status: str

    @rpc
    def start(self) -> None:
        super().start()
        definition = self.config.definition
        params = {**definition.params, **self.config.params}
        self._engine = Engine(
            definition.model_copy(update={"params": params}),
            f"{definition.state_machine.name}-{uuid.uuid4().hex[:8]}",
        )
        self._executor = ThreadPoolExecutor(thread_name_prefix=definition.state_machine.name)
        self._timers = {}
        self._snapshot = None
        self._status = ""
        asyncio.run_coroutine_threadsafe(self._begin(), self._running_loop()).result()
        for name in definition.inputs:
            self._subscribe(getattr(self, name), partial(_input, name))
        self._subscribe(self.action_result, _action_result)

    @rpc
    def stop(self) -> None:
        for handle in getattr(self, "_timers", {}).values():
            handle.cancel()
        super().stop()
        if self._executor is not None:
            self._executor.shutdown(wait=False, cancel_futures=True)

    @rpc
    def send_event(self, name: str, data: dict[str, str | float | bool] | None = None) -> str:
        """Send a declared external event and return the status after it is handled."""
        events = self.config.definition.events
        if name not in events:
            return f"Unknown event {name!r}. Declared events: {', '.join(events) or 'none'}"
        event = ExternalEvent(name, dict(data or {}))
        asyncio.run_coroutine_threadsafe(self._send(event), self._running_loop()).result()
        return self.status()

    @rpc
    def status(self) -> str:
        """The active path, vars, last event and time entered, as JSON."""
        return self._status

    @skill
    def machine_status(self) -> str:
        """Show the state machine's active states, vars and last event as JSON."""
        return self.status()

    @skill
    def send_machine_event(self, name: str) -> str:
        """Send an external event, such as pause or resume, to the state machine.

        Args:
            name: One of the events the state machine file declares.
        """
        return self.send_event(name)

    def _running_loop(self) -> asyncio.AbstractEventLoop:
        if self._loop is None or not self._loop.is_running():
            raise RuntimeError(f"{type(self).__name__}._loop is not running")
        return self._loop

    def _state_file(self) -> Path:
        name = self.config.definition.state_machine.name
        return self.config.state_file or MACHINES_DIR / f"{name}.json"

    async def _begin(self) -> None:
        saved = self._load()
        now = self._running_loop().time()
        effects = self._engine.start(now) if saved is None else self._engine.restore(saved, now)
        self._settle("start", effects)

    async def _send(self, event: Event) -> None:
        self._handle(event)

    def _subscribe(self, stream: In[_T], convert: Callable[[_T], Event | None]) -> None:
        def on_message(msg: _T) -> None:
            event = convert(msg)
            loop = self._loop
            if event is not None and loop is not None and loop.is_running():
                loop.call_soon_threadsafe(self._handle, event)

        self.register_disposable(Disposable(stream.subscribe(on_message)))

    def _handle(self, event: Event) -> None:
        try:
            effects = self._engine.handle(event, self._running_loop().time())
        except (StateMachineError, ValueError) as error:
            logger.error("State machine dropped an event", event=event, error=str(error))
            return
        self._settle(_describe(event), effects)

    def _settle(self, last_event: str, effects: list[Effect]) -> None:
        for effect in effects:
            self._apply(effect)
        snapshot = self._engine.snapshot()
        if snapshot == self._snapshot:
            return
        if self._snapshot is None or snapshot.active != self._snapshot.active:
            self._entered = time.time()
        self._snapshot = snapshot
        self._save(snapshot)
        self._status = json.dumps(
            {
                "active": list(snapshot.active),
                "vars": snapshot.vars,
                "last_event": last_event,
                "entered": self._entered,
            }
        )
        self.machine_state.publish(String(self._status))

    def _apply(self, effect: Effect) -> None:
        loop = self._running_loop()
        if isinstance(effect, Publish):
            output = getattr(self, effect.output)
            output.publish(_message(output.type, effect.value))
        elif isinstance(effect, StartCall):
            call = self._method(effect.target)
            args = effect.args
            if effect.nowait:
                self._run(lambda: call(**args), None, None)
            else:
                goal_id = effect.goal_id
                self._run(
                    lambda: call(**args),
                    lambda result: CallReply(goal_id, True, _plain(result)),
                    lambda error: CallReply(goal_id, False, reason=str(error)),
                )
        elif isinstance(effect, StartAction):
            start = self._method(effect.target)
            goal_id, args = effect.goal_id, effect.args
            self._run(
                lambda: start(goal_id=goal_id, **args),
                None,
                lambda error: ActionResult(goal_id, "failed", f"rejected: {error}"),
            )
        elif isinstance(effect, CancelAction):
            cancel = self._method(f"{effect.target.split('.')[0]}.cancel_action")
            goal_id = effect.goal_id
            self._run(lambda: cancel(goal_id=goal_id), None, None)
        elif isinstance(effect, ArmTimer):
            self._timers[effect.timer_id] = loop.call_at(
                effect.deadline, self._fire, effect.timer_id
            )
        elif isinstance(effect, ClearTimer):
            handle = self._timers.pop(effect.timer_id, None)
            if handle is not None:
                handle.cancel()

    def _fire(self, timer_id: str) -> None:
        self._timers.pop(timer_id, None)
        self._handle(TimerFired(timer_id))

    def _method(self, target: str) -> Callable[..., object]:
        role, method = target.split(".")
        return getattr(getattr(self, role), method)  # type: ignore[no-any-return]

    def _run(
        self,
        call: Callable[[], object],
        on_reply: Callable[[object], Event] | None,
        on_error: Callable[[Exception], Event] | None,
    ) -> None:
        def done(future: asyncio.Future[object]) -> None:
            try:
                result = future.result()
            except Exception as error:
                if on_error is None:
                    logger.warning("State machine call failed", error=str(error))
                else:
                    self._handle(on_error(error))
                return
            if on_reply is not None:
                self._handle(on_reply(result))

        self._running_loop().run_in_executor(self._executor, call).add_done_callback(done)

    def _load(self) -> Snapshot | None:
        path = self._state_file()
        if not path.exists():
            return None
        try:
            data = json.loads(path.read_text())
            return Snapshot(dict(data["vars"]), tuple(data["active"]))
        except (OSError, ValueError, KeyError, TypeError) as error:
            logger.warning("Ignoring an unreadable state file", path=str(path), error=str(error))
            return None

    def _save(self, snapshot: Snapshot) -> None:
        path = self._state_file()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f"{path.name}.tmp")
        temporary.write_text(json.dumps({"vars": snapshot.vars, "active": list(snapshot.active)}))
        os.replace(temporary, path)


def _plain(value: object) -> JsonValue:
    if value is None or isinstance(value, bool | int | float | str):
        return value
    if isinstance(value, np.ndarray):
        return _plain(value.tolist())
    if isinstance(value, np.generic):
        return _plain(value.item())
    if isinstance(value, list | tuple):
        return [_plain(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    slots = getattr(type(value), "__slots__", None)
    names = list(slots) if slots is not None else list(vars(value))
    return {name: _plain(getattr(value, name)) for name in names if not name.startswith("_")}


def _message(message_type: type, value: JsonValue) -> object:
    return message_type(**value) if isinstance(value, dict) else message_type(value)


def _input(name: str, msg: object) -> InputMessage:
    fields = _plain(msg)
    return InputMessage(name, fields if isinstance(fields, dict) else {"data": fields})


def _action_result(msg: GoalStatus) -> ActionResult | None:
    status = _FINAL_STATUS.get(msg.status)
    if status is None:
        return None
    return ActionResult(msg.goal_id.id, status, msg.text)  # type: ignore[arg-type]


def _describe(event: Event) -> str:
    if isinstance(event, InputMessage | ExternalEvent):
        return event.name
    if isinstance(event, ActionResult):
        return f"action {event.status}"
    if isinstance(event, CallReply):
        return "call succeeded" if event.ok else "call failed"
    return "timer"


def message_type(name: str) -> type:
    """Resolve a message type name like std_msgs.String, from dimos.msgs first, then dimos_lcm."""
    match = _MESSAGE_TYPE.fullmatch(name)
    if match is None:
        raise DefinitionError(f"expected a message type like std_msgs.String, got {name!r}")
    package, type_name = match.groups()
    for module_name in (f"dimos.msgs.{package}.{type_name}", f"dimos_lcm.{package}"):
        try:
            module = importlib.import_module(module_name)
        except ImportError:
            continue
        found = getattr(module, type_name, None)
        if isinstance(found, type):
            return found
    raise DefinitionError(f"unknown message type {name!r}")


def _role_methods(definition: StateMachineDefinition) -> dict[str, dict[str, set[str]]]:
    roles: dict[str, dict[str, set[str]]] = {}

    def add(target: str, params: Iterable[str]) -> None:
        role, method = target.split(".")
        roles.setdefault(role, {}).setdefault(method, set()).update(params)

    for state in definition.states:
        steps = [*state.entry, *state.exit, *(step for t in state.transitions for step in t.run)]
        for step in steps:
            if isinstance(step, CallStep):
                add(step.call, step.args)
        if isinstance(state.do, CallActivity):
            add(state.do.call, state.do.args)
        elif isinstance(state.do, ActionActivity):
            add(state.do.action, [*state.do.args, "goal_id"])
            add(f"{state.do.action.split('.')[0]}.cancel_action", ["goal_id"])
    return roles


def _refs(definition: StateMachineDefinition) -> list[RefSpec]:
    refs: list[RefSpec] = []
    for role, methods in _role_methods(definition).items():
        spec_name = "".join(part.title() for part in role.split("_")) + "Role"
        method_specs = [MethodSpec(name, tuple(sorted(params))) for name, params in methods.items()]
        refs.append(RefSpec(role, make_spec_class(spec_name, method_specs)))
    return refs


def _ports(definition: StateMachineDefinition) -> list[PortSpec]:
    return [
        PortSpec(name, message_type(type_name), "in")
        for name, type_name in definition.inputs.items()
    ] + [
        PortSpec(name, message_type(type_name), "out")
        for name, type_name in definition.outputs.items()
    ]


def state_machine(path: Path | str) -> Blueprint:
    """Load a state machine file and return a blueprint that runs it."""
    definition = load_definition(path)
    try:
        cls = make_module_class(StateMachineModule, _ports(definition), refs=_refs(definition))
    except (DefinitionError, TypeError, ValueError) as error:
        raise DefinitionError(f"{path}: {error}") from error
    return cls.blueprint(definition=definition, instance_name=definition.state_machine.name)
