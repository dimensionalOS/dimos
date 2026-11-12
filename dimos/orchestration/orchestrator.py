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

"""The Orchestrator module, and the checks of a definition against the running blueprint."""

from collections import deque
from collections.abc import Callable, Iterator, Mapping
from dataclasses import dataclass, replace
from functools import partial
from pathlib import Path
import threading
import time

from dimos.core.core import rpc
from dimos.core.introspection.module.info import RpcInfo, StreamDescriptor
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import Transport
from dimos.orchestration import engine
from dimos.orchestration.checks import CheckedDefinition, check
from dimos.orchestration.definition import OrchestratorDefinition, Signal
from dimos.orchestration.engine import (
    DEFAULT_LIMITS,
    Call,
    ClearTimer,
    Effect,
    FaultReport,
    Limits,
    QueueItem,
    ReturnItem,
    SendCancel,
    SendStart,
    SetTimer,
    SignalItem,
    Timer,
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.expressions import UNKNOWN, Shape
from dimos.orchestration.loader import LoadError, load_and_check
from dimos.orchestration.plain import field_to_plain, from_plain, shape_of, to_plain
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A start may run for as long as its node is active, so its RPC gets no practical deadline.
_START_TIMEOUT_S = 365 * 24 * 3600.0
_VARIADIC = ("VAR_POSITIONAL", "VAR_KEYWORD")
_BY_NAME = ("POSITIONAL_OR_KEYWORD", "KEYWORD_ONLY")


@dataclass(frozen=True)
class Deployed:
    """What the running blueprint offers the modules a definition names."""

    modules: frozenset[str]
    rpcs: Mapping[str, RpcInfo]
    """By `Module.rpc`."""
    streams: Mapping[str, type]
    """Message types by `Module.stream`."""


def bind(definition: OrchestratorDefinition, deployed: Deployed) -> CheckedDefinition:
    """Check a definition against the running blueprint. Its problems hold every one found."""
    shapes, shape_problems = _shapes(definition, deployed)
    parameters = {
        activity.name: {
            param.name: shape_of(param.annotation) for param in deployed.rpcs[activity.start].params
        }
        for activity in definition.activities
        if activity.start in deployed.rpcs
    }
    checked = check(definition, shapes, parameters)
    problems = [
        *_activity_problems(definition, deployed),
        *_args_problems(definition, deployed),
        *shape_problems,
        *checked.problems,
    ]
    return replace(checked, problems=tuple(dict.fromkeys(problems)))


def _topic(target: str) -> str:
    """The RPC name of a target written `Module.rpc`."""
    return target.replace(".", "/", 1)


def _missing(deployed: Deployed, target: str, offered: Mapping[str, object], kind: str) -> str:
    """Why the blueprint does not offer a `Module.name` target. Empty when it does."""
    module, _, name = target.partition(".")
    if module not in deployed.modules:
        return f"names `{target}`, and `{module}` is not a deployed module"
    if target not in offered:
        return f"names `{target}`, and `{module}` has no {kind} `{name}`"
    return ""


def _required(info: RpcInfo) -> list[str]:
    return [p.name for p in info.params if p.default is None and p.kind not in _VARIADIC]


def _activity_problems(definition: OrchestratorDefinition, deployed: Deployed) -> Iterator[str]:
    for activity in definition.activities:
        for field, target in (("start", activity.start), ("cancel", activity.cancel)):
            missing = _missing(deployed, target, deployed.rpcs, "RPC") if target else ""
            if missing:
                yield f"activity `{activity.name}`: `{field}` {missing}"
        cancel = deployed.rpcs.get(activity.cancel or "")
        for name in _required(cancel) if cancel is not None else ():
            yield (
                f"activity `{activity.name}`: `cancel` RPC `{activity.cancel}` needs `{name}`, "
                "and a cancel is called with no arguments"
            )


def _args_problems(definition: OrchestratorDefinition, deployed: Deployed) -> Iterator[str]:
    starts = {activity.name: activity.start for activity in definition.activities}
    for node in definition.nodes:
        target = starts.get(node.activity or "", "")
        info = deployed.rpcs.get(target)
        if info is None:
            continue
        named = {p.name for p in info.params if p.kind in _BY_NAME}
        takes_any = any(p.kind == "VAR_KEYWORD" for p in info.params)
        for name in node.args:
            if name not in named and not takes_any:
                yield (
                    f"node `{node.name}`: `args` names `{name}`, "
                    f"which is not a parameter of `{target}`"
                )
        for name in _required(info):
            if name not in node.args:
                yield f"node `{node.name}`: `args` has no entry for `{name}`, which `{target}` needs"


def _shapes(
    definition: OrchestratorDefinition, deployed: Deployed
) -> tuple[dict[str, Shape], list[str]]:
    """The shape of every signal and result key, and what is wrong with any of them."""
    shapes: dict[str, Shape] = {}
    problems: list[str] = []
    for signal in definition.signals:
        missing = _missing(deployed, signal.stream, deployed.streams, "stream")
        if missing:
            problems.append(f"signal `{signal.name}`: `stream` {missing}")
            continue
        message = deployed.streams[signal.stream]
        shape = shape_of(message)
        if signal.field is not None and isinstance(shape, dict) and signal.field not in shape:
            problems.append(
                f"signal `{signal.name}`: `field` names `{signal.field}`, "
                f"which is not a field of `{message.__name__}`"
            )
        elif signal.field is not None:
            shape = shape[signal.field] if isinstance(shape, dict) else UNKNOWN
        shapes[signal.name] = shape

    starts = {activity.name: activity.start for activity in definition.activities}
    writers: dict[str, tuple[str, Shape]] = {}
    for node in definition.nodes:
        info = deployed.rpcs.get(starts.get(node.activity or "", ""))
        if node.result is None or info is None:
            continue
        shape = shape_of(info.return_annotation)
        first, earlier = writers.setdefault(node.result, (node.name, shape))
        if earlier is not UNKNOWN and shape is not UNKNOWN and repr(earlier) != repr(shape):
            problems.append(
                f"key `{node.result}` is written by `{first}` and `{node.name}`, "
                "and their `start` RPCs return different shapes"
            )
    shapes.update({key: shape for key, (_, shape) in writers.items()})
    return shapes, problems


class OrchestratorConfig(ModuleConfig):
    file: Path
    cancel_deadline_s: float = DEFAULT_LIMITS.cancel_deadline_s
    transition_cap: int = DEFAULT_LIMITS.transition_cap


class Orchestrator(Module):
    """Runs the state machine of an orchestrator file by calling other modules' RPCs."""

    config: OrchestratorConfig

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._definition: OrchestratorDefinition | None = None
        self._checked: CheckedDefinition | None = None
        self._rpcs: Mapping[str, RpcInfo] = {}
        self._queue: deque[QueueItem] = deque()
        self._wake = threading.Condition()
        self._running = False
        self._thread: threading.Thread | None = None
        # Only the engine thread touches the timers.
        self._timers: list[Timer] = []
        self._transports: list[Transport[object]] = []
        self._unsubscribes: list[Callable[[], None]] = []
        self._unreadable: set[str] = set()

    @rpc
    def start(self) -> None:
        super().start()
        self._definition = load_and_check(self.config.file)

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        """Check the file against the running blueprint, then start the engine. Runs once."""
        definition = self._definition
        if definition is None or self._checked is not None:
            logger.info("Orchestrator ignores a change to the system's modules")
            return
        deployed, descriptors = self._describe(definition, modules)
        checked = bind(definition, deployed)
        if checked.problems:
            raise LoadError(self.config.file, checked.problems)

        self._checked = checked
        self._rpcs = deployed.rpcs
        for activity in definition.activities:
            self.rpc.rpc_timeouts[_topic(activity.start)] = _START_TIMEOUT_S
        with self._wake:
            self._running = True
        for signal in definition.signals:
            transport = descriptors[signal.stream].transport.build()
            self._transports.append(transport)
            self._unsubscribes.append(transport.subscribe(partial(self._on_message, signal)))
        self._thread = threading.Thread(
            target=self._run, args=(checked,), name="orchestrator-engine", daemon=True
        )
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._halt()
        # The engine thread ends first, so no start it is sending follows the cancels.
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._cancel_all()
        super().stop()

    def _describe(
        self, definition: OrchestratorDefinition, modules: list[RPCClient]
    ) -> tuple[Deployed, dict[str, StreamDescriptor]]:
        """Ask each module the definition names for its RPCs or its streams."""
        clients = {module.remote_name: module for module in modules}
        targets = [t for a in definition.activities for t in (a.start, a.cancel) if t]
        rpcs: dict[str, RpcInfo] = {}
        for module in sorted({target.partition(".")[0] for target in targets} & clients.keys()):
            infos: list[RpcInfo] = clients[module].get_rpc_signatures()
            rpcs.update({f"{module}.{info.name}": info for info in infos})
        descriptors: dict[str, StreamDescriptor] = {}
        streams = {signal.stream.partition(".")[0] for signal in definition.signals}
        for module in sorted(streams & clients.keys()):
            found: list[StreamDescriptor] = clients[module].get_stream_descriptors()
            descriptors.update({f"{module}.{descriptor.name}": descriptor for descriptor in found})
        message_types = {name: descriptor.type for name, descriptor in descriptors.items()}
        return Deployed(frozenset(clients), rpcs, message_types), descriptors

    def _run(self, checked: CheckedDefinition) -> None:
        limits = Limits(self.config.cancel_deadline_s, self.config.transition_cap)
        try:
            runtime, effects = engine.start(checked, time.monotonic(), limits)
            self._perform(effects)
            while (item := self._next()) is not None:
                runtime, effects = engine.step(checked, runtime, item, time.monotonic(), limits)
                self._perform(effects)
        except Exception:
            logger.exception("Orchestrator engine stopped on an error")
            self._halt()
            self._cancel_all()

    def _next(self) -> QueueItem | None:
        """The next queue item, after adding the timers that are due. None once stopped."""
        with self._wake:
            while self._running:
                now = time.monotonic()
                for timer in sorted(self._timers, key=lambda timer: timer.due):
                    if timer.due <= now:
                        self._timers.remove(timer)
                        engine.enqueue(self._queue, TimerItem(timer))
                if self._queue:
                    return self._queue.popleft()
                due = min((timer.due for timer in self._timers), default=None)
                self._wake.wait(None if due is None else due - now)
            return None

    def _add(self, item: QueueItem) -> None:
        with self._wake:
            if self._running:
                engine.enqueue(self._queue, item)
                self._wake.notify()

    def _perform(self, effects: list[Effect]) -> None:
        for effect in effects:
            match effect:
                case SendStart(call) | SendCancel(call):
                    self._send(call)
                case SetTimer(timer):
                    self._timers.append(timer)
                case ClearTimer(timer):
                    if timer in self._timers:
                        self._timers.remove(timer)
                case TransitionRecord(node, trigger, goto):
                    logger.info("Orchestrator transition", node=node, trigger=trigger, goto=goto)
                case FaultReport(rule, node, event, blackboard):
                    # The logger keeps the keyword event for its own message.
                    logger.error(
                        "Orchestrator fault",
                        rule=rule,
                        node=node,
                        handling=event,
                        blackboard=dict(blackboard),
                    )
                    self._halt()

    def _send(self, call: Call) -> None:
        with self._wake:
            if call.kind == "start" and not self._running:
                return
        name = _topic(call.rpc)
        annotations = {param.name: param.annotation for param in self._rpcs[call.rpc].params}
        try:
            arguments = {
                key: from_plain(value, annotations.get(key, object))
                for key, value in call.args.items()
            }
            if call.node is None:
                # A cancel sent by a fault. The engine ignores its return.
                self.rpc.call_nowait(name, ((), arguments))
            else:
                self.rpc.call(name, ((), arguments), partial(self._returned, call))
        except Exception as error:
            logger.error("Orchestrator could not send an RPC", rpc=call.rpc, error=str(error))
            self._add(ReturnItem(call, raised=True))

    def _returned(self, call: Call, value: object) -> None:
        checked = self._checked
        wanted = (
            checked is not None
            and call.kind == "start"
            and call.node is not None
            and checked.tree.nodes[call.node].result is not None
        )
        if isinstance(value, BaseException):
            logger.warning("Orchestrator RPC raised", rpc=call.rpc, error=str(value))
            self._add(ReturnItem(call, raised=True))
        elif not wanted:
            self._add(ReturnItem(call))
        else:
            try:
                self._add(ReturnItem(call, to_plain(value)))
            except TypeError as error:
                logger.error("Orchestrator cannot keep a result", rpc=call.rpc, error=str(error))
                self._add(ReturnItem(call, raised=True))

    def _on_message(self, signal: Signal, message: object) -> None:
        try:
            if signal.field is None:
                value = to_plain(message, signal.name)
            else:
                value = field_to_plain(message, signal.field, signal.name)
            item = SignalItem(signal.name, value)
        except (AttributeError, TypeError) as error:
            if signal.name not in self._unreadable:
                self._unreadable.add(signal.name)
                logger.error(
                    "Orchestrator cannot read a signal", signal=signal.name, error=str(error)
                )
            return
        self._add(item)

    def _halt(self) -> None:
        """Take no more items and drop every stream subscription."""
        with self._wake:
            self._running = False
            self._queue.clear()
            self._wake.notify_all()
            unsubscribes, self._unsubscribes = self._unsubscribes, []
            transports, self._transports = self._transports, []
        for unsubscribe in unsubscribes:
            unsubscribe()
        for transport in transports:
            transport.stop()

    def _cancel_all(self) -> None:
        """Send the cancel of every activity without waiting, as a fault does."""
        checked = self._checked
        if checked is None:
            return
        cancels = [activity.cancel for activity in checked.tree.activities.values()]
        for target in dict.fromkeys(cancel for cancel in cancels if cancel is not None):
            self.rpc.call_nowait(_topic(target), ((), {}))
