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

"""The Orchestrator module."""

from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import replace
from functools import partial
import json
from pathlib import Path
import threading
import time

from pydantic import JsonValue

from dimos.core.core import rpc
from dimos.core.introspection.module.info import RpcInfo, StreamDescriptor
from dimos.core.module import Module, ModuleConfig
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In, Out, Transport
from dimos.msgs.std_msgs.String import String
from dimos.orchestration import engine
from dimos.orchestration.binding import Deployed, bind, resolve, streams
from dimos.orchestration.checks import CheckedDefinition
from dimos.orchestration.definition import OrchestratorDefinition, StreamValue
from dimos.orchestration.engine import (
    DEFAULT_LIMITS,
    AfterTimer,
    Call,
    ClearTimer,
    Effect,
    FaultReport,
    Limits,
    QueueItem,
    ReturnItem,
    Runtime,
    SendCancel,
    SendStart,
    SetTimer,
    StreamItem,
    Timer,
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.expressions import Plain
from dimos.orchestration.journal import EditItem, Journal, Json, definition_to_plain
from dimos.orchestration.loader import LoadError, check_source, load_and_check
from dimos.orchestration.plain import field_to_plain, from_plain, to_plain
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# A start may run for as long as its node is active, so its RPC gets no practical deadline.
_START_TIMEOUT_S = 365 * 24 * 3600.0


def _handled(item: QueueItem) -> str:
    """What a step handled, in words."""
    match item:
        case StreamItem(stream):
            return f"message on {stream}"
        case ReturnItem(call, _, raised):
            return f"{call.kind} of {call.rpc} {'raised' if raised else 'returned'}"
        case TimerItem(AfterTimer(node)):
            return f"after timer of {node}"
        case TimerItem(timer):
            return f"cancel deadline of {timer.node}"


def _state(checked: CheckedDefinition, runtime: Runtime, now: float) -> dict[str, Plain]:
    """The active nodes, the values that change, the RPCs still out and the timers."""
    constants = checked.definition.constants
    calls: list[Plain] = [
        {
            "node": call.node,
            "kind": call.kind,
            "phase": call.phase,
            "rpc": call.rpc,
            "args": dict(call.args),
        }
        for call in runtime.calls
    ]
    timers: list[Plain] = [
        {
            "node": timer.node,
            "kind": "after" if isinstance(timer, AfterTimer) else "cancel_deadline",
            "due_in_s": timer.due - now,
        }
        for timer in runtime.timers
    ]
    return {
        "active": [name for name in checked.tree.nodes if name in runtime.active],
        "blackboard": {k: v for k, v in runtime.blackboard.items() if k not in constants},
        "calls": calls,
        "timers": timers,
    }


class OrchestratorConfig(ModuleConfig):
    file: Path
    cancel_deadline_s: float = DEFAULT_LIMITS.cancel_deadline_s
    transition_cap: int = DEFAULT_LIMITS.transition_cap
    # How many engine steps `journal` keeps for tools that watch or rewind the orchestrator.
    journal_size: int = 10000


class Orchestrator(Module):
    """Runs the state machine of an orchestrator file by calling other modules' RPCs."""

    config: OrchestratorConfig

    orchestrator_state: Out[String]
    # Requests from an operator. The file names what each one does.
    orchestrator_request: In[String]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._definition: OrchestratorDefinition | None = None
        self._checked: CheckedDefinition | None = None
        self._rpcs: Mapping[str, RpcInfo] = {}
        # Each deployed module's RPC address, by the name the file uses for it.
        self._addresses: dict[str, str] = {}
        self._queue: deque[QueueItem] = deque()
        self._wake = threading.Condition()
        self._running = False
        self._thread: threading.Thread | None = None
        # Only the engine thread touches the timers.
        self._timers: list[Timer] = []
        self._transports: list[Transport[object]] = []
        self._unsubscribes: list[Callable[[], None]] = []
        self._unreadable: set[str] = set()
        self._published = 0
        self._journal = Journal(self.config.journal_size)
        self._modules: list[RPCClient] = []

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
        namespace = (self.config.instance_name or "").rpartition("/")[0]
        self._addresses = {module.instance_name: module.remote_name for module in modules}
        definition = resolve(definition, namespace, set(self._addresses))
        deployed, descriptors = self._describe(definition, modules)
        checked = bind(definition, deployed)
        if checked.problems:
            raise LoadError(self.config.file, checked.problems)
        self._modules = modules
        self._begin(definition, checked, deployed, descriptors)

    def _begin(
        self,
        definition: OrchestratorDefinition,
        checked: CheckedDefinition,
        deployed: Deployed,
        descriptors: dict[str, StreamDescriptor],
    ) -> None:
        """Subscribe to the signals' streams and start the engine thread on a checked definition."""
        self._checked = checked
        self._rpcs = deployed.rpcs
        for activity in definition.activities:
            self.rpc.rpc_timeouts[self._address(activity.start)] = _START_TIMEOUT_S
        with self._wake:
            self._running = True
        for stream, values in streams(definition).items():
            transport = descriptors[stream].transport.build()
            self._transports.append(transport)
            received = partial(self._on_message, stream, values)
            self._unsubscribes.append(transport.subscribe(received))
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

    @rpc
    def definition(self) -> Json:
        """The orchestrator file as plain JSON, with its path and its text."""
        definition = self._definition
        if definition is None:
            raise RuntimeError("the orchestrator has not loaded its file")
        return {
            "file": str(self.config.file),
            "source": self.config.file.read_text(),
            "definition": definition_to_plain(definition),
        }

    @rpc
    def state(self) -> Json:
        """The blackboard, active nodes, calls under way, timers and outcomes, as of step `seq`."""
        with self._wake:
            running = self._running
        return {**self._journal.snapshot(), "running": running}

    @rpc
    def journal(self, after: int = 0, limit: int = 500) -> Json:
        """Up to `limit` engine steps after step `after`: what came in, what changed, what was sent.

        `oldest` is the first step still kept, so a reader that asked for an older one missed some.
        """
        return self._journal.since(after, limit)

    @rpc
    def set_value(self, key: str, value: JsonValue) -> None:
        """Write a var or a stream value on the blackboard, as if its stream had published it.

        The engine handles it in turn with everything else, so its transitions fire as usual.
        """
        definition = self._definition
        if definition is None or self._checked is None:
            raise RuntimeError("the orchestrator is not running")
        if key in definition.constants:
            raise ValueError(f"`{key}` is a constant and cannot be set")
        settable = {*definition.vars, *(value.name for value in definition.stream_values)}
        if key not in settable:
            raise ValueError(f"`{key}` is not a var or stream value (they are: {sorted(settable)})")
        self._add(EditItem.of(key, value))

    @rpc
    def set_source(self, source: str) -> list[str]:
        """Replace the orchestrator file with `source` and run it from its start.

        The text is checked against the running blueprint first: if anything is wrong, nothing changes and the
        problems come back. Otherwise the activities under way are cancelled, the file is written, and the engine
        starts over with a fresh blackboard and journal (so a reader sees `seq` go back to 1).
        """
        if self._checked is None:
            raise RuntimeError("the orchestrator is not running")
        try:
            definition = check_source(source, self.config.file)
        except LoadError as error:
            return list(error.problems)
        deployed, descriptors = self._describe(definition, self._modules)
        checked = bind(definition, deployed)
        if checked.problems:
            return list(checked.problems)
        self._halt()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._cancel_all()
        self.config.file.write_text(source)
        self._definition = definition
        self._timers = []
        self._journal = Journal(self.config.journal_size)
        self._begin(definition, checked, deployed, descriptors)
        return []

    def _describe(
        self, definition: OrchestratorDefinition, modules: list[RPCClient]
    ) -> tuple[Deployed, dict[str, StreamDescriptor]]:
        """Ask each module the definition names for its RPCs or its streams."""
        clients = {module.instance_name: module for module in modules}
        targets = [t for a in definition.activities for t in (a.start, a.cancel) if t]
        rpcs: dict[str, RpcInfo] = {}
        for module in sorted({target.partition(".")[0] for target in targets} & clients.keys()):
            infos: list[RpcInfo] = clients[module].get_rpc_signatures()
            rpcs.update({f"{module}.{info.name}": info for info in infos})
        descriptors: dict[str, StreamDescriptor] = {}
        publishers = {stream.partition(".")[0] for stream in streams(definition)}
        for module in sorted(publishers & clients.keys()):
            found: list[StreamDescriptor] = clients[module].get_stream_descriptors()
            descriptors.update({f"{module}.{descriptor.name}": descriptor for descriptor in found})
        message_types = {name: descriptor.type for name, descriptor in descriptors.items()}
        return Deployed(frozenset(clients), rpcs, message_types), descriptors

    def _run(self, checked: CheckedDefinition) -> None:
        limits = Limits(self.config.cancel_deadline_s, self.config.transition_cap)
        runtime = Runtime({})
        try:
            now = time.monotonic()
            runtime, effects = engine.start(checked, now, limits)
            self._journal.record(None, runtime, effects)
            self._perform(effects)
            self._publish(checked, runtime, now, "startup", effects)
            while (item := self._next()) is not None:
                before, now = runtime, time.monotonic()
                runtime, effects = engine.step(checked, runtime, item, now, limits)
                self._journal.record(item, runtime, effects)
                self._perform(effects)
                # A step that only wrote stream values is not published.
                if effects or runtime.calls != before.calls or runtime.timers != before.timers:
                    self._publish(checked, runtime, now, _handled(item), effects)
        except Exception as error:
            logger.exception("Orchestrator engine stopped on an error")
            self._halt()
            self._cancel_all()
            stopped = replace(runtime, active=frozenset(), calls=(), timers=())
            rule = f"an engine error: {type(error).__name__}: {error}"
            self._publish(checked, stopped, time.monotonic(), "engine error", [], rule)

    def _publish(
        self,
        checked: CheckedDefinition,
        runtime: Runtime,
        now: float,
        handled: str,
        effects: list[Effect],
        error: str | None = None,
    ) -> None:
        """Publish what one step did, the state after it and the definition, as JSON."""
        self._published += 1
        transitions: list[Plain] = [
            {"node": effect.node, "cause": effect.cause, "goto": effect.goto}
            for effect in effects
            if isinstance(effect, TransitionRecord)
        ]
        faults: list[Plain] = [
            {"rule": effect.rule, "node": effect.node}
            for effect in effects
            if isinstance(effect, FaultReport)
        ]
        if error is not None:
            faults = [{"rule": error, "node": None}]
        message: dict[str, Plain] = {
            "seq": self._published,
            "handled": handled,
            "transitions": transitions,
            "fault": faults[0] if faults else None,
            **_state(checked, runtime, now),
            "definition": checked.definition.model_dump(mode="json", by_alias=True),
        }
        self.orchestrator_state.publish(String(json.dumps(message)))

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
                self._wake.wait(None if due is None else min(due - now, threading.TIMEOUT_MAX))
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
                case TransitionRecord(node, cause, goto):
                    logger.info("Orchestrator transition", node=node, cause=cause, goto=goto)
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

    def _address(self, target: str) -> str:
        """The RPC name of a target written `Module.rpc`, at that module's address."""
        module, _, rpc = target.partition(".")
        return f"{self._addresses.get(module, module)}/{rpc}"

    def _send(self, call: Call) -> None:
        with self._wake:
            if call.kind == "start" and not self._running:
                return
        name = self._address(call.rpc)
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

    def _on_message(self, stream: str, values: tuple[StreamValue, ...], message: object) -> None:
        """Queue one message with every stream value read from it. One that cannot be read drops it."""
        read: dict[str, Plain] = {}
        for value in values:
            try:
                read[value.name] = field_to_plain(message, value.field, value.name)
            except (AttributeError, TypeError) as error:
                if value.name not in self._unreadable:
                    self._unreadable.add(value.name)
                    logger.error(
                        "Orchestrator cannot read a stream value",
                        value=value.name,
                        error=str(error),
                    )
                return
        self._add(StreamItem(stream, read))

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
        for target in checked.tree.cancel_rpcs:
            self.rpc.call_nowait(self._address(target), ((), {}))
