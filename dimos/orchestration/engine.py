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

"""The engine: a function from a runtime and one queue item to a new runtime and effects."""

from collections import deque
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Literal

from dimos.orchestration.checks import CheckedDefinition
from dimos.orchestration.definition import Activity, Node, Transition
from dimos.orchestration.expressions import ExpressionError, Plain, evaluate, evaluate_bool


@dataclass(frozen=True)
class Call:
    """One RPC that has not returned."""

    id: int
    node: str
    kind: Literal["start", "cancel"]
    phase: Literal["held", "sent"]
    uses: frozenset[str]


@dataclass(frozen=True)
class Timer:
    node: str
    transition: int
    due: float


@dataclass(frozen=True)
class Pending:
    leaving: str
    goto: str
    waiting: frozenset[str]


@dataclass(frozen=True)
class QueueItem:
    kind: Literal["signal", "start_return", "cancel_return", "timer"]
    call: Call | None = None
    timer: Timer | None = None
    key: str | None = None
    value: Plain = None
    raised: bool = False


@dataclass(frozen=True)
class Runtime:
    blackboard: Mapping[str, Plain]
    active: frozenset[str] = frozenset()
    calls: tuple[Call, ...] = ()
    timers: tuple[Timer, ...] = ()
    pending: tuple[Pending, ...] = ()
    calls_made: int = 0


@dataclass(frozen=True)
class SendStart:
    call: Call
    rpc: str
    args: Mapping[str, Plain]


@dataclass(frozen=True)
class SendCancel:
    call: Call
    rpc: str


@dataclass(frozen=True)
class SetTimer:
    timer: Timer


@dataclass(frozen=True)
class ClearTimer:
    timer: Timer


@dataclass(frozen=True)
class TransitionRecord:
    """One transition taken. `trigger` is None for a switch node's."""

    node: str
    trigger: str | None
    goto: str


@dataclass(frozen=True)
class FaultReport:
    rule: str
    node: str | None
    event: str
    blackboard: Mapping[str, Plain]


Effect = SendStart | SendCancel | SetTimer | ClearTimer | TransitionRecord | FaultReport


def start(checked: CheckedDefinition, now: float) -> tuple[Runtime, list[Effect]]:
    """Enter the root of a definition that has no problems."""
    if checked.problems:
        raise ValueError(f"the definition has {len(checked.problems)} problems")
    definition = checked.definition
    work = _Step(checked, Runtime({**definition.constants, **definition.vars}), now)
    return work.run("startup", lambda: work.enter(work.tree.nodes[definition.root]))


def step(
    checked: CheckedDefinition, runtime: Runtime, item: QueueItem, now: float
) -> tuple[Runtime, list[Effect]]:
    """Handle one queue item."""
    work = _Step(checked, runtime, now)
    return work.run(item.kind, lambda: work.handle(item))


class _FaultError(Exception):
    def __init__(self, rule: str, node: Node) -> None:
        super().__init__(rule)
        self.rule = rule
        self.node = node.name


class _Step:
    """The state of one step, changed in place and frozen into a Runtime at the end."""

    def __init__(self, checked: CheckedDefinition, runtime: Runtime, now: float) -> None:
        self.expressions = checked.expressions
        self.tree = checked.tree
        self.now = now
        self.blackboard = dict(runtime.blackboard)
        self.active = set(runtime.active)
        self.calls = list(runtime.calls)
        self.timers = list(runtime.timers)
        self.calls_made = runtime.calls_made
        self.effects: list[Effect] = []
        self.raised: deque[tuple[str, Node]] = deque()

    def run(self, event: str, first: Callable[[], None]) -> tuple[Runtime, list[Effect]]:
        try:
            first()
            self.settle()
        except _FaultError as fault:
            self.active.clear()
            self.calls.clear()
            self.timers.clear()
            self.effects.append(FaultReport(fault.rule, fault.node, event, dict(self.blackboard)))
        runtime = Runtime(
            self.blackboard,
            frozenset(self.active),
            tuple(self.calls),
            tuple(self.timers),
            (),
            self.calls_made,
        )
        return runtime, self.effects

    def handle(self, item: QueueItem) -> None:
        if item.kind == "timer":
            self.handle_timer(item.timer)
        elif item.kind in ("start_return", "cancel_return"):
            self.handle_return(item)

    def handle_timer(self, timer: Timer | None) -> None:
        if timer is None or timer not in self.timers:
            return
        self.timers.remove(timer)
        node = self.tree.nodes[timer.node]
        transition = node.transitions[timer.transition]
        if self.passes(node, transition):
            self.take(node, transition, "after")

    def handle_return(self, item: QueueItem) -> None:
        returned = item.call.id if item.call is not None else None
        call = next((call for call in self.calls if call.id == returned), None)
        if call is None:
            return
        self.calls.remove(call)
        if call.kind == "cancel":
            return
        node = self.tree.nodes[call.node]
        if item.raised:
            self.raised.append(("failed", node))
            return
        if node.result is not None:
            self.blackboard[node.result] = item.value
        if node.inside:
            self.enter_inside(node)
        else:
            self.raised.append(("succeeded", node))

    def settle(self) -> None:
        """Handle every event the engine raised, in the order raised."""
        while self.raised:
            event, node = self.raised.popleft()
            if node.name not in self.active:
                continue
            if event == "switch":
                self.offer(node, None)
            elif event == "failed":
                self.fail(node)
            else:
                self.offer(node, event)

    def offer(self, node: Node, trigger: str | None) -> bool:
        """Take the first transition of a node on a trigger whose `if` passes."""
        for transition in node.transitions:
            if transition.trigger != trigger or transition.after is not None:
                continue
            if self.passes(node, transition):
                self.take(node, transition, trigger)
                return True
        return False

    def fail(self, node: Node) -> None:
        for candidate in (node, *self.tree.ancestors(node)):
            if self.offer(candidate, "failed"):
                return
        raise _FaultError("a `failed` that no node handles", node)

    def take(self, node: Node, transition: Transition, trigger: str | None) -> None:
        values = {var: self.evaluate(node, source) for var, source in transition.set.items()}
        self.blackboard.update(values)
        self.effects.append(TransitionRecord(node.name, trigger, transition.goto))
        self.leave_inside(node)
        self.complete(node, transition.goto)

    def complete(self, node: Node, goto: str) -> None:
        """Finish a transition once everything inside its node has been left."""
        self.leave(node)
        if goto in self.tree.nodes:
            self.enter(self.tree.nodes[goto])
            return
        parent = self.tree.parent[node.name]
        if goto == "failed" or not any(name in self.active for name in parent.parallel):
            self.raised.append((goto, parent))

    def leave_inside(self, node: Node) -> None:
        for name in node.inside:
            if name in self.active:
                self.leave_inside(self.tree.nodes[name])
                self.leave(self.tree.nodes[name])

    def leave(self, node: Node) -> None:
        self.active.discard(node.name)
        self.calls = [c for c in self.calls if c.node != node.name or c.kind != "start"]
        activity = self.tree.activity(node)
        if activity is not None and activity.cancel is not None:
            self.effects.append(SendCancel(self.call(node, "cancel", activity), activity.cancel))

    def enter(self, node: Node) -> None:
        self.active.add(node.name)
        activity = self.tree.activity(node)
        if activity is None:
            self.enter_inside(node)
            return
        args = {name: self.evaluate(node, source) for name, source in node.args.items()}
        if node.result is not None:
            self.blackboard.pop(node.result, None)
        self.effects.append(SendStart(self.call(node, "start", activity), activity.start, args))

    def enter_inside(self, node: Node) -> None:
        if node.initial is not None:
            self.enter(self.tree.nodes[node.initial])
        for name in node.parallel:
            self.enter(self.tree.nodes[name])
        if node.is_switch:
            self.raised.append(("switch", node))

    def call(self, node: Node, kind: Literal["start", "cancel"], activity: Activity) -> Call:
        self.calls_made += 1
        call = Call(self.calls_made, node.name, kind, "sent", frozenset(activity.uses))
        self.calls.append(call)
        return call

    def evaluate(self, node: Node, source: str) -> Plain:
        try:
            return evaluate(self.expressions[source], self.blackboard)
        except ExpressionError as error:
            raise _FaultError(f"an expression error: {error}", node) from None

    def passes(self, node: Node, transition: Transition) -> bool:
        if transition.if_ is None:
            return True
        try:
            return evaluate_bool(self.expressions[transition.if_], self.blackboard)
        except ExpressionError as error:
            raise _FaultError(f"an expression error: {error}", node) from None
