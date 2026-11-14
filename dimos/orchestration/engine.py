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
from collections.abc import Callable, Mapping, MutableSequence
from dataclasses import dataclass, replace
import math
from typing import Literal

from dimos.orchestration.checks import CheckedDefinition
from dimos.orchestration.definition import Check, Node, Outcome, Transition
from dimos.orchestration.expressions import ExpressionError, Plain, evaluate, evaluate_bool

CallKind = Literal["start", "cancel"]
CallPhase = Literal["held", "sent"]
EngineEvent = Outcome | Literal["succeeded"]


@dataclass(frozen=True)
class Call:
    """One RPC that has not returned. `node` is None for a cancel sent by a fault."""

    id: int
    node: str | None
    kind: CallKind
    phase: CallPhase
    uses: frozenset[str]
    rpc: str
    args: Mapping[str, Plain]


@dataclass(frozen=True)
class AfterTimer:
    """The clock of one `after` transition, by its place on the node."""

    node: str
    due: float
    transition: int


@dataclass(frozen=True)
class CancelDeadline:
    """When the cancel sent on leaving a node must have returned."""

    node: str
    due: float
    call_id: int


Timer = AfterTimer | CancelDeadline


@dataclass(frozen=True)
class SignalItem:
    key: str
    value: Plain = None


@dataclass(frozen=True)
class ReturnItem:
    """A start or a cancel that returned `value`, or raised."""

    call: Call
    value: Plain = None
    raised: bool = False


@dataclass(frozen=True)
class TimerItem:
    timer: Timer


QueueItem = SignalItem | ReturnItem | TimerItem


@dataclass(frozen=True)
class Runtime:
    blackboard: Mapping[str, Plain]
    active: frozenset[str] = frozenset()
    calls: tuple[Call, ...] = ()
    timers: tuple[Timer, ...] = ()
    calls_made: int = 0


@dataclass(frozen=True)
class Limits:
    cancel_deadline_s: float = 2.0
    transition_cap: int = 100


@dataclass(frozen=True)
class SendStart:
    call: Call


@dataclass(frozen=True)
class SendCancel:
    call: Call


@dataclass(frozen=True)
class SetTimer:
    timer: Timer


@dataclass(frozen=True)
class ClearTimer:
    timer: Timer


@dataclass(frozen=True)
class TransitionRecord:
    """One transition taken, or one redirect to a fallback. `trigger` is None for a switch,
    `after` for a timer and the check kind for a redirect.
    """

    node: str
    trigger: str | None
    goto: str


@dataclass(frozen=True)
class FaultReport:
    rule: str
    node: str
    event: str
    blackboard: Mapping[str, Plain]


Effect = SendStart | SendCancel | SetTimer | ClearTimer | TransitionRecord | FaultReport
DEFAULT_LIMITS = Limits()


def enqueue(items: MutableSequence[QueueItem], item: QueueItem) -> None:
    """Add an item to a queue. A signal replaces an unhandled item for the same signal."""
    if isinstance(item, SignalItem):
        for index, waiting in enumerate(items):
            if isinstance(waiting, SignalItem) and waiting.key == item.key:
                items[index] = item
                return
    items.append(item)


def start(
    checked: CheckedDefinition, now: float, limits: Limits = DEFAULT_LIMITS
) -> tuple[Runtime, list[Effect]]:
    """Enter the root of a definition that has no problems."""
    if checked.problems:
        raise ValueError(f"the definition has {len(checked.problems)} problems")
    definition = checked.definition
    work = _Step(checked, Runtime({**definition.constants, **definition.vars}), now, limits)
    return work.run("startup", lambda: work.enter(work.root))


def step(
    checked: CheckedDefinition,
    runtime: Runtime,
    item: QueueItem,
    now: float,
    limits: Limits = DEFAULT_LIMITS,
) -> tuple[Runtime, list[Effect]]:
    """Handle one queue item."""
    work = _Step(checked, runtime, now, limits)
    return work.run(_event(item), lambda: work.handle(item))


def _event(item: QueueItem) -> str:
    """What a fault report calls a queue item."""
    match item:
        case SignalItem(key):
            return key
        case ReturnItem(call):
            return f"{call.kind}_return"
        case TimerItem():
            return "timer"


class _FaultError(Exception):
    def __init__(self, rule: str, node: Node) -> None:
        super().__init__(rule)
        self.rule = rule
        self.node = node.name


class _Step:
    """The state of one step, changed in place and frozen into a Runtime at the end."""

    def __init__(
        self, checked: CheckedDefinition, runtime: Runtime, now: float, limits: Limits
    ) -> None:
        self.expressions = checked.expressions
        self.tree = checked.tree
        self.root = self.tree.nodes[checked.definition.root]
        self.before = runtime
        self.now = now
        self.limits = limits
        self.blackboard = dict(runtime.blackboard)
        self.active = set(runtime.active)
        self.calls = list(runtime.calls)
        self.timers = list(runtime.timers)
        self.calls_made = runtime.calls_made
        self.effects: list[Effect] = []
        self.raised_events: deque[tuple[EngineEvent | None, Node]] = deque()
        self.taken = 0

    def run(self, event: str, action: Callable[[], None]) -> tuple[Runtime, list[Effect]]:
        try:
            action()
            self.drain()
            if self.blackboard != self.before.blackboard or self.active != self.before.active:
                while self.walk(self.root, self.offer_events, stop_at_first=True):
                    self.drain()
        except _FaultError as fault:
            self.fault(fault, event)
        runtime = Runtime(
            self.blackboard,
            frozenset(self.active),
            tuple(self.calls),
            tuple(self.timers),
            self.calls_made,
        )
        return runtime, self.effects

    def fault(self, fault: _FaultError, event: str) -> None:
        """Stop everything. Starts and timers this step was about to send are dropped."""
        fresh = {e.timer for e in self.effects if isinstance(e, SetTimer)} & set(self.timers)
        self.effects = [
            effect
            for effect in self.effects
            if not isinstance(effect, SendStart)
            and not (isinstance(effect, SetTimer) and effect.timer in fresh)
        ]
        self.effects += [ClearTimer(timer) for timer in self.timers if timer not in fresh]
        self.active.clear()
        self.calls.clear()
        self.timers.clear()
        for rpc in self.tree.cancel_rpcs:
            self.calls_made += 1
            call = Call(self.calls_made, None, "cancel", "sent", frozenset(), rpc, {})
            self.effects.append(SendCancel(call))
        self.effects.append(FaultReport(fault.rule, fault.node, event, dict(self.blackboard)))

    def handle(self, item: QueueItem) -> None:
        match item:
            case SignalItem(key, value):
                self.blackboard[key] = value
                self.walk(self.root, lambda node: self.offer(node, key), stop_at_first=False)
            case ReturnItem():
                self.handle_return(item)
            case TimerItem(timer):
                self.handle_timer(timer)

    def handle_timer(self, timer: Timer) -> None:
        if timer not in self.timers:
            return
        self.timers.remove(timer)
        node = self.tree.nodes[timer.node]
        if isinstance(timer, CancelDeadline):
            raise _FaultError("a `cancel` that has not returned by its deadline", node)
        transition = node.transitions[timer.transition]
        if self.passes(node, transition):
            self.take(node, transition, "after")

    def handle_return(self, item: ReturnItem) -> None:
        call = next((call for call in self.calls if call.id == item.call.id), None)
        if call is None or call.node is None:
            return
        self.calls.remove(call)
        node = self.tree.nodes[call.node]
        if call.kind == "cancel":
            self.clear_timers(
                [t for t in self.timers if isinstance(t, CancelDeadline) and t.call_id == call.id]
            )
            if item.raised:
                raise _FaultError("a `cancel` that raised", node)
            self.release()
        elif item.raised:
            self.raised_events.append(("failed", node))
        else:
            if node.result is not None:
                self.blackboard[node.result] = item.value
            if node.inside:
                self.enter_inside(node)
            else:
                self.raised_events.append(("succeeded", node))

    def release(self) -> None:
        """Send every held start that no cancel still out holds back."""
        for index, call in enumerate(self.calls):
            if call.phase == "held" and not self.held_back(call.uses):
                self.calls[index] = replace(call, phase="sent")
                self.effects.append(SendStart(self.calls[index]))

    def held_back(self, uses: frozenset[str]) -> bool:
        return any(call.kind == "cancel" and call.uses & uses for call in self.calls)

    def drain(self) -> None:
        """Handle every event the engine raised, in the order raised."""
        while self.raised_events:
            event, node = self.raised_events.popleft()
            if node.name not in self.active:
                continue
            if event == "failed":
                self.fail(node)
            else:
                self.offer(node, event)

    def walk(self, node: Node, offer: Callable[[Node], bool], stop_at_first: bool) -> bool:
        """Offer an event to a node, then to the active nodes inside it. The outermost wins."""
        if node.name not in self.active:
            return False
        if offer(node):
            return True
        taken = False
        for name in [name for name in node.inside if name in self.active]:
            if self.walk(self.tree.nodes[name], offer, stop_at_first):
                if stop_at_first:
                    return True
                taken = True
                self.drain()
        return taken

    def offer(self, node: Node, trigger: str | None) -> bool:
        """Take the first transition of a node on a trigger whose guard passes."""
        for transition in node.transitions:
            if transition.trigger != trigger or transition.after is not None:
                continue
            if self.passes(node, transition):
                self.take(node, transition, trigger)
                return True
        return False

    def offer_events(self, node: Node) -> bool:
        """Take the first transition of a node on a defined event that is true."""
        for transition in node.transitions:
            trigger = transition.trigger
            if trigger is None or trigger not in self.tree.events:
                continue
            if self.holds(node, self.tree.events[trigger]) and self.passes(node, transition):
                self.take(node, transition, trigger)
                return True
        return False

    def fail(self, node: Node) -> None:
        for candidate in (node, *self.tree.ancestors(node)):
            if self.offer(candidate, "failed"):
                return
        raise _FaultError("a `failed` that no node handles", node)

    def take(self, node: Node, transition: Transition, trigger: str | None) -> None:
        self.taken += 1
        if self.taken > self.limits.transition_cap:
            raise _FaultError("too many transitions in a row with no outside event", node)
        values = {var: self.evaluate(node, source) for var, source in transition.set.items()}
        self.blackboard.update(values)
        self.effects.append(TransitionRecord(node.name, trigger, transition.goto))
        graceful = transition.mode == "graceful"
        self.leave_inside(node, graceful)
        self.complete(node, transition.goto, graceful)

    def complete(self, node: Node, goto: str, graceful: bool) -> None:
        """Finish a transition once everything inside its node has been left."""
        self.leave(node)
        failed = self.failed_check(node, node.ensures) if graceful else None
        if failed is not None:
            goto = self.fallback(node, failed, "ensures").name
        if goto in self.tree.nodes:
            self.enter(self.tree.nodes[goto])
            return
        outcome: Outcome = "failed" if goto == "failed" else "done"
        parent = self.tree.parent[node.name]
        if outcome == "failed" or not any(name in self.active for name in parent.parallel):
            self.raised_events.append((outcome, parent))

    def leave_inside(self, node: Node, graceful: bool) -> None:
        for name in node.inside:
            if name not in self.active:
                continue
            inner = self.tree.nodes[name]
            self.leave_inside(inner, graceful)
            self.leave(inner)
            if graceful and self.failed_check(inner, inner.ensures) is not None:
                raise _FaultError("a failed `ensures` check on a node an outer node left", inner)

    def leave(self, node: Node) -> None:
        self.active.discard(node.name)
        self.calls = [c for c in self.calls if c.node != node.name or c.kind != "start"]
        self.clear_timers(
            [t for t in self.timers if isinstance(t, AfterTimer) and t.node == node.name]
        )
        activity = self.tree.activity(node)
        if activity is None or activity.cancel is None:
            return
        call = self.track_call(node, "cancel", "sent", activity.resources, activity.cancel, {})
        self.effects.append(SendCancel(call))
        due = self.now + self.limits.cancel_deadline_s
        self.set_timer(CancelDeadline(node.name, due, call.id))

    def enter(self, node: Node) -> None:
        failed = self.failed_check(node, node.requires)
        if failed is not None:
            self.enter(self.fallback(node, failed, "requires"))
            return
        self.active.add(node.name)
        for index, transition in enumerate(node.transitions):
            if transition.after is not None:
                due = self.now + self.seconds(node, transition.after)
                self.set_timer(AfterTimer(node.name, due, index))
        activity = self.tree.activity(node)
        if activity is None:
            self.enter_inside(node)
            return
        args = {name: self.evaluate(node, source) for name, source in node.args.items()}
        if node.result is not None:
            self.blackboard.pop(node.result, None)
        uses = activity.resources
        phase: CallPhase = "held" if self.held_back(uses) else "sent"
        call = self.track_call(node, "start", phase, uses, activity.start, args)
        if phase == "sent":
            self.effects.append(SendStart(call))

    def enter_inside(self, node: Node) -> None:
        if node.initial is not None:
            self.enter(self.tree.nodes[node.initial])
        for name in node.parallel:
            self.enter(self.tree.nodes[name])
        if node.is_switch:
            self.raised_events.append((None, node))

    def failed_check(self, node: Node, checks: tuple[Check, ...]) -> Check | None:
        for check in checks:
            if not all(self.holds(node, source) for source in check.expect):
                return check
        return None

    def fallback(self, node: Node, check: Check, kind: Literal["requires", "ensures"]) -> Node:
        if check.fallback is None:
            raise _FaultError(f"a failed `{kind}` check with no fallback", node)
        self.effects.append(TransitionRecord(node.name, kind, check.fallback))
        return self.tree.nodes[check.fallback]

    def track_call(
        self,
        node: Node,
        kind: CallKind,
        phase: CallPhase,
        uses: frozenset[str],
        rpc: str,
        args: Mapping[str, Plain],
    ) -> Call:
        self.calls_made += 1
        call = Call(self.calls_made, node.name, kind, phase, uses, rpc, args)
        self.calls.append(call)
        return call

    def set_timer(self, timer: Timer) -> None:
        self.timers.append(timer)
        self.effects.append(SetTimer(timer))

    def clear_timers(self, timers: list[Timer]) -> None:
        for timer in timers:
            self.timers.remove(timer)
            self.effects.append(ClearTimer(timer))

    def evaluate(self, node: Node, source: str) -> Plain:
        try:
            return evaluate(self.expressions[source], self.blackboard)
        except ExpressionError as error:
            raise _FaultError(f"an expression error: {error}", node) from None

    def seconds(self, node: Node, source: str) -> float:
        value = self.evaluate(node, source)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
        ):
            raise _FaultError(
                f"an expression error: `{source}` gave {value!r}, not a finite number", node
            )
        return value

    def holds(self, node: Node, source: str) -> bool:
        try:
            return evaluate_bool(self.expressions[source], self.blackboard)
        except ExpressionError as error:
            raise _FaultError(f"an expression error: {error}", node) from None

    def passes(self, node: Node, transition: Transition) -> bool:
        return transition.if_ is None or self.holds(node, transition.if_)
