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

"""Static checks on an orchestrator definition."""

from collections import Counter
from collections.abc import Iterable, Iterator, Mapping
from dataclasses import dataclass
from typing import get_args

from dimos.orchestration.definition import (
    Activity,
    Node,
    OrchestratorDefinition,
    Outcome,
    Transition,
)
from dimos.orchestration.expressions import (
    NUMBER,
    UNKNOWN,
    Compiler,
    Expression,
    ExpressionError,
    Shape,
    Wanted,
)

OUTCOMES: tuple[str, ...] = get_args(Outcome)
RESERVED = ("succeeded", *OUTCOMES)


@dataclass(frozen=True)
class CheckedDefinition:
    """A definition, what is wrong with it, and its expressions compiled by source."""

    problems: tuple[str, ...]
    expressions: Mapping[str, Expression]
    tree: "Tree"

    @property
    def definition(self) -> OrchestratorDefinition:
        return self.tree.definition


def check(
    definition: OrchestratorDefinition,
    shapes: Mapping[str, Shape] | None = None,
    parameters: Mapping[str, Mapping[str, Shape]] | None = None,
) -> CheckedDefinition:
    """Run every static check. Name and nesting problems are returned alone.

    `shapes` are of signals and results. `parameters` are of each activity's `start`, by name.
    """
    problems = [*_name_problems(definition), *_nesting_problems(definition)]
    expressions: dict[str, Expression] = {}
    tree = Tree(definition)
    if not problems:
        expressions, problems = _compile(definition, shapes or {}, parameters or {})
        problems += [
            *_node_problems(tree),
            *_transition_problems(tree, expressions),
            *_fallback_problems(tree),
            *_resource_problems(tree),
        ]
    return CheckedDefinition(tuple(dict.fromkeys(problems)), expressions, tree)


def _listed(names: Iterable[str]) -> str:
    return " and ".join(f"`{name}`" for name in sorted(names))


class Tree:
    """A definition indexed by name. It is sound only when names and nesting have no problems."""

    def __init__(self, definition: OrchestratorDefinition) -> None:
        self.definition = definition
        self.nodes = {node.name: node for node in definition.nodes}
        self.activities = {activity.name: activity for activity in definition.activities}
        self.events = {event.name: event.when for event in definition.events}
        self.signals = {signal.name for signal in definition.signals}
        self.parent = {
            inner: self.nodes[node.name] for node in definition.nodes for inner in node.inside
        }

    def ancestors(self, node: Node) -> Iterator[Node]:
        while node.name in self.parent:
            node = self.parent[node.name]
            yield node

    def siblings(self, node: Node) -> set[str]:
        parent = self.parent.get(node.name)
        return set(parent.children) - {node.name} if parent is not None else set()

    def activity(self, node: Node) -> Activity | None:
        return self.activities.get(node.activity) if node.activity is not None else None

    @property
    def cancel_rpcs(self) -> tuple[str, ...]:
        """Every cancel RPC the definition names, once each."""
        cancels = [activity.cancel for activity in self.activities.values()]
        return tuple(dict.fromkeys(cancel for cancel in cancels if cancel is not None))


def _name_problems(definition: OrchestratorDefinition) -> Iterator[str]:
    signals = [signal.name for signal in definition.signals]
    events = [event.name for event in definition.events]
    nodes = [node.name for node in definition.nodes]
    activities = [activity.name for activity in definition.activities]
    results = {node.result for node in definition.nodes if node.result is not None}
    keys = Counter([*definition.constants, *definition.vars, *signals, *results])
    for kind, names in (("node", nodes), ("activity", activities), ("event", events)):
        for name, count in Counter(names).items():
            if count > 1:
                yield f"{kind} `{name}` is defined more than once"
    for name in (*keys, *events):
        if name in RESERVED:
            yield f"`{name}` is reserved, and no key or event may have that name"
    for name in sorted(set(keys) & set(events)):
        yield f"`{name}` is both a key and an event"
    for name in nodes:
        if name in OUTCOMES:
            yield f"`{name}` is reserved, and no node may have that name"
    for key, count in keys.items():
        if count > 1:
            yield f"key `{key}` is declared more than once"
    for key in keys:
        for longer in keys:
            if longer.startswith(key + "."):
                yield f"key `{key}` is a prefix of key `{longer}`"


def _nesting_problems(definition: OrchestratorDefinition) -> Iterator[str]:
    holders: dict[str, list[str]] = {node.name: [] for node in definition.nodes}
    if definition.root not in holders:
        yield f"`root` names `{definition.root}`, which is not a node"
    for node in definition.nodes:
        for inner in node.inside:
            if inner in holders:
                holders[inner].append(node.name)
            else:
                yield f"node `{node.name}` holds `{inner}`, which is not a node"
    for name, inside in holders.items():
        if name != definition.root and len(inside) != 1:
            where = _listed(inside) if inside else "no node"
            yield f"node `{name}` is inside {where}, and it must be inside exactly one node"
        above = inside[:1]
        visited: set[str] = set()
        while above and above[0] != name and above[0] not in visited:
            visited.add(above[0])
            above = holders[above[0]][:1]
        if above and above[0] == name:
            yield f"node `{name}` is inside itself"


def _node_problems(tree: Tree) -> Iterator[str]:
    resources = tree.definition.resources
    for activity in tree.definition.activities:
        if activity.uses and activity.cancel is None:
            yield f"activity `{activity.name}` has `uses`, and no `cancel`"
        for resource in activity.uses:
            if resource not in resources:
                yield f"activity `{activity.name}` uses `{resource}`, which is not a resource"
    for node in tree.definition.nodes:
        where = f"node `{node.name}`"
        if node.activity is None and (node.args or node.result is not None):
            yield f"{where} has `args` or `result`, and no activity"
        if node.activity is not None and node.activity not in tree.activities:
            yield f"{where} names `{node.activity}`, which is not an activity"
        for resource in node.claims:
            if resource not in resources:
                yield f"{where} claims `{resource}`, which is not a resource"
        if node.children and node.initial not in node.children:
            yield f"{where}: `initial` must name one of its children"
        if node.initial is not None and not node.children:
            yield f"{where} has `initial`, and no children"
        if node.outcomes and not node.inside:
            yield f"{where} has `outcomes`, and no nodes inside"


def _outcomes_read(expressions: Mapping[str, Expression], transition: Transition) -> set[str]:
    """Which of `succeeded`, `failed` and `done` the `if` of a transition reads."""
    expression = expressions.get(transition.if_ or "")
    return set(RESERVED).intersection(expression.reads) if expression is not None else set()


def _transition_problems(tree: Tree, expressions: Mapping[str, Expression]) -> Iterator[str]:
    for node in tree.definition.nodes:
        where = f"node `{node.name}`"
        parent = tree.parent.get(node.name)
        above = list(tree.ancestors(node))
        reads = [_outcomes_read(expressions, transition) for transition in node.transitions]
        if node.is_switch:
            if node.activity is not None or node.inside:
                yield f"{where} is a switch node, and it has an activity or nodes inside"
            if not all(transition.is_condition for transition in node.transitions):
                yield f"{where} is a switch node, and a transition has an `on` or an `after`"
            if node.transitions[-1].if_ is not None:
                yield f"{where} is a switch node, and its last transition has an `if`"
        if parent is not None and not any(n.transitions for n in (node, *above)):
            yield f"{where} has no way out through its own transitions or an ancestor's"
        ended = any(t.goto == "done" for name in node.inside for t in tree.nodes[name].transitions)
        if parent is not None and ended and not any("done" in read for read in reads):
            yield f"{where} can be ended with `done`, and no `if` of its transitions reads `done`"
        for transition, read in zip(node.transitions, reads, strict=True):
            on, goto = transition.on, transition.goto
            if on is not None and on not in tree.signals:
                yield f"{where}: `on` names `{on}`, which is not a signal"
            if "succeeded" in read and node.inside:
                yield f"{where} has nodes inside, and a transition that reads `succeeded`"
            if "failed" in read and not transition.is_condition:
                yield f"{where}: `failed` is never true in a transition with an `on` or an `after`"
            if parent is not None and parent.parallel:
                if goto not in parent.outcomes:
                    yield (
                        f"{where} is inside the parallel node `{parent.name}`, and "
                        f"`goto` names `{goto}`, which is not an outcome of that node"
                    )
            elif (
                goto != node.name
                and goto not in tree.siblings(node)
                and (parent is None or goto not in parent.outcomes)
            ):
                yield (
                    f"{where}: `goto` names `{goto}`, which is not a sibling, "
                    "the node itself or an outcome of its parent"
                )
            for var in transition.set:
                if var not in tree.definition.vars:
                    yield f"{where} sets `{var}`, which is not a var"
            if transition.is_condition and goto == node.name and not read:
                yield f"{where}: a transition with only an `if` points back at its own node"
            for outer in above:
                if on is not None and any(t.on == on and t.if_ is None for t in outer.transitions):
                    yield (
                        f"{where}: its transition on `{on}` is never taken, "
                        f"since `{outer.name}` has one with no `if`"
                    )
                if (
                    transition.is_condition
                    and not read
                    and any(t.is_condition and t.if_ == transition.if_ for t in outer.transitions)
                ):
                    yield (
                        f"{where}: its transition on `{transition.if_}` is never taken, "
                        f"since `{outer.name}` has one with the same `if`"
                    )
            if transition.mode == "immediate" and goto in tree.nodes:
                for name in sorted(_started_on_entering(tree, goto, set())):
                    if tree.activities[name].uses:
                        yield (
                            f"{where}: the immediate transition to `{goto}` starts "
                            f"`{name}`, which uses a resource"
                        )


def _started_on_entering(tree: Tree, name: str, seen: set[str]) -> set[str]:
    """The activities that can start in the step that enters a node or falls back from it."""
    node = tree.nodes.get(name)
    if node is None or name in seen:
        return set()
    seen.add(name)
    following = [check.fallback for check in node.requires if check.fallback is not None]
    started: set[str] = set()
    if node.activity is not None:
        started = {node.activity} & tree.activities.keys()
    elif node.is_switch:
        following += [transition.goto for transition in node.transitions]
    elif node.initial is not None:
        following.append(node.initial)
    else:
        following += node.parallel
    return started.union(*(_started_on_entering(tree, inner, seen) for inner in following))


def _fallback_problems(tree: Tree) -> Iterator[str]:
    for node in tree.definition.nodes:
        for fallback in (c.fallback for c in (*node.requires, *node.ensures)):
            if fallback is not None and fallback not in tree.siblings(node):
                yield f"node `{node.name}`: fallback `{fallback}` is not a sibling"
    fallbacks = {
        node.name: {c.fallback for c in node.requires if c.fallback is not None}
        for node in tree.definition.nodes
    }
    reported: set[str] = set()
    for name in fallbacks:
        reached: set[str] = set()
        frontier = [name]
        while frontier:
            found = fallbacks.get(frontier.pop(), set()) - reached
            frontier += found
            reached |= found
        if name in reached and name not in reported:
            yield f"node `{name}`: the fallbacks of `requires` form a loop"
            reported |= reached


def _activity_needs(tree: Tree, node: Node) -> frozenset[str]:
    activity = tree.activity(node)
    return activity.resources if activity is not None else frozenset()


def _needs(tree: Tree, name: str) -> set[str]:
    """Every resource a node and the nodes inside it could need, cancel RPCs included."""
    node = tree.nodes[name]
    needs = {*node.claims, *_activity_needs(tree, node)}
    for inner in node.inside:
        needs |= _needs(tree, inner)
    return needs


def _resource_problems(tree: Tree) -> Iterator[str]:
    for node in tree.definition.nodes:
        for index, first in enumerate(node.parallel):
            for second in node.parallel[index + 1 :]:
                shared = _needs(tree, first) & _needs(tree, second)
                if shared:
                    yield (
                        f"`{first}` and `{second}`, inside the parallel node `{node.name}`, "
                        f"can both need {_listed(shared)}"
                    )
        inner = {need for name in node.inside for need in _needs(tree, name)}
        shared = inner & _activity_needs(tree, node)
        if shared:
            yield f"node `{node.name}`: its activity shares {_listed(shared)} with a node inside it"


def _sources(
    definition: OrchestratorDefinition, parameters: Mapping[str, Mapping[str, Shape]]
) -> Iterator[tuple[str, str, Wanted]]:
    """Every expression but an `if`, with where it is and what it must give."""
    for event in definition.events:
        yield f"event `{event.name}`: `when`", event.when, True
    for node in definition.nodes:
        where = f"node `{node.name}`"
        for condition in (*node.requires, *node.ensures):
            for source in condition.expect:
                yield f"{where}: `expect`", source, True
        wanted = parameters.get(node.activity, {}) if node.activity is not None else {}
        for parameter, source in node.args.items():
            yield f"{where}: `args` entry `{parameter}`", source, wanted.get(parameter, UNKNOWN)
        for transition in node.transitions:
            if transition.after is not None:
                yield f"{where}: `after`", transition.after, NUMBER
            for var, source in transition.set.items():
                yield f"{where}: `set` of `{var}`", source, definition.vars.get(var, UNKNOWN)


def _compile(
    definition: OrchestratorDefinition,
    shapes: Mapping[str, Shape],
    parameters: Mapping[str, Mapping[str, Shape]],
) -> tuple[dict[str, Expression], list[str]]:
    written = [signal.name for signal in definition.signals]
    written += [node.result for node in definition.nodes if node.result is not None]
    keys: dict[str, Shape] = {
        **definition.constants,
        **definition.vars,
        **{key: shapes.get(key, UNKNOWN) for key in written},
    }
    expressions: dict[str, Expression] = {}
    problems: list[str] = []

    def add(compiler: Compiler, where: str, source: str, gives: Wanted) -> None:
        try:
            expressions[source] = compiler.compile(source, gives)
        except ExpressionError as error:
            problems.append(f"{where}: {error}")

    plain = Compiler(keys)
    for where, source, gives in _sources(definition, parameters):
        add(plain, where, source, gives)
    events = {event.name: expressions.get(event.when) for event in definition.events}
    broken = [name for name, when in events.items() if when is None]
    guards = Compiler(
        {**keys, **dict.fromkeys((*RESERVED, *broken), False)},
        {name: when for name, when in events.items() if when is not None},
    )
    for node in definition.nodes:
        for transition in node.transitions:
            if transition.if_ is not None:
                add(guards, f"node `{node.name}`: `if`", transition.if_, True)
    return expressions, problems
