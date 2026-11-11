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

from dimos.orchestration.definition import Activity, Node, OrchestratorDefinition
from dimos.orchestration.expressions import (
    NUMBER,
    UNKNOWN,
    Compiler,
    Expression,
    ExpressionError,
    Shape,
    Wanted,
)

OUTCOMES = ("done", "failed")
RESERVED = ("succeeded", *OUTCOMES)


@dataclass(frozen=True)
class CheckedDefinition:
    """A definition, what is wrong with it, and its expressions compiled by source."""

    definition: OrchestratorDefinition
    problems: tuple[str, ...]
    expressions: Mapping[str, Expression]
    tree: "Tree"


def check(
    definition: OrchestratorDefinition, shapes: Mapping[str, Shape] | None = None
) -> CheckedDefinition:
    """Run every loader check. `shapes` gives the shape of signals and results.

    Problems with names or nesting are returned alone, since the other checks need a sound tree.
    """
    problems = [*_name_problems(definition), *_nesting_problems(definition)]
    expressions: dict[str, Expression] = {}
    tree = Tree(definition)
    if not problems:
        expressions, problems = _compile(definition, shapes or {})
        problems += [
            *_node_problems(tree),
            *_transition_problems(tree),
            *_fallback_problems(tree),
            *_resource_problems(tree),
        ]
    return CheckedDefinition(definition, tuple(dict.fromkeys(problems)), expressions, tree)


def _listed(names: Iterable[str]) -> str:
    return " and ".join(f"`{name}`" for name in sorted(names))


class Tree:
    """A definition indexed by name. It is sound only when names and nesting have no problems."""

    def __init__(self, definition: OrchestratorDefinition) -> None:
        self.definition = definition
        self.nodes = {node.name: node for node in definition.nodes}
        self.activities = {activity.name: activity for activity in definition.activities}
        self.events = {event.name for event in definition.events}
        self.triggers = self.events | {signal.name for signal in definition.signals}
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


def _name_problems(definition: OrchestratorDefinition) -> Iterator[str]:
    signals = [signal.name for signal in definition.signals]
    events = [event.name for event in definition.events]
    nodes = [node.name for node in definition.nodes]
    activities = [activity.name for activity in definition.activities]
    for kind, names in (("node", nodes), ("activity", activities), ("event", events)):
        for name, count in Counter(names).items():
            if count > 1:
                yield f"{kind} `{name}` is defined more than once"
    for name in (*signals, *events):
        if name in RESERVED:
            yield f"`{name}` is reserved, and no signal or event may have that name"
    for name in sorted(set(signals) & set(events)):
        yield f"`{name}` is both a signal and an event"
    for name in nodes:
        if name in OUTCOMES:
            yield f"`{name}` is reserved, and no node may have that name"
    results = {node.result for node in definition.nodes if node.result is not None}
    keys = Counter([*definition.constants, *definition.vars, *signals, *results])
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
        if node.wind_down is not None:
            yield f"{where} sets `wind_down`, and wind-down nodes are not supported"
        if node.outcomes and not node.inside:
            yield f"{where} has `outcomes`, and no nodes inside"


def _transition_problems(tree: Tree) -> Iterator[str]:
    for node in tree.definition.nodes:
        where = f"node `{node.name}`"
        parent = tree.parent.get(node.name)
        above = list(tree.ancestors(node))
        if node.is_switch:
            if node.activity is not None or node.inside:
                yield f"{where} is a switch node, and it has an activity or nodes inside"
            if any(t.trigger is not None or t.after is not None for t in node.transitions):
                yield f"{where} is a switch node, and a transition has a `trigger` or an `after`"
            if node.transitions[-1].if_ is not None:
                yield f"{where} is a switch node, and its last transition has an `if`"
        if parent is not None and not any(n.transitions for n in (node, *above)):
            yield f"{where} has no way out through its own transitions or an ancestor's"
        ended = any(t.goto == "done" for name in node.inside for t in tree.nodes[name].transitions)
        if parent is not None and ended and all(t.trigger != "done" for t in node.transitions):
            yield f"{where} can be ended with `done`, and it has no transition on `done`"
        for transition in node.transitions:
            trigger, goto = transition.trigger, transition.goto
            if trigger is not None and trigger not in tree.triggers and trigger not in RESERVED:
                yield f"{where}: trigger `{trigger}` is not a signal or an event"
            if trigger == "succeeded" and node.inside:
                yield f"{where} has nodes inside, and a transition on `succeeded`"
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
            if trigger in tree.events and goto == node.name:
                yield f"{where}: a transition on the event `{trigger}` points back at its own node"
            for outer in above:
                if trigger in tree.triggers and any(
                    t.trigger == trigger and t.if_ is None for t in outer.transitions
                ):
                    yield (
                        f"{where}: its transition on `{trigger}` is never taken, "
                        f"since `{outer.name}` has one with no `if`"
                    )
            if transition.mode == "immediate" and goto in tree.nodes:
                for name in sorted(_started_on_entering(tree, goto, set())):
                    if tree.activities[name].uses:
                        yield (
                            f"{where}: the immediate transition to `{goto}` starts "
                            f"`{name}`, which uses a resource"
                        )


def _started_on_entering(tree: Tree, name: str, seen: set[str]) -> set[str]:
    """The activities that can start in the step that enters a node."""
    node = tree.nodes.get(name)
    if node is None or name in seen:
        return set()
    seen.add(name)
    if node.activity is not None:
        return {node.activity} & tree.activities.keys()
    if node.is_switch:
        following = [transition.goto for transition in node.transitions]
    elif node.children:
        following = [node.initial] if node.initial is not None else []
    else:
        following = list(node.parallel)
    return {activity for inner in following for activity in _started_on_entering(tree, inner, seen)}


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


def _activity_needs(tree: Tree, node: Node) -> set[str]:
    activity = tree.activity(node)
    if activity is None:
        return set()
    return {*activity.uses, *([activity.cancel] if activity.cancel is not None else [])}


def _needs(tree: Tree, name: str) -> set[str]:
    """Every resource a node and the nodes inside it could need."""
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
        shared = _activity_needs(tree, node) & inner
        if shared:
            yield f"node `{node.name}`: its activity shares {_listed(shared)} with a node inside it"


def _sources(definition: OrchestratorDefinition) -> Iterator[tuple[str, str, Wanted]]:
    """Every expression in a definition, with where it is and what it must give."""
    for event in definition.events:
        yield f"event `{event.name}`: `when`", event.when, True
    for node in definition.nodes:
        where = f"node `{node.name}`"
        for check in (*node.requires, *node.ensures):
            for source in check.expect:
                yield f"{where}: `expect`", source, True
        for parameter, source in node.args.items():
            yield f"{where}: `args` entry `{parameter}`", source, UNKNOWN
        for transition in node.transitions:
            if transition.if_ is not None:
                yield f"{where}: `if`", transition.if_, True
            if transition.after is not None:
                yield f"{where}: `after`", transition.after, NUMBER
            for var, source in transition.set.items():
                yield f"{where}: `set` of `{var}`", source, definition.vars.get(var, UNKNOWN)


def _compile(
    definition: OrchestratorDefinition, shapes: Mapping[str, Shape]
) -> tuple[dict[str, Expression], list[str]]:
    written = [signal.name for signal in definition.signals]
    written += [node.result for node in definition.nodes if node.result is not None]
    compiler = Compiler(
        {
            **definition.constants,
            **definition.vars,
            **{key: shapes.get(key, UNKNOWN) for key in written},
        }
    )
    expressions: dict[str, Expression] = {}
    problems: list[str] = []
    for where, source, gives in _sources(definition):
        try:
            expressions[source] = compiler.compile(source, gives)
        except ExpressionError as error:
            problems.append(f"{where}: {error}")
    return expressions, problems
