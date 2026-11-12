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

import pytest

pytest.importorskip("cel_expr_python")

from dimos.orchestration.checks import check
from dimos.orchestration.definition import (
    Check,
    Event,
    Node,
    OrchestratorDefinition,
    Signal,
    Transition,
)

Edits = dict[str, dict[str, object]]
TOP = ""
AS_PARALLEL = {"children": (), "initial": None, "parallel": ("NextWaypoint", "Navigating")}

# Each row edits the patrol definition by node name and gives the problem that must follow.
# A name that starts with + is a node to add.
BROKEN: list[tuple[Edits, str]] = [
    ({"+Estopped": {}}, "node `Estopped` is defined more than once"),
    ({"+done": {}}, "`done` is reserved, and no node"),
    (
        {TOP: {"signals": (Signal(name="failed", stream="A.b"),)}},
        "`failed` is reserved, and no signal or event",
    ),
    (
        {TOP: {"signals": (Signal(name="estop", stream="A.b"),)}},
        "`estop` is both a signal and an event",
    ),
    ({TOP: {"vars": {"route": 0}}}, "key `route` is declared more than once"),
    ({TOP: {"vars": {"nav": 0}}}, "key `nav` is a prefix of key `nav.goal`"),
    ({TOP: {"root": "Nowhere"}}, "`root` names `Nowhere`, which is not a node"),
    ({"Patrolling": {"children": ("NextWaypoint", "Lost")}}, "holds `Lost`, which is not a node"),
    (
        {"Demo": {"children": ("Patrolling", "Estopped", "Navigating")}},
        "node `Navigating` is inside `Demo` and `Patrolling`",
    ),
    ({"+Lost": {}}, "node `Lost` is inside no node"),
    ({"+A": {"children": ["B"]}, "+B": {"children": ["A"]}}, "node `A` is inside itself"),
    ({TOP: {"resources": ()}}, "activity `go_to` uses `base`, which is not a resource"),
    ({"Navigating": {"activity": None}}, "node `Navigating` has `args` or `result`"),
    ({"Navigating": {"activity": "fly"}}, "names `fly`, which is not an activity"),
    ({"Navigating": {"claims": ("arm",)}}, "claims `arm`, which is not a resource"),
    ({"Patrolling": {"initial": "Estopped"}}, "`initial` must name one of its children"),
    ({"Estopped": {"initial": "Patrolling"}}, "node `Estopped` has `initial`, and no children"),
    ({"Estopped": {"outcomes": ("done",)}}, "has `outcomes`, and no nodes inside"),
    ({"NextWaypoint": {"activity": "go_to"}}, "is a switch node, and it has an activity"),
    (
        {"NextWaypoint": {"transitions": (Transition(goto="Navigating", if_="true"),)}},
        "is a switch node, and its last transition has an `if`",
    ),
    (
        {
            "NextWaypoint": {
                "transitions": (
                    Transition(goto="Navigating", trigger="arrived"),
                    Transition(goto="Navigating"),
                )
            }
        },
        "is a switch node, and a transition has a `trigger`",
    ),
    (
        {"Demo": {"children": ("Patrolling", "Estopped", "Idle")}, "+Idle": {}},
        "node `Idle` has no way out",
    ),
    (
        {
            "Patrolling": {"outcomes": ("done",)},
            "Navigating": {"transitions": (Transition(goto="done", trigger="arrived"),)},
        },
        "node `Patrolling` can be ended with `done`",
    ),
    (
        {"Estopped": {"transitions": (Transition(goto="Patrolling", trigger="whenever"),)}},
        "trigger `whenever` is not a signal or an event",
    ),
    (
        {"Patrolling": {"transitions": (Transition(goto="Estopped", trigger="succeeded"),)}},
        "node `Patrolling` has nodes inside, and a transition on `succeeded`",
    ),
    (
        {"Navigating": {"transitions": (Transition(goto="Estopped", trigger="arrived"),)}},
        "`goto` names `Estopped`, which is not a sibling",
    ),
    (
        {"Navigating": {"transitions": (Transition(goto="done", trigger="arrived"),)}},
        "`goto` names `done`, which is not a sibling",
    ),
    ({"Patrolling": AS_PARALLEL}, "`goto` names `Navigating`, which is not an outcome"),
    (
        {"Estopped": {"transitions": (Transition(goto="Patrolling", set={"route": "[]"}),)}},
        "sets `route`, which is not a var",
    ),
    (
        {"Estopped": {"transitions": (Transition(goto="Estopped", trigger="estop_cleared"),)}},
        "on the event `estop_cleared` points back at its own node",
    ),
    (
        {"Navigating": {"transitions": (Transition(goto="NextWaypoint", trigger="estop"),)}},
        "its transition on `estop` is never taken, since `Patrolling` has one",
    ),
    (
        {
            "Estopped": {
                "transitions": (
                    Transition(goto="Patrolling", trigger="estop_cleared", mode="immediate"),
                )
            }
        },
        "the immediate transition to `Patrolling` starts `go_to`, which uses a resource",
    ),
    (
        {"Navigating": {"requires": (Check(expect=["true"], fallback="Estopped"),)}},
        "fallback `Estopped` is not a sibling",
    ),
    (
        {
            "Patrolling": {"requires": (Check(expect=["true"], fallback="Estopped"),)},
            "Estopped": {"requires": (Check(expect=["true"], fallback="Patrolling"),)},
        },
        "the fallbacks of `requires` form a loop",
    ),
    (
        {"Patrolling": AS_PARALLEL, "NextWaypoint": {"activity": "go_to"}},
        "`NextWaypoint` and `Navigating`, inside the parallel node `Patrolling`, can both need",
    ),
    (
        {"Patrolling": {"activity": "go_to"}},
        "its activity shares `FakeBase.stop` and `base` with a node inside it",
    ),
    (
        {TOP: {"events": (Event(name="estop", when="estop_buton"),)}},
        "event `estop`: `when`: `estop_buton`: <input>:1:1: undeclared reference",
    ),
    (
        {"Estopped": {"transitions": (Transition(goto="Patrolling", if_="patrol.index"),)}},
        "`if`: `patrol.index` gives INT, and BOOL is needed",
    ),
    (
        {"Estopped": {"transitions": (Transition(goto="Patrolling", after="'soon'"),)}},
        "`after`: `'soon'` gives STRING, and INT or DOUBLE is needed",
    ),
    (
        {"Estopped": {"transitions": (Transition(goto="Patrolling", set={"patrol.index": 1.5}),)}},
        "`set` of `patrol.index`: `1.5` gives DOUBLE, and INT is needed",
    ),
]


def edited(definition: OrchestratorDefinition, edits: Edits) -> OrchestratorDefinition:
    nodes = [node.model_copy(update=edits.get(node.name, {})) for node in definition.nodes]
    nodes += [Node(name=name[1:], **fields) for name, fields in edits.items() if name[:1] == "+"]
    return definition.model_copy(update={**edits.get(TOP, {}), "nodes": tuple(nodes)})


def test_patrol_has_no_problems(patrol: OrchestratorDefinition) -> None:
    checked = check(patrol)
    assert checked.problems == ()
    assert checked.expressions["patrol.index + 1"].reads == ("patrol.index",)


def test_shapes_are_checked_when_given(patrol: OrchestratorDefinition) -> None:
    (problem,) = check(patrol, {"nav_status": {"status": 0}, "nav.goal": ""}).problems
    assert "event `arrived`: `when`" in problem
    assert "cannot be the operand of a select" in problem


@pytest.mark.parametrize(("edits", "problem"), BROKEN)
def test_broken_definitions(patrol: OrchestratorDefinition, edits: Edits, problem: str) -> None:
    problems = check(edited(patrol, edits)).problems
    assert any(problem in found for found in problems), problems
