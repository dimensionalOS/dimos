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

import sys

import pytest

# TODO: remove this skip once the pin is cel-expr-python 0.1.4, which supports Python 3.10.
if sys.version_info < (3, 11):
    pytest.skip("cel-expr-python 0.1.3 needs Python 3.11", allow_module_level=True)

from dimos.orchestration.checks import check
from dimos.orchestration.definition import (
    Activity,
    Check,
    Node,
    OrchestratorDefinition,
    Transition,
)

Edits = dict[str, dict[str, object]]
TOP = ""
AS_PARALLEL = {"children": (), "initial": None, "parallel": ("NextWaypoint", "Navigating")}

# Each row edits the patrol definition by node name and gives the problem that must follow.
# A name that starts with + is a node to add.
BROKEN: list[tuple[Edits, str]] = [
    (
        {"Demo": {"children": ("Patrolling", "Estopped", "Navigating")}},
        "node `Navigating` is inside `Demo` and `Patrolling`",
    ),
    (
        {TOP: {"activities": (Activity(name="go_to", start="FakeBase.go_to", uses=("base",)),)}},
        "activity `go_to` has `uses`, and no `cancel`",
    ),
    (
        {"NextWaypoint": {"transitions": (Transition(goto="Navigating", if_="true"),)}},
        "is a switch node, and its last transition has an `if`",
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
        {"Navigating": {"transitions": (Transition(goto="NextWaypoint", trigger="estop"),)}},
        "its transition on `estop` is never taken, since `Patrolling` has one",
    ),
    (
        {"Estopped": {"requires": (Check(expect=["true"], fallback="Patrolling"),)}},
        "the immediate transition to `Estopped` starts `go_to`, which uses a resource",
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
        {"Estopped": {"transitions": (Transition(goto="Patrolling", if_="patrol.index"),)}},
        "`if`: `patrol.index` gives INT, and BOOL is needed",
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
