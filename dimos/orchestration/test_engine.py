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

from dimos.orchestration.checks import CheckedDefinition, check
from dimos.orchestration.definition import Activity, Node, OrchestratorDefinition, Transition
from dimos.orchestration.engine import (
    Call,
    QueueItem,
    Runtime,
    SendCancel,
    SendStart,
    TransitionRecord,
    start,
    step,
)

BASE = frozenset({"base"})

# Work runs `power` and holds a switch node, Pick, and a node that runs `drive`, Move.
WORK = OrchestratorDefinition(
    root="Work",
    constants={"top_speed": 0.5},
    vars={"laps": 0},
    resources=["base"],
    activities=[
        Activity(name="power", start="Base.enable", cancel="Base.disable"),
        Activity(name="drive", start="Base.drive", cancel="Base.stop", uses=["base"]),
    ],
    nodes=[
        Node(
            name="Work",
            activity="power",
            children=["Pick", "Move"],
            initial="Pick",
            transitions=[Transition(trigger="failed", goto="Work")],
        ),
        Node(name="Pick", transitions=[Transition(goto="Move")]),
        Node(
            name="Move",
            activity="drive",
            args={"speed": "top_speed"},
            result="drive.id",
            transitions=[Transition(trigger="succeeded", goto="Pick", set={"laps": "laps + 1"})],
        ),
    ],
)
POWER = Call(1, "Work", "start", "sent", frozenset())
DRIVE = Call(2, "Move", "start", "sent", BASE)


@pytest.fixture
def checked() -> CheckedDefinition:
    checked = check(WORK)
    assert checked.problems == ()
    return checked


@pytest.fixture
def moving(checked: CheckedDefinition) -> Runtime:
    """Work after both nodes were entered, with the start of `drive` not yet returned."""
    runtime, effects = start(checked, 0.0)
    assert effects == [SendStart(POWER, "Base.enable", {})]
    assert runtime.active == {"Work"}

    runtime, effects = step(checked, runtime, QueueItem("start_return", call=POWER), 1.0)
    assert effects == [
        TransitionRecord("Pick", None, "Move"),
        SendStart(DRIVE, "Base.drive", {"speed": 0.5}),
    ]
    assert runtime.active == {"Work", "Move"}
    return runtime


def test_a_start_return_raises_succeeded(checked: CheckedDefinition, moving: Runtime) -> None:
    returned = QueueItem("start_return", call=DRIVE, value=7)
    runtime, effects = step(checked, moving, returned, 2.0)
    stop = Call(3, "Move", "cancel", "sent", BASE)
    again = Call(4, "Move", "start", "sent", BASE)
    assert effects == [
        TransitionRecord("Move", "succeeded", "Pick"),
        SendCancel(stop, "Base.stop"),
        TransitionRecord("Pick", None, "Move"),
        SendStart(again, "Base.drive", {"speed": 0.5}),
    ]
    assert runtime.active == {"Work", "Move"}
    assert runtime.calls == (stop, again)
    assert runtime.blackboard == {"top_speed": 0.5, "laps": 1}


def test_a_failed_walks_up_and_a_stale_return_is_discarded(
    checked: CheckedDefinition, moving: Runtime
) -> None:
    raised = QueueItem("start_return", call=DRIVE, raised=True)
    runtime, effects = step(checked, moving, raised, 2.0)
    assert effects == [
        TransitionRecord("Work", "failed", "Work"),
        SendCancel(Call(3, "Move", "cancel", "sent", BASE), "Base.stop"),
        SendCancel(Call(4, "Work", "cancel", "sent", frozenset()), "Base.disable"),
        SendStart(Call(5, "Work", "start", "sent", frozenset()), "Base.enable", {}),
    ]
    assert runtime.active == {"Work"}

    late = QueueItem("start_return", call=DRIVE, value=7)
    assert step(checked, runtime, late, 3.0) == (runtime, [])
