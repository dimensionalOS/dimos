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

from dataclasses import dataclass

import pytest

pytest.importorskip("cel_expr_python")

from dimos.orchestration.checks import check
from dimos.orchestration.definition import (
    Activity,
    Check,
    Event,
    Node,
    OrchestratorDefinition,
    Signal,
    Transition,
)
from dimos.orchestration.engine import (
    ClearTimer,
    Effect,
    FaultReport,
    Limits,
    QueueItem,
    Runtime,
    SendCancel,
    SendStart,
    SetTimer,
    Timer,
    TransitionRecord,
    enqueue,
    start,
    step,
)
from dimos.orchestration.expressions import Plain

# What happens outside the engine: a kind, a signal or node name, and a value if it has one.
# The kinds are signal, returns, raises, canceled, cancel raises, timer and deadline.
Happening = tuple[str, str] | tuple[str, str, Plain]


@dataclass
class Case:
    """A definition, the effects of starting it, and the effects of each happening in turn."""

    name: str
    definition: OrchestratorDefinition
    startup: list[str]
    script: list[tuple[float, Happening, list[str]]]
    limits: Limits = Limits()


def signals(*names: str) -> list[Signal]:
    return [Signal(name=name, stream=f"Panel.{name}") for name in names]


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
MOVING: list[tuple[float, Happening, list[str]]] = [
    (1.0, ("returns", "Work"), ["Pick --> Move", "start Base.drive(speed=0.5)"]),
]
LEFT_MOVE = [
    *MOVING,
    (
        2.0,
        ("returns", "Move", 7),
        ["Move -succeeded-> Pick", "cancel Base.stop", "set deadline Move 4.0", "Pick --> Move"],
    ),
]

SIGNALS = OrchestratorDefinition(
    root="Top",
    vars={"presses": 0},
    signals=signals("press"),
    nodes=[
        Node(name="Top", children=["Outer", "Rest"], initial="Outer"),
        Node(
            name="Outer",
            children=["Left", "Right"],
            initial="Left",
            transitions=[Transition(trigger="press", if_="presses > 1", goto="Rest")],
        ),
        Node(
            name="Left",
            transitions=[Transition(trigger="press", goto="Right", set={"presses": "presses + 1"})],
        ),
        Node(
            name="Right",
            transitions=[Transition(trigger="press", goto="Left", set={"presses": "presses + 1"})],
        ),
        Node(name="Rest", transitions=[Transition(trigger="press", goto="Outer")]),
    ],
)

EVENTS = OrchestratorDefinition(
    root="Top",
    constants={"low": 10.0},
    signals=signals("level"),
    events=[Event(name="empty", when="level < low"), Event(name="filled", when="level >= low")],
    nodes=[
        Node(name="Top", children=["Run", "Rest"], initial="Run"),
        Node(name="Run", transitions=[Transition(trigger="empty", goto="Rest")]),
        Node(name="Rest", transitions=[Transition(trigger="filled", goto="Run")]),
    ],
)

TIMERS = OrchestratorDefinition(
    root="Top",
    constants={"pause": 2.0},
    signals=signals("skip", "again"),
    nodes=[
        Node(name="Top", children=["Wait", "Go"], initial="Wait"),
        Node(
            name="Wait",
            transitions=[
                Transition(after="pause", goto="Go"),
                Transition(trigger="skip", goto="Go"),
                Transition(trigger="again", goto="Wait"),
            ],
        ),
        Node(name="Go", transitions=[Transition(after=5, goto="Wait")]),
    ],
)

CHECKS = OrchestratorDefinition(
    root="Top",
    vars={"ready": False, "tidy": False},
    signals=signals("go", "leave", "halt"),
    nodes=[
        Node(name="Top", children=["Work", "Safe", "Stop"], initial="Work"),
        Node(
            name="Work",
            requires=[Check(expect=["ready"], fallback="Safe")],
            ensures=[Check(expect=["tidy"], fallback="Safe")],
            transitions=[
                Transition(trigger="leave", goto="Stop"),
                Transition(trigger="halt", mode="immediate", goto="Stop"),
            ],
        ),
        Node(
            name="Safe",
            transitions=[Transition(trigger="go", goto="Work", set={"ready": True})],
        ),
        Node(name="Stop", transitions=[Transition(trigger="go", goto="Work")]),
    ],
)

# Job is inside Work. Its `ensures` has no fallback and nothing handles its `failed`.
INNER = OrchestratorDefinition(
    root="Top",
    vars={"tidy": False},
    signals=signals("leave"),
    activities=[
        Activity(name="grip", start="Arm.grip", cancel="Arm.release"),
        Activity(name="drive", start="Base.drive", cancel="Base.stop"),
        Activity(name="turn", start="Base.turn", cancel="Base.stop"),
    ],
    nodes=[
        Node(name="Top", children=["Work", "Stop"], initial="Work"),
        Node(
            name="Work",
            children=["Job"],
            initial="Job",
            transitions=[Transition(trigger="leave", goto="Stop")],
        ),
        Node(name="Job", activity="grip", ensures=[Check(expect=["tidy"])]),
        Node(name="Stop", transitions=[Transition(trigger="leave", goto="Work")]),
    ],
)

PARALLEL = OrchestratorDefinition(
    root="Top",
    signals=signals("left", "right", "all", "bad"),
    nodes=[
        Node(name="Top", children=["Both", "After"], initial="Both"),
        Node(
            name="Both",
            parallel=["Left", "Right"],
            outcomes=["done", "failed"],
            transitions=[
                Transition(trigger="done", goto="After"),
                Transition(trigger="failed", goto="After"),
            ],
        ),
        Node(
            name="Left",
            transitions=[
                Transition(trigger="left", goto="done"),
                Transition(trigger="all", goto="done"),
            ],
        ),
        Node(
            name="Right",
            transitions=[
                Transition(trigger="right", goto="done"),
                Transition(trigger="all", goto="done"),
                Transition(trigger="bad", goto="failed"),
            ],
        ),
        Node(name="After", transitions=[Transition(trigger="all", goto="Both")]),
    ],
)

LOOP = OrchestratorDefinition(
    root="Top",
    nodes=[
        Node(name="Top", children=["Ping", "Pong"], initial="Ping"),
        Node(name="Ping", transitions=[Transition(goto="Pong")]),
        Node(name="Pong", transitions=[Transition(goto="Ping")]),
    ],
)

CASES = [
    Case(
        "an activity on a node with nodes inside, a switch node and a held start",
        WORK,
        ["start Base.enable()"],
        [
            *LEFT_MOVE,
            (2.5, ("canceled", "Move"), ["clear deadline Move 4.0", "start Base.drive(speed=0.5)"]),
        ],
    ),
    Case(
        "a failed walks up to the first node that handles it",
        WORK,
        ["start Base.enable()"],
        [
            *MOVING,
            (
                2.0,
                ("raises", "Move"),
                [
                    "Work -failed-> Work",
                    "cancel Base.stop",
                    "set deadline Move 4.0",
                    "cancel Base.disable",
                    "set deadline Work 4.0",
                    "start Base.enable()",
                ],
            ),
        ],
    ),
    Case(
        "a cancel that misses its deadline is a fault",
        WORK,
        ["start Base.enable()"],
        [
            *LEFT_MOVE,
            (
                4.0,
                ("deadline", "Move"),
                [
                    "cancel Base.disable",
                    "cancel Base.stop",
                    "fault at Move: a `cancel` that has not returned by its deadline",
                ],
            ),
        ],
    ),
    Case(
        "a cancel that raises is a fault",
        WORK,
        ["start Base.enable()"],
        [
            *LEFT_MOVE,
            (
                2.5,
                ("cancel raises", "Move"),
                [
                    "clear deadline Move 4.0",
                    "cancel Base.disable",
                    "cancel Base.stop",
                    "fault at Move: a `cancel` that raised",
                ],
            ),
        ],
    ),
    Case(
        "a signal fires on every message, and the outermost node whose `if` passes takes it",
        SIGNALS,
        [],
        [
            (1.0, ("signal", "press"), ["Left -press-> Right"]),
            (2.0, ("signal", "press"), ["Right -press-> Left"]),
            (3.0, ("signal", "press"), ["Outer -press-> Rest"]),
        ],
    ),
    Case(
        "a defined event is taken when its expression becomes true",
        EVENTS,
        [],
        [
            (1.0, ("signal", "level", 50.0), []),
            (2.0, ("signal", "level", 5.0), ["Run -empty-> Rest"]),
            (3.0, ("signal", "level", 5.0), []),
            (4.0, ("signal", "level", 60.0), ["Rest -filled-> Run"]),
        ],
    ),
    Case(
        "a timer starts on entering its node and stops on leaving it",
        TIMERS,
        ["set timer Wait 2.0"],
        [
            (
                1.0,
                ("signal", "again"),
                ["Wait -again-> Wait", "clear timer Wait 2.0", "set timer Wait 3.0"],
            ),
            (
                1.5,
                ("signal", "skip"),
                ["Wait -skip-> Go", "clear timer Wait 3.0", "set timer Go 6.5"],
            ),
            (6.5, ("timer", "Go"), ["Go -after-> Wait", "set timer Wait 8.5"]),
        ],
    ),
    Case(
        "a failed check goes to its fallback, and an immediate transition skips `ensures`",
        CHECKS,
        ["Work -requires-> Safe"],
        [
            (1.0, ("signal", "go"), ["Safe -go-> Work"]),
            (2.0, ("signal", "leave"), ["Work -leave-> Stop", "Work -ensures-> Safe"]),
            (3.0, ("signal", "go"), ["Safe -go-> Work"]),
            (4.0, ("signal", "halt"), ["Work -halt-> Stop"]),
        ],
    ),
    Case(
        "a failed `ensures` on a node an outer node left is a fault",
        INNER,
        ["start Arm.grip()"],
        [
            (
                1.0,
                ("signal", "leave"),
                [
                    "Work -leave-> Stop",
                    "cancel Arm.release",
                    "set deadline Job 3.0",
                    "clear deadline Job 3.0",
                    "cancel Arm.release",
                    "cancel Base.stop",
                    "fault at Job: a failed `ensures` check on a node an outer node left",
                ],
            ),
        ],
    ),
    Case(
        "a `failed` that no node handles is a fault",
        INNER,
        ["start Arm.grip()"],
        [
            (
                1.0,
                ("raises", "Job"),
                [
                    "cancel Arm.release",
                    "cancel Base.stop",
                    "fault at Job: a `failed` that no node handles",
                ],
            ),
        ],
    ),
    Case(
        "a parallel node raises `done` when every child has reached it",
        PARALLEL,
        [],
        [
            (1.0, ("signal", "left"), ["Left -left-> done"]),
            (2.0, ("signal", "right"), ["Right -right-> done", "Both -done-> After"]),
        ],
    ),
    Case(
        "a signal is offered to each child of a parallel node",
        PARALLEL,
        [],
        [
            (
                1.0,
                ("signal", "all"),
                ["Left -all-> done", "Right -all-> done", "Both -done-> After"],
            )
        ],
    ),
    Case(
        "a child reaching `failed` fails its parallel node at once",
        PARALLEL,
        [],
        [(1.0, ("signal", "bad"), ["Right -bad-> failed", "Both -failed-> After"])],
    ),
    Case(
        "too many transitions with no outside event is a fault",
        LOOP,
        [
            "Ping --> Pong",
            "Pong --> Ping",
            "Ping --> Pong",
            "fault at Pong: too many transitions in a row with no outside event",
        ],
        [],
        Limits(transition_cap=3),
    ),
]


def label(timer: Timer) -> str:
    kind = "timer" if timer.cancel is None else "deadline"
    return f"{kind} {timer.node} {timer.due}"


def render(effect: Effect) -> str:
    match effect:
        case TransitionRecord(node, trigger, goto):
            return f"{node} -{trigger or ''}-> {goto}"
        case SendStart(call):
            args = ", ".join(f"{name}={value!r}" for name, value in call.args.items())
            return f"start {call.rpc}({args})"
        case SendCancel(call):
            return f"cancel {call.rpc}"
        case SetTimer(timer):
            return f"set {label(timer)}"
        case ClearTimer(timer):
            return f"clear {label(timer)}"
        case FaultReport(rule, node):
            return f"fault at {node}: {rule}"


def queue_item(runtime: Runtime, happening: Happening) -> QueueItem:
    kind, name, *rest = happening
    value = rest[0] if rest else None
    if kind == "signal":
        return QueueItem("signal", key=name, value=value)
    if kind in ("timer", "deadline"):
        timers = [t for t in runtime.timers if t.node == name and label(t).startswith(kind)]
        return QueueItem("timer", timer=timers[0])
    sent = "start" if kind in ("returns", "raises") else "cancel"
    call = next(call for call in runtime.calls if call.node == name and call.kind == sent)
    return QueueItem(f"{sent}_return", call=call, value=value, raised="raises" in kind)


@pytest.mark.parametrize("case", CASES, ids=lambda case: case.name)
def test_cases(case: Case) -> None:
    checked = check(case.definition)
    assert checked.problems == ()
    runtime, effects = start(checked, 0.0, case.limits)
    assert [render(effect) for effect in effects] == case.startup
    for now, happening, expected in case.script:
        item = queue_item(runtime, happening)
        runtime, effects = step(checked, runtime, item, now, case.limits)
        assert [render(effect) for effect in effects] == expected, happening


def test_a_stale_return_and_a_cleared_timer_are_discarded() -> None:
    checked = check(WORK)
    runtime, _ = start(checked, 0.0)
    runtime, _ = step(checked, runtime, queue_item(runtime, ("returns", "Work")), 1.0)
    drive = queue_item(runtime, ("returns", "Move", 7))
    runtime, _ = step(checked, runtime, queue_item(runtime, ("raises", "Move")), 2.0)
    assert runtime.active == {"Work"}

    assert step(checked, runtime, drive, 3.0) == (runtime, [])
    cleared = QueueItem("timer", timer=Timer("Work", 9.0, transition=0))
    assert step(checked, runtime, cleared, 3.0) == (runtime, [])


def test_enqueue_replaces_an_unhandled_item_for_the_same_signal() -> None:
    items = [QueueItem("signal", key="level", value=1.0), QueueItem("timer")]
    enqueue(items, QueueItem("signal", key="level", value=2.0))
    enqueue(items, QueueItem("signal", key="skip"))
    assert items == [
        QueueItem("signal", key="level", value=2.0),
        QueueItem("timer"),
        QueueItem("signal", key="skip"),
    ]
