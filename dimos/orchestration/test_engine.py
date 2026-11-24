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
from typing import Literal

import pytest

from dimos.orchestration.checks import check
from dimos.orchestration.definition import (
    Activity,
    Check,
    Node,
    OrchestratorDefinition,
    StreamValue,
    Transition,
)
from dimos.orchestration.engine import (
    AfterTimer,
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
    enqueue,
    start,
    step,
)
from dimos.orchestration.expressions import Plain

HappeningKind = Literal["message", "returns", "raises", "canceled", "timer", "deadline"]
# What happens outside the engine: a kind, a stream or node name, and what it carries if anything.
Happening = tuple[HappeningKind, str] | tuple[HappeningKind, str, Plain]


@dataclass
class Case:
    """A scripted run of a definition, where None stands for the patrol fixture."""

    name: str
    definition: OrchestratorDefinition | None
    startup: list[str]
    script: list[tuple[float, Happening, list[str]]]
    limits: Limits = Limits()


def values(*names: str) -> list[StreamValue]:
    return [StreamValue(name=name, stream=f"Panel.{name}", field="data") for name in names]


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
            transitions=[Transition(on="failed", goto="Work")],
        ),
        Node(name="Pick", transitions=[Transition(goto="Move")]),
        Node(
            name="Move",
            activity="drive",
            args={"speed": "top_speed"},
            result="drive.id",
            transitions=[Transition(on="succeeded", goto="Pick", set={"laps": "laps + 1"})],
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
        ["Move -on succeeded-> Pick", "cancel Base.stop", "set deadline Move 4.0", "Pick --> Move"],
    ),
]

STREAMS = OrchestratorDefinition(
    root="Top",
    vars={"presses": 0},
    nodes=[
        Node(name="Top", children=["Outer", "Rest"], initial="Outer"),
        Node(
            name="Outer",
            children=["Left", "Right"],
            initial="Left",
            transitions=[Transition(on="Panel.press", if_="presses > 1", goto="Rest")],
        ),
        Node(
            name="Left",
            transitions=[
                Transition(on="Panel.press", goto="Right", set={"presses": "presses + 1"})
            ],
        ),
        Node(
            name="Right",
            transitions=[Transition(on="Panel.press", goto="Left", set={"presses": "presses + 1"})],
        ),
        Node(name="Rest", transitions=[Transition(on="Panel.press", goto="Outer")]),
    ],
)

CONDITIONS = OrchestratorDefinition(
    root="Top",
    constants={"low": 10.0},
    stream_values=values("level"),
    nodes=[
        Node(name="Top", children=["Run", "Rest"], initial="Run"),
        Node(name="Run", transitions=[Transition(if_="level < low", goto="Rest")]),
        Node(name="Rest", transitions=[Transition(if_="level >= low", goto="Run")]),
    ],
)

TIMERS = OrchestratorDefinition(
    root="Top",
    constants={"pause": 2.0},
    nodes=[
        Node(name="Top", children=["Wait", "Go"], initial="Wait"),
        Node(
            name="Wait",
            transitions=[
                Transition(after="pause", goto="Go"),
                Transition(on="Panel.skip", goto="Go"),
                Transition(on="Panel.again", goto="Wait"),
            ],
        ),
        Node(name="Go", transitions=[Transition(after=5, goto="Wait")]),
    ],
)

NEVER = OrchestratorDefinition(
    root="Top",
    constants={"rate": 0.0},
    nodes=[
        Node(name="Top", children=["Wait", "Go"], initial="Wait"),
        Node(name="Wait", transitions=[Transition(after="1.0 / rate", goto="Go")]),
        Node(name="Go", transitions=[Transition(after=5, goto="Wait")]),
    ],
)

CHECKS = OrchestratorDefinition(
    root="Top",
    vars={"ready": False, "tidy": False},
    nodes=[
        Node(name="Top", children=["Work", "Safe", "Stop"], initial="Work"),
        Node(
            name="Work",
            requires=[Check(expect=["ready"], fallback="Safe")],
            ensures=[Check(expect=["tidy"], fallback="Safe")],
            transitions=[
                Transition(on="Panel.leave", goto="Stop"),
                Transition(on="Panel.halt", mode="immediate", goto="Stop"),
            ],
        ),
        Node(
            name="Safe",
            transitions=[Transition(on="Panel.go", goto="Work", set={"ready": True})],
        ),
        Node(name="Stop", transitions=[Transition(on="Panel.go", goto="Work")]),
    ],
)

PARALLEL = OrchestratorDefinition(
    root="Top",
    nodes=[
        Node(name="Top", children=["Both", "After"], initial="Both"),
        Node(
            name="Both",
            parallel=["Left", "Right"],
            outcomes=["done", "failed"],
            transitions=[
                Transition(on="done", goto="After"),
                Transition(on="failed", goto="After"),
            ],
        ),
        Node(name="Left", transitions=[Transition(on="Panel.all", goto="done")]),
        Node(
            name="Right",
            transitions=[
                Transition(on="Panel.all", goto="done"),
                Transition(on="Panel.bad", goto="failed"),
            ],
        ),
        Node(name="After", transitions=[Transition(on="Panel.all", goto="Both")]),
    ],
)

# Entering Both starts drive and a timer on Left. Then the requires of Right fails.
HALFWAY = OrchestratorDefinition(
    root="Top",
    activities=[Activity(name="drive", start="Base.drive", cancel="Base.stop")],
    nodes=[
        Node(name="Top", children=["Idle", "Both"], initial="Idle"),
        Node(name="Idle", transitions=[Transition(on="Panel.go", goto="Both")]),
        Node(
            name="Both",
            parallel=["Left", "Right"],
            outcomes=["done"],
            transitions=[Transition(on="done", goto="Idle")],
        ),
        Node(name="Left", activity="drive", transitions=[Transition(after=5, goto="done")]),
        Node(name="Right", requires=[Check(expect=["false"])]),
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

WAITING = OrchestratorDefinition(
    root="Top",
    stream_values=values("ok"),
    nodes=[
        Node(name="Top", children=["Wait", "Go"], initial="Wait"),
        Node(
            name="Wait",
            transitions=[Transition(after=2, goto="Go"), Transition(if_="ok", goto="Go")],
        ),
        Node(name="Go", transitions=[Transition(on="Panel.ok", goto="Wait")]),
    ],
)

DOCKING = OrchestratorDefinition(
    root="Top",
    vars={"tries": 0},
    activities=[Activity(name="dock", start="Dock.dock")],
    nodes=[
        Node(name="Top", children=["Docking", "Docked", "Stuck"], initial="Docking"),
        Node(
            name="Docking",
            activity="dock",
            transitions=[
                Transition(on="succeeded", goto="Docked"),
                Transition(
                    on="failed", if_="tries < 1", goto="Docking", set={"tries": "tries + 1"}
                ),
                Transition(on="failed", goto="Stuck"),
            ],
        ),
        Node(name="Docked", transitions=[Transition(on="Panel.again", goto="Docking")]),
        Node(name="Stuck", transitions=[Transition(on="Panel.again", goto="Docking")]),
    ],
)

ARRIVED = "nav_state == SUCCEEDED && nav_goal_id == nav.goal"
NEXT = "NextWaypoint -if patrol.index < size(route)-> Navigating"
PATROL_STARTUP = [NEXT, "start FakeBase.go_to(name='lobby')"]


def waypoint(now: float, goal: str, then: str) -> list[tuple[float, Happening, list[str]]]:
    """One waypoint of the patrol: go_to returns a goal, the goal is reached, stop returns."""
    reached: Plain = {"nav_state": 3, "nav_goal_id": goal}
    return [
        (now, ("returns", "Navigating", goal), []),
        (
            now + 1.0,
            ("message", "FakeBase.status", reached),
            [
                f"Navigating -if {ARRIVED}-> NextWaypoint",
                "cancel FakeBase.stop",
                f"set deadline Navigating {now + 3.0}",
                "NextWaypoint --> Navigating" if then == "lobby" else NEXT,
            ],
        ),
        (
            now + 1.5,
            ("canceled", "Navigating"),
            [f"clear deadline Navigating {now + 3.0}", f"start FakeBase.go_to(name='{then}')"],
        ),
    ]


CASES = [
    Case(
        "the patrol visits every waypoint and starts the route again",
        None,
        PATROL_STARTUP,
        [
            *waypoint(1.0, "goal 1", "lab"),
            *waypoint(3.0, "goal 2", "kitchen"),
            *waypoint(5.0, "goal 3", "lobby"),
        ],
    ),
    Case(
        "the patrol faults when `go_to` raises",
        None,
        PATROL_STARTUP,
        [
            (
                1.0,
                ("raises", "Navigating"),
                [
                    "cancel FakeBase.stop",
                    "fault at Navigating on start_return: a `failed` that no node handles",
                ],
            ),
        ],
    ),
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
        "a failed walks up, and the start of power waits for its own cancel",
        WORK,
        ["start Base.enable()"],
        [
            *MOVING,
            (
                2.0,
                ("raises", "Move"),
                [
                    "Work -on failed-> Work",
                    "cancel Base.stop",
                    "set deadline Move 4.0",
                    "cancel Base.disable",
                    "set deadline Work 4.0",
                ],
            ),
            (2.5, ("canceled", "Work"), ["clear deadline Work 4.0", "start Base.enable()"]),
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
                    "fault at Move on timer: a `cancel` that has not returned by its deadline",
                ],
            ),
        ],
    ),
    Case(
        "a message is offered on every arrival, and the outermost node whose `if` passes takes it",
        STREAMS,
        [],
        [
            (1.0, ("message", "Panel.press"), ["Left -on Panel.press-> Right"]),
            (2.0, ("message", "Panel.press"), ["Right -on Panel.press-> Left"]),
            (3.0, ("message", "Panel.press"), ["Outer -on Panel.press-> Rest"]),
        ],
    ),
    Case(
        "a transition with only an `if` is taken when its expression becomes true",
        CONDITIONS,
        [],
        [
            (1.0, ("message", "Panel.level", {"level": 50.0}), []),
            (2.0, ("message", "Panel.level", {"level": 5.0}), ["Run -if level < low-> Rest"]),
            (3.0, ("message", "Panel.level", {"level": 5.0}), []),
            (4.0, ("message", "Panel.level", {"level": 60.0}), ["Rest -if level >= low-> Run"]),
        ],
    ),
    Case(
        "a timer starts on entering its node and stops on leaving it",
        TIMERS,
        ["set timer Wait 2.0"],
        [
            (
                1.0,
                ("message", "Panel.again"),
                ["Wait -on Panel.again-> Wait", "clear timer Wait 2.0", "set timer Wait 3.0"],
            ),
            (
                1.5,
                ("message", "Panel.skip"),
                ["Wait -on Panel.skip-> Go", "clear timer Wait 3.0", "set timer Go 6.5"],
            ),
            (6.5, ("timer", "Go"), ["Go -after-> Wait", "set timer Wait 8.5"]),
        ],
    ),
    Case(
        "a failed check goes to its fallback, and an immediate transition skips `ensures`",
        CHECKS,
        ["Work -requires-> Safe"],
        [
            (1.0, ("message", "Panel.go"), ["Safe -on Panel.go-> Work"]),
            (
                2.0,
                ("message", "Panel.leave"),
                ["Work -on Panel.leave-> Stop", "Work -ensures-> Safe"],
            ),
            (3.0, ("message", "Panel.go"), ["Safe -on Panel.go-> Work"]),
            (4.0, ("message", "Panel.halt"), ["Work -on Panel.halt-> Stop"]),
        ],
    ),
    Case(
        "a message is offered to each child of a parallel node, which is done when all are",
        PARALLEL,
        [],
        [
            (
                1.0,
                ("message", "Panel.all"),
                [
                    "Left -on Panel.all-> done",
                    "Right -on Panel.all-> done",
                    "Both -on done-> After",
                ],
            )
        ],
    ),
    Case(
        "a child reaching `failed` fails its parallel node at once",
        PARALLEL,
        [],
        [
            (
                1.0,
                ("message", "Panel.bad"),
                ["Right -on Panel.bad-> failed", "Both -on failed-> After"],
            )
        ],
    ),
    Case(
        "a fault drops the starts and timers of its own step",
        HALFWAY,
        [],
        [
            (
                1.0,
                ("message", "Panel.go"),
                [
                    "Idle -on Panel.go-> Both",
                    "cancel Base.stop",
                    "fault at Right on Panel.go: a failed `requires` check with no fallback",
                ],
            ),
        ],
    ),
    Case(
        "too many transitions with no outside event is a fault",
        LOOP,
        [
            "Ping --> Pong",
            "Pong --> Ping",
            "Ping --> Pong",
            "fault at Pong on startup: too many transitions in a row with no outside event",
        ],
        [],
        Limits(transition_cap=3),
    ),
    Case(
        "an `after` that is not a finite number is a fault",
        NEVER,
        [
            "fault at Wait on startup: an expression error: "
            "`1.0 / rate` gave inf, not a finite number",
        ],
        [],
    ),
    Case(
        "an expression error is a fault, and a fault clears the timers of earlier steps",
        WAITING,
        ["set timer Wait 2.0"],
        [
            (
                1.0,
                ("message", "Panel.ok", {"ok": "yes"}),
                [
                    "clear timer Wait 2.0",
                    "fault at Wait on Panel.ok: an expression error: "
                    "`ok` gave 'yes', and true or false is needed",
                ],
            ),
        ],
    ),
    Case(
        "a `failed` is taken by the first transition on it whose `if` passes",
        DOCKING,
        ["start Dock.dock()"],
        [
            (
                1.0,
                ("raises", "Docking"),
                ["Docking -on failed-> Docking", "start Dock.dock()"],
            ),
            (2.0, ("raises", "Docking"), ["Docking -on failed-> Stuck"]),
        ],
    ),
]


def label(timer: Timer) -> str:
    kind = "timer" if isinstance(timer, AfterTimer) else "deadline"
    return f"{kind} {timer.node} {timer.due}"


def render(effect: Effect) -> str:
    match effect:
        case TransitionRecord(node, cause, goto):
            return f"{node} -{cause or ''}-> {goto}"
        case SendStart(call):
            args = ", ".join(f"{name}={value!r}" for name, value in call.args.items())
            return f"start {call.rpc}({args})"
        case SendCancel(call):
            return f"cancel {call.rpc}"
        case SetTimer(timer):
            return f"set {label(timer)}"
        case ClearTimer(timer):
            return f"clear {label(timer)}"
        case FaultReport(rule, node, event):
            return f"fault at {node} on {event}: {rule}"


def queue_item(runtime: Runtime, happening: Happening) -> QueueItem:
    kind, name, *rest = happening
    value = rest[0] if rest else None
    if kind == "message":
        return StreamItem(name, value or {})
    if kind in ("timer", "deadline"):
        timers = [t for t in runtime.timers if t.node == name and label(t).startswith(kind)]
        return TimerItem(timers[0])
    sent = "start" if kind in ("returns", "raises") else "cancel"
    call = next(call for call in runtime.calls if call.node == name and call.kind == sent)
    return ReturnItem(call, value, raised=kind == "raises")


@pytest.mark.parametrize("case", CASES, ids=lambda case: case.name)
def test_cases(case: Case, patrol: OrchestratorDefinition) -> None:
    checked = check(case.definition or patrol)
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
    cleared = TimerItem(AfterTimer("Work", 9.0, transition=0))
    assert step(checked, runtime, cleared, 3.0) == (runtime, [])


def test_enqueue_replaces_an_unhandled_item_for_the_same_stream() -> None:
    timer = TimerItem(AfterTimer("Wait", 2.0, transition=0))
    items: list[QueueItem] = [StreamItem("Panel.level", {"level": 1.0}), timer]
    enqueue(items, StreamItem("Panel.level", {"level": 2.0}))
    enqueue(items, StreamItem("Panel.skip"))
    assert items == [StreamItem("Panel.level", {"level": 2.0}), timer, StreamItem("Panel.skip")]
