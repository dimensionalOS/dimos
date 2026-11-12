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

from collections.abc import Sequence
import heapq
import itertools
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
import pytest

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.orchestration import engine
from dimos.orchestration.demo.fake_robot import FakeBase, FakeBattery, FakeEstop
from dimos.orchestration.engine import (
    Effect,
    FaultReport,
    Limits,
    QueueItem,
    ReturnItem,
    SendCancel,
    SendStart,
    SetTimer,
    SignalItem,
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.orchestrator import Deployed, bind
from dimos.orchestration.plain import to_plain

PATROL = Path(__file__).parent / "patrol.toml"
FAKES = (FakeBase, FakeBattery, FakeEstop)

# One waypoint, from reaching it to setting off for the next.
VISIT = [
    ("Navigating", "arrived", "Scanning"),
    ("Sweeping", "succeeded", "done"),
    ("Dwelling", "after", "done"),
    ("Scanning", "done", "NextWaypoint"),
    ("NextWaypoint", None, "Navigating"),
]


def _deployed() -> Deployed:
    return Deployed(
        modules=frozenset(fake.__name__ for fake in FAKES),
        rpcs={
            f"{fake.__name__}.{name}": extract_rpc_info(fn)
            for fake in FAKES
            for name, fn in fake.rpcs.items()
        },
        streams={
            "FakeBase.status": GoalStatus,
            "FakeBattery.percent": Float32,
            "FakeEstop.pressed": Bool,
        },
    )


def test_the_patrol_file_runs_one_loop_of_its_route() -> None:
    checked, problems = bind(load_and_check(PATROL), _deployed())
    assert problems == []

    # The world answers as the fake modules do, on a clock the test owns.
    limits = Limits(cancel_deadline_s=2.0, transition_cap=100)
    order = itertools.count()
    world: list[tuple[float, int, QueueItem]] = []
    records: list[tuple[str, str | None, str]] = []
    goals: list[str] = []

    def later(when: float, item: QueueItem) -> None:
        heapq.heappush(world, (when, next(order), item))

    def perform(effects: Sequence[Effect], now: float) -> None:
        for effect in effects:
            match effect:
                case TransitionRecord(node, trigger, goto):
                    records.append((node, trigger, goto))
                case SetTimer(timer):
                    later(timer.due, TimerItem(timer))
                case SendStart(call) if call.rpc == "FakeBase.go_to":
                    goals.append(str(call.args["name"]))
                    if len(goals) == 2:
                        later(now + 0.01, ReturnItem(call, raised=True))
                        continue
                    goal = f"goal-{len(goals)}"
                    reached = GoalStatus(goal_id=GoalID(id=goal), status=GoalStatus.SUCCEEDED)
                    later(now + 0.01, ReturnItem(call, goal))
                    later(now + 1.0, SignalItem("nav_status", to_plain(reached)))
                case SendStart(call) | SendCancel(call):
                    later(now + (0.5 if call.rpc == "FakeBase.spin" else 0.01), ReturnItem(call))
                case FaultReport():
                    pytest.fail(f"the patrol faulted: {effect}")

    later(0.1, SignalItem("estop_button", False))
    runtime, effects = engine.start(checked, 0.0, limits)
    perform(effects, 0.0)
    wanted = ["lobby", "lab", "lab", "kitchen", "lobby"]
    while len(goals) < len(wanted):
        now, _, item = heapq.heappop(world)
        runtime, effects = engine.step(checked, runtime, item, now, limits)
        perform(effects, now)

    # The second goal fails and is tried again, and the route starts over after its last place.
    assert goals == wanted
    assert records == [
        ("Operating", "requires", "Estopped"),
        ("Estopped", "estop_cleared", "Operating"),
        ("NextWaypoint", None, "Navigating"),
        *VISIT,
        ("Navigating", "failed", "Navigating"),
        *VISIT,
        *VISIT,
    ]
