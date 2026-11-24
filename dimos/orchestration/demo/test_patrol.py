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

from dimos_lcm.actionlib_msgs import GoalStatus
import pytest

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.orchestration import engine
from dimos.orchestration.binding import Deployed, bind
from dimos.orchestration.demo.fake_robot import (
    FAILING_GOAL,
    FakeBase,
    FakeBaseConfig,
    FakeBattery,
    FakeEstop,
)
from dimos.orchestration.engine import (
    Effect,
    FaultReport,
    QueueItem,
    ReturnItem,
    SendCancel,
    SendStart,
    SetTimer,
    StreamItem,
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.loader import load_and_check

PATROL = Path(__file__).parent / "patrol.toml"
FAKES = (FakeBase, FakeBattery, FakeEstop)

# One waypoint, from reaching it to setting off for the next.
VISIT = [
    ("Navigating", "if nav_state == SUCCEEDED && nav_goal_id == nav.goal", "Scanning"),
    ("Sweeping", "on succeeded", "done"),
    ("Dwelling", "after", "done"),
    ("Scanning", "on done", "NextWaypoint"),
]
NEXT = ("NextWaypoint", "if patrol.index < size(route)", "Navigating")


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
    checked = bind(load_and_check(PATROL), _deployed())
    assert checked.problems == ()

    # The world answers as the fake modules do, on a clock the test owns.
    base = FakeBaseConfig()
    order = itertools.count()
    world: list[tuple[float, int, QueueItem]] = []
    records: list[tuple[str, str | None, str]] = []
    goals: list[str] = []

    def later(when: float, item: QueueItem) -> None:
        heapq.heappush(world, (when, next(order), item))

    def perform(effects: Sequence[Effect], now: float) -> None:
        for effect in effects:
            match effect:
                case TransitionRecord(node, cause, goto):
                    records.append((node, cause, goto))
                case SetTimer(timer):
                    later(timer.due, TimerItem(timer))
                case SendStart(call) if call.rpc == "FakeBase.go_to":
                    goals.append(str(call.args["name"]))
                    if len(goals) == FAILING_GOAL:
                        later(now + 0.01, ReturnItem(call, raised=True))
                        continue
                    goal = f"goal-{len(goals)}"
                    later(now + 0.01, ReturnItem(call, goal))
                    arrived = {"nav_state": GoalStatus.SUCCEEDED, "nav_goal_id": goal}
                    later(now + base.arrive_s, StreamItem("FakeBase.status", arrived))
                case SendStart(call) if call.rpc == "FakeBase.spin":
                    later(now + float(str(call.args["turns"])) * base.spin_s, ReturnItem(call))
                case SendStart(call) | SendCancel(call):
                    later(now + 0.01, ReturnItem(call))
                case FaultReport():
                    pytest.fail(f"the patrol faulted: {effect}")

    later(0.1, StreamItem("FakeEstop.pressed", {"estop_button": False}))
    runtime, effects = engine.start(checked, 0.0)
    perform(effects, 0.0)
    wanted = ["lobby", "lab", "lab", "kitchen", "lobby"]
    while len(goals) < len(wanted):
        now, _, item = heapq.heappop(world)
        runtime, effects = engine.step(checked, runtime, item, now)
        perform(effects, now)

    # The second goal fails and is tried again, and the route starts over after its last place.
    assert goals == wanted
    assert records == [
        ("Operating", "requires", "Estopped"),
        ("Estopped", "if !estop_button", "Operating"),
        NEXT,
        *VISIT,
        NEXT,
        ("Navigating", "on failed", "Navigating"),
        *VISIT,
        NEXT,
        *VISIT,
        ("NextWaypoint", None, "Navigating"),
    ]
