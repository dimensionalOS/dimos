# Copyright 2026 Dimensional Inc.
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

"""The patrol file fits the modules it drives, and a low battery sends the robot to charge."""

from collections import deque
from collections.abc import Sequence
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time
import pytest

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.navigation.global_planner.mls_planner.mls_planner_native import MLSPlannerNative
from dimos.orchestration import engine
from dimos.orchestration.binding import Deployed, bind
from dimos.orchestration.checks import CheckedDefinition
from dimos.orchestration.engine import (
    Effect,
    FaultReport,
    QueueItem,
    ReturnItem,
    SendCancel,
    SendStart,
    SignalItem,
    TransitionRecord,
)
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.plain import to_plain
from dimos.simulation.go2_sim.fake_battery import FakeBattery

PATROL = Path(__file__).parent / "go2_sim_patrol.toml"
MODULES = (MLSPlannerNative, FakeBattery)


def _checked() -> CheckedDefinition:
    deployed = Deployed(
        modules=frozenset(module.__name__ for module in MODULES),
        rpcs={
            f"{module.__name__}.{name}": extract_rpc_info(fn)
            for module in MODULES
            for name, fn in module.rpcs.items()
        },
        streams={"MLSPlannerNative.nav_status": GoalStatus, "FakeBattery.percent": Float32},
    )
    return bind(load_and_check(PATROL), deployed)


def test_the_patrol_file_fits_the_planner_and_the_battery() -> None:
    assert _checked().problems == ()


def test_go_to_returns_a_stamp_the_file_can_compare() -> None:
    # A return type that does not resolve has no shape, and the stamp comparison goes unchecked.
    assert extract_rpc_info(MLSPlannerNative.go_to).return_annotation is Time


def test_a_low_battery_sends_the_patrol_to_the_charger_and_back() -> None:
    checked = _checked()
    calls: list[str] = []
    records: list[tuple[str, str | None, str]] = []
    queue: deque[QueueItem] = deque()

    def perform(effects: Sequence[Effect]) -> None:
        for effect in effects:
            match effect:
                case TransitionRecord(node, trigger, goto):
                    records.append((node, trigger, goto))
                case SendStart(call) if call.rpc == "MLSPlannerNative.go_to":
                    calls.append(f"go_to({call.args['x']}, {call.args['y']})")
                    # Each goal's stamp is its place in the calls made.
                    queue.append(ReturnItem(call, to_plain(Time(len(calls), 0))))
                case SendStart(call) | SendCancel(call):
                    calls.append(call.rpc.split(".")[1])
                    queue.append(ReturnItem(call))
                case FaultReport():
                    pytest.fail(f"the patrol faulted: {effect}")

    runtime, effects = engine.start(checked, 0.0)

    def run(*items: QueueItem) -> None:
        nonlocal runtime
        queue.extend(items)
        while queue:
            runtime, effects = engine.step(checked, runtime, queue.popleft(), 0.0)
            perform(effects)

    def reached() -> SignalItem:
        status = GoalStatus(goal_id=GoalID(stamp=Time(len(calls), 0)), status=GoalStatus.SUCCEEDED)
        return SignalItem("nav_status", to_plain(status))

    perform(effects)
    run(SignalItem("battery_pct", 15.0))
    run(reached())
    run(SignalItem("battery_pct", 96.0))

    # The leg under way is canceled, and the patrol takes up the same waypoint after charging.
    assert calls == [
        "go_to(4.0, 4.0)",
        "stop_navigation",
        "go_to(1.0, 1.0)",
        "stop_navigation",
        "start_charging",
        "stop_charging",
        "go_to(4.0, 4.0)",
    ]
    assert records == [
        ("NextWaypoint", None, "Navigating"),
        ("Patrolling", "battery_low", "Charging"),
        ("ReturnToCharger", "arrived", "OnCharger"),
        ("OnCharger", "charged", "done"),
        ("Charging", "done", "Patrolling"),
        ("NextWaypoint", None, "Navigating"),
    ]
