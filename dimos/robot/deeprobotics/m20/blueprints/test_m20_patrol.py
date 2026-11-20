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

"""The patrol file fits the modules it drives, and loops the goals between charges."""

from collections import deque
from collections.abc import Sequence
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.sensor_msgs import BatteryState as LCMBatteryState
from dimos_lcm.std_msgs import Time
import pytest

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
from dimos.msgs.std_msgs.String import String
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
    SetTimer,
    StreamItem,
    Timer,
    TimerItem,
)
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.orchestrator import Orchestrator
from dimos.orchestration.plain import field_to_plain, to_plain
from dimos.robot.deeprobotics.m20.connection import M20Connection

PATROL = Path(__file__).parent / "m20_patrol.toml"
MODULES = (MLSPlannerNative, M20Connection, LocalMapRelocalization, Orchestrator)
GOALS = [
    "hardware",
    "kitchenette",
    "lobby",
    "common_area",
    "side_entrance",
    "bathroom",
    "garage",
    "flag",
    "robot_area",
    "hugo",
]


def _checked() -> CheckedDefinition:
    deployed = Deployed(
        modules=frozenset(module.__name__ for module in MODULES),
        rpcs={
            f"{module.__name__}.{name}": extract_rpc_info(fn)
            for module in MODULES
            for name, fn in module.rpcs.items()
        },
        streams={
            "MLSPlannerNative.nav_status": GoalStatus,
            "M20Connection.battery": BatteryState,
            "Orchestrator.orchestrator_request": String,
        },
    )
    return bind(load_and_check(PATROL), deployed)


def test_the_patrol_file_fits_the_planner_the_connection_and_the_relocalizer() -> None:
    assert _checked().problems == ()


def _battery(status: int, level: float) -> StreamItem:
    state = BatteryState(voltage=80.0, percentage=level, power_supply_status=status)
    values = {
        "battery": field_to_plain(state, "percentage"),
        "power": field_to_plain(state, "power_supply_status"),
    }
    return StreamItem("M20Connection.battery", values)


DRAIN = 0.04
SWAPPED = 0.25
CHARGING = LCMBatteryState.POWER_SUPPLY_STATUS_CHARGING
DISCHARGING = LCMBatteryState.POWER_SUPPLY_STATUS_DISCHARGING
NOT_CHARGING = LCMBatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING


def _walk(
    undocks: int,
    drain: float = DRAIN,
    failed_docks: int = 0,
    stalled_docks: int = 0,
    silent: str | None = None,
    dock_at: str | None = None,
) -> tuple[list[str], list[str], list[float], list[FaultReport]]:
    """Run the file from the charger until the given undock.

    Every drive uses `drain` of the battery, and every dock after the first `failed_docks`
    charges it to 75% and then 80%, except the next `stalled_docks`, which stop charging
    at 75%. Sitting down swaps in a battery at 25%. The planner never reports on the
    `silent` goal. An operator asks to dock during the first drive to `dock_at`.
    Time passes only when nothing else is waiting, so timers fire last.
    """
    checked = _checked()
    calls: list[str] = []
    facing: list[str] = []
    undocked_at: list[float] = []
    faults: list[FaultReport] = []
    level = 0.85
    docks = 0
    requested = False
    queue: deque[QueueItem] = deque([_battery(CHARGING, level)])
    timers: list[Timer] = []
    runtime = engine.Runtime({})

    def perform(effects: Sequence[Effect]) -> None:
        nonlocal level, docks, requested
        for effect in effects:
            match effect:
                case SendStart(call) if call.rpc == "LocalMapRelocalization.location":
                    calls.append(str(call.args["name"]))
                    pose = PoseStamped(frame_id="premap", position=[len(calls), 0.0, 0.0])
                    queue.append(ReturnItem(call, to_plain(pose)))
                case SendStart(call) if call.rpc == "MLSPlannerNative.go_to":
                    if call.args.get("face_heading") is True:
                        facing.append(calls[-1])
                    stamp = Time(len(calls), 0)
                    queue.append(ReturnItem(call, to_plain(stamp)))
                    if calls[-1] == silent:
                        continue
                    level = round(level - drain, 2)
                    queue.append(_battery(DISCHARGING, level))
                    if calls[-1] == dock_at and not requested:
                        requested = True
                        request = {"request": field_to_plain(String("dock"), "data")}
                        queue.append(StreamItem("Orchestrator.orchestrator_request", request))
                    status = GoalStatus(goal_id=GoalID(stamp=stamp), status=GoalStatus.SUCCEEDED)
                    reached = {
                        "nav_state": field_to_plain(status, "status"),
                        "nav_stamp": field_to_plain(status, "goal_id.stamp"),
                    }
                    queue.append(StreamItem("MLSPlannerNative.nav_status", reached))
                case SendStart(call) if call.rpc == "M20Connection.undock":
                    calls.append("undock")
                    battery = runtime.blackboard["battery"]
                    assert isinstance(battery, float)
                    undocked_at.append(battery)
                    if len(undocked_at) < undocks:
                        queue.append(_battery(DISCHARGING, level))
                        queue.append(ReturnItem(call))
                case SendStart(call) if call.rpc == "M20Connection.dock":
                    calls.append("dock")
                    docks += 1
                    if docks <= failed_docks:
                        queue.append(ReturnItem(call, raised=True))
                        continue
                    queue.append(ReturnItem(call))
                    if docks <= failed_docks + stalled_docks:
                        level = 0.75
                        queue.extend([_battery(CHARGING, level), _battery(NOT_CHARGING, level)])
                        continue
                    level = 0.80
                    queue.extend([_battery(CHARGING, 0.75), _battery(CHARGING, level)])
                case SendStart(call) if call.rpc == "M20Connection.liedown":
                    calls.append("sit")
                    queue.append(ReturnItem(call, True))
                    level = SWAPPED
                    queue.append(_battery(DISCHARGING, level))
                case SendStart(call) if call.rpc == "M20Connection.standup":
                    calls.append("stand")
                    queue.append(ReturnItem(call, True))
                case SendStart(call) | SendCancel(call):
                    queue.append(ReturnItem(call))
                case SetTimer(timer):
                    timers.append(timer)
                case FaultReport():
                    faults.append(effect)

    now = 0.0
    runtime, effects = engine.start(checked, now)
    perform(effects)
    while queue or timers:
        if queue:
            item = queue.popleft()
        else:
            timer = min(timers, key=lambda timer: timer.due)
            timers.remove(timer)
            now = max(now, timer.due)
            item = TimerItem(timer)
        runtime, effects = engine.step(checked, runtime, item, now)
        perform(effects)
    return calls, facing, undocked_at, faults


def test_the_robot_loops_until_the_battery_is_low_then_charges_and_resumes() -> None:
    calls, facing, undocked_at, faults = _walk(undocks=3)

    assert faults == []
    # 85% and 4% per drive: the fourth goal reads 69%, so the robot docks in the middle of it
    # and resumes it after charging. From 80%, the third goal reads 68%.
    dock = ["dock_approach", "predock", "dock"]
    assert calls == ["undock", *GOALS[:4], *dock, "undock", *GOALS[3:6], *dock, "undock"]
    assert undocked_at == pytest.approx([0.85, 0.80, 0.80])
    # Only the two docking points ask the planner to turn to their heading.
    assert facing == ["dock_approach", "predock"] * 2


def test_after_three_failed_docks_the_robot_walks_a_loop_and_docks_again() -> None:
    calls, _, undocked_at, faults = _walk(undocks=2, drain=0.02, failed_docks=3)

    assert faults == []
    # 2% per drive: the eighth goal reads 69%. After three failed docks the robot resumes
    # that goal and walks all ten before it heads for the dock again. The next goal's
    # lookup has started by the time the low battery takes over.
    dock = ["dock_approach", "predock", "dock"]
    loop = [*GOALS[7:], *GOALS[:7]]
    assert calls == ["undock", *GOALS[:8], *dock * 3, *loop, GOALS[7], *dock, "undock"]
    assert undocked_at == pytest.approx([0.85, 0.80])


def test_a_flat_robot_sits_until_a_charged_battery_goes_in_then_docks() -> None:
    calls, _, undocked_at, faults = _walk(undocks=2, failed_docks=3)

    assert faults == []
    # After three failed docks the loop drains the battery to 9% at its ninth goal.
    # The swapped battery reads 25%, so the robot stands and heads straight for the dock.
    dock = ["dock_approach", "predock", "dock"]
    flat = [*GOALS[3:], *GOALS[:2]]
    after_swap = ["sit", "stand", GOALS[1], *dock, "undock"]
    assert calls == ["undock", *GOALS[:4], *dock * 3, *flat, *after_swap]
    assert undocked_at == pytest.approx([0.85, 0.80])


def test_a_dock_that_stops_charging_docks_again() -> None:
    calls, _, undocked_at, faults = _walk(undocks=3, stalled_docks=1)

    assert faults == []
    # The first dock stops charging at 75%, so after a minute the robot undocks, drives the
    # approach again and docks. The second dock charges to 80%.
    dock = ["dock_approach", "predock", "dock"]
    assert calls == ["undock", *GOALS[:4], *dock, "undock", *dock, "undock"]
    assert undocked_at == pytest.approx([0.85, 0.75, 0.80])


def test_a_goal_the_planner_never_finishes_is_skipped_after_ten_minutes() -> None:
    calls, _, _, faults = _walk(undocks=2, silent=GOALS[1])

    assert faults == []
    # The planner never reports on kitchenette, so its timeout skips it without a retry.
    # Each reported drive uses 4%, and side_entrance reads 69%.
    dock = ["dock_approach", "predock", "dock"]
    assert calls == ["undock", *GOALS[:5], *dock, "undock"]


def test_an_operator_request_docks_once_then_the_robot_resumes() -> None:
    calls, _, undocked_at, faults = _walk(undocks=3, dock_at=GOALS[1])

    assert faults == []
    # "dock" arrives on the way to kitchenette. After charging the robot resumes that goal,
    # and docks next when the battery is low, so one request docks once.
    dock = ["dock_approach", "predock", "dock"]
    after = [GOALS[1], GOALS[2], GOALS[3], *dock, "undock"]
    assert calls == ["undock", *GOALS[:2], *dock, "undock", *after]
    assert undocked_at == pytest.approx([0.85, 0.80, 0.80])
