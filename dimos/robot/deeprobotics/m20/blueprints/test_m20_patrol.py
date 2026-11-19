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

"""The patrol file fits the modules it drives, and docks once from every goal before it finishes."""

from collections import deque
from collections.abc import Sequence
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.sensor_msgs import BatteryState as LCMBatteryState
from dimos_lcm.std_msgs import Time

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.mapping.relocalization.lidar.module import LocalMapRelocalization
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.BatteryState import BatteryState
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
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.plain import field_to_plain, to_plain
from dimos.robot.deeprobotics.m20.connection import M20Connection

PATROL = Path(__file__).parent / "m20_patrol.toml"
MODULES = (MLSPlannerNative, M20Connection, LocalMapRelocalization)
GOALS = [
    "couches",
    "workshop",
    "lobby",
    "common_area",
    "side_entrance",
    "bathroom",
    "garage",
    "flag",
    "robot_area",
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
        },
    )
    return bind(load_and_check(PATROL), deployed)


def test_the_patrol_file_fits_the_planner_the_connection_and_the_relocalizer() -> None:
    assert _checked().problems == ()


def _battery(status: int) -> StreamItem:
    state = BatteryState(voltage=80.0, percentage=1.0, power_supply_status=status)
    values = {
        "battery": field_to_plain(state, "percentage"),
        "power": field_to_plain(state, "power_supply_status"),
    }
    return StreamItem("M20Connection.battery", values)


def _walk() -> tuple[list[str], list[str], list[TransitionRecord], list[FaultReport]]:
    """Run the file from the charger with every goal, dock and undock succeeding."""
    checked = _checked()
    calls: list[str] = []
    facing: list[str] = []
    records: list[TransitionRecord] = []
    faults: list[FaultReport] = []
    queue: deque[QueueItem] = deque([_battery(LCMBatteryState.POWER_SUPPLY_STATUS_CHARGING)])

    def perform(effects: Sequence[Effect]) -> None:
        for effect in effects:
            match effect:
                case TransitionRecord():
                    records.append(effect)
                case SendStart(call) if call.rpc == "LocalMapRelocalization.location":
                    name = str(call.args["name"])
                    calls.append(name)
                    pose = PoseStamped(frame_id="premap", position=[len(calls), 0.0, 0.0])
                    queue.append(ReturnItem(call, to_plain(pose)))
                case SendStart(call) if call.rpc == "MLSPlannerNative.go_to":
                    if call.args.get("face_heading") is True:
                        facing.append(calls[-1])
                    stamp = Time(len(calls), 0)
                    queue.append(ReturnItem(call, to_plain(stamp)))
                    status = GoalStatus(goal_id=GoalID(stamp=stamp), status=GoalStatus.SUCCEEDED)
                    reached = {
                        "nav_state": field_to_plain(status, "status"),
                        "nav_stamp": field_to_plain(status, "goal_id.stamp"),
                    }
                    queue.append(StreamItem("MLSPlannerNative.nav_status", reached))
                case SendStart(call) if call.rpc == "M20Connection.undock":
                    calls.append("undock")
                    queue.append(_battery(LCMBatteryState.POWER_SUPPLY_STATUS_DISCHARGING))
                    queue.append(ReturnItem(call))
                case SendStart(call) if call.rpc == "M20Connection.dock":
                    calls.append("dock")
                    queue.append(_battery(LCMBatteryState.POWER_SUPPLY_STATUS_CHARGING))
                    queue.append(ReturnItem(call))
                case SendStart(call) | SendCancel(call):
                    queue.append(ReturnItem(call))
                case SetTimer(timer):
                    queue.append(TimerItem(timer))
                case FaultReport():
                    faults.append(effect)

    runtime, effects = engine.start(checked, 0.0)
    perform(effects)
    while queue:
        runtime, effects = engine.step(checked, runtime, queue.popleft(), 0.0)
        perform(effects)
    return calls, facing, records, faults


def test_the_robot_docks_once_from_every_goal_then_stays_on_the_charger() -> None:
    calls, facing, records, faults = _walk()

    assert faults == []
    # Each cycle walks the loop plus one goal, so cycle n docks from goal n.
    expected = []
    for cycle in range(len(GOALS)):
        loop = [
            GOALS[(cycle * (len(GOALS) + 1) + step) % len(GOALS)] for step in range(len(GOALS) + 1)
        ]
        expected += ["undock", *loop, "dock_approach", "predock", "dock"]
    assert calls == expected
    assert [loop[-4] for loop in _cycles(calls)] == GOALS
    # Only the two docking points ask the planner to turn to their heading.
    assert facing == ["dock_approach", "predock"] * len(GOALS)
    assert records[-1] == TransitionRecord("TopUp", "if cycle.count >= size(goals)", "Finished")


def _cycles(calls: list[str]) -> list[list[str]]:
    cycles: list[list[str]] = []
    for call in calls:
        if call == "undock":
            cycles.append([])
        cycles[-1].append(call)
    return cycles
