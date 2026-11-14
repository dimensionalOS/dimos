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

"""The dock cycle file fits the modules it drives, and walks five cycles off and back on the dock."""

from collections import deque
from collections.abc import Sequence
import math
from pathlib import Path

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.std_msgs import Time

from dimos.core.introspection.module.info import extract_rpc_info
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
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
    SignalItem,
    TimerItem,
    TransitionRecord,
)
from dimos.orchestration.loader import load_and_check
from dimos.orchestration.plain import to_plain
from dimos.robot.deeprobotics.m20.connection import M20Connection

DOCK_CYCLE = Path(__file__).parent / "m20_dock_cycle.toml"
MODULES = (MLSPlannerNative, M20Connection)


def _checked() -> CheckedDefinition:
    deployed = Deployed(
        modules=frozenset(module.__name__ for module in MODULES),
        rpcs={
            f"{module.__name__}.{name}": extract_rpc_info(fn)
            for module in MODULES
            for name, fn in module.rpcs.items()
        },
        streams={"MLSPlannerNative.nav_status": GoalStatus, "M20Connection.odom": PoseStamped},
    )
    return bind(load_and_check(DOCK_CYCLE), deployed)


def test_the_dock_cycle_file_fits_the_planner_and_the_connection() -> None:
    assert _checked().problems == ()


def _pose(x: float, y: float, z: float, yaw_deg: float) -> PoseStamped:
    half_yaw = math.radians(yaw_deg) / 2
    return PoseStamped(
        frame_id="map",
        position=[x, y, z],
        orientation=[0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)],
    )


# On the dock at (2, 1) facing 30 degrees. The undock leaves the robot 1.2 m back along the
# dock's axis, 0.4 m to its left and turned to 40 degrees.
DOCKED = _pose(2.0, 1.0, 0.2, 30.0)
UNDOCKED = _pose(0.7608, 0.7464, 0.48, 40.0)


def _walk(docks_that_raise: int) -> tuple[list[str], list[TransitionRecord], list[FaultReport]]:
    checked = _checked()
    calls: list[str] = []
    records: list[TransitionRecord] = []
    faults: list[FaultReport] = []
    queue: deque[QueueItem] = deque()

    def perform(effects: Sequence[Effect]) -> None:
        nonlocal docks_that_raise
        for effect in effects:
            match effect:
                case TransitionRecord(goto=goto):
                    records.append(effect)
                    if goto == "OnDock":
                        queue.append(SignalItem("odom", to_plain(DOCKED)))
                case SendStart(call) if call.rpc == "MLSPlannerNative.go_to":
                    goal = call.args["pose"]
                    position = goal["pose"]["position"]
                    calls.append(
                        f"go_to({goal['header']['frame_id']}, {position['x']:.2f}, "
                        f"{position['y']:.2f}, {position['z']:.2f})"
                    )
                    # Each goal's stamp is its place in the calls made, and every goal is reached.
                    stamp = Time(len(calls), 0)
                    queue.append(ReturnItem(call, to_plain(stamp)))
                    status = GoalStatus(goal_id=GoalID(stamp=stamp), status=GoalStatus.SUCCEEDED)
                    queue.append(SignalItem("nav_status", to_plain(status)))
                case SendStart(call) if call.rpc == "M20Connection.undock":
                    calls.append("undock")
                    queue.append(SignalItem("odom", to_plain(UNDOCKED)))
                    queue.append(ReturnItem(call))
                case SendStart(call) if call.rpc == "M20Connection.dock":
                    calls.append("dock")
                    queue.append(ReturnItem(call, raised=docks_that_raise > 0))
                    docks_that_raise -= 1
                case SendStart(call) | SendCancel(call):
                    calls.append(call.rpc.split(".")[1])
                    queue.append(ReturnItem(call))
                case SetTimer(timer):
                    queue.append(TimerItem(timer))
                case FaultReport():
                    faults.append(effect)

    queue.append(SignalItem("odom", to_plain(DOCKED)))
    runtime, effects = engine.start(checked, 0.0)
    perform(effects)
    while queue:
        runtime, effects = engine.step(checked, runtime, queue.popleft(), 0.0)
        perform(effects)
    return calls, records, faults


# Both spots sit on the dock's axis, the predock spot level with the undock spot.
APPROACH = [
    "go_to(map, -5.97, -3.60, 0.00)",
    "stop_navigation",
    "go_to(map, 0.96, 0.40, 0.00)",
    "stop_navigation",
    "dock",
]


def test_the_robot_docks_five_times_and_the_mission_ends() -> None:
    calls, records, faults = _walk(docks_that_raise=0)

    assert faults == []
    assert calls == ["undock", *APPROACH] * 5
    assert records[-1] == TransitionRecord("NextCycle", None, "done")


def test_a_dock_that_keeps_failing_is_tried_three_times_then_faults() -> None:
    calls, records, faults = _walk(docks_that_raise=3)

    assert calls[: 1 + 3 * len(APPROACH)] == ["undock", *APPROACH * 3]
    assert [record.node for record in records].count("Docking") == 3
    assert len(faults) == 1
