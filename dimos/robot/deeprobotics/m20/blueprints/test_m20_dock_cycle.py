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
import pytest

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


def test_the_robot_docks_five_times_and_the_mission_ends() -> None:
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
                case SendStart(call) | SendCancel(call):
                    calls.append(call.rpc.split(".")[1])
                    queue.append(ReturnItem(call))
                case SetTimer(timer):
                    queue.append(TimerItem(timer))
                case FaultReport():
                    pytest.fail(f"the dock cycle faulted: {effect}")

    # The robot stands at (2, 1) on its way off the dock, facing 30 degrees.
    half_yaw = math.radians(30.0) / 2
    odom = PoseStamped(
        frame_id="map",
        position=[2.0, 1.0, 0.48],
        orientation=[0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)],
    )
    queue.append(SignalItem("odom", to_plain(odom)))

    runtime, effects = engine.start(checked, 0.0)
    perform(effects)
    while queue:
        runtime, effects = engine.step(checked, runtime, queue.popleft(), 0.0)
        perform(effects)

    assert (
        calls
        == [
            "undock",
            "go_to(map, -0.60, -0.50, 0.00)",
            "stop_navigation",
            "go_to(map, 2.00, 1.00, 0.00)",
            "stop_navigation",
            "dock",
        ]
        * 5
    )
    assert records[-1] == ("NextCycle", None, "done")
    assert runtime.active == frozenset({"Mission"})
