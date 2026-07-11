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

from __future__ import annotations

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.manipulation_operator import ActionResult, OperatorStatus
from dimos.manipulation.planning.spec.models import PlanningSceneInfo
from dimos.manipulation.visualization.types import TargetEvaluation
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.state import FeasibilityStatus


class StatusOnlyServer:
    pass


class StatusOnlyTelemetry:
    pass


class StatusOnlyBackend:
    pass


class StatusOnlyOperator:
    def status(self) -> OperatorStatus:
        return OperatorStatus(state="IDLE", error="", has_plan=False)

    def get_init_joints(self, robot_name: str) -> None:
        _ = robot_name
        return None

    def cancel(self) -> ActionResult:
        return ActionResult(True, "cancel=True")


def make_gui() -> ViserPanelGui:
    return ViserPanelGui(
        StatusOnlyServer(),
        PlanningSceneInfo(robots={}),
        StatusOnlyOperator(),
        {},
        ViserVisualizationConfig(),
    )


@pytest.mark.parametrize(
    ("result", "success", "collision_free", "expected"),
    [
        ({"status": "FEASIBLE"}, True, True, FeasibilityStatus.FEASIBLE),
        ({"status": "COLLISION"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_START"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "COLLISION_AT_GOAL"}, False, False, FeasibilityStatus.COLLISION),
        ({"status": "NO_SOLUTION"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "SINGULARITY"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "JOINT_LIMITS"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "TIMEOUT"}, False, False, FeasibilityStatus.IK_FAILED),
        ({"status": "IK_SUCCEEDED"}, False, False, FeasibilityStatus.INVALID),
    ],
)
def test_gui_feasibility_status_uses_exact_status_mapping(
    result: TargetEvaluation,
    success: bool,
    collision_free: bool,
    expected: FeasibilityStatus,
) -> None:
    gui = make_gui()

    assert gui._feasibility_status(result, success, collision_free) == expected


def test_group_status_composes_shared_panel_state_without_robot_dropdown() -> None:
    gui = make_gui()
    values: dict[str, str] = {}
    gui.state.selected_group_ids = ("left/manipulator", "right/gripper")
    gui.state.error = "planner unavailable"
    gui.state.target_status = gui.state.target_status.FEASIBLE
    gui.state.plan_state.status = gui.state.plan_state.status.FRESH
    gui._stale_robot_names = lambda _group_ids: ("right",)  # type: ignore[method-assign]
    gui._set_handle_value = values.__setitem__  # type: ignore[method-assign]

    gui._update_status_text()

    assert "robot" not in gui._handles
    assert values == {
        "status": "### Status\n\n**State:** planner unavailable\n\n"
        "Target: `feasible` · Plan: `fresh`\n\nState stale: `True (right)`",
        "target_summary": "Feasibility: `unknown`",
    }
