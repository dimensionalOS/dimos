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

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    PanelRuntime,
    PanelState,
    PlanStatus,
    TargetStatus,
)


def test_panel_can_plan_from_fault_after_planning_failure() -> None:
    state = PanelState(
        selected_robot="arm",
        selected_group_ids=(PlanningGroupID("arm/manipulator"),),
        runtime=PanelRuntime.RUNNING,
        backend_status=BackendConnectionStatus.READY,
        target_status=TargetStatus.FEASIBLE,
        manipulation_state="FAULT",
    )

    assert state.can_plan() is True


def test_panel_cannot_plan_without_a_selected_group() -> None:
    state = PanelState(
        runtime=PanelRuntime.RUNNING,
        backend_status=BackendConnectionStatus.READY,
        target_status=TargetStatus.FEASIBLE,
        manipulation_state="IDLE",
    )

    assert state.can_plan() is False


def test_sequence_change_marks_a_fresh_plan_stale() -> None:
    state = PanelState(plan_state=PanelState().plan_state)
    state.plan_state.status = PlanStatus.FRESH

    state.next_sequence_id()

    assert state.plan_state.status == PlanStatus.STALE
    assert state.target_status == TargetStatus.CHECKING


def test_selection_epoch_change_resets_plan_and_invalidates_sequence() -> None:
    state = PanelState(selected_group_ids=(PlanningGroupID("arm/manipulator"),))
    state.plan_state.status = PlanStatus.FRESH

    assert state.advance_selection_epoch() == 1

    assert state.latest_sequence_id == 1
    assert state.plan_state.status == PlanStatus.NONE


def test_cancel_is_available_for_running_preview_and_execute_actions() -> None:
    state = PanelState()

    for action in (ActionStatus.RUNNING, ActionStatus.PREVIEWING, ActionStatus.EXECUTING):
        state.action_status = action
        assert state.can_cancel() is True
