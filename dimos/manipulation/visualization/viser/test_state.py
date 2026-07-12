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

from collections.abc import Callable
import threading

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    OperationWorker,
    PanelRuntime,
    PanelState,
    PlanStatus,
    TargetEvaluationWorker,
    TargetStatus,
)


def test_panel_cannot_plan_from_fault_without_explicit_reset() -> None:
    state = PanelState(
        selected_robot="arm",
        selected_group_ids=(PlanningGroupID("arm/manipulator"),),
        runtime=PanelRuntime.RUNNING,
        backend_status=BackendConnectionStatus.READY,
        target_status=TargetStatus.FEASIBLE,
        manipulation_state="FAULT",
    )

    assert state.can_plan() is False


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


def test_operation_worker_uses_per_operation_timeout() -> None:
    errors: list[str] = []
    worker = OperationWorker(errors.append, timeout_seconds=1.0)
    worker.submit(lambda: None, timeout_seconds=0.25)

    request = worker._requests.get_nowait()

    assert worker._operation_timeout(request) == 0.25


def test_operation_worker_uses_operation_error_callback_on_timeout() -> None:
    default_errors: list[str] = []
    operation_errors: list[str] = []
    release = threading.Event()
    finished = threading.Event()
    worker = OperationWorker(default_errors.append)

    def operation() -> None:
        release.wait(timeout=1.0)
        finished.set()

    worker.submit(
        operation,
        timeout_seconds=0.001,
        on_error=operation_errors.append,
    )

    worker._run_operation(worker._requests.get_nowait())
    release.set()
    assert finished.wait(timeout=1.0)

    assert default_errors == []
    assert operation_errors == ["Operation timed out after 0.0s"]


class FakeTargetEvaluationWorker(TargetEvaluationWorker):
    def __init__(self, calls: list[Callable[[], None]]) -> None:
        self.calls = calls
