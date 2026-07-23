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
from dataclasses import dataclass

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningSceneInfo
from dimos.manipulation.visualization.operator import OperatorStatus, TargetEvaluationResult
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    FeasibilityStatus,
    OperationWorker,
    PanelRuntime,
    PlanStatus,
    TargetEvaluationWorker,
    TargetStatus,
)
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory


class EmptyServer:
    pass


class FakeOperatorBackend:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def cancel(self) -> bool:
        self.cancel_calls += 1
        return True


class FakeOperator:
    def __init__(self, module: FakeOperatorBackend | None = None) -> None:
        self.module = module or FakeOperatorBackend()

    def status(self) -> OperatorStatus:
        return OperatorStatus(state="IDLE", error="", has_plan=False)

    def get_init_joints(self, robot_name: str) -> None:
        return None

    def cancel(self) -> bool:
        return self.module.cancel()

    def preview(self, *_args: object, **_kwargs: object) -> bool:
        return True


@dataclass
class FakeStopOperationWorker(OperationWorker):
    stop_calls: list[float | None]

    def __init__(self, stop_calls: list[float | None]) -> None:
        self.stop_calls = stop_calls

    def stop(self, timeout: float | None = 2.0) -> None:
        self.stop_calls.append(timeout)


@dataclass
class FakeStopEvaluationWorker(TargetEvaluationWorker):
    stop_calls: list[float | None]

    def __init__(self, stop_calls: list[float | None]) -> None:
        self.stop_calls = stop_calls

    def stop(self, timeout: float | None = 2.0) -> None:
        self.stop_calls.append(timeout)


class FakeTimeoutSubmitWorker(OperationWorker):
    def __init__(self, submissions: list[dict[str, float]]) -> None:
        self.submissions = submissions

    def submit(
        self,
        operation: Callable[[], None],
        *,
        timeout_seconds: float | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        kwargs = {}
        if timeout_seconds is not None:
            kwargs["timeout_seconds"] = timeout_seconds
        self.submissions.append(kwargs)


class FakeOperationSubmitWorker(OperationWorker):
    def __init__(self, submissions: list[Callable[[], None]]) -> None:
        self.submissions = submissions

    def submit(
        self,
        operation: Callable[[], None],
        *,
        timeout_seconds: float | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        self.submissions.append(operation)


class FakeOperationErrorWorker(OperationWorker):
    def __init__(self, errors: list[Callable[[str], None]]) -> None:
        self.errors = errors

    def submit(
        self,
        operation: Callable[[], None],
        *,
        timeout_seconds: float | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        if on_error is not None:
            self.errors.append(on_error)


class FakeRestartableOperationWorker(FakeOperationSubmitWorker):
    def __init__(
        self, submissions: list[Callable[[], None]], stop_calls: list[float | None]
    ) -> None:
        super().__init__(submissions)
        self.stop_calls = stop_calls

    def stop(self, timeout: float | None = 2.0) -> None:
        self.stop_calls.append(timeout)


def planning_group(robot: str, name: str, joints: tuple[str, ...]) -> PlanningGroup:
    return PlanningGroup(
        f"{robot}/{name}",
        robot,
        name,
        tuple(f"{robot}/{joint}" for joint in joints),
        joints,
        "base",
        None,
    )


def make_gui(module: FakeOperatorBackend | None = None) -> ViserPanelGui:
    module = module or FakeOperatorBackend()
    return ViserPanelGui(
        EmptyServer(),
        PlanningSceneInfo(robots={}),
        FakeOperator(module),
        {},
        ViserVisualizationConfig(),
    )


@pytest.mark.parametrize(
    ("result", "success", "collision_free", "expected"),
    [
        (
            TargetEvaluationResult(True, "FEASIBLE", "", True),
            True,
            True,
            FeasibilityStatus.FEASIBLE,
        ),
        (TargetEvaluationResult(False, "COLLISION", ""), False, False, FeasibilityStatus.COLLISION),
        (
            TargetEvaluationResult(False, "COLLISION_AT_START", ""),
            False,
            False,
            FeasibilityStatus.COLLISION,
        ),
        (
            TargetEvaluationResult(False, "COLLISION_AT_GOAL", ""),
            False,
            False,
            FeasibilityStatus.COLLISION,
        ),
        (
            TargetEvaluationResult(False, "NO_SOLUTION", ""),
            False,
            False,
            FeasibilityStatus.IK_FAILED,
        ),
        (
            TargetEvaluationResult(False, "SINGULARITY", ""),
            False,
            False,
            FeasibilityStatus.IK_FAILED,
        ),
        (
            TargetEvaluationResult(False, "JOINT_LIMITS", ""),
            False,
            False,
            FeasibilityStatus.IK_FAILED,
        ),
        (TargetEvaluationResult(False, "TIMEOUT", ""), False, False, FeasibilityStatus.IK_FAILED),
        (
            TargetEvaluationResult(False, "IK_SUCCEEDED", ""),
            False,
            False,
            FeasibilityStatus.INVALID,
        ),
    ],
)
def test_gui_feasibility_status_uses_exact_status_mapping(
    result: TargetEvaluationResult,
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


def test_gui_close_uses_bounded_operation_worker_stop(monkeypatch: pytest.MonkeyPatch) -> None:
    stop_timeouts: list[float | None] = []
    gui = make_gui()
    gui._operation_worker.stop()
    gui._worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeStopOperationWorker(stop_timeouts))
    monkeypatch.setattr(gui, "_worker", FakeStopEvaluationWorker([]))

    gui.close()

    assert stop_timeouts == [2.0]


def test_gui_only_preview_submits_timeout_override(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[dict[str, float]] = []
    gui = make_gui()
    gui.config = ViserVisualizationConfig(preview_request_timeout=0.25)
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeTimeoutSubmitWorker(submissions))
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.plan_state.status = PlanStatus.FRESH
    gui._submit_preview()

    assert submissions == [{"timeout_seconds": 0.25}]


def test_gui_preview_enters_previewing_before_worker_runs(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    submissions: list[Callable[[], None]] = []
    gui = make_gui()
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeOperationSubmitWorker(submissions))
    monkeypatch.setattr(gui, "refresh", lambda: None)
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "COMPLETED"
    gui.state.selected_group_ids = ("arm/manipulator",)
    gui.state.plan_state.status = PlanStatus.FRESH
    gui.state.plan_state.group_ids = gui.state.selected_group_ids
    gui.state.plan_state.target_sequence_id = gui.state.latest_sequence_id
    gui.state.plan_state.plan = GeneratedPlan(
        group_ids=gui.state.selected_group_ids,
        trajectory=JointTrajectory(),
        path=[JointState({"name": [], "position": []})],
    )

    assert gui.state.can_execute() is True

    gui._submit_preview()

    assert gui.state.action_status == ActionStatus.PREVIEWING
    assert gui.state.can_execute() is False
    assert len(submissions) == 1


def test_gui_selection_change_clears_invalidated_preview(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    submissions: list[Callable[[], None]] = []
    gui = make_gui()
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeOperationSubmitWorker(submissions))
    monkeypatch.setattr(gui, "refresh", lambda: None)
    groups = [
        planning_group("arm", "manipulator", ("j1",)),
        planning_group("arm", "gripper", ("j2",)),
    ]
    monkeypatch.setattr(gui, "list_planning_groups", lambda: groups)
    monkeypatch.setattr(gui, "_build_joint_sliders", lambda: None)
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "COMPLETED"
    gui.state.selected_group_ids = (groups[0].id,)
    gui.state.plan_state.status = PlanStatus.FRESH
    gui.state.plan_state.group_ids = gui.state.selected_group_ids
    gui.state.plan_state.target_sequence_id = gui.state.latest_sequence_id
    gui.state.plan_state.plan = GeneratedPlan(
        group_ids=gui.state.selected_group_ids,
        trajectory=JointTrajectory(),
        path=[JointState({"name": [], "position": []})],
    )

    gui._submit_preview()
    gui._toggle_group_selected(groups[1].id)
    submissions[0]()

    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.last_result == "preview=False"


def test_gui_selection_change_ignores_invalidated_preview_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    errors: list[Callable[[str], None]] = []
    gui = make_gui()
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeOperationErrorWorker(errors))
    monkeypatch.setattr(gui, "refresh", lambda: None)
    groups = [
        planning_group("arm", "manipulator", ("j1",)),
        planning_group("arm", "gripper", ("j2",)),
    ]
    monkeypatch.setattr(gui, "list_planning_groups", lambda: groups)
    monkeypatch.setattr(gui, "_build_joint_sliders", lambda: None)
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "COMPLETED"
    gui.state.selected_group_ids = (groups[0].id,)
    gui.state.plan_state.status = PlanStatus.FRESH
    gui.state.plan_state.group_ids = gui.state.selected_group_ids
    gui.state.plan_state.target_sequence_id = gui.state.latest_sequence_id
    gui.state.plan_state.plan = GeneratedPlan(
        group_ids=gui.state.selected_group_ids,
        trajectory=JointTrajectory(),
        path=[JointState({"name": [], "position": []})],
    )

    gui._submit_preview()
    gui._toggle_group_selected(groups[1].id)
    errors[0]("preview timed out")

    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.error == ""
    assert gui.state.last_result == "preview=False"


def test_gui_cancel_bypasses_operation_worker(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[Callable[[], None]] = []
    stop_calls: list[float | None] = []
    module = FakeOperatorBackend()
    gui = make_gui(module)
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui, "_operation_worker", FakeRestartableOperationWorker(submissions, stop_calls)
    )
    gui.state.action_status = ActionStatus.PREVIEWING

    gui._submit_cancel()
    gui.close()

    assert submissions == []
    assert stop_calls == [0.0]
    assert module.cancel_calls == 1
    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.last_result == "cancel=True"


def test_gui_cancelled_planning_clears_active_plan_state(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[Callable[[], None]] = []
    stop_calls: list[float | None] = []
    module = FakeOperatorBackend()
    gui = make_gui(module)
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui, "_operation_worker", FakeRestartableOperationWorker(submissions, stop_calls)
    )
    stale_operation_id = gui._next_operation_id()
    gui.state.action_status = ActionStatus.RUNNING
    gui.state.plan_state.status = PlanStatus.PLANNING
    assert gui.state.plan_state.status == PlanStatus.PLANNING

    gui._submit_cancel()
    gui._finish_operation("plan_to_joints=True", operation_id=stale_operation_id)
    gui.close()

    assert submissions == []
    assert module.cancel_calls == 1
    assert stop_calls == [0.0]
    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.plan_state.status == PlanStatus.FAILED
    assert gui.state.last_result == "cancel=True"


@pytest.mark.parametrize(
    ("submit", "expected_error"),
    [
        ("_submit_plan", "Cannot plan until target is feasible and manipulation is idle"),
        ("_submit_preview", "No fresh plan to preview"),
        (
            "_submit_execute",
            "Cannot execute: require feasible fresh plan",
        ),
    ],
)
def test_gui_guard_errors_keep_action_idle(
    submit: str, expected_error: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    submissions: list[Callable[[], None]] = []
    gui = make_gui()
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeOperationSubmitWorker(submissions))
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.selected_robot = "arm"
    gui.state.action_status = ActionStatus.IDLE

    getattr(gui, submit)()

    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.error == expected_error
    assert submissions == []


def test_gui_ignores_stale_timed_out_operation_finish() -> None:
    gui = make_gui()
    old_operation_id = gui._next_operation_id()
    gui._set_operation_error("Operation timed out after 5.0s", old_operation_id)
    gui.state.action_status = ActionStatus.FAILED

    gui._finish_operation("preview=True", operation_id=old_operation_id)

    assert gui.state.action_status == ActionStatus.FAILED
    assert gui.state.error == "Operation timed out after 5.0s"
