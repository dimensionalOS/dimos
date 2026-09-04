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
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.planning.spec.config import RobotModelConfig
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
from dimos.robot.assets.model import RobotModel


class EmptyServer:
    pass


class FakeOperatorBackend:
    def __init__(self) -> None:
        self.cancel_calls = 0
        self.scan_calls: list[tuple[list[str], float]] = []

    def cancel(self) -> bool:
        self.cancel_calls += 1
        return True

    def scan_from_here(self, prompts: list[str], timeout: float) -> dict[str, int]:
        self.scan_calls.append((prompts, timeout))
        return {"detected": 2, "refreshed": 4, "total": 4}


class FakeOperator:
    def __init__(self, module: FakeOperatorBackend | None = None) -> None:
        self.module = module or FakeOperatorBackend()

    def status(self) -> OperatorStatus:
        return OperatorStatus(state="IDLE", error="", has_plan=False)

    def get_init_joints(self) -> None:
        return None

    def cancel(self) -> bool:
        return self.module.cancel()

    def preview(self, *_args: object, **_kwargs: object) -> bool:
        return True

    def scan_from_here(self, prompts: list[str], timeout: float) -> dict[str, int]:
        return self.module.scan_from_here(prompts, timeout)


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


def planning_group(name: str, joints: tuple[str, ...]) -> PlanningGroup:
    return PlanningGroup(
        name,
        joints,
        "base",
        None,
    )


def make_gui(module: FakeOperatorBackend | None = None) -> ViserPanelGui:
    module = module or FakeOperatorBackend()
    return ViserPanelGui(
        EmptyServer(),
        PlanningSceneInfo(
            model=RobotModelConfig(
                model=RobotModel.from_file(Path("/tmp/model.urdf")), joint_names=[]
            )
        ),
        FakeOperator(module),
        lambda: None,
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


def test_group_status_composes_shared_panel_state(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gui = make_gui()
    values: dict[str, str] = {}
    gui.state.selected_group_ids = ("manipulator", "gripper")
    gui.state.error = "planner unavailable"
    gui.state.target_status = gui.state.target_status.FEASIBLE
    gui.state.plan_state.status = gui.state.plan_state.status.FRESH
    monkeypatch.setattr(gui, "_stale_models", lambda _group_ids: ("model",))
    monkeypatch.setattr(gui, "_set_handle_value", values.__setitem__)

    gui._update_status_text()

    assert values == {
        "status": "### Status\n\n**State:** planner unavailable\n\n"
        "Target: `feasible` · Plan: `fresh`\n\nState stale: `True`",
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


def test_scan_is_debounced_and_reports_accumulated_count(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    submissions: list[Callable[[], None]] = []
    backend = FakeOperatorBackend()
    gui = make_gui(backend)
    gui.config = ViserVisualizationConfig(
        scan_from_here_enabled=True,
        scan_prompts=("cup", "bottle"),
        scan_timeout=12.0,
    )
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeOperationSubmitWorker(submissions))
    monkeypatch.setattr(gui, "refresh", lambda: None)
    scan_status = SimpleNamespace(content="")
    gui._handles["scan_status"] = scan_status
    gui.state.action_status = ActionStatus.IDLE

    gui._submit_scan()
    gui._submit_scan()
    submissions[0]()

    assert backend.scan_calls == [(["cup", "bottle"], 12.0)]
    assert scan_status.content == "Scan complete: 2 detected, 4 planner objects total."
    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.last_result == "scan=2, total=4"


def test_named_camera_viewpoint_round_trip_lives_in_panel_session() -> None:
    gui = make_gui()
    camera = SimpleNamespace(
        position=(1.0, 2.0, 3.0),
        look_at=(0.1, 0.2, 0.3),
        up_direction=(0.0, 0.0, 1.0),
        fov=0.8,
    )
    event = SimpleNamespace(client=SimpleNamespace(camera=camera))
    name = SimpleNamespace(value="table overview")
    choices = SimpleNamespace(options=["(none)"], value="(none)")
    status = SimpleNamespace(content="")
    gui._handles.update(
        {
            "viewpoint_name": name,
            "viewpoint_choices": choices,
            "viewpoint_status": status,
        }
    )

    gui._save_viewpoint(event)
    camera.position = (9.0, 9.0, 9.0)
    camera.look_at = (8.0, 8.0, 8.0)
    camera.up_direction = (0.0, 1.0, 0.0)
    camera.fov = 1.2
    gui._restore_viewpoint(event)

    assert choices.options == ["table overview"]
    assert camera.position == (1.0, 2.0, 3.0)
    assert camera.look_at == (0.1, 0.2, 0.3)
    assert camera.up_direction == (0.0, 0.0, 1.0)
    assert camera.fov == 0.8
    assert status.content == "Restored `table overview`."


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
    gui.state.selected_group_ids = ("manipulator",)
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
        planning_group("manipulator", ("j1",)),
        planning_group("gripper", ("j2",)),
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
        planning_group("manipulator", ("j1",)),
        planning_group("gripper", ("j2",)),
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
