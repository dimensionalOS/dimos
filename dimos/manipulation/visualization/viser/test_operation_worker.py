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
import threading

import pytest

pytest.importorskip("viser", reason="Viser optional dependency is not installed")

from dimos.manipulation.planning.groups.models import PlanningGroup
from dimos.manipulation.visualization.types import RobotInfo
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.gui import ViserPanelGui
from dimos.manipulation.visualization.viser.state import (
    ActionStatus,
    BackendConnectionStatus,
    OperationWorker,
    PanelRuntime,
    PlanStatus,
    TargetEvaluationWorker,
    TargetStatus,
)
from dimos.msgs.sensor_msgs.JointState import JointState


class EmptyServer:
    pass


class EmptyWorldMonitor:
    def get_current_joint_state(self, robot_id: str) -> None:
        return None

    def is_state_stale(self, robot_id: str, max_age: float = 1.0) -> bool:
        return False


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


class FakeRestartableOperationWorker(FakeOperationSubmitWorker):
    def __init__(
        self, submissions: list[Callable[[], None]], stop_calls: list[float | None]
    ) -> None:
        super().__init__(submissions)
        self.stop_calls = stop_calls

    def stop(self, timeout: float | None = 2.0) -> None:
        self.stop_calls.append(timeout)


class FakeOperationAdapter:
    def __init__(self) -> None:
        self.cancel_calls = 0

    def list_robots(self) -> list[str]:
        return []

    def robot_id_for_name(self, robot_name: str) -> str | None:
        return None

    def get_state(self) -> str:
        return "IDLE"

    def get_robot_info(self, robot_name: str) -> RobotInfo | None:
        return None

    def list_planning_groups(self) -> list[PlanningGroup]:
        return []

    def get_current_joint_state(self, robot_name: str) -> None:
        return None

    def get_ee_pose(self, robot_name: str, joint_state: JointState | None = None) -> None:
        return None

    def get_error(self) -> str:
        return ""

    def get_robot_config(self, robot_name: str) -> None:
        return None

    def is_state_stale(self, robot_name: str, max_age: float = 1.0) -> bool:
        return False

    def cancel(self) -> bool:
        self.cancel_calls += 1
        return True

    def plan_to_joints(self, joints: JointState, robot_name: str | None = None) -> bool:
        return True

    def robot_items(self) -> list[tuple[str, str, object]]:
        return []

    def plan_to_joint_targets(self, joint_targets: dict[str, JointState]) -> bool:
        return True


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


def test_gui_close_uses_bounded_operation_worker_stop(monkeypatch: pytest.MonkeyPatch) -> None:
    stop_timeouts: list[float | None] = []
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        FakeOperationAdapter(),
        ViserVisualizationConfig(),
    )
    gui._operation_worker.stop()
    gui._worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeStopOperationWorker(stop_timeouts))
    monkeypatch.setattr(gui, "_worker", FakeStopEvaluationWorker([]))

    gui.close()

    assert stop_timeouts == [2.0]


def test_gui_only_preview_submits_timeout_override(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[dict[str, float]] = []
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        FakeOperationAdapter(),
        ViserVisualizationConfig(preview_request_timeout=0.25),
    )
    gui._operation_worker.stop()
    monkeypatch.setattr(gui, "_operation_worker", FakeTimeoutSubmitWorker(submissions))
    gui.state.runtime = PanelRuntime.RUNNING
    gui.state.backend_status = BackendConnectionStatus.READY
    gui.state.selected_group_ids = ("arm:manipulator",)
    gui.state.target_joints = JointState({"name": ["arm/j1"], "position": [1.0]})
    gui.state.target_status = TargetStatus.FEASIBLE
    gui.state.manipulation_state = "IDLE"

    gui._submit_plan()
    gui.state.plan_state.status = PlanStatus.FRESH
    gui._submit_preview()

    assert "timeout_seconds" not in submissions[0]
    assert submissions[1]["timeout_seconds"] == 0.25


def test_gui_cancel_bypasses_operation_worker(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[Callable[[], None]] = []
    stop_calls: list[float | None] = []
    adapter = FakeOperationAdapter()
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        adapter,
        ViserVisualizationConfig(),
    )
    gui._operation_worker.stop()
    monkeypatch.setattr(
        gui, "_operation_worker", FakeRestartableOperationWorker(submissions, stop_calls)
    )
    gui.state.action_status = ActionStatus.PREVIEWING

    gui._submit_cancel()
    gui.close()

    assert submissions == []
    assert stop_calls == [0.0]
    assert adapter.cancel_calls == 1
    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.last_result == "cancel=True"


def test_gui_cancelled_planning_clears_active_plan_state(monkeypatch: pytest.MonkeyPatch) -> None:
    submissions: list[Callable[[], None]] = []
    stop_calls: list[float | None] = []
    adapter = FakeOperationAdapter()
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        adapter,
        ViserVisualizationConfig(),
    )
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
    assert adapter.cancel_calls == 1
    assert stop_calls == [0.0]
    assert gui.state.action_status == ActionStatus.IDLE
    assert gui.state.plan_state.status == PlanStatus.FAILED
    assert gui.state.last_result == "cancel=True"


@pytest.mark.parametrize(
    ("submit", "expected_error"),
    [
        ("_submit_plan", "Cannot plan until target is feasible and manipulation is idle"),
        ("_submit_preview", "No fresh plan to preview"),
        ("_submit_execute", "Panel execution disabled; set allow_plan_execute=True to enable"),
    ],
)
def test_gui_guard_errors_keep_action_idle(
    submit: str, expected_error: str, monkeypatch: pytest.MonkeyPatch
) -> None:
    submissions: list[Callable[[], None]] = []
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        FakeOperationAdapter(),
        ViserVisualizationConfig(),
    )
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
    gui = ViserPanelGui(
        EmptyServer(),
        EmptyWorldMonitor(),
        FakeOperationAdapter(),
        ViserVisualizationConfig(),
    )
    old_operation_id = gui._next_operation_id()
    gui._set_operation_error("Operation timed out after 5.0s", old_operation_id)
    gui.state.action_status = ActionStatus.FAILED

    gui._finish_operation("preview=True", operation_id=old_operation_id)

    assert gui.state.action_status == ActionStatus.FAILED
    assert gui.state.error == "Operation timed out after 5.0s"
