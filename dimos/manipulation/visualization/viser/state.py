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
from dataclasses import dataclass, field
from enum import Enum
import queue
import threading
from typing import Literal

from dimos.manipulation.planning.spec.models import PlanningGroupID
from dimos.manipulation.visualization.types import RobotInfo, TargetEvaluation, TargetSetEvaluation
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FeasibilityStatus(str, Enum):
    UNKNOWN = "unknown"
    FEASIBLE = "feasible"
    IK_FAILED = "ik_failed"
    COLLISION = "collision"
    INVALID = "invalid"


class PanelRuntime(str, Enum):
    STOPPED = "stopped"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    FAILED = "failed"


class BackendConnectionStatus(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    WAITING_FOR_ROBOT = "waiting_for_robot"
    READY = "ready"


class TargetStatus(str, Enum):
    EMPTY = "empty"
    DIRTY = "dirty"
    CHECKING = "checking"
    FEASIBLE = "feasible"
    INFEASIBLE = "infeasible"


class PlanStatus(str, Enum):
    NONE = "none"
    PLANNING = "planning"
    FRESH = "fresh"
    STALE = "stale"
    EXECUTING = "executing"
    FAILED = "failed"


class ActionStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"
    PREVIEWING = "previewing"
    EXECUTING = "executing"
    CANCELLING = "cancelling"
    CLEARING_PLAN = "clearing_plan"
    FAILED = "failed"


PreviewSource = Literal["cartesian", "joints"]


@dataclass
class FeasibilityState:
    status: FeasibilityStatus = FeasibilityStatus.UNKNOWN
    message: str = ""
    sequence_id: int = 0


@dataclass
class PanelPlanState:
    status: PlanStatus = PlanStatus.NONE
    group_ids: tuple[PlanningGroupID, ...] = ()
    robot: str | None = None
    target_pose: Pose | None = None
    target_joints: list[float] | None = None
    start_joints_snapshot: dict[PlanningGroupID, list[float]] = field(default_factory=dict)
    planned_path: list[JointState] | None = None


@dataclass
class PanelState:
    selected_robot: str | None = None
    selected_group_ids: tuple[PlanningGroupID, ...] = ()
    auxiliary_group_ids: tuple[PlanningGroupID, ...] = ()
    pose_targets: dict[PlanningGroupID, Pose] = field(default_factory=dict)
    group_joint_targets: dict[PlanningGroupID, JointState] = field(default_factory=dict)
    target_joints: JointState | None = None
    last_valid_target_joints: JointState | None = None
    group_poses: dict[PlanningGroupID, Pose] = field(default_factory=dict)
    group_diagnostics: dict[PlanningGroupID, str] = field(default_factory=dict)
    runtime: PanelRuntime = PanelRuntime.STOPPED
    backend_status: BackendConnectionStatus = BackendConnectionStatus.DISCONNECTED
    target_status: TargetStatus = TargetStatus.EMPTY
    action_status: ActionStatus = ActionStatus.IDLE
    manipulation_state: str = "DISCONNECTED"
    robot_info: RobotInfo | None = None
    current_joints: list[float] | None = None
    current_ee_pose: Pose | None = None
    cartesian_target: Pose | None = None
    joint_target: list[float] | None = None
    feasibility: FeasibilityState = field(default_factory=FeasibilityState)
    latest_sequence_id: int = 0
    plan_state: PanelPlanState = field(default_factory=PanelPlanState)
    error: str = ""
    last_result: str = ""

    def next_sequence_id(self) -> int:
        self.latest_sequence_id += 1
        self.feasibility = FeasibilityState(sequence_id=self.latest_sequence_id)
        self.target_status = TargetStatus.CHECKING
        self.mark_plan_stale()
        return self.latest_sequence_id

    def mark_plan_stale(self) -> None:
        if self.plan_state.status == PlanStatus.FRESH:
            self.plan_state.status = PlanStatus.STALE

    def can_plan(self) -> bool:
        return (
            self.runtime == PanelRuntime.RUNNING
            and self.backend_status == BackendConnectionStatus.READY
            and bool(self.selected_group_ids)
            and self.action_status == ActionStatus.IDLE
            and self.target_status == TargetStatus.FEASIBLE
            and self.target_joints is not None
            and self.manipulation_state in {"IDLE", "COMPLETED", "FAULT"}
            and self.plan_state.status != PlanStatus.PLANNING
        )

    def can_preview(self) -> bool:
        return (
            self.runtime == PanelRuntime.RUNNING
            and self.backend_status == BackendConnectionStatus.READY
            and self.action_status == ActionStatus.IDLE
            and self.plan_state.status == PlanStatus.FRESH
        )

    def can_cancel(self) -> bool:
        return self.action_status in {
            ActionStatus.RUNNING,
            ActionStatus.PREVIEWING,
            ActionStatus.EXECUTING,
        } or (self.manipulation_state == "EXECUTING")

    def can_execute(
        self,
        current_tolerance: float,
        action_status: ActionStatus | None = None,
    ) -> bool:
        plan = self.plan_state
        effective_action_status = action_status or self.action_status
        if not (
            self.runtime == PanelRuntime.RUNNING
            and self.backend_status == BackendConnectionStatus.READY
            and effective_action_status == ActionStatus.IDLE
            and self.target_status == TargetStatus.FEASIBLE
            and self.manipulation_state in {"IDLE", "COMPLETED"}
            and plan.status == PlanStatus.FRESH
            and plan.group_ids == self.selected_group_ids
            and bool(plan.start_joints_snapshot)
            and self.target_joints is not None
        ):
            return False
        if self.current_joints is None:
            return False
        # Multi-group freshness is checked by group snapshot when available. The
        # robot-level current-joint fallback preserves one-group legacy tests.
        if len(plan.start_joints_snapshot) == 1:
            snapshot = next(iter(plan.start_joints_snapshot.values()))
            if len(snapshot) != len(self.current_joints):
                return False
            return all(
                abs(expected - current) <= current_tolerance
                for expected, current in zip(snapshot, self.current_joints, strict=False)
            )
        return True

    @property
    def connected(self) -> bool:
        return self.backend_status in {
            BackendConnectionStatus.WAITING_FOR_ROBOT,
            BackendConnectionStatus.READY,
        }

    @property
    def module_state(self) -> str:
        if self.backend_status == BackendConnectionStatus.DISCONNECTED:
            return "DISCONNECTED"
        if self.backend_status == BackendConnectionStatus.WAITING_FOR_ROBOT:
            return "WAITING_FOR_ROBOT"
        return self.manipulation_state


@dataclass
class TargetEvaluationRequest:
    sequence_id: int
    source: PreviewSource
    group_ids: tuple[PlanningGroupID, ...]
    robot_name: str | None = None
    auxiliary_group_ids: tuple[PlanningGroupID, ...] = ()
    pose: Pose | None = None
    pose_targets: dict[PlanningGroupID, Pose] = field(default_factory=dict)
    joints: JointState | None = None
    joint_targets: dict[PlanningGroupID, JointState] = field(default_factory=dict)
    check_collision: bool = True


class TargetEvaluationWorker:
    """Latest-target-wins background evaluator.

    User callbacks own target visuals immediately. This worker only computes
    feasibility/joint solutions; stale sequence IDs are ignored by the GUI
    apply step.
    """

    def __init__(
        self,
        handler: Callable[[TargetEvaluationRequest], TargetEvaluation | TargetSetEvaluation],
        apply_result: Callable[
            [TargetEvaluationRequest, TargetEvaluation | TargetSetEvaluation], None
        ],
    ) -> None:
        self._handler = handler
        self._apply_result = apply_result
        self._requests: queue.Queue[TargetEvaluationRequest] = queue.Queue(maxsize=1)
        self._submit_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="ViserTargetEvaluationWorker", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float | None = 2.0) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        if self._thread is not None and not self._thread.is_alive():
            self._thread = None

    def submit(self, request: TargetEvaluationRequest) -> None:
        with self._submit_lock:
            while True:
                try:
                    self._requests.get_nowait()
                except queue.Empty:
                    break
            self._requests.put_nowait(request)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            while True:
                try:
                    request = self._requests.get_nowait()
                except queue.Empty:
                    break
            try:
                result = self._handler(request)
                self._apply_result(request, result)
            except Exception:
                logger.warning("Target evaluation worker caught unhandled exception", exc_info=True)


class OperationWorker:
    """Single-worker operation queue for Viser panel actions."""

    def __init__(
        self,
        on_error: Callable[[str], None],
        timeout_seconds: float | None = None,
    ) -> None:
        self._on_error = on_error
        self._timeout_seconds = timeout_seconds
        self._requests: queue.Queue[OperationRequest] = queue.Queue(maxsize=1)
        self._submit_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="ViserOperationWorker", daemon=True)
        self._thread.start()

    def stop(self, timeout: float | None = 2.0) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=timeout)
        if self._thread is not None and not self._thread.is_alive():
            self._thread = None

    def submit(
        self,
        operation: Callable[[], None],
        *,
        timeout_seconds: float | None = None,
        on_error: Callable[[str], None] | None = None,
    ) -> None:
        request = OperationRequest(
            operation=operation,
            timeout_seconds=timeout_seconds,
            on_error=on_error or self._on_error,
        )
        with self._submit_lock:
            while True:
                try:
                    self._requests.get_nowait()
                except queue.Empty:
                    break
            self._requests.put_nowait(request)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self._run_operation(request)
            except Exception as e:
                request.on_error(str(e))

    def _run_operation(self, request: OperationRequest) -> None:
        timeout = self._operation_timeout(request)
        if timeout is None:
            request.operation()
            return

        error: Exception | None = None

        def run() -> None:
            nonlocal error
            try:
                request.operation()
            except Exception as e:
                error = e

        thread = threading.Thread(target=run, name="ViserOperation", daemon=True)
        thread.start()
        thread.join(timeout=max(timeout, 0.0))
        if thread.is_alive():
            request.on_error(f"Operation timed out after {timeout:.1f}s")
            return
        if error is not None:
            raise error

    def _operation_timeout(self, request: OperationRequest) -> float | None:
        timeout = request.timeout_seconds
        if timeout is None:
            timeout = self._timeout_seconds
        return None if timeout is None else float(timeout)


@dataclass(frozen=True)
class OperationRequest:
    operation: Callable[[], None]
    timeout_seconds: float | None
    on_error: Callable[[str], None]
