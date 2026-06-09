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
import time
from typing import Any, Literal

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.sensor_msgs.JointState import JointState


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
    PREVIEWING = "previewing"
    EXECUTING = "executing"
    CANCELLING = "cancelling"
    CLEARING_PLAN = "clearing_plan"


PreviewSource = Literal["cartesian", "joints"]


@dataclass
class FeasibilityState:
    status: FeasibilityStatus = FeasibilityStatus.UNKNOWN
    message: str = ""
    sequence_id: int = 0


@dataclass
class PanelPlanState:
    status: PlanStatus = PlanStatus.NONE
    robot: str | None = None
    target_pose: Pose | None = None
    target_joints: list[float] | None = None
    start_joints_snapshot: list[float] | None = None
    planned_path: list[JointState] | None = None


@dataclass
class PanelSession:
    selected_robot: str | None = None
    runtime: PanelRuntime = PanelRuntime.STOPPED
    backend_status: BackendConnectionStatus = BackendConnectionStatus.DISCONNECTED
    target_status: TargetStatus = TargetStatus.EMPTY
    action_status: ActionStatus = ActionStatus.IDLE
    manipulation_state: str = "DISCONNECTED"
    robot_info: dict[str, Any] | None = None
    current_joints: list[float] | None = None
    current_ee_pose: Pose | None = None
    cartesian_target: Pose | None = None
    joint_target: list[float] | None = None
    feasibility: FeasibilityState = field(default_factory=FeasibilityState)
    latest_sequence_id: int = 0
    sync_source: PreviewSource | None = None
    plan_state: PanelPlanState = field(default_factory=PanelPlanState)
    error: str = ""

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
            and self.selected_robot is not None
            and self.action_status == ActionStatus.IDLE
            and self.target_status == TargetStatus.FEASIBLE
            and self.manipulation_state in {"IDLE", "COMPLETED"}
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
        return self.action_status in {ActionStatus.PREVIEWING, ActionStatus.EXECUTING} or (
            self.manipulation_state == "EXECUTING"
        )

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
            and plan.robot == self.selected_robot
            and plan.start_joints_snapshot is not None
            and self.current_joints is not None
        ):
            return False
        if len(plan.start_joints_snapshot) != len(self.current_joints):
            return False
        return all(
            abs(expected - current) <= current_tolerance
            for expected, current in zip(
                plan.start_joints_snapshot, self.current_joints, strict=False
            )
        )

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
class PreviewRequest:
    sequence_id: int
    source: PreviewSource
    robot_name: str
    pose: Pose | None = None
    joints: JointState | None = None


class PreviewWorker:
    def __init__(
        self,
        handler: Callable[[PreviewRequest], dict[str, Any]],
        apply_result: Callable[[PreviewRequest, dict[str, Any]], None],
        debounce_seconds: float,
    ) -> None:
        self._handler = handler
        self._apply_result = apply_result
        self._debounce_seconds = debounce_seconds
        self._requests: queue.Queue[PreviewRequest] = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="ViserPreviewWorker", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def submit(self, request: PreviewRequest) -> None:
        while True:
            try:
                self._requests.get_nowait()
            except queue.Empty:
                break
        try:
            self._requests.put_nowait(request)
        except queue.Full:
            pass

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                request = self._requests.get(timeout=0.1)
            except queue.Empty:
                continue
            time.sleep(self._debounce_seconds)
            while True:
                try:
                    request = self._requests.get_nowait()
                except queue.Empty:
                    break
            result = self._handler(request)
            self._apply_result(request, result)
