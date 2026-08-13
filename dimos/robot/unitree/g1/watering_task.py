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

"""Lifecycle-managed G1 plant-watering task.

This module owns orchestration only. Target providers publish a typed,
world-frame observation; a replaceable base-pose provider says where the mobile
base is; the existing stance and last-mile functions choose and servo the base
pose; and the manipulation module remains the authority for arm planning and
execution.

The behavior is intentionally a small explicit state machine. Every phase is
observable through ``get_status`` and ``watering_status``, cancellation has a
single owner, and the task can be tested without starting transports or MuJoCo.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Any, Protocol
import uuid

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.agents.skill_result import SkillResult
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.manipulation.mobile.target_observation import (
    TargetObservation,
    copy_pose_stamped,
)
from dimos.manipulation.planning.spec.models import PlanningGroupID, RobotName
from dimos.manipulation.skill_errors import ManipulationSkillError
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.manip_last_mile import Twist2D, servo_step
from dimos.robot.unitree.g1.manip_stance import (
    DEFAULT_MAP_PATH,
    POUR_Z,
    TIP_RADIANS,
    PourReachMap,
    pot_in_base_frame,
    select_stance,
    tool_yaw_for,
)
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WateringManipulationSpec(Spec, Protocol):
    """The narrow slice of manipulation used by the watering task."""

    def latch_base_pose(
        self, pose: PoseStamped | None = None, robot_name: RobotName | None = None
    ) -> bool: ...

    def plan_to_pose(
        self,
        pose: Pose,
        robot_name: RobotName | None = None,
        group_id: PlanningGroupID | None = None,
    ) -> bool: ...

    def clear_planned_path(self) -> bool: ...

    def get_error(self) -> str: ...

    def cancel(self) -> bool: ...

    def reset(self) -> SkillResult[ManipulationSkillError]: ...

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float | None = None,
        pitch: float | None = None,
        yaw: float | None = None,
        robot_name: str | None = None,
        group_id: str | None = None,
    ) -> SkillResult[ManipulationSkillError]: ...

    def go_init(
        self, robot_name: str | None = None, group_id: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...


class BaseCommandSink(Protocol):
    """Boundary used by the state machine to command the mobile base."""

    def send(self, command: Twist2D) -> None: ...

    def stop(self) -> None: ...


class WateringState(str, Enum):
    IDLE = "idle"
    WAITING_INPUT = "waiting_input"
    RESETTING = "resetting"
    APPROACHING = "approaching"
    SETTLING = "settling"
    LATCHING_BASE = "latching_base"
    VERIFYING_REACH = "verifying_reach"
    NUDGING = "nudging"
    MOVING_OVER_TARGET = "moving_over_target"
    TIPPING = "tipping"
    HOLDING = "holding"
    RETURNING = "returning"
    COMPLETED = "completed"
    CANCELLED = "cancelled"
    FAILED = "failed"


TERMINAL_WATERING_STATES = {
    WateringState.COMPLETED,
    WateringState.CANCELLED,
    WateringState.FAILED,
}


@dataclass(frozen=True)
class WateringStatus:
    run_id: str | None
    target_id: str | None
    state: WateringState
    message: str
    attempt: int
    started_at: float | None
    updated_at: float

    @property
    def terminal(self) -> bool:
        return self.state in TERMINAL_WATERING_STATES


@dataclass(frozen=True)
class WateringRunResult:
    success: bool
    state: WateringState
    message: str


@dataclass(frozen=True)
class WateringInputSnapshot:
    target: TargetObservation
    base_pose: PoseStamped


class WateringTaskConfig(ModuleConfig):
    target_id: str = "plant_pot_1"
    robot_name: str = "g1"
    group_id: str = "g1/right_arm"
    reach_map_path: Path = DEFAULT_MAP_PATH
    servo_hz: float = Field(default=10.0, gt=0.0)
    servo_timeout: float = Field(default=90.0, gt=0.0)
    input_wait_timeout: float = Field(default=10.0, gt=0.0)
    target_max_age: float = Field(default=2.0, gt=0.0)
    base_pose_max_age: float = Field(default=2.0, gt=0.0)
    settle_seconds: float = Field(default=3.0, ge=0.0)
    hold_seconds: float = Field(default=3.0, ge=0.0)
    verify_attempts: int = Field(default=3, ge=1)
    reach_margin_cells: int = Field(default=3, ge=1)
    stop_repetitions: int = Field(default=5, ge=1)
    auto_start: bool = False
    task_join_timeout: float = Field(default=DEFAULT_THREAD_JOIN_TIMEOUT, ge=0.0)


class WateringInputs:
    """Thread-safe ownership of the task's two live input streams."""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._target: TargetObservation | None = None
        self._base_pose: PoseStamped | None = None

    def update_target(self, observation: TargetObservation) -> None:
        copied = TargetObservation(
            object_id=observation.object_id,
            label=observation.label,
            pose=copy_pose_stamped(observation.pose),
            source=observation.source,
            observed_at=float(observation.observed_at),
            confidence=float(observation.confidence),
        )
        with self._condition:
            self._target = copied
            self._condition.notify_all()

    def update_base_pose(self, base_pose: PoseStamped) -> None:
        with self._condition:
            self._base_pose = copy_pose_stamped(base_pose)
            self._condition.notify_all()

    def snapshot(self, target_id: str) -> WateringInputSnapshot | None:
        with self._condition:
            return self._snapshot_locked(target_id)

    def wait_ready(
        self,
        target_id: str,
        timeout: float,
        cancelled: threading.Event,
        monotonic: Callable[[], float],
    ) -> WateringInputSnapshot | None:
        deadline = monotonic() + timeout
        with self._condition:
            while not cancelled.is_set():
                if (snapshot := self._snapshot_locked(target_id)) is not None:
                    return snapshot
                remaining = deadline - monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=min(remaining, 0.1))
        return None

    def wake(self) -> None:
        with self._condition:
            self._condition.notify_all()

    def _snapshot_locked(self, target_id: str) -> WateringInputSnapshot | None:
        if self._target is None or self._base_pose is None or self._target.object_id != target_id:
            return None
        target = TargetObservation(
            object_id=self._target.object_id,
            label=self._target.label,
            pose=copy_pose_stamped(self._target.pose),
            source=self._target.source,
            observed_at=self._target.observed_at,
            confidence=self._target.confidence,
        )
        return WateringInputSnapshot(
            target=target,
            base_pose=copy_pose_stamped(self._base_pose),
        )


class _WateringCancelledError(RuntimeError):
    pass


class _WateringFailureError(RuntimeError):
    pass


StatusCallback = Callable[[WateringState, str, int], None]
Waiter = Callable[[float], bool]


class WateringSequence:
    """Transport-free watering state machine used by the runtime module and tests."""

    def __init__(
        self,
        config: WateringTaskConfig,
        inputs: WateringInputs,
        base: BaseCommandSink,
        manipulation: WateringManipulationSpec,
        reach_map: PourReachMap,
        cancelled: threading.Event,
        transition: StatusCallback,
        wait: Waiter,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
    ) -> None:
        self._config = config
        self._inputs = inputs
        self._base = base
        self._manipulation = manipulation
        self._reach = reach_map
        self._cancelled = cancelled
        self._transition = transition
        self._wait = wait
        self._monotonic = monotonic
        self._wall_time = wall_time

    def run(self, target_id: str) -> WateringRunResult:
        try:
            self._transition(
                WateringState.WAITING_INPUT,
                f"Waiting for world-frame target '{target_id}' and base pose",
                0,
            )
            snapshot = self._inputs.wait_ready(
                target_id,
                self._config.input_wait_timeout,
                self._cancelled,
                self._monotonic,
            )
            self._check_cancelled()
            if snapshot is None:
                raise _WateringFailureError(
                    f"No target '{target_id}' and base pose within "
                    f"{self._config.input_wait_timeout:.1f}s"
                )
            self._validate_snapshot(snapshot, target_id)

            self._transition(WateringState.RESETTING, "Resetting manipulation state", 0)
            reset = self._manipulation.reset()
            if not reset.is_success():
                raise _WateringFailureError(f"Manipulation reset failed: {reset}")

            self._approach(target_id, snapshot)
            target, yaw = self._latch_and_verify(target_id)
            self._pour(target, yaw)
            message = f"Watered target '{target_id}'"
            self._transition(WateringState.COMPLETED, message, 0)
            return WateringRunResult(True, WateringState.COMPLETED, message)
        except _WateringCancelledError:
            self._cancel_manipulation()
            message = f"Watering target '{target_id}' was cancelled"
            self._transition(WateringState.CANCELLED, message, 0)
            return WateringRunResult(False, WateringState.CANCELLED, message)
        except _WateringFailureError as exc:
            self._cancel_manipulation()
            message = str(exc)
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        except Exception as exc:
            self._cancel_manipulation()
            logger.exception("Unhandled watering task failure")
            message = f"Unexpected watering failure: {exc}"
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        finally:
            self._base.stop()

    def _approach(self, target_id: str, initial: WateringInputSnapshot) -> None:
        pot = self._pot_xy(initial)
        base_x, base_y, _ = self._base_pose(initial)
        stance = select_stance(
            pot,
            approach_yaw=math.atan2(pot[1] - base_y, pot[0] - base_x),
            reach_map=self._reach,
            margin_cells=self._config.reach_margin_cells,
        )
        self._transition(
            WateringState.APPROACHING,
            f"Servoing target to base offset ({stance.offset[0]:.2f}, {stance.offset[1]:.2f})",
            0,
        )
        deadline = self._monotonic() + self._config.servo_timeout
        period = 1.0 / self._config.servo_hz
        while self._monotonic() < deadline:
            self._check_cancelled()
            snapshot = self._require_snapshot(target_id)
            seen = self._seen_offset(snapshot)
            command = servo_step(
                seen,
                stance.offset,
                self._reach,
                margin_cells=self._config.reach_margin_cells,
            )
            if command.is_stop:
                self._base.stop()
                self._transition(
                    WateringState.SETTLING,
                    f"Base stopped with target at ({seen[0]:.2f}, {seen[1]:.2f})",
                    0,
                )
                self._wait_or_cancel(self._config.settle_seconds)
                return
            self._base.send(command)
            self._wait_or_cancel(period)
        raise _WateringFailureError(
            f"Last-mile servo timed out after {self._config.servo_timeout:.1f}s"
        )

    def _latch_and_verify(self, target_id: str) -> tuple[TargetObservation, float]:
        for attempt in range(1, self._config.verify_attempts + 1):
            snapshot = self._require_snapshot(target_id)
            self._transition(
                WateringState.LATCHING_BASE,
                "Latching the stopped base pose into the planning world",
                attempt,
            )
            if not self._manipulation.latch_base_pose(snapshot.base_pose, self._config.robot_name):
                raise _WateringFailureError(f"Base latch failed: {self._manipulation.get_error()}")

            pot = self._pot_xy(snapshot)
            seen = self._seen_offset(snapshot)
            _, _, base_yaw = self._base_pose(snapshot)
            yaw = tool_yaw_for(seen, base_yaw)
            self._transition(
                WateringState.VERIFYING_REACH,
                f"Planning upright and tipped pour poses (attempt {attempt})",
                attempt,
            )
            solved = self._verify_poses(pot, yaw)
            if all(solved.values()):
                return snapshot.target, yaw

            failed = ", ".join(name for name, ok in solved.items() if not ok)
            error = self._manipulation.get_error()
            reset = self._manipulation.reset()
            if not reset.is_success():
                raise _WateringFailureError(
                    f"Reach verification failed ({failed}) and reset failed: {reset}"
                )
            if attempt == self._config.verify_attempts:
                raise _WateringFailureError(
                    f"Pour poses did not plan after {attempt} attempts ({failed}): {error}"
                )
            self._nudge(seen, attempt)

        raise AssertionError("verify_attempts is constrained to at least one")

    def _verify_poses(self, pot: tuple[float, float], yaw: float) -> dict[str, bool]:
        poses = {
            "upright": Pose(
                Vector3(pot[0], pot[1], POUR_Z),
                Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            ),
            "tipped": Pose(
                Vector3(pot[0], pot[1], POUR_Z),
                Quaternion.from_euler(Vector3(TIP_RADIANS, 0.0, yaw)),
            ),
        }
        solved: dict[str, bool] = {}
        for name, pose in poses.items():
            self._check_cancelled()
            solved[name] = self._manipulation.plan_to_pose(
                pose, self._config.robot_name, self._config.group_id
            )
        self._manipulation.clear_planned_path()
        return solved

    def _nudge(self, seen: tuple[float, float], attempt: int) -> None:
        command = Twist2D(vx=-0.1 if seen[0] < 0.3 else 0.1)
        self._transition(
            WateringState.NUDGING,
            f"Verification failed; nudging base before attempt {attempt + 1}",
            attempt,
        )
        period = 1.0 / self._config.servo_hz
        for _ in range(round(self._config.servo_hz)):
            self._check_cancelled()
            self._base.send(command)
            self._wait_or_cancel(period)
        self._base.stop()
        self._transition(WateringState.SETTLING, "Settling after verification nudge", attempt)
        self._wait_or_cancel(self._config.settle_seconds)

    def _pour(self, target: TargetObservation, yaw: float) -> None:
        pot = (float(target.pose.position.x), float(target.pose.position.y))
        self._check_cancelled()
        self._transition(
            WateringState.MOVING_OVER_TARGET,
            f"Moving right tool over target at ({pot[0]:.2f}, {pot[1]:.2f})",
            0,
        )
        over = self._manipulation.move_to_pose(
            x=pot[0],
            y=pot[1],
            z=POUR_Z,
            roll=0.0,
            pitch=0.0,
            yaw=yaw,
            robot_name=self._config.robot_name,
            group_id=self._config.group_id,
        )
        if not over.is_success():
            self._manipulation.reset()
            raise _WateringFailureError(f"Failed to move over target: {over}")

        self._check_cancelled()
        self._transition(WateringState.TIPPING, "Tipping the watering tool", 0)
        tipped = self._manipulation.move_to_pose(
            x=pot[0],
            y=pot[1],
            z=POUR_Z,
            roll=float(TIP_RADIANS),
            pitch=0.0,
            yaw=yaw,
            robot_name=self._config.robot_name,
            group_id=self._config.group_id,
        )
        if not tipped.is_success():
            self._manipulation.reset()
            raise _WateringFailureError(f"Failed to tip over target: {tipped}")

        self._transition(WateringState.HOLDING, "Holding the tipped pose", 0)
        self._wait_or_cancel(self._config.hold_seconds)
        self._transition(WateringState.RETURNING, "Returning the right arm to init", 0)
        home = self._manipulation.go_init(self._config.robot_name, self._config.group_id)
        if not home.is_success():
            raise _WateringFailureError(f"Pour completed but arm return failed: {home}")

    def _require_snapshot(self, target_id: str) -> WateringInputSnapshot:
        snapshot = self._inputs.snapshot(target_id)
        if snapshot is None:
            raise _WateringFailureError(f"Target '{target_id}' or base pose disappeared")
        self._validate_snapshot(snapshot, target_id)
        return snapshot

    def _validate_snapshot(self, snapshot: WateringInputSnapshot, target_id: str) -> None:
        target = snapshot.target
        if target.object_id != target_id:
            raise _WateringFailureError(
                f"Received target '{target.object_id}' while running '{target_id}'"
            )
        if target.pose.frame_id != "world":
            raise _WateringFailureError(
                f"Target pose must be world-framed, got '{target.pose.frame_id}'"
            )
        if snapshot.base_pose.frame_id not in ("", "world"):
            raise _WateringFailureError(
                f"Base pose must be world-framed, got '{snapshot.base_pose.frame_id}'"
            )
        now = self._wall_time()
        target_age = now - target.observed_at
        base_pose_age = now - float(snapshot.base_pose.ts)
        if target_age > self._config.target_max_age:
            raise _WateringFailureError(f"Target observation is stale ({target_age:.2f}s old)")
        if base_pose_age > self._config.base_pose_max_age:
            raise _WateringFailureError(f"Base pose is stale ({base_pose_age:.2f}s old)")
        values = (
            *target.pose.position,
            *target.pose.orientation,
            *snapshot.base_pose.position,
            *snapshot.base_pose.orientation,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise _WateringFailureError("Target or base pose contains non-finite values")

    @staticmethod
    def _pot_xy(snapshot: WateringInputSnapshot) -> tuple[float, float]:
        return float(snapshot.target.pose.position.x), float(snapshot.target.pose.position.y)

    @staticmethod
    def _base_pose(snapshot: WateringInputSnapshot) -> tuple[float, float, float]:
        base_pose = snapshot.base_pose
        return (
            float(base_pose.position.x),
            float(base_pose.position.y),
            float(base_pose.orientation.to_euler().z),
        )

    def _seen_offset(self, snapshot: WateringInputSnapshot) -> tuple[float, float]:
        x, y, yaw = self._base_pose(snapshot)
        return pot_in_base_frame(self._pot_xy(snapshot), (x, y), yaw)

    def _wait_or_cancel(self, seconds: float) -> None:
        self._check_cancelled()
        if seconds > 0.0 and self._wait(seconds):
            raise _WateringCancelledError
        self._check_cancelled()

    def _check_cancelled(self) -> None:
        if self._cancelled.is_set():
            raise _WateringCancelledError

    def _cancel_manipulation(self) -> None:
        try:
            self._manipulation.cancel()
        except Exception:
            logger.warning("Manipulation cancellation after task failure failed", exc_info=True)


class _PortBaseCommandSink:
    def __init__(self, publish: Callable[[Twist], None], stop_repetitions: int) -> None:
        self._publish = publish
        self._stop_repetitions = stop_repetitions

    def send(self, command: Twist2D) -> None:
        self._publish(
            Twist(
                linear=[command.vx, command.vy, 0.0],
                angular=[0.0, 0.0, command.wz],
            )
        )

    def stop(self) -> None:
        for _ in range(self._stop_repetitions):
            self.send(Twist2D())


class WateringTaskModule(Module):
    """Own one cancellable watering run and publish its explicit state."""

    config: WateringTaskConfig
    _manipulation: WateringManipulationSpec

    target_observation: In[TargetObservation]
    base_pose: In[PoseStamped]
    base_command: Out[Twist]
    watering_status: Out[WateringStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        now = time.time()
        self._inputs = WateringInputs()
        self._cancel_event = threading.Event()
        self._task_thread: threading.Thread | None = None
        self._status_lock = threading.RLock()
        self._status = WateringStatus(
            run_id=None,
            target_id=None,
            state=WateringState.IDLE,
            message="Ready",
            attempt=0,
            started_at=None,
            updated_at=now,
        )
        self._base = _PortBaseCommandSink(self.base_command.publish, self.config.stop_repetitions)

    @rpc
    def start(self) -> None:
        super().start()
        target_unsubscribe = self.target_observation.subscribe(self._inputs.update_target)
        base_pose_unsubscribe = self.base_pose.subscribe(self._inputs.update_base_pose)
        for unsubscribe in (target_unsubscribe, base_pose_unsubscribe):
            self.register_disposable(
                Disposable(unsubscribe) if callable(unsubscribe) else unsubscribe
            )
        self._publish_status(self._status)
        if self.config.auto_start:
            self.start_watering()

    @rpc
    def stop(self) -> None:
        self._request_cancel()
        thread = self._task_thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=self.config.task_join_timeout)
            if thread.is_alive():
                logger.warning("Watering task thread did not stop before module shutdown")
        self._base.stop()
        super().stop()

    @rpc
    def start_watering(self, target_id: str | None = None) -> bool:
        """Start one background watering run for the configured target."""
        selected_target = target_id or self.config.target_id
        with self._status_lock:
            if self._task_thread is not None and self._task_thread.is_alive():
                return False
            self._cancel_event.clear()
            started_at = time.time()
            run_id = str(uuid.uuid4())
            self._status = WateringStatus(
                run_id=run_id,
                target_id=selected_target,
                state=WateringState.WAITING_INPUT,
                message="Starting watering task",
                attempt=0,
                started_at=started_at,
                updated_at=started_at,
            )
            self._task_thread = threading.Thread(
                target=self._run,
                args=(selected_target,),
                name=f"WateringTask-{run_id[:8]}",
                daemon=True,
            )
            thread = self._task_thread
        self._publish_status(self._status)
        thread.start()
        return True

    @rpc
    def cancel_watering(self) -> bool:
        """Request cancellation and stop both base and arm motion."""
        thread = self._task_thread
        if thread is None or not thread.is_alive():
            return False
        self._request_cancel()
        return True

    @rpc
    def get_status(self) -> WateringStatus:
        """Return the latest immutable task status snapshot."""
        with self._status_lock:
            return self._status

    def _run(self, target_id: str) -> None:
        try:
            reach_map = PourReachMap.load(self.config.reach_map_path)
            sequence = WateringSequence(
                config=self.config,
                inputs=self._inputs,
                base=self._base,
                manipulation=self._manipulation,
                reach_map=reach_map,
                cancelled=self._cancel_event,
                transition=self._transition,
                wait=self._cancel_event.wait,
            )
            sequence.run(target_id)
        except Exception as exc:
            logger.exception("Failed to initialize watering sequence")
            self._transition(WateringState.FAILED, f"Task initialization failed: {exc}", 0)

    def _request_cancel(self) -> None:
        self._cancel_event.set()
        self._inputs.wake()
        self._base.stop()
        thread = self._task_thread
        if thread is not None and thread.is_alive():
            try:
                self._manipulation.cancel()
            except Exception:
                logger.warning("Manipulation cancellation failed", exc_info=True)

    def _transition(self, state: WateringState, message: str, attempt: int) -> None:
        with self._status_lock:
            previous = self._status
            status = WateringStatus(
                run_id=previous.run_id,
                target_id=previous.target_id,
                state=state,
                message=message,
                attempt=attempt,
                started_at=previous.started_at,
                updated_at=time.time(),
            )
            self._status = status
        logger.info("Watering task [%s]: %s", state.value, message)
        self._publish_status(status)

    def _publish_status(self, status: WateringStatus) -> None:
        self.watering_status.publish(status)
