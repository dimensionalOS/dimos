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

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
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
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.robot.unitree.g1.manip_config import G1_NOMINAL_PELVIS_Z
from dimos.robot.unitree.g1.manip_last_mile import (
    ApproachControllerConfig,
    ApproachPhase,
    approach_step,
)
from dimos.robot.unitree.g1.manip_stance import (
    DEFAULT_MAP_PATH,
    TIP_RADIANS,
    PourReachMap,
    pot_in_base_frame,
    select_stance,
    tool_yaw_for,
    wrap_angle,
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
        pre_lift: bool = True,
    ) -> SkillResult[ManipulationSkillError]: ...

    def go_init(
        self, robot_name: str | None = None, group_id: str | None = None
    ) -> SkillResult[ManipulationSkillError]: ...


class ApproachCommandSink(Protocol):
    """Boundary used by the state machine to start and stop path following."""

    def start(self, path: NavPath) -> None: ...

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
    nominal_pelvis_height: float = Field(default=G1_NOMINAL_PELVIS_Z, gt=0.0)
    stop_repetitions: int = Field(default=5, ge=1)
    approach_max_distance: float = Field(default=2.0, gt=0.0)
    approach_position_tolerance: float = Field(default=0.06, gt=0.0)
    approach_yaw_tolerance: float = Field(default=math.radians(5.0), gt=0.0)
    max_target_drift: float = Field(default=0.15, gt=0.0)
    max_base_pose_jump: float = Field(default=0.25, gt=0.0)
    approach_motion_enabled: bool = False
    pour_motion_enabled: bool = False
    motion_enabled: bool = True
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
ApproachPreviewCallback = Callable[[NavPath, PoseStamped], None]


def build_approach_preview(
    snapshot: WateringInputSnapshot,
    reach_map: PourReachMap,
    margin_cells: int,
) -> tuple[NavPath, PoseStamped]:
    """Build the explicit straight path and final stance used by the controller."""
    pot = (float(snapshot.target.pose.position.x), float(snapshot.target.pose.position.y))
    base = snapshot.base_pose
    to_pot_x = pot[0] - float(base.position.x)
    to_pot_y = pot[1] - float(base.position.y)
    # The final pose must be a manipulation stance, not a copy of whatever
    # direction the robot happened to face before walking. Copying the initial
    # yaw made RPP align to the path, approach the pot, and then rotate back to
    # that unrelated heading. Orient the reach map's preferred right-arm offset
    # along the current line of sight instead: the torso faces the plant with
    # the exact lateral bearing that the right arm's capability map expects.
    preferred_offset = reach_map.best_offset(margin_cells)
    preferred_bearing = math.atan2(preferred_offset[1], preferred_offset[0])
    approach_yaw = (
        wrap_angle(math.atan2(to_pot_y, to_pot_x) - preferred_bearing)
        if math.hypot(to_pot_x, to_pot_y) > 1e-9
        else float(base.orientation.to_euler().z)
    )
    stance = select_stance(
        pot,
        approach_yaw=approach_yaw,
        reach_map=reach_map,
        margin_cells=margin_cells,
    )
    goal = PoseStamped(
        ts=base.ts,
        frame_id="world",
        position=[stance.x, stance.y, float(base.position.z)],
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, stance.yaw)),
    )
    start = copy_pose_stamped(base)
    start.frame_id = "world"
    dx = stance.x - float(base.position.x)
    dy = stance.y - float(base.position.y)
    # This is a pose path, not merely an XY polyline: align to the first
    # segment before translating, then retain the reach-map-selected yaw on the
    # final pose so manipulation gets the base orientation it planned around.
    if math.hypot(dx, dy) > 1e-9:
        start.orientation = Quaternion.from_euler(Vector3(0.0, 0.0, math.atan2(dy, dx)))
    return NavPath(ts=base.ts, frame_id="world", poses=[start, goal]), goal


class WateringSequence:
    """Transport-free watering state machine used by the runtime module and tests."""

    def __init__(
        self,
        config: WateringTaskConfig,
        inputs: WateringInputs,
        approach_commands: ApproachCommandSink,
        manipulation: WateringManipulationSpec,
        reach_map: PourReachMap,
        cancelled: threading.Event,
        transition: StatusCallback,
        wait: Waiter,
        approach_preview: ApproachPreviewCallback | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
    ) -> None:
        self._config = config
        self._inputs = inputs
        self._approach_commands = approach_commands
        self._manipulation = manipulation
        self._reach = reach_map
        self._cancelled = cancelled
        self._transition = transition
        self._wait = wait
        self._approach_preview = approach_preview
        self._monotonic = monotonic
        self._wall_time = wall_time
        self._controller_config = ApproachControllerConfig(
            position_tolerance=config.approach_position_tolerance,
            yaw_tolerance=config.approach_yaw_tolerance,
            max_approach_distance=config.approach_max_distance,
        )

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
            target, yaw, pour_z = self._latch_and_verify(target_id)
            self._pour(target, yaw, pour_z)
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
            self._approach_commands.stop()

    def run_approach(self, target_id: str) -> WateringRunResult:
        """Drive to the previewed stance without invoking manipulation."""
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
            self._approach(target_id, snapshot)
            message = f"Reached approach stance for target '{target_id}'; arm was not moved"
            self._transition(WateringState.COMPLETED, message, 0)
            return WateringRunResult(True, WateringState.COMPLETED, message)
        except _WateringCancelledError:
            message = f"Approach to target '{target_id}' was cancelled"
            self._transition(WateringState.CANCELLED, message, 0)
            return WateringRunResult(False, WateringState.CANCELLED, message)
        except _WateringFailureError as exc:
            message = str(exc)
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        except Exception as exc:
            logger.exception("Unhandled watering approach failure")
            message = f"Unexpected watering approach failure: {exc}"
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        finally:
            self._approach_commands.stop()

    def run_pour(self, target_id: str) -> WateringRunResult:
        """Pour from the current stance without commanding base motion."""
        try:
            self._approach_commands.stop()
            self._transition(
                WateringState.WAITING_INPUT,
                f"Waiting for world-frame target '{target_id}' and stopped base pose",
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

            target, yaw, pour_z = self._latch_and_verify(target_id)
            self._pour(target, yaw, pour_z)
            message = f"Poured at target '{target_id}' without moving the base"
            self._transition(WateringState.COMPLETED, message, 0)
            return WateringRunResult(True, WateringState.COMPLETED, message)
        except _WateringCancelledError:
            self._cancel_manipulation()
            message = f"Pour at target '{target_id}' was cancelled"
            self._transition(WateringState.CANCELLED, message, 0)
            return WateringRunResult(False, WateringState.CANCELLED, message)
        except _WateringFailureError as exc:
            self._cancel_manipulation()
            message = str(exc)
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        except Exception as exc:
            self._cancel_manipulation()
            logger.exception("Unhandled watering pour failure")
            message = f"Unexpected watering pour failure: {exc}"
            self._transition(WateringState.FAILED, message, 0)
            return WateringRunResult(False, WateringState.FAILED, message)
        finally:
            self._approach_commands.stop()

    def _approach(self, target_id: str, initial: WateringInputSnapshot) -> None:
        path, goal = build_approach_preview(
            initial,
            self._reach,
            self._config.reach_margin_cells,
        )
        if self._approach_preview is not None:
            self._approach_preview(path, goal)
        initial_step = approach_step(
            self._base_pose(initial),
            (
                float(goal.position.x),
                float(goal.position.y),
                float(goal.orientation.to_euler().z),
            ),
            self._controller_config,
        )
        if initial_step.blocked:
            raise _WateringFailureError(f"Approach blocked: {initial_step.message}")

        # The watering state machine owns the goal and safety supervision; the
        # navigation module owns the control law and publishes autonomy Twist.
        self._approach_commands.start(path)
        deadline = self._monotonic() + self._config.servo_timeout
        period = 1.0 / self._config.servo_hz
        initial_pot = self._pot_xy(initial)
        previous_base = self._base_pose(initial)
        previous_phase: ApproachPhase | None = None
        while self._monotonic() < deadline:
            self._check_cancelled()
            snapshot = self._require_snapshot(target_id)
            current_pot = self._pot_xy(snapshot)
            target_drift = math.hypot(
                current_pot[0] - initial_pot[0],
                current_pot[1] - initial_pot[1],
            )
            if target_drift > self._config.max_target_drift:
                raise _WateringFailureError(
                    f"Target moved {target_drift:.2f} m during approach; stopping"
                )
            current_base = self._base_pose(snapshot)
            base_jump = math.hypot(
                current_base[0] - previous_base[0],
                current_base[1] - previous_base[1],
            )
            if base_jump > self._config.max_base_pose_jump:
                raise _WateringFailureError(
                    f"Base pose jumped {base_jump:.2f} m between control ticks; stopping"
                )
            previous_base = current_base
            step = approach_step(
                current_base,
                (
                    float(goal.position.x),
                    float(goal.position.y),
                    float(goal.orientation.to_euler().z),
                ),
                self._controller_config,
            )
            if step.blocked:
                raise _WateringFailureError(f"Approach blocked: {step.message}")
            if step.phase is not previous_phase:
                self._transition(
                    WateringState.APPROACHING,
                    f"{step.phase.value}: distance={step.distance_error:.2f} m, "
                    f"heading={math.degrees(step.path_heading_error):.1f} deg, "
                    f"final_yaw={math.degrees(step.stance_yaw_error):.1f} deg",
                    0,
                )
                previous_phase = step.phase
            if step.arrived:
                self._approach_commands.stop()
                self._transition(
                    WateringState.SETTLING,
                    f"Base reached stance at ({current_base[0]:.2f}, {current_base[1]:.2f})",
                    0,
                )
                self._wait_or_cancel(self._config.settle_seconds)
                settled_snapshot = self._require_snapshot(target_id)
                settled_step = approach_step(
                    self._base_pose(settled_snapshot),
                    (
                        float(goal.position.x),
                        float(goal.position.y),
                        float(goal.orientation.to_euler().z),
                    ),
                    self._controller_config,
                )
                if settled_step.arrived:
                    return
                previous_base = self._base_pose(settled_snapshot)
                previous_phase = None
                self._transition(
                    WateringState.APPROACHING,
                    "Settling moved the base outside stance tolerance; resuming path follower",
                    0,
                )
                self._approach_commands.start(path)
                continue
            self._wait_or_cancel(period)
        raise _WateringFailureError(
            f"Path follower timed out after {self._config.servo_timeout:.1f}s"
        )

    def _latch_and_verify(self, target_id: str) -> tuple[TargetObservation, float, float]:
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
            pour_z = self._pour_world_z(snapshot)
            self._transition(
                WateringState.VERIFYING_REACH,
                f"Planning upright and tipped pour poses at world z={pour_z:.2f} m "
                f"(attempt {attempt})",
                attempt,
            )
            solved = self._verify_poses(pot, yaw, pour_z)
            if all(solved.values()):
                return snapshot.target, yaw, pour_z

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
            self._transition(
                WateringState.VERIFYING_REACH,
                f"Retrying pose planning without moving the verified stance ({attempt + 1})",
                attempt,
            )

        raise AssertionError("verify_attempts is constrained to at least one")

    def _pour_world_z(self, snapshot: WateringInputSnapshot) -> float:
        """Convert the ground-relative reach-map height into the live LIO world."""
        ground_z = float(snapshot.base_pose.position.z) - self._config.nominal_pelvis_height
        return ground_z + self._reach.pour_z

    def _verify_poses(self, pot: tuple[float, float], yaw: float, pour_z: float) -> dict[str, bool]:
        poses = {
            "upright": Pose(
                Vector3(pot[0], pot[1], pour_z),
                Quaternion.from_euler(Vector3(0.0, 0.0, yaw)),
            ),
            "tipped": Pose(
                Vector3(pot[0], pot[1], pour_z),
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

    def _pour(self, target: TargetObservation, yaw: float, pour_z: float) -> None:
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
            z=pour_z,
            roll=0.0,
            pitch=0.0,
            yaw=yaw,
            robot_name=self._config.robot_name,
            group_id=self._config.group_id,
            pre_lift=False,
        )
        if not over.is_success():
            self._manipulation.reset()
            raise _WateringFailureError(f"Failed to move over target: {over}")

        self._check_cancelled()
        self._transition(WateringState.TIPPING, "Tipping the watering tool", 0)
        tipped = self._manipulation.move_to_pose(
            x=pot[0],
            y=pot[1],
            z=pour_z,
            roll=float(TIP_RADIANS),
            pitch=0.0,
            yaw=yaw,
            robot_name=self._config.robot_name,
            group_id=self._config.group_id,
            pre_lift=False,
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


class _PortApproachCommandSink:
    def __init__(
        self,
        publish_path: Callable[[NavPath], None],
        publish_stop: Callable[[Bool], None],
        stop_repetitions: int,
    ) -> None:
        self._publish_path = publish_path
        self._publish_stop = publish_stop
        self._stop_repetitions = stop_repetitions

    def start(self, path: NavPath) -> None:
        self._publish_path(path)

    def stop(self) -> None:
        for _ in range(self._stop_repetitions):
            self._publish_stop(Bool(data=True))


class WateringTaskModule(Module):
    """Own one cancellable watering run and publish its explicit state."""

    config: WateringTaskConfig
    _manipulation: WateringManipulationSpec

    target_observation: In[TargetObservation]
    base_pose: In[PoseStamped]
    operator_command: In[Twist]
    approach_command_path: Out[NavPath]
    stop_approach: Out[Bool]
    approach_path: Out[NavPath]
    approach_goal: Out[PoseStamped]
    watering_status: Out[WateringStatus]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        now = time.time()
        self._inputs = WateringInputs()
        self._cancel_event = threading.Event()
        self._task_thread: threading.Thread | None = None
        self._run_controls_arm = False
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
        self._approach_commands = _PortApproachCommandSink(
            self.approach_command_path.publish,
            self.stop_approach.publish,
            self.config.stop_repetitions,
        )

    @rpc
    def start(self) -> None:
        super().start()
        target_unsubscribe = self.target_observation.subscribe(self._inputs.update_target)
        base_pose_unsubscribe = self.base_pose.subscribe(self._inputs.update_base_pose)
        operator_unsubscribe = self.operator_command.subscribe(self._on_operator_command)
        for unsubscribe in (target_unsubscribe, base_pose_unsubscribe, operator_unsubscribe):
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
        self._approach_commands.stop()
        super().stop()

    @rpc
    def start_watering(self, target_id: str | None = None) -> bool:
        """Start one background watering run for the configured target."""
        selected_target = target_id or self.config.target_id
        if not self.config.motion_enabled:
            self._transition(
                WateringState.IDLE,
                "Autonomous motion is disabled; inspect preview_watering() first",
                0,
            )
            return False
        return self._start_run(
            selected_target,
            runner=self._run,
            message="Starting watering task",
            thread_name="WateringTask",
            controls_arm=True,
        )

    @rpc
    def start_approach(self, target_id: str | None = None) -> bool:
        """Drive only to the previewed stance; never plan or move the arm."""
        selected_target = target_id or self.config.target_id
        if not self.config.approach_motion_enabled:
            self._transition(
                WateringState.IDLE,
                "Base-only approach motion is disabled",
                0,
            )
            return False
        return self._start_run(
            selected_target,
            runner=self._run_approach,
            message="Starting base-only approach",
            thread_name="WateringApproach",
            controls_arm=False,
        )

    @rpc
    def start_pour(self, target_id: str | None = None) -> bool:
        """Pour from the current verified stance without commanding the base."""
        selected_target = target_id or self.config.target_id
        if not self.config.pour_motion_enabled:
            self._transition(
                WateringState.IDLE,
                "Pour-only arm motion is disabled",
                0,
            )
            return False
        return self._start_run(
            selected_target,
            runner=self._run_pour,
            message="Starting pour-only task from the current stance",
            thread_name="WateringPour",
            controls_arm=True,
        )

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

    @rpc
    def preview_watering(self, target_id: str | None = None) -> NavPath | None:
        """Publish and return the straight approach without commanding motion."""
        selected_target = target_id or self.config.target_id
        snapshot = self._inputs.snapshot(selected_target)
        if snapshot is None:
            self._transition(
                WateringState.IDLE,
                f"Cannot preview: target '{selected_target}' or base pose is unavailable",
                0,
            )
            return None
        reach_map = PourReachMap.load(self.config.reach_map_path)
        path, goal = build_approach_preview(
            snapshot,
            reach_map,
            self.config.reach_margin_cells,
        )
        self._publish_approach_preview(path, goal)
        self._transition(
            WateringState.IDLE,
            f"Previewed stance ({goal.position.x:.2f}, {goal.position.y:.2f}, "
            f"yaw={math.degrees(goal.orientation.to_euler().z):.1f} deg)",
            0,
        )
        return path

    def _run(self, target_id: str) -> None:
        try:
            self._make_sequence().run(target_id)
        except Exception as exc:
            logger.exception("Failed to initialize watering sequence")
            self._transition(WateringState.FAILED, f"Task initialization failed: {exc}", 0)

    def _run_approach(self, target_id: str) -> None:
        try:
            self._make_sequence().run_approach(target_id)
        except Exception as exc:
            logger.exception("Failed to initialize watering approach")
            self._transition(WateringState.FAILED, f"Approach initialization failed: {exc}", 0)

    def _run_pour(self, target_id: str) -> None:
        try:
            self._make_sequence().run_pour(target_id)
        except Exception as exc:
            logger.exception("Failed to initialize watering pour")
            self._transition(WateringState.FAILED, f"Pour initialization failed: {exc}", 0)

    def _make_sequence(self) -> WateringSequence:
        return WateringSequence(
            config=self.config,
            inputs=self._inputs,
            approach_commands=self._approach_commands,
            manipulation=self._manipulation,
            reach_map=PourReachMap.load(self.config.reach_map_path),
            cancelled=self._cancel_event,
            transition=self._transition,
            wait=self._cancel_event.wait,
            approach_preview=self._publish_approach_preview,
        )

    def _start_run(
        self,
        target_id: str,
        *,
        runner: Callable[[str], None],
        message: str,
        thread_name: str,
        controls_arm: bool,
    ) -> bool:
        with self._status_lock:
            if self._task_thread is not None and self._task_thread.is_alive():
                return False
            self._cancel_event.clear()
            self._run_controls_arm = controls_arm
            started_at = time.time()
            run_id = str(uuid.uuid4())
            self._status = WateringStatus(
                run_id=run_id,
                target_id=target_id,
                state=WateringState.WAITING_INPUT,
                message=message,
                attempt=0,
                started_at=started_at,
                updated_at=started_at,
            )
            self._task_thread = threading.Thread(
                target=runner,
                args=(target_id,),
                name=f"{thread_name}-{run_id[:8]}",
                daemon=True,
            )
            thread = self._task_thread
        self._publish_status(self._status)
        thread.start()
        return True

    def _request_cancel(self) -> None:
        self._cancel_event.set()
        self._inputs.wake()
        self._approach_commands.stop()
        thread = self._task_thread
        if self._run_controls_arm and thread is not None and thread.is_alive():
            try:
                self._manipulation.cancel()
            except Exception:
                logger.warning("Manipulation cancellation failed", exc_info=True)

    def _on_operator_command(self, _command: Twist) -> None:
        thread = self._task_thread
        if thread is not None and thread.is_alive():
            logger.warning("Operator command received; cancelling autonomous watering")
            self._request_cancel()

    def _publish_approach_preview(self, path: NavPath, goal: PoseStamped) -> None:
        self.approach_path.publish(path)
        self.approach_goal.publish(goal)

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
