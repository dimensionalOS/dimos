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

"""Track a planner's timed base trajectory on a twist base."""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
from typing import Any

from pydantic import ConfigDict, NonNegativeFloat, PositiveFloat

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()

_FB_CLAMP_LINEAR = 0.15
_FB_CLAMP_YAW = 0.4
_SPEED_SLACK = 1e-3


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


@dataclass
class PlanarBaseTrajectoryTaskConfig:
    joint_names: list[str]
    priority: int = 10
    kp: tuple[float, float, float] = (1.0, 1.0, 1.0)
    max_linear: float = 1.0
    max_angular: float = 2.0
    start_position_tolerance: float = 0.1
    start_orientation_tolerance: float = 0.1
    position_goal_tolerance: float = 0.05
    orientation_goal_tolerance: float = 0.1
    max_tracking_error: float = 0.5
    max_yaw_tracking_error: float = 0.5
    settle_timeout: float = 2.0
    stop_hold_s: float = 0.5
    stale_pose_timeout: float = 0.3


class PlanarBaseTrajectoryTask(BaseControlTask):
    """Follow an (x, y, yaw) trajectory in the odometry frame with feedforward and feedback.

    The trajectory's three columns are x, y and yaw in that order; yaw may be
    unwrapped. Commands are body-frame (vx, vy, wz) on the twist joints, which
    report odometry through their position fields. A finished, failed, cancelled
    or preempted run reports its final state at once and streams zero velocity
    for ``stop_hold_s``.
    """

    def __init__(self, name: str, config: PlanarBaseTrajectoryTaskConfig) -> None:
        if len(config.joint_names) != 3:
            raise ValueError(
                f"PlanarBaseTrajectoryTask '{name}' needs 3 joints (vx, vy, wz), "
                f"got {len(config.joint_names)}"
            )
        self._name = name
        self._config = config
        self._joints = list(config.joint_names)
        self._lock = threading.Lock()
        self._state = TrajectoryState.IDLE
        self._error = ""
        self._trajectory: JointTrajectory | None = None
        self._start_t: float | None = None
        self._elapsed = 0.0
        self._stop_until: float | None = None
        self._stop_pending = False
        self._armed_t: float | None = None
        self._last_pose_t: float | None = None

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=frozenset(self._joints),
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        return (
            self._state == TrajectoryState.EXECUTING
            or self._stop_pending
            or self._stop_until is not None
        )

    def execute(self, trajectory: JointTrajectory) -> TrajectoryExecutionResult:
        """Accept a trajectory; its start is checked against odometry on the first tick."""
        problem = _trajectory_problem(trajectory, self._config)
        if problem:
            return TrajectoryExecutionResult(TrajectoryExecutionStatus.INVALID_TRAJECTORY, problem)
        with self._lock:
            if self._state == TrajectoryState.EXECUTING:
                return TrajectoryExecutionResult(
                    TrajectoryExecutionStatus.ALREADY_EXECUTING,
                    f"Base trajectory task '{self._name}' is already executing",
                )
            self._trajectory = trajectory
            self._start_t = None
            self._elapsed = 0.0
            self._stop_until = None
            self._stop_pending = False
            self._armed_t = None
            self._error = ""
            self._last_pose_t = None
            self._state = TrajectoryState.EXECUTING
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    def cancel(self) -> bool:
        with self._lock:
            if self._state != TrajectoryState.EXECUTING:
                return False
            self._stop(None, TrajectoryState.ABORTED, "cancelled")
        return True

    def get_status(self) -> TrajectoryStatus:
        with self._lock:
            duration = self._trajectory.duration if self._trajectory is not None else 0.0
            progress = 1.0 if duration <= 0.0 else min(1.0, self._elapsed / duration)
            return TrajectoryStatus(
                state=self._state,
                progress=progress,
                time_elapsed=self._elapsed,
                time_remaining=max(0.0, duration - self._elapsed),
                error=self._error,
            )

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if self._stop_pending:
                self._stop_until = state.t_now + self._config.stop_hold_s
                self._stop_pending = False
            if self._stop_until is not None:
                if state.t_now >= self._stop_until:
                    self._stop_until = None
                return self._command(0.0, 0.0, 0.0)
            if self._state != TrajectoryState.EXECUTING or self._trajectory is None:
                return None
            if self._armed_t is None:
                self._armed_t = state.t_now

            pose = self._read_pose(state)
            if pose is None:
                last_seen = self._last_pose_t if self._last_pose_t is not None else self._armed_t
                if state.t_now - last_seen > self._config.stale_pose_timeout:
                    return self._fail(state.t_now, "base odometry is unavailable")
                return self._command(0.0, 0.0, 0.0)

            first_tick = self._start_t is None
            if self._start_t is None:
                self._start_t = state.t_now
            self._elapsed = state.t_now - self._start_t
            reference, reference_velocity = self._trajectory.sample(self._elapsed)
            position_error = math.hypot(reference[0] - pose[0], reference[1] - pose[1])
            yaw_error = abs(angle_diff(reference[2], pose[2]))

            config = self._config
            if first_tick and (
                position_error > config.start_position_tolerance
                or yaw_error > config.start_orientation_tolerance
            ):
                return self._fail(
                    state.t_now,
                    f"trajectory starts {position_error:.3f} m / {yaw_error:.3f} rad from the "
                    "base's odometry",
                )
            if (
                position_error > config.max_tracking_error
                or yaw_error > config.max_yaw_tracking_error
            ):
                return self._fail(
                    state.t_now,
                    f"base is {position_error:.3f} m / {yaw_error:.3f} rad off the trajectory",
                )
            if self._elapsed >= self._trajectory.duration:
                if (
                    position_error < config.position_goal_tolerance
                    and yaw_error < config.orientation_goal_tolerance
                ):
                    self._stop(state.t_now, TrajectoryState.COMPLETED, "")
                    return self._command(0.0, 0.0, 0.0)
                if self._elapsed - self._trajectory.duration > config.settle_timeout:
                    return self._fail(state.t_now, "base did not reach the end of the trajectory")
                # Settling is pure feedback; the last point's velocity would carry it past.
                reference_velocity = [0.0, 0.0, 0.0]

            cos_yaw, sin_yaw = math.cos(pose[2]), math.sin(pose[2])
            dx, dy, dyaw = reference_velocity
            feedback = self._feedback(reference, pose)
            vx = cos_yaw * dx + sin_yaw * dy + feedback[0]
            vy = -sin_yaw * dx + cos_yaw * dy + feedback[1]
            speed = math.hypot(vx, vy)
            if speed > config.max_linear:
                vx, vy = vx * config.max_linear / speed, vy * config.max_linear / speed
            return self._command(vx, vy, _clamp(dyaw + feedback[2], config.max_angular))

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        with self._lock:
            if joints & set(self._joints) and self._state == TrajectoryState.EXECUTING:
                logger.warning(f"PlanarBaseTrajectoryTask '{self._name}' preempted by {by_task}")
                self._stop(None, TrajectoryState.ABORTED, f"preempted by {by_task}")

    def _read_pose(self, state: CoordinatorState) -> tuple[float, float, float] | None:
        positions = state.joints.joint_positions
        x = positions.get(self._joints[0])
        y = positions.get(self._joints[1])
        yaw = positions.get(self._joints[2])
        if x is None or y is None or yaw is None:
            return None
        pose = (float(x), float(y), float(yaw))
        if not all(math.isfinite(value) for value in pose):
            return None
        self._last_pose_t = state.t_now
        return pose

    def _feedback(
        self, reference: list[float], pose: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        x, y, yaw = pose
        ex_world = reference[0] - x
        ey_world = reference[1] - y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        ex_body = cos_yaw * ex_world + sin_yaw * ey_world
        ey_body = -sin_yaw * ex_world + cos_yaw * ey_world
        kp = self._config.kp
        return (
            _clamp(kp[0] * ex_body, _FB_CLAMP_LINEAR),
            _clamp(kp[1] * ey_body, _FB_CLAMP_LINEAR),
            _clamp(kp[2] * angle_diff(reference[2], yaw), _FB_CLAMP_YAW),
        )

    def _fail(self, t_now: float, message: str) -> JointCommandOutput:
        logger.error(f"PlanarBaseTrajectoryTask '{self._name}' aborted: {message}")
        self._stop(t_now, TrajectoryState.ABORTED, message)
        return self._command(0.0, 0.0, 0.0)

    def _stop(self, t_now: float | None, final_state: TrajectoryState, error: str) -> None:
        self._state = final_state
        self._error = error
        if t_now is None:
            # Off the tick there is no t_now, so the next tick starts the hold.
            self._stop_pending = True
        else:
            self._stop_until = t_now + self._config.stop_hold_s

    def _command(self, vx: float, vy: float, wz: float) -> JointCommandOutput:
        return JointCommandOutput(
            joint_names=self._joints,
            velocities=[vx, vy, wz],
            mode=ControlMode.VELOCITY,
        )


def _trajectory_problem(
    trajectory: JointTrajectory | None, config: PlanarBaseTrajectoryTaskConfig
) -> str:
    if trajectory is None or not trajectory.points:
        return "Base trajectory has no points"
    previous_time: float | None = None
    for point in trajectory.points:
        if len(point.positions) != 3 or len(point.velocities) != 3:
            return "Base trajectory points must carry x, y and yaw"
        values = (*point.positions, *point.velocities, point.time_from_start)
        if not all(math.isfinite(value) for value in values):
            return "Base trajectory contains non-finite values"
        if previous_time is None and point.time_from_start != 0.0:
            return "Base trajectory must start at t=0"
        if previous_time is not None and point.time_from_start <= previous_time:
            return "Base trajectory has non-increasing timestamps"
        previous_time = point.time_from_start
        vx, vy, wz = point.velocities
        if (
            math.hypot(vx, vy) > config.max_linear + _SPEED_SLACK
            or abs(wz) > config.max_angular + _SPEED_SLACK
        ):
            return (
                f"Base trajectory moves at {math.hypot(vx, vy):.3f} m/s / {abs(wz):.3f} rad/s, "
                f"over the base's {config.max_linear} m/s / {config.max_angular} rad/s"
            )
    return ""


class PlanarBaseTrajectoryTaskParams(BaseConfig):
    model_config = ConfigDict(allow_inf_nan=False)

    kp: tuple[NonNegativeFloat, NonNegativeFloat, NonNegativeFloat] = (1.0, 1.0, 1.0)
    max_linear: PositiveFloat = 1.0
    max_angular: PositiveFloat = 2.0
    start_position_tolerance: PositiveFloat = 0.1
    start_orientation_tolerance: PositiveFloat = 0.1
    position_goal_tolerance: PositiveFloat = 0.05
    orientation_goal_tolerance: PositiveFloat = 0.1
    max_tracking_error: PositiveFloat = 0.5
    max_yaw_tracking_error: PositiveFloat = 0.5
    settle_timeout: NonNegativeFloat = 2.0
    stop_hold_s: NonNegativeFloat = 0.5
    stale_pose_timeout: PositiveFloat = 0.3


def create_task(cfg: Any, hardware: Any) -> PlanarBaseTrajectoryTask:
    params = PlanarBaseTrajectoryTaskParams.model_validate(cfg.params)
    return PlanarBaseTrajectoryTask(
        cfg.name,
        PlanarBaseTrajectoryTaskConfig(
            joint_names=list(cfg.joint_names), priority=cfg.priority, **params.model_dump()
        ),
    )
