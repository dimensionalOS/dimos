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

"""Quest and planner reference producers. Neither can issue arm commands."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import math
import time

from dimos.control.group_command import GroupCommandOutput
from dimos.control.task import BaseControlTask, CoordinatorState, ResourceClaim
from dimos.control.tasks.trajectory_task.trajectory_task import (
    JointTrajectoryTask,
    JointTrajectoryTaskConfig,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.robot.manipulators.openyam.joints import OPENYAM_ARM_JOINTS
from dimos.robot.manipulators.openyam.trimode_connection import TrimodeLifecycle
from dimos.robot.manipulators.openyam.trimode_tracker import (
    REFERENCE_TIMEOUT,
    ArmSample,
    Bounds,
    JointGoal,
    PoseGoal,
    PositionGoal,
    Vector,
    bounded,
    measured_bounded,
    tcp_pose,
)
from dimos.teleop.webxr.controller_types import Buttons

JOINTS = tuple(f"arm/{j}" for j in OPENYAM_ARM_JOINTS)
POSE, XYZ, JOINT = "arm/tcp_pose", "arm/tcp_position", "arm/joint_reference"
QUEST_TIMEOUT = 0.25
QUEST_FRAME = "trimode_right_monotonic_deadline"
# Private to TrimodeQuest's Buttons stream; reserved bit, not a digital button.
QUEST_FRESH = 1 << 15
SETTLE_TIMEOUT = 2.0
SETTLE_POSITION = 0.10  # rad
SETTLE_VELOCITY = 0.08  # rad/s


ReadFeedback = Callable[[], ArmSample | None]


def _begin(lifecycle: TrimodeLifecycle, interface: str) -> str | None:
    return lifecycle.open_reference(interface.split("/")[1], time.monotonic() + REFERENCE_TIMEOUT)


class QuestReference(BaseControlTask):
    def __init__(
        self,
        interface: str,
        read: ReadFeedback,
        takeover: Callable[[], None],
        lifecycle: TrimodeLifecycle,
    ) -> None:
        if interface not in (POSE, XYZ):
            raise ValueError("Quest interface must be TCP pose or position")
        self._name = "teleop_openyam"
        self._interface, self._read, self._takeover = interface, read, takeover
        self._lifecycle = lifecycle
        self._session: str | None = None
        self._held = self._estopped = False
        self._need_release = True
        self._buttons_at = self._buttons_deadline = float("-inf")
        self._pose: PoseStamped | None = None
        self._anchor: tuple[PoseStamped, PoseStamped] | None = None

    def claim(self) -> ResourceClaim:
        return ResourceClaim(frozenset(), priority=20)

    def is_active(self) -> bool:
        return self._session is not None

    @property
    def blocks_plan(self) -> bool:
        return self._held and time.monotonic() < self._buttons_deadline

    def _end(self, *, idle_hold: bool = False) -> None:
        session, self._session = self._session, None
        self._pose = self._anchor = None
        if session is not None:
            self._lifecycle.close_reference(session, idle_hold=idle_hold)

    def set_estop(self, estopped: bool) -> None:
        self._estopped = estopped
        if estopped:
            self._end()
            self._need_release = True

    def stop(self) -> None:
        self.set_estop(True)

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        self._end()
        self._need_release = True
        return True

    def on_left_cartesian_command(self, pose: PoseStamped, t_now: float) -> bool:
        return False

    def on_right_cartesian_command(self, pose: PoseStamped, t_now: float) -> bool:
        if self._session is None:
            return False
        now = time.monotonic()
        if (
            pose.frame_id != QUEST_FRAME
            or not now < pose.ts <= now + QUEST_TIMEOUT
            or (self._pose is not None and now >= self._pose.ts)
        ):
            self._end()
            self._need_release = True
            return False
        values = _pose_vector(pose)
        if not all(math.isfinite(v) for v in values) or not math.isclose(
            math.hypot(*values[3:]), 1.0, rel_tol=0.0, abs_tol=1e-5
        ):
            self._end()
            self._need_release = True
            return False
        self._pose = pose
        return True

    def on_teleop_buttons(self, msg: Buttons, t_now: float) -> bool:
        expired = t_now >= self._buttons_at + QUEST_TIMEOUT
        self._buttons_at = t_now
        self._buttons_deadline = time.monotonic() + QUEST_TIMEOUT
        self._held = msg.right_grip
        if self._estopped or not (msg.data & QUEST_FRESH):
            self._end()
            self._need_release = True
        elif not self._held:
            self._end(idle_hold=not expired)
            self._need_release = False
        elif expired:
            self._end()
            self._need_release = True
        elif self._session is None and not self._need_release:
            self._takeover()  # Close the plan session before granting grip authority.
            self._session = _begin(self._lifecycle, self._interface)
            self._opened_at = time.monotonic()
            self._need_release = True
        return True

    def compute(self, state: CoordinatorState) -> GroupCommandOutput | None:
        session = self._session
        if session is None:
            return None
        deadline = self._buttons_deadline
        if self._pose is not None:
            deadline = min(deadline, self._pose.ts)
        now = time.monotonic()
        if now >= deadline:
            self._end()
            self._need_release = True
            return None
        if self._pose is None:
            return None
        feedback = self._read()
        if feedback is None or (feedback.at >= self._opened_at and feedback.session != session):
            self._end()
            self._need_release = True
            return None
        if self._anchor is None:
            xyz = tcp_pose(feedback.q)
            world = PoseStamped(frame_id="world", position=xyz[:3], orientation=(*xyz[4:], xyz[3]))
            self._anchor = self._pose, world
        controller, robot = self._anchor
        delta = self._pose - controller
        orientation = (
            delta.orientation * robot.orientation if self._interface == POSE else robot.orientation
        )
        target = PoseStamped(position=robot.position + delta.position, orientation=orientation)
        cls = PoseGoal if self._interface == POSE else PositionGoal
        return GroupCommandOutput(
            self._interface,
            cls(session, now, min(deadline, now + REFERENCE_TIMEOUT), _pose_vector(target)),
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints:
            self._need_release = True
            self._end()


def _pose_vector(pose: PoseStamped) -> Vector:
    p, r = pose.position, pose.orientation
    return p.x, p.y, p.z, r.w, r.x, r.y, r.z


class PlannedReference(JointTrajectoryTask):
    """Canonical trajectory RPC/status, without a position-servo fallback."""

    def __init__(
        self,
        bounds: Bounds,
        read: ReadFeedback,
        blocked: Callable[[], bool],
        lifecycle: TrimodeLifecycle,
    ) -> None:
        super().__init__(JointTrajectoryTaskConfig(joint_names=JOINTS, priority=10))
        self._bounds, self._read, self._blocked = bounds, read, blocked
        self._lifecycle = lifecycle
        self._session: str | None = None
        self._plan: JointTrajectory | None = None
        self._began: float | None = None
        self._error = ""

    def on_joint_command(self, msg: JointState, t_now: float) -> bool:
        return False

    def is_active(self) -> bool:
        return self._state == TrajectoryState.EXECUTING

    def _valid_measurement(self, reading: ArmSample | None) -> bool:
        return (
            reading is not None
            and 0 <= time.monotonic() - reading.at < REFERENCE_TIMEOUT
            and measured_bounded(reading.q, self._bounds)
            and len(reading.dq) == len(JOINTS)
            and all(math.isfinite(v) for v in reading.dq)
        )

    def execute(
        self, trajectory: JointTrajectory, current_positions: Mapping[str, float]
    ) -> TrajectoryExecutionResult:
        status = TrajectoryExecutionStatus
        if self._estopped:
            return TrajectoryExecutionResult(status.ESTOPPED)
        if self._blocked() or self.is_active() or self._state == TrajectoryState.FAULT:
            return TrajectoryExecutionResult(
                status.INVALID_TRAJECTORY, "arm unavailable for new plan"
            )
        if (
            trajectory is None
            or not self._validate_trajectory(trajectory)
            or set(trajectory.joint_names) != set(JOINTS)
        ):
            return TrajectoryExecutionResult(
                status.INVALID_TRAJECTORY, "expected a complete six-joint plan"
            )
        columns = [trajectory.joint_names.index(j) for j in JOINTS]
        points = [
            TrajectoryPoint(
                time_from_start=p.time_from_start,
                positions=[p.positions[i] for i in columns],
                velocities=[p.velocities[i] for i in columns],
            )
            for p in trajectory.points
        ]
        if any(not bounded(tuple(p.positions), self._bounds) for p in points):
            return TrajectoryExecutionResult(
                status.INVALID_TRAJECTORY, "reference outside joint limits"
            )
        reading = self._read()
        if reading is None or not self._valid_measurement(reading):
            return TrajectoryExecutionResult(status.START_STATE_UNAVAILABLE)
        if any(
            abs(q - ref) > self._config.start_position_tolerance
            for q, ref in zip(reading.q, points[0].positions, strict=True)
        ):
            return TrajectoryExecutionResult(status.START_STATE_MISMATCH)
        session = _begin(self._lifecycle, JOINT)
        if session is None:
            return TrajectoryExecutionResult(status.START_STATE_UNAVAILABLE)
        self._session, self._opened_at = session, time.monotonic()
        self._plan = JointTrajectory(points=points, joint_names=list(JOINTS))
        self._began, self._error = None, ""
        self._state = TrajectoryState.EXECUTING
        return TrajectoryExecutionResult(status.ACCEPTED)

    def _clear_active_trajectory(self) -> None:
        super()._clear_active_trajectory()
        session, self._session = self._session, None
        self._plan = None
        if session is not None:
            self._lifecycle.close_reference(
                session, idle_hold=self._state is TrajectoryState.COMPLETED
            )

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        self.cancel()
        return self.reset()

    def stop(self) -> None:
        self.cancel()

    def _finish(self, state: TrajectoryState, error: str = "") -> None:
        self._state, self._error = state, error
        self._clear_active_trajectory()

    def compute(self, state: CoordinatorState) -> GroupCommandOutput | None:
        plan, session = self._plan, self._session
        if plan is None or session is None:
            return None
        if self._began is None:
            self._began = state.t_now
        elapsed = state.t_now - self._began
        remaining = plan.duration + SETTLE_TIMEOUT - elapsed
        measured = self._read()
        if measured is None or (measured.at >= self._opened_at and measured.session != session):
            fault = measured.fault if measured is not None else None
            self._finish(TrajectoryState.FAULT, fault or "arm feedback interrupted")
            return None
        if not self._valid_measurement(measured):
            self._finish(TrajectoryState.FAULT, "invalid or expired arm feedback")
            return None
        if remaining <= 0:
            self._finish(TrajectoryState.FAULT, "measured endpoint did not settle")
            return None
        ref = tuple(plan.sample(elapsed)[0])
        terminal = elapsed >= plan.duration
        assert measured is not None
        if (
            terminal
            and measured.session == session
            and measured.terminal_at is not None
            and measured.at > measured.terminal_at
            and max(abs(a - b) for a, b in zip(ref, measured.q, strict=True)) <= SETTLE_POSITION
            and max(map(abs, measured.dq)) <= SETTLE_VELOCITY
        ):
            self._finish(TrajectoryState.COMPLETED)
            return None
        now = time.monotonic()
        return GroupCommandOutput(
            JOINT, JointGoal(session, now, now + REFERENCE_TIMEOUT, ref, terminal)
        )

    def get_progress(self, t_now: float) -> float:
        return self.get_status(t_now).progress

    def get_status(self, t_now: float) -> TrajectoryStatus:
        if self._plan is not None:
            elapsed = max(0.0, t_now - self._began) if self._began is not None else 0.0
            duration = self._plan.duration
            return TrajectoryStatus(
                self._state,
                min(1.0, elapsed / duration) if duration else 1.0,
                elapsed,
                max(0.0, duration - elapsed),
            )
        return TrajectoryStatus(
            self._state,
            float(self._state == TrajectoryState.COMPLETED),
            error=self._error if self._state == TrajectoryState.FAULT else "",
        )
