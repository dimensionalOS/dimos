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

"""Quest controller poses as the policy's 30 Hz reference motion.

The trained controller never consumed an operator pose: it consumed a recorded
reference row - joint positions, joint velocities and the TCP pose *at the same
timestamp*. This module produces exactly that tuple from a Streamed WebXR
controller pose, with no privileged simulator input:

* the operator pose is anchored to the measured TCP pose at engagement, so the
  first delta is zero and the arm never jumps at the start of a session;
* `OpenYamPinkPoseTargetSolver` (the same streaming IK the Quest teleoperation
  blueprint already runs) turns the anchored Cartesian target into ``q_ref``;
* ``dq_ref`` is a finite difference of consecutive ``q_ref`` over the *actual*
  elapsed time, and a wrong or stale ``dt`` is refused rather than divided by;
* the goal TCP pose is the forward kinematics of ``q_ref``, so the actor always
  sees a reference it can achieve.

That last point is also this module's honesty boundary: IK-projected references
are a *deployment adaptation*, not proven equivalent to the recorded retargeted
training motion. The requested operator target and the achievable FK target are
reported separately so a projection never hides tracking error.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import threading
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.openyam.policy.contract import (
    PolicyContractError,
    PolicyManifest,
)
from dimos.robot.manipulators.openyam.policy.reference import ReferenceRow
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from dimos.control.tasks.pose_target_ik import PinkPoseTargetSolver

#: Name of the IK frame the operator drives in the deployed URDF.
TOOL_FRAME = "gripper_tip"

GoalFk = Callable[[NDArray[np.float64]], tuple[NDArray[np.float64], NDArray[np.float64]]]


class ReferenceRefusedError(PolicyContractError):
    """A reference tick that cannot be produced without inventing values."""

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


@dataclass(frozen=True)
class ReferenceDiagnostics:
    """One tick's requested, achievable and projected reference geometry."""

    requested_position: NDArray[np.float64]
    requested_quaternion_wxyz: NDArray[np.float64]
    achievable_position: NDArray[np.float64]
    achievable_quaternion_wxyz: NDArray[np.float64]
    projection_error_m: float
    projection_error_rad: float
    dt_s: float
    #: How far the IK solution was clipped back into the trained joint envelope.
    envelope_clip_rad: float = 0.0

    def as_dict(self) -> dict[str, float | list[float]]:
        return {
            "requested_position": self.requested_position.tolist(),
            "requested_quaternion_wxyz": self.requested_quaternion_wxyz.tolist(),
            "achievable_position": self.achievable_position.tolist(),
            "achievable_quaternion_wxyz": self.achievable_quaternion_wxyz.tolist(),
            "projection_error_m": self.projection_error_m,
            "projection_error_rad": self.projection_error_rad,
            "dt_s": self.dt_s,
            "envelope_clip_rad": self.envelope_clip_rad,
        }


def pose_matrix(
    position: tuple[float, float, float] | NDArray, quaternion_xyzw: NDArray
) -> NDArray:
    """4x4 transform from a position and an ``xyzw`` quaternion (zero norm -> identity)."""
    values = np.asarray(quaternion_xyzw, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(values))
    if norm < 1e-9:
        raise ReferenceRefusedError("quaternion", "controller orientation has zero norm")
    rotation = Rotation.from_quat(values / norm).as_matrix()
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = np.asarray(position, dtype=np.float64).reshape(3)
    return matrix


def controller_matrix(pose: Pose | PoseStamped) -> NDArray:
    """Transform of a WebXR controller pose message."""
    orientation = pose.orientation
    return pose_matrix(
        (float(pose.position.x), float(pose.position.y), float(pose.position.z)),
        np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
    )


def _quat_wxyz(matrix: NDArray) -> NDArray[np.float64]:
    x, y, z, w = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return np.array([w, x, y, z], dtype=np.float64)


def _continuous(quaternion: NDArray, previous: NDArray | None) -> NDArray:
    """Keep the quaternion on the same double cover as the previous tick."""
    if previous is not None and float(np.dot(quaternion, previous)) < 0.0:
        return -quaternion
    return quaternion


class QuestReferenceGenerator:
    """Anchored Quest pose -> IK joints -> finite-difference velocity -> FK goal."""

    def __init__(
        self,
        manifest: PolicyManifest,
        *,
        solver: PinkPoseTargetSolver,
        goal_fk: GoalFk,
        joint_names: tuple[str, ...],
        solver_joint_names: tuple[str, ...] | None = None,
        tool_frame: str = TOOL_FRAME,
        dt_band: tuple[float, float] = (0.6, 1.5),
        envelope_margin_rad: float = 0.015,
        max_reference_velocity_rad_s: float | None = None,
    ) -> None:
        self.manifest = manifest
        self._solver = solver
        self._goal_fk = goal_fk
        self._joint_names = tuple(joint_names)
        # The IK model names the same joints with its robot prefix
        # (``arm/yam_joint1``); the policy row keeps the trained bare order.
        self._solver_names = tuple(solver_joint_names or joint_names)
        self._tool_frame = tool_frame
        self._period_s = manifest.tick_period_s
        #: Accepted ratio of the actual tick interval to the nominal period. The
        #: coordinator delivers ~30 Hz with a few milliseconds of jitter, so the
        #: reference is differenced over the *actual* interval; anything outside
        #: this band is a stalled or doubled tick, not a policy period.
        self._dt_min_ratio = float(min(dt_band))
        self._dt_max_ratio = float(max(dt_band))
        #: The trained joint envelope, pulled in by more than the measured
        #: soft-limit penetration, so a reference can never wedge a hinge.
        margin = float(envelope_margin_rad)
        self._envelope_lower = np.asarray(manifest.robot.sim_lower, dtype=np.float64) + margin
        self._envelope_upper = np.asarray(manifest.robot.sim_upper, dtype=np.float64) - margin
        #: A reference step may not exceed the solver's own velocity bound for
        #: the interval it covers; anything larger is a jump, not motion.
        self._max_velocity_rad_s = (
            float(max_reference_velocity_rad_s)
            if max_reference_velocity_rad_s is not None
            else None
        )
        self._lock = threading.RLock()
        self._q_prev: NDArray[np.float64] | None = None
        self._tcp_anchor: NDArray[np.float64] | None = None
        self._controller_anchor: NDArray[np.float64] | None = None
        self._t_prev: float | None = None
        self._quat_prev: NDArray[np.float64] | None = None
        self._diagnostics: ReferenceDiagnostics | None = None
        self._anchor_q: NDArray[np.float64] | None = None

    # ------------------------------------------------------------------ state
    @property
    def anchored(self) -> bool:
        return self._q_prev is not None

    @property
    def diagnostics(self) -> ReferenceDiagnostics | None:
        return self._diagnostics

    @property
    def last_reference_q(self) -> NDArray[np.float64] | None:
        return None if self._q_prev is None else self._q_prev.copy()

    def release(self) -> None:
        """Drop the anchor and the previous sample; a new session re-anchors."""
        with self._lock:
            self._q_prev = None
            self._tcp_anchor = None
            self._controller_anchor = None
            self._t_prev = None
            self._quat_prev = None
            self._diagnostics = None
            self._anchor_q = None
            self._solver.reset()

    # ----------------------------------------------------------------- anchor
    def anchor(
        self,
        q_measured: NDArray[np.float32] | NDArray[np.float64],
        pose: Pose | PoseStamped,
        t_now: float,
    ) -> ReferenceRow:
        """Start a session at the measured pose and the operator's current pose.

        The initial reference velocity is explicitly zero and ``q_ref`` is the
        measured pose: the trained reset starts the plant on its reference row,
        so engagement adds no motion the operator did not ask for.
        """
        with self._lock:
            q = np.asarray(q_measured, dtype=np.float64).reshape(-1)
            if q.shape != (len(self._joint_names),):
                raise ReferenceRefusedError("joints", f"expected {len(self._joint_names)} joints")
            if not np.isfinite(q).all():
                raise ReferenceRefusedError("joints", "measured pose is not finite")
            self._solver.reset()
            self._q_prev = q.copy()
            self._anchor_q = q.copy()
            self._tcp_anchor = self._fk_frame()
            self._controller_anchor = controller_matrix(pose)
            self._t_prev = float(t_now)
            position, quaternion = self._goal_fk(q)
            self._quat_prev = quaternion
            self._diagnostics = ReferenceDiagnostics(
                requested_position=self._tcp_anchor[:3, 3].copy(),
                requested_quaternion_wxyz=_quat_wxyz(self._tcp_anchor),
                achievable_position=np.asarray(position, dtype=np.float64).copy(),
                achievable_quaternion_wxyz=quaternion.copy(),
                projection_error_m=0.0,
                projection_error_rad=0.0,
                dt_s=0.0,
            )
            return self._row(0, q, np.zeros_like(q), position, quaternion)

    def _fk_frame(self) -> NDArray[np.float64]:
        """Operator tool frame at the anchored pose, from the IK model's FK."""
        assert self._q_prev is not None
        frames = self._solver.frame_poses(
            JointState(name=list(self._solver_names), position=self._q_prev.tolist()),
            (self._tool_frame,),
        )
        return pose_to_matrix(frames[self._tool_frame])

    # ----------------------------------------------------------------- update
    def update(
        self,
        pose: Pose | PoseStamped,
        t_now: float,
        *,
        tick_index: int,
    ) -> ReferenceRow:
        """Produce the reference row for ``tick_index`` or refuse with a reason."""
        with self._lock:
            if self._q_prev is None or self._t_prev is None:
                raise ReferenceRefusedError("anchor", "generator is not anchored")
            now = float(t_now)
            dt = now - self._t_prev
            if not math.isfinite(dt) or dt <= 0.0:
                raise ReferenceRefusedError("dt", f"non-positive interval {dt!r}")
            if not (
                self._period_s * self._dt_min_ratio <= dt <= self._period_s * self._dt_max_ratio
            ):
                raise ReferenceRefusedError(
                    "dt",
                    f"interval {dt:.6f}s is outside the "
                    f"[{self._period_s * self._dt_min_ratio:.6f}, "
                    f"{self._period_s * self._dt_max_ratio:.6f}]s band around the "
                    f"{self._period_s:.6f}s policy period",
                )

            delta = controller_matrix(pose) @ np.linalg.inv(self._controller_anchor)
            requested = delta @ self._tcp_anchor
            requested_pose = matrix_to_pose(requested)
            target = PoseStamped(
                ts=now,
                frame_id="",
                position=requested_pose.position,
                orientation=requested_pose.orientation,
            )
            command = self._solver.step(
                {self._tool_frame: target},
                JointState(name=list(self._solver_names), position=self._q_prev.tolist()),
                dt,
            )
            if command is None:
                raise ReferenceRefusedError("ik", "the streaming IK returned no command")
            q_raw = np.asarray(command.position, dtype=np.float64).reshape(-1)
            if q_raw.shape != self._q_prev.shape or not np.isfinite(q_raw).all():
                raise ReferenceRefusedError("ik", "the streaming IK returned an invalid pose")
            # The IK runs on the deployed URDF, whose limits are wider than the
            # plant the policy was trained on (joint 6: +-2.0944 rad vs
            # [-2.35619, 1.8326]). Clip every reference into the trained
            # envelope before anything derives from it, so the actor, the goal
            # FK and the recorded velocity all describe a reachable pose.
            q_ref = np.clip(q_raw, self._envelope_lower, self._envelope_upper)
            envelope_clip = float(np.abs(q_ref - q_raw).max())
            step = float(np.abs(q_ref - self._q_prev).max())
            if self._max_velocity_rad_s is not None:
                step_limit = self._max_velocity_rad_s * dt + 1e-3
                if step > step_limit:
                    raise ReferenceRefusedError(
                        "jump", f"reference step {step:.4f} rad exceeds {step_limit:.4f} rad"
                    )
            dq_ref = (q_ref - self._q_prev) / dt
            position, quaternion = self._goal_fk(q_ref)
            quaternion = _continuous(np.asarray(quaternion, dtype=np.float64), self._quat_prev)

            achievable = self._solver.frame_poses(
                JointState(name=list(self._solver_names), position=q_ref.tolist()),
                (self._tool_frame,),
            )[self._tool_frame]
            achievable_matrix = pose_to_matrix(achievable)
            projection_m = float(np.linalg.norm(achievable_matrix[:3, 3] - requested[:3, 3]))
            projection_rad = float(
                Rotation.from_matrix(achievable_matrix[:3, :3] @ requested[:3, :3].T).magnitude()
            )

            self._diagnostics = ReferenceDiagnostics(
                requested_position=requested[:3, 3].copy(),
                requested_quaternion_wxyz=_quat_wxyz(requested),
                achievable_position=achievable_matrix[:3, 3].copy(),
                achievable_quaternion_wxyz=_quat_wxyz(achievable_matrix),
                projection_error_m=projection_m,
                projection_error_rad=projection_rad,
                dt_s=dt,
                envelope_clip_rad=envelope_clip,
            )
            self._q_prev = q_ref
            self._t_prev = now
            self._quat_prev = quaternion
            return self._row(int(tick_index), q_ref, dq_ref, position, quaternion)

    # ------------------------------------------------------------------- rows
    def _row(
        self,
        index: int,
        q: NDArray[np.float64],
        dq: NDArray[np.float64],
        position: NDArray[np.float64],
        quaternion_wxyz: NDArray[np.float64],
    ) -> ReferenceRow:
        return ReferenceRow(
            index=int(index),
            time_s=float(index) * self._period_s,
            q=np.asarray(q, dtype=np.float32).copy(),
            dq=np.asarray(dq, dtype=np.float32).copy(),
            goal_position=np.asarray(position, dtype=np.float32).copy(),
            goal_quaternion_wxyz=np.asarray(quaternion_wxyz, dtype=np.float32).copy(),
        )
