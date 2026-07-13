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

"""Coupled dual-arm IK task for the Unitree G1 humanoid.

Solves both 7-DOF arms as one 14-DOF damped-least-squares problem with a
weighted posture cost, so the elbows keep a natural configuration instead
of drifting into the torso (which independent per-arm IK does on a
redundant arm). Joint limits are clamped every solver iteration.

Wrist targets arrive as *absolute* poses in the pelvis frame via
``on_cartesian_command``; the coordinator routes them here with
``frame_id = "<task_name>/left"`` or ``"<task_name>/right"``. This is
head-relative retargeting (the operator's hand pose relative to the
headset *is* the wrist target), not the delta-clutch semantics of
``teleop_ik`` — there is no engage-time origin.

Engage comes from the coordinator's ``teleop_buttons`` broadcast: both
index triggers held past ``engage_threshold``. While disengaged (after a
first engage) the task keeps outputting its last solution, so the arms
hold where the operator left them instead of snapping back to the
lower-priority servo task's default pose.

Participates in joint-level arbitration; pair with a lower-priority
``servo`` task over the same joints as the pre-first-engage holder,
exactly like the GR00T WBC blueprint already does.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.linalg import norm, solve
import pinocchio

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import pose_to_se3
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs.Pose import Pose
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.teleop.quest.quest_types import Buttons

logger = setup_logger()

# Short arm-joint names in the G1 dex convention (left arm then right arm).
# Matches make_humanoid_joints("g1")[15:] after the hardware-id prefix is
# stripped, and the ``<name>_joint`` suffix added below matches the URDF.
ARM_JOINT_SHORT_NAMES: tuple[str, ...] = (
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

# Joints locked out of the full G1 model so the reduced model is exactly
# the two arms rooted at a fixed pelvis. Names that don't exist in the
# model are skipped, so robot-only and whole-scene URDFs both work.
_LOCK_JOINTS = (
    "floating_base_joint",
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
)


@dataclass(frozen=True)
class G1DualArmIKSolverConfig:
    """Tuning for the streaming 14-DOF DLS solve (per control tick)."""

    max_iter: int = 12
    damp: float = 1e-2
    dt: float = 0.25
    position_cost: float = 8.0
    orientation_cost: float = 2.0
    posture_cost: float = 0.01
    max_velocity: float = 8.0


class G1DualArmIK:
    """Pinocchio DLS IK over the G1 arms as one 14-DOF problem."""

    # Posture-cost weights per arm joint (left then right): pull shoulders
    # and elbows toward neutral, leave wrist roll/yaw nearly free.
    _POSTURE_WEIGHTS = np.array(
        [4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1, 4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1],
        dtype=np.float64,
    )

    def __init__(self, urdf_path: str | Path, config: G1DualArmIKSolverConfig | None = None):
        self._config = config or G1DualArmIKSolverConfig()
        full_model = pinocchio.buildModelFromUrdf(str(urdf_path))
        lock_joints = [
            full_model.getJointId(name) for name in _LOCK_JOINTS if full_model.existJointName(name)
        ]
        full_q0 = np.zeros(full_model.nq)
        self._model = pinocchio.buildReducedModel(full_model, lock_joints, full_q0)

        if self._model.nq != len(ARM_JOINT_SHORT_NAMES):
            raise RuntimeError(
                f"G1 arm IK expected {len(ARM_JOINT_SHORT_NAMES)} DOF, got {self._model.nq}"
            )

        # q-vector order follows the reduced model, not our canonical list.
        # Expose it so the task can map between hardware names and q slots.
        self.joint_order: tuple[str, ...] = tuple(
            name.removesuffix("_joint") for name in self._model.names[1:]
        )
        missing = set(ARM_JOINT_SHORT_NAMES) - set(self.joint_order)
        if missing:
            raise RuntimeError(f"G1 arm IK model is missing arm joints: {sorted(missing)}")

        self._model.addFrame(
            pinocchio.Frame(
                "L_ee",
                self._model.getJointId("left_wrist_yaw_joint"),
                pinocchio.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pinocchio.FrameType.OP_FRAME,
            )
        )
        self._model.addFrame(
            pinocchio.Frame(
                "R_ee",
                self._model.getJointId("right_wrist_yaw_joint"),
                pinocchio.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pinocchio.FrameType.OP_FRAME,
            )
        )
        self._data = self._model.createData()
        self._left_frame_id = self._model.getFrameId("L_ee")
        self._right_frame_id = self._model.getFrameId("R_ee")
        self._q_default = np.zeros(self._model.nq)
        self._posture_weights = self._reorder_posture_weights()

    def _reorder_posture_weights(self) -> NDArray[np.float64]:
        by_name = dict(zip(ARM_JOINT_SHORT_NAMES, self._POSTURE_WEIGHTS, strict=True))
        return np.array([by_name[name] for name in self.joint_order], dtype=np.float64)

    def forward_wrists(self, q: NDArray[np.floating[Any]]) -> tuple[pinocchio.SE3, pinocchio.SE3]:
        """FK of both wrist EE frames at configuration ``q`` (pelvis frame)."""
        pinocchio.forwardKinematics(self._model, self._data, np.asarray(q, dtype=np.float64))
        pinocchio.updateFramePlacements(self._model, self._data)
        return (
            self._data.oMf[self._left_frame_id].copy(),
            self._data.oMf[self._right_frame_id].copy(),
        )

    def solve(
        self,
        left_wrist: pinocchio.SE3,
        right_wrist: pinocchio.SE3,
        q_init: NDArray[np.floating[Any]] | None,
    ) -> NDArray[np.float64]:
        cfg = self._config
        q = np.asarray(q_init if q_init is not None else self._q_default, dtype=np.float64).copy()
        q = np.clip(q, self._model.lowerPositionLimit, self._model.upperPositionLimit)

        for _ in range(cfg.max_iter):
            pinocchio.forwardKinematics(self._model, self._data, q)
            pinocchio.updateFramePlacements(self._model, self._data)
            errors: list[NDArray[np.float64]] = []
            jacobians: list[NDArray[np.float64]] = []

            for frame_id, target in (
                (self._left_frame_id, left_wrist),
                (self._right_frame_id, right_wrist),
            ):
                current = self._data.oMf[frame_id]
                frame_error = current.actInv(target)
                err = pinocchio.log(frame_error).vector
                weight = np.array(
                    [cfg.position_cost] * 3 + [cfg.orientation_cost] * 3,
                    dtype=np.float64,
                )
                jac = pinocchio.computeFrameJacobian(
                    self._model,
                    self._data,
                    q,
                    frame_id,
                    pinocchio.ReferenceFrame.LOCAL,
                )
                jac = -pinocchio.Jlog6(frame_error.inverse()) @ jac
                errors.append(np.sqrt(weight) * err)
                jacobians.append(np.sqrt(weight)[:, None] * jac)

            posture_weight = np.sqrt(cfg.posture_cost) * self._posture_weights
            errors.append(posture_weight * (q - self._q_default))
            jacobians.append(np.diag(posture_weight))

            err_stack = np.concatenate(errors)
            jac_stack = np.vstack(jacobians)
            lhs = jac_stack @ jac_stack.T + cfg.damp * np.eye(jac_stack.shape[0])
            try:
                velocity = -jac_stack.T @ solve(lhs, err_stack)
            except np.linalg.LinAlgError:
                velocity = -jac_stack.T @ np.linalg.lstsq(lhs, err_stack, rcond=None)[0]

            velocity_norm = norm(velocity)
            if velocity_norm > cfg.max_velocity:
                velocity *= cfg.max_velocity / velocity_norm

            q = pinocchio.integrate(self._model, q, velocity * cfg.dt)
            q = np.clip(q, self._model.lowerPositionLimit, self._model.upperPositionLimit)

        return np.asarray(q, dtype=np.float64)


@dataclass
class G1DualArmIKTaskConfig:
    """Configuration for the dual-arm IK task.

    Attributes:
        joint_names: The 14 hardware arm-joint names ("g1/left_shoulder_pitch",
            ... left arm then right arm, dex order).
        model_path: G1 URDF with the full 29-DOF body; legs/waist/base are
            locked out internally.
        priority: Arbitration priority. Set above the arm-holder servo task
            and below the WBC task (which claims disjoint joints anyway).
        timeout: While engaged, freeze at the last solution if no fresh wrist
            target arrived within this many seconds (0 = never).
        max_joint_speed_deg_s: Per-joint rate limit on the commanded output.
            Each tick the command moves from the reference (measured pose)
            toward the IK solution by at most this rate, so a distant
            solution — first engage, re-engage after moving your hands —
            glides instead of snapping.
        engage_threshold: Analog trigger level on *both* controllers that
            engages tracking.
        solver: DLS solver tuning.
    """

    joint_names: list[str]
    model_path: str | Path
    priority: int = 20
    timeout: float = 0.5
    max_joint_speed_deg_s: float = 120.0
    engage_threshold: float = 0.5
    solver: G1DualArmIKSolverConfig = field(default_factory=G1DualArmIKSolverConfig)


class G1DualArmIKTask(BaseControlTask):
    """Coupled dual-arm IK from absolute wrist targets in the pelvis frame.

    Routing: the teleop module stamps wrist poses with
    ``frame_id = f"{task_name}/left"`` / ``.../right"``; the coordinator
    resolves the prefix to this task and ``on_cartesian_command`` reads the
    hand back off the suffix.
    """

    def __init__(self, name: str, config: G1DualArmIKTaskConfig) -> None:
        if len(config.joint_names) != len(ARM_JOINT_SHORT_NAMES):
            raise ValueError(
                f"G1DualArmIKTask '{name}' needs exactly "
                f"{len(ARM_JOINT_SHORT_NAMES)} arm joints, got {len(config.joint_names)}"
            )
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)

        self._ik = G1DualArmIK(config.model_path, config.solver)
        # Hardware joint name for each reduced-model q slot: strip the
        # "g1/" prefix from configured names to match on short names.
        short_to_hw = {hw.split("/", 1)[-1]: hw for hw in config.joint_names}
        missing = [s for s in self._ik.joint_order if s not in short_to_hw]
        if missing:
            raise ValueError(f"G1DualArmIKTask '{name}' joint_names missing: {missing}")
        self._hw_names_in_q_order = [short_to_hw[s] for s in self._ik.joint_order]

        self._lock = threading.Lock()
        self._targets: dict[str, pinocchio.SE3] = {}
        self._last_target_time = 0.0
        self._engaged = False
        self._last_solution: NDArray[np.float64] | None = None
        self._active = False

        logger.info(
            f"G1DualArmIKTask {name} initialized (model: {config.model_path}, "
            f"engage: both triggers > {config.engage_threshold})"
        )

    @property
    def name(self) -> str:
        return self._name

    def claim(self) -> ResourceClaim:
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        with self._lock:
            if not self._active:
                return False
            return self._last_solution is not None or (self._engaged and len(self._targets) == 2)

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Store the latest absolute wrist target for one hand.

        The hand is the frame_id suffix after the task name
        ("<task>/left" or "<task>/right").
        """
        frame_id = getattr(pose, "frame_id", "")
        hand = frame_id.rsplit("/", 1)[-1]
        if hand not in ("left", "right"):
            logger.warning(f"G1DualArmIKTask {self._name}: unroutable frame_id {frame_id!r}")
            return False
        with self._lock:
            self._targets[hand] = pose_to_se3(pose)
            self._last_target_time = t_now
        return True

    def on_buttons(self, msg: Buttons) -> bool:
        """Both index triggers held → engage arm tracking."""
        threshold = self._config.engage_threshold
        engaged = (
            max(msg.left_trigger_analog, float(msg.left_trigger)) > threshold
            and max(msg.right_trigger_analog, float(msg.right_trigger)) > threshold
        )
        with self._lock:
            if engaged != self._engaged:
                self._engaged = engaged
                logger.info(
                    f"G1DualArmIKTask {self._name}: "
                    f"{'engaged' if engaged else 'disengaged — holding last pose'}"
                )
        return True

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        with self._lock:
            if not self._active:
                return None
            engaged = self._engaged
            targets = dict(self._targets)
            last_solution = self._last_solution
            stale = (
                self._config.timeout > 0
                and state.t_now - self._last_target_time > self._config.timeout
            )

        tracking = engaged and len(targets) == 2 and not stale
        if not tracking:
            # Hold where the operator left the arms. Before the first
            # engage there is nothing to hold and the lower-priority
            # servo task keeps the arms at its default pose.
            return self._hold_output(last_solution)

        q_current = self._measured_q(state)
        # Warm-start from the last command, not the measured pose: the
        # measured pose lags the command by the servo dynamics, and
        # re-solving from it makes the output oscillate.
        q_init = last_solution if last_solution is not None else q_current
        try:
            q_solution = self._ik.solve(targets["left"], targets["right"], q_init)
        except Exception:
            logger.exception(f"G1DualArmIKTask {self._name}: IK solve failed")
            return self._hold_output(last_solution)

        # Rate-limit the command toward the solution so a distant target
        # (first engage, re-engage after moving) glides instead of snapping.
        reference = last_solution if last_solution is not None else q_current
        if reference is not None:
            step = math.radians(self._config.max_joint_speed_deg_s) * max(state.dt, 1e-3)
            q_solution = reference + np.clip(q_solution - reference, -step, step)

        with self._lock:
            self._last_solution = q_solution
        return JointCommandOutput(
            joint_names=list(self._hw_names_in_q_order),
            positions=q_solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def _hold_output(self, solution: NDArray[np.float64] | None) -> JointCommandOutput | None:
        if solution is None:
            return None
        return JointCommandOutput(
            joint_names=list(self._hw_names_in_q_order),
            positions=solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def _measured_q(self, state: CoordinatorState) -> NDArray[np.float64] | None:
        positions = []
        for joint_name in self._hw_names_in_q_order:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                return None
            positions.append(pos)
        return np.array(positions, dtype=np.float64)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        if joints & self._joint_names:
            logger.warning(
                f"G1DualArmIKTask {self._name} preempted by {by_task} on joints {joints}"
            )

    def start(self) -> None:
        with self._lock:
            self._active = True
        logger.info(f"G1DualArmIKTask {self._name} started")

    def stop(self) -> None:
        with self._lock:
            self._active = False
        logger.info(f"G1DualArmIKTask {self._name} stopped")


class G1DualArmIKTaskParams(BaseConfig):
    model_path: str | Path
    timeout: float = 0.5
    max_joint_speed_deg_s: float = 120.0
    engage_threshold: float = 0.5
    max_iter: int = 12
    damp: float = 1e-2
    dt: float = 0.25
    position_cost: float = 8.0
    orientation_cost: float = 2.0
    posture_cost: float = 0.01
    max_velocity: float = 8.0


def create_task(cfg: Any, hardware: Any) -> G1DualArmIKTask:
    params = G1DualArmIKTaskParams.model_validate(cfg.params)
    return G1DualArmIKTask(
        cfg.name,
        G1DualArmIKTaskConfig(
            joint_names=cfg.joint_names,
            model_path=params.model_path,
            priority=cfg.priority,
            timeout=params.timeout,
            max_joint_speed_deg_s=params.max_joint_speed_deg_s,
            engage_threshold=params.engage_threshold,
            solver=G1DualArmIKSolverConfig(
                max_iter=params.max_iter,
                damp=params.damp,
                dt=params.dt,
                position_cost=params.position_cost,
                orientation_cost=params.orientation_cost,
                posture_cost=params.posture_cost,
                max_velocity=params.max_velocity,
            ),
        ),
    )
