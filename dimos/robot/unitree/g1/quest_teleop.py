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

"""Quest WebXR teleop for the G1 GR00T WBC stack."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
from typing import Any

import numpy as np
from numpy.linalg import norm, solve
import pinocchio as pin
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.teleop.quest.quest_teleop_module import Hand, QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.quest.quest_types import Buttons, QuestControllerState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

logger = setup_logger()

_HW_ID = "g1"
_URDF_PATH = Path(__file__).resolve().parent / "g1.urdf"

_ARM_JOINT_NAMES: tuple[str, ...] = (
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
_PIN_ARM_JOINT_NAMES: tuple[str, ...] = tuple(f"{name}_joint" for name in _ARM_JOINT_NAMES)
_FULL_ARM_JOINT_NAMES: tuple[str, ...] = tuple(f"{_HW_ID}/{name}" for name in _ARM_JOINT_NAMES)

_T_ROBOT_OPENXR = np.array(
    [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
    dtype=np.float64,
)
_T_OPENXR_ROBOT = np.array(
    [[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]],
    dtype=np.float64,
)
_CONST_HEAD_POSE = np.array(
    [[1, 0, 0, 0], [0, 1, 0, 1.5], [0, 0, 1, -0.2], [0, 0, 0, 1]],
    dtype=np.float64,
)
_CONST_LEFT_ARM_POSE = np.array(
    [[1, 0, 0, -0.15], [0, 1, 0, 1.13], [0, 0, 1, -0.3], [0, 0, 0, 1]],
    dtype=np.float64,
)
_CONST_RIGHT_ARM_POSE = np.array(
    [[1, 0, 0, 0.15], [0, 1, 0, 1.13], [0, 0, 1, -0.3], [0, 0, 0, 1]],
    dtype=np.float64,
)


def _safe_mat_update(prev_mat: np.ndarray, mat: np.ndarray) -> tuple[np.ndarray, bool]:
    det = np.linalg.det(mat)
    if not np.isfinite(det) or np.isclose(det, 0.0, atol=1e-6):
        return prev_mat, False
    return mat, True


@dataclass(frozen=True)
class G1DualArmIKConfig:
    max_iter: int = 12
    damp: float = 1e-2
    dt: float = 0.25
    position_cost: float = 8.0
    orientation_cost: float = 2.0
    posture_cost: float = 0.01
    max_velocity: float = 8.0


class G1DualArmIK:
    """Pinocchio DLS IK over the G1 arms as one 14-DOF problem."""

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
    _POSTURE_WEIGHTS = np.array(
        [4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1, 4.0, 3.0, 0.1, 3.0, 1.0, 1.0, 0.1],
        dtype=np.float64,
    )

    def __init__(
        self, urdf_path: Path = _URDF_PATH, config: G1DualArmIKConfig | None = None
    ) -> None:
        self._config = config or G1DualArmIKConfig()
        full_model = pin.buildModelFromUrdf(str(urdf_path))
        lock_joints = [
            full_model.getJointId(name)
            for name in self._LOCK_JOINTS
            if full_model.existJointName(name)
        ]
        full_q0 = np.zeros(full_model.nq)
        self._model = pin.buildReducedModel(full_model, lock_joints, full_q0)

        if self._model.nq != len(_ARM_JOINT_NAMES):
            raise RuntimeError(
                f"G1 arm IK expected {len(_ARM_JOINT_NAMES)} DOF, got {self._model.nq}"
            )

        self._model.addFrame(
            pin.Frame(
                "L_ee",
                self._model.getJointId("left_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self._model.addFrame(
            pin.Frame(
                "R_ee",
                self._model.getJointId("right_wrist_yaw_joint"),
                pin.SE3(np.eye(3), np.array([0.05, 0.0, 0.0])),
                pin.FrameType.OP_FRAME,
            )
        )
        self._data = self._model.createData()
        self._left_frame_id = self._model.getFrameId("L_ee")
        self._right_frame_id = self._model.getFrameId("R_ee")
        self._q_default = np.zeros(self._model.nq)

    def solve(
        self,
        left_wrist: np.ndarray,
        right_wrist: np.ndarray,
        q_init: np.ndarray | None,
    ) -> np.ndarray:
        cfg = self._config
        q = np.asarray(q_init if q_init is not None else self._q_default, dtype=np.float64).copy()
        q = np.clip(q, self._model.lowerPositionLimit, self._model.upperPositionLimit)
        left_target = pin.SE3(left_wrist[:3, :3].copy(), left_wrist[:3, 3].copy())
        right_target = pin.SE3(right_wrist[:3, :3].copy(), right_wrist[:3, 3].copy())

        for _ in range(cfg.max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)
            errors: list[np.ndarray] = []
            jacobians: list[np.ndarray] = []

            for frame_id, target in (
                (self._left_frame_id, left_target),
                (self._right_frame_id, right_target),
            ):
                current = self._data.oMf[frame_id]
                frame_error = current.actInv(target)
                err = pin.log(frame_error).vector
                weight = np.array(
                    [cfg.position_cost] * 3 + [cfg.orientation_cost] * 3,
                    dtype=np.float64,
                )
                jac = pin.computeFrameJacobian(
                    self._model,
                    self._data,
                    q,
                    frame_id,
                    pin.ReferenceFrame.LOCAL,
                )
                jac = -pin.Jlog6(frame_error.inverse()) @ jac
                errors.append(np.sqrt(weight) * err)
                jacobians.append(np.sqrt(weight)[:, None] * jac)

            posture_weight = np.sqrt(cfg.posture_cost) * self._POSTURE_WEIGHTS
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

            q = pin.integrate(self._model, q, velocity * cfg.dt)
            q = np.clip(q, self._model.lowerPositionLimit, self._model.upperPositionLimit)

        return np.asarray(q, dtype=np.float64)


class G1QuestTeleopConfig(QuestTeleopConfig):
    """Configuration for Quest-driven G1 teleop."""

    linear_scale: float = 0.3
    yaw_scale: float = 0.3
    strafe_scale: float = 0.3
    right_stick_mode: str = "yaw"
    # Larger than the obvious 0.05 because Quest stick rest values often drift
    # to ~0.10–0.15 — anything inside this radius is treated as neutral.
    deadzone: float = 0.18
    workspace_scale: float = 0.7
    waist_offset: tuple[float, float, float] = (0.15, 0.0, 0.45)
    shoulder_y_correction: float = 0.08
    # Engage = both index triggers pulled (>50%). Frees X/A so the operator
    # can use their thumbs for the joysticks without releasing arm tracking.
    # Override to ("primary",) for X/A or ("grip",) for the middle-finger grips.
    arm_engage_buttons: tuple[str, ...] = Field(default=("trigger",))


class G1QuestTeleopModule(QuestTeleopModule):
    """Quest WebXR controller teleop for G1 locomotion and bimanual arms."""

    config: G1QuestTeleopConfig

    joint_state: In[JointState]
    joint_command: Out[JointState]
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._raw_pose_lock = threading.Lock()
        self._head_xr = _CONST_HEAD_POSE.copy()
        self._left_xr = _CONST_LEFT_ARM_POSE.copy()
        self._right_xr = _CONST_RIGHT_ARM_POSE.copy()
        self._have_head = False
        self._have_left = False
        self._have_right = False
        # Track engage transitions so we log the state change exactly once and
        # the operator can confirm the X+A handshake worked.
        self._arms_engaged_state = False
        # Same idea for cmd_vel: log when the joystick starts/stops producing
        # a non-zero Twist so the operator can confirm the joy path is alive
        # without sifting through 50 Hz noise.
        self._cmd_vel_moving = False

        self._state_lock = threading.Lock()
        self._latest_arm_q: np.ndarray | None = None
        self._current_arm_q = np.zeros(len(_ARM_JOINT_NAMES), dtype=np.float64)
        self._ik: G1DualArmIK | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.joint_state.subscribe(self._on_joint_state)))
        logger.info("G1QuestTeleopModule ready on Quest port %d", self.config.server_port)

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_joy_bytes(self, data: bytes) -> None:
        """Dump raw axes the moment a Joy frame arrives, before any downstream
        logic. Lets us see exactly what the Quest browser is sending —
        whether the thumbstick is at zero, missing, or scaled wrong."""
        from dimos.msgs.sensor_msgs.Joy import Joy as _Joy

        try:
            msg = _Joy.lcm_decode(data)
        except Exception as exc:
            logger.warning("G1 Quest: failed to decode Joy: %s", exc)
            return super()._on_joy_bytes(data)
        axes = list(msg.axes or [])
        # Throttle: only log when values change by more than 0.01 from the
        # last logged frame, so a still controller doesn't spam.
        prev_key = f"_last_joy_axes_{msg.frame_id}"
        prev = getattr(self, prev_key, None)
        if prev is None or any(abs(axes[i] - prev[i]) > 0.01 for i in range(min(len(axes), len(prev)))):
            logger.info("G1 Quest joy[%s]: axes=%s", msg.frame_id, [round(a, 3) for a in axes])
            setattr(self, prev_key, list(axes))
        super()._on_joy_bytes(data)

    def _on_pose_bytes(self, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        matrix = pose_to_matrix(msg)
        with self._raw_pose_lock:
            if msg.frame_id == "head":
                self._head_xr, self._have_head = _safe_mat_update(self._head_xr, matrix)
            elif msg.frame_id == "left":
                self._left_xr, self._have_left = _safe_mat_update(self._left_xr, matrix)
            elif msg.frame_id == "right":
                self._right_xr, self._have_right = _safe_mat_update(self._right_xr, matrix)

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        by_name = {
            (name.split("/", 1)[1] if "/" in name else name): float(pos)
            for name, pos in zip(msg.name, msg.position, strict=False)
        }
        if not all(name in by_name for name in _ARM_JOINT_NAMES):
            return
        arm_q = np.asarray([by_name[name] for name in _ARM_JOINT_NAMES], dtype=np.float64)
        with self._state_lock:
            self._latest_arm_q = arm_q

    def _handle_engage(self) -> None:
        return

    def _should_publish(self, hand: Hand) -> bool:
        return False

    def _publish_button_state(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        buttons = Buttons.from_controllers(left, right)
        buttons.pack_analog_triggers(
            left=left.trigger if left is not None else 0.0,
            right=right.trigger if right is not None else 0.0,
        )
        self.buttons.publish(buttons)
        self._publish_cmd_vel(left, right)
        self._publish_arm_target(left, right)

    def _publish_cmd_vel(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        def dz(value: float) -> float:
            return 0.0 if abs(value) < self.config.deadzone else value

        if right is not None and right.thumbstick_press:
            # E-stop: one zero Twist; let downstream re-arming come from a
            # different source (Babylon WASD, etc.) without us spamming.
            if self._cmd_vel_moving:
                self.cmd_vel.publish(Twist.zero())
                self._cmd_vel_moving = False
            return

        left_x = dz(left.thumbstick.x if left is not None else 0.0)
        left_y = dz(left.thumbstick.y if left is not None else 0.0)
        right_x = dz(right.thumbstick.x if right is not None else 0.0)

        vx = -left_y * self.config.linear_scale
        vy = 0.0
        yaw_rate = 0.0
        if self.config.right_stick_mode == "strafe":
            vy = -right_x * self.config.strafe_scale
            yaw_rate = -left_x * self.config.yaw_scale
        else:
            yaw_rate = -right_x * self.config.yaw_scale

        moving = abs(vx) > 0.0 or abs(vy) > 0.0 or abs(yaw_rate) > 0.0
        # Only publish when there's something to publish (i.e. sticks
        # actually outside the deadzone), OR on the one tick we transition
        # from moving back to neutral so the robot gets a definitive stop.
        # Otherwise we spam /g1/cmd_vel at the control-loop rate with zeros
        # and clobber other cmd_vel sources (Babylon WASD, agent commands).
        if moving:
            self.cmd_vel.publish(
                Twist(
                    linear=Vector3(vx, vy, 0.0),
                    angular=Vector3(0.0, 0.0, yaw_rate),
                )
            )
        elif self._cmd_vel_moving:
            self.cmd_vel.publish(Twist.zero())

        if moving != self._cmd_vel_moving:
            self._cmd_vel_moving = moving
            if moving:
                logger.info(
                    "G1 Quest cmd_vel active: vx=%.2f vy=%.2f wz=%.2f", vx, vy, yaw_rate
                )
            else:
                logger.info("G1 Quest cmd_vel zeroed")

    def _arms_engaged(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> bool:
        if left is None or right is None:
            return False

        def engaged(controller: QuestControllerState) -> bool:
            for button in self.config.arm_engage_buttons:
                if button == "primary" and controller.primary:
                    return True
                if button == "grip" and controller.grip > 0.5:
                    return True
                if button == "trigger" and controller.trigger > 0.5:
                    return True
            return False

        return engaged(left) and engaged(right)

    def _publish_arm_target(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        engaged = self._arms_engaged(left, right)
        if engaged != self._arms_engaged_state:
            self._arms_engaged_state = engaged
            if engaged:
                logger.info("G1 Quest arms engaged (X+A held)")
            else:
                logger.info("G1 Quest arms disengaged")
        if not engaged:
            return
        if self._ik is None:
            try:
                self._ik = G1DualArmIK()
            except Exception:
                logger.exception("G1 Quest arm IK failed to initialize")
                return
        ik = self._ik

        with self._raw_pose_lock:
            if not (self._have_head and self._have_left and self._have_right):
                return
            head_xr = self._head_xr.copy()
            left_xr = self._left_xr.copy()
            right_xr = self._right_xr.copy()

        with self._state_lock:
            q_init = (
                self._latest_arm_q.copy()
                if self._latest_arm_q is not None
                else self._current_arm_q.copy()
            )

        left_wrist, right_wrist = self._controller_wrist_targets(head_xr, left_xr, right_xr)
        try:
            sol_q = ik.solve(left_wrist, right_wrist, q_init)
        except Exception:
            logger.exception("G1 Quest arm IK failed")
            return

        self._current_arm_q = sol_q.copy()
        self.joint_command.publish(
            JointState(
                name=list(_FULL_ARM_JOINT_NAMES),
                position=[float(value) for value in sol_q],
            )
        )

    def _controller_wrist_targets(
        self,
        head_xr: np.ndarray,
        left_xr: np.ndarray,
        right_xr: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        head = _T_ROBOT_OPENXR @ head_xr @ _T_OPENXR_ROBOT
        left_wrist = _T_ROBOT_OPENXR @ left_xr @ _T_OPENXR_ROBOT
        right_wrist = _T_ROBOT_OPENXR @ right_xr @ _T_OPENXR_ROBOT

        head_yaw = math.atan2(head[1, 0], head[0, 0])
        cos_y = math.cos(-head_yaw)
        sin_y = math.sin(-head_yaw)
        inv_yaw = np.array(
            [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        left_wrist = left_wrist.copy()
        right_wrist = right_wrist.copy()
        left_delta = inv_yaw @ (left_wrist[:3, 3] - head[:3, 3])
        right_delta = inv_yaw @ (right_wrist[:3, 3] - head[:3, 3])
        left_wrist[:3, :3] = inv_yaw @ left_wrist[:3, :3]
        right_wrist[:3, :3] = inv_yaw @ right_wrist[:3, :3]

        left_delta *= self.config.workspace_scale
        right_delta *= self.config.workspace_scale
        waist_x, waist_y, waist_z = self.config.waist_offset
        left_wrist[:3, 3] = left_delta + np.array([waist_x, waist_y, waist_z])
        right_wrist[:3, 3] = right_delta + np.array([waist_x, waist_y, waist_z])
        left_wrist[1, 3] -= self.config.shoulder_y_correction
        right_wrist[1, 3] += self.config.shoulder_y_correction
        return left_wrist, right_wrist


g1_quest_teleop = G1QuestTeleopModule.blueprint

__all__ = ["G1DualArmIK", "G1QuestTeleopModule", "g1_quest_teleop"]
