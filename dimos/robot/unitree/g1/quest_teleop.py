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

"""Quest WebXR teleop retargeting for the G1 humanoid.

Thin :class:`QuestTeleopModule` subclass: it maps operator body pose to
robot wrist targets and joystick input to base velocity, and nothing else.
IK lives in the ``g1_dual_arm_ik`` control task; engage/disengage gating
lives there too (both index triggers, via the coordinator's
``teleop_buttons`` broadcast).

Retargeting is *absolute and head-relative*: each controller pose relative
to the headset, yaw-normalized and workspace-scaled, becomes a wrist pose
relative to the robot's waist. Where the operator's hands are relative to
their head is where the G1's wrists go — there is no delta-clutch origin,
which keeps the human-to-robot correspondence consistent for imitation
learning episodes.

Outputs:
    - left_controller_output / right_controller_output: absolute wrist
      targets in the pelvis frame, ``frame_id = "<arm_task_name>/left|right"``
      (remap onto ``coordinator_cartesian_command`` in the blueprint).
    - teleop_buttons: digital buttons + analog triggers (engage source for
      the IK task).
    - cmd_vel: thumbstick locomotion (left stick forward/back + yaw or
      strafe on the right stick; right stick press = zero-Twist e-stop).

Inputs:
    - color_image: optional camera feed pushed to the headset as JPEG.

Requires the Quest web client to stream the headset pose (frame_id
"head") alongside the controller poses.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.teleop.quest.quest_extensions import _push_jpeg
from dimos.teleop.quest.quest_teleop_module import QuestTeleopConfig, QuestTeleopModule
from dimos.teleop.quest.quest_types import Buttons, Hand, QuestControllerState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

logger = setup_logger()

# WebXR frame (X=right, Y=up, Z=back) <-> robot frame (X=fwd, Y=left, Z=up)
# conjugation for full 4x4 transforms, matching the retargeting math this
# was tuned with (do not mix with webxr_to_robot(), which also applies a
# per-controller grip rotation).
_T_ROBOT_OPENXR = np.array(
    [[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]],
    dtype=np.float64,
)
_T_OPENXR_ROBOT = np.array(
    [[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]],
    dtype=np.float64,
)


def _valid_pose_matrix(mat: np.ndarray) -> bool:
    det = np.linalg.det(mat)
    return bool(np.isfinite(det)) and not np.isclose(det, 0.0, atol=1e-6)


class G1QuestTeleopConfig(QuestTeleopConfig):
    """Configuration for Quest-driven G1 teleop retargeting."""

    # Task the wrist targets are routed to (frame_id prefix).
    arm_task_name: str = "dual_arm_ik"

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
    video_jpeg_quality: int = 70


class G1QuestTeleopModule(QuestTeleopModule):
    """Quest WebXR retargeting for G1 locomotion and bimanual arms."""

    # Own process: per-frame JPEG encodes must not share a GIL with control
    # modules (measured arm latency when placed with G1WholeBodyConnection).
    dedicated_worker = True

    config: G1QuestTeleopConfig

    color_image: In[Image]
    cmd_vel: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Raw WebXR matrices; None until the first valid sample so we never
        # retarget from a fabricated pose.
        self._head_xr: np.ndarray | None = None
        self._hand_xr: dict[Hand, np.ndarray | None] = {Hand.LEFT: None, Hand.RIGHT: None}
        # Log cmd_vel activity on transitions only, so a still controller
        # doesn't spam at the control-loop rate.
        self._cmd_vel_moving = False

    async def handle_color_image(self, msg: Image) -> None:
        _push_jpeg(self, msg, self.config.video_jpeg_quality)

    def _on_pose_bytes(self, data: bytes) -> None:
        """Store raw WebXR matrices; the head pose rides the same stream.

        Degenerate matrices (tracking loss) are dropped, keeping the last
        good sample.
        """
        msg = PoseStamped.lcm_decode(data)
        try:
            matrix = pose_to_matrix(msg)
        except ValueError:
            # Zero/NaN quaternion during tracking loss; keep the last pose.
            return
        if not _valid_pose_matrix(matrix):
            return
        with self._lock:
            if msg.frame_id == "head":
                self._head_xr = matrix
            elif msg.frame_id in ("left", "right"):
                hand = Hand.LEFT if msg.frame_id == "left" else Hand.RIGHT
                self._hand_xr[hand] = matrix

    def _handle_engage(self) -> None:
        """Engage gating lives in the g1_dual_arm_ik task, not here."""

    def _should_publish(self, hand: Hand) -> bool:
        return self._head_xr is not None and self._hand_xr[hand] is not None

    def _get_output_pose(self, hand: Hand) -> PoseStamped | None:
        head_xr = self._head_xr
        hand_xr = self._hand_xr[hand]
        if head_xr is None or hand_xr is None:
            return None
        wrist = self._retarget_wrist(head_xr, hand_xr, hand)
        pose = matrix_to_pose(wrist)
        suffix = "left" if hand == Hand.LEFT else "right"
        return PoseStamped(
            position=pose.position,
            orientation=pose.orientation,
            ts=time.time(),
            frame_id=f"{self.config.arm_task_name}/{suffix}",
        )

    def _retarget_wrist(self, head_xr: np.ndarray, hand_xr: np.ndarray, hand: Hand) -> np.ndarray:
        """Head-relative absolute retarget of one controller to a wrist pose.

        Yaw-normalizes around the headset so the operator can face any
        direction, scales the reach, and anchors the result to the robot's
        waist in the pelvis frame.
        """
        head: np.ndarray = _T_ROBOT_OPENXR @ head_xr @ _T_OPENXR_ROBOT
        wrist: np.ndarray = _T_ROBOT_OPENXR @ hand_xr @ _T_OPENXR_ROBOT

        head_yaw = math.atan2(head[1, 0], head[0, 0])
        cos_y = math.cos(-head_yaw)
        sin_y = math.sin(-head_yaw)
        inv_yaw = np.array(
            [[cos_y, -sin_y, 0.0], [sin_y, cos_y, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        wrist = wrist.copy()
        delta = inv_yaw @ (wrist[:3, 3] - head[:3, 3])
        wrist[:3, :3] = inv_yaw @ wrist[:3, :3]

        delta *= self.config.workspace_scale
        waist_x, waist_y, waist_z = self.config.waist_offset
        wrist[:3, 3] = delta + np.array([waist_x, waist_y, waist_z])
        correction = self.config.shoulder_y_correction
        wrist[1, 3] += -correction if hand == Hand.LEFT else correction
        return wrist

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
        self.teleop_buttons.publish(buttons)
        self._publish_cmd_vel(left, right)

    def _publish_cmd_vel(
        self,
        left: QuestControllerState | None,
        right: QuestControllerState | None,
    ) -> None:
        def dz(value: float) -> float:
            return 0.0 if abs(value) < self.config.deadzone else value

        if right is not None and right.thumbstick_press:
            # E-stop: one zero Twist; let downstream re-arming come from a
            # different source without us spamming.
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
        # Only publish while the sticks are outside the deadzone, plus one
        # definitive stop on the moving→neutral transition. Publishing zeros
        # at the loop rate would clobber other cmd_vel producers (nav stack,
        # agent commands).
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
                logger.info("G1 Quest cmd_vel active: vx=%.2f vy=%.2f wz=%.2f", vx, vy, yaw_rate)
            else:
                logger.info("G1 Quest cmd_vel zeroed")
