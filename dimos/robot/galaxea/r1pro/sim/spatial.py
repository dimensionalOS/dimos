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

"""R1 bindings for the shared spatial workstation; motion stays in sim controllers."""

import json
from typing import Any, Protocol

from fastapi import HTTPException
import numpy as np
from pydantic import BaseModel, ConfigDict, Field
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.spec.utils import Spec
from dimos.teleop.spatial.module import SpatialTeleopModule, SpatialViewerConfig
from dimos.teleop.webxr.controller_types import Hand


class CoordinatorStopSpec(Spec, Protocol):
    def set_estop(self, estopped: bool) -> bool: ...


class R1ServoSpec(Spec, Protocol):
    def set_joint(self, name: str, position: float) -> None: ...
    def home(self) -> None: ...


class Jog(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    session: str
    epoch: int
    joint: str
    value: float


class R1SpatialConfig(SpatialViewerConfig):
    linear_speed: float = Field(default=0.2, gt=0, le=0.4)
    angular_speed: float = Field(default=0.3, gt=0, le=0.5)


class R1ProSpatialTeleop(SpatialTeleopModule):
    config: R1SpatialConfig
    dedicated_worker = True
    cmd_vel: Out[Twist]
    _coordinator: CoordinatorStopSpec
    _servo: R1ServoSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._clutch_frames: dict[Hand, np.ndarray] = {}

    def _publish_safe_command(self) -> None:
        self.cmd_vel.publish(Twist.zero())

    def _reset_controller_state(self) -> None:
        super()._reset_controller_state()
        self._clutch_frames.clear()

    def _dispatch_text_message(self, payload: str) -> bool:
        valid = super()._dispatch_text_message(payload)
        if not valid or self._operator_paused:
            self._publish_safe_command()
            return False
        data = json.loads(payload)
        drive = data.get("drive", [0, 0, 0])
        self.cmd_vel.publish(
            Twist(
                linear=Vector3(
                    drive[0] * self.config.linear_speed, drive[1] * self.config.linear_speed, 0
                ),
                angular=Vector3(0, 0, drive[2] * self.config.angular_speed),
            )
        )
        for hand in Hand:
            controller = self._controllers[hand]
            if controller is None or controller.grip < 0.5:
                self._clutch_frames.pop(hand, None)
        return True

    def _publish_msg(self, hand: Hand, output_msg: PoseStamped) -> None:
        # Capture the configured shoulder frame at clutch engagement. This
        # matches the arm-subtree IK, including the actual torso posture. Base
        # driving then carries the arms instead of moving the operator reference.
        side = "left" if hand == Hand.LEFT else "right"
        group = self.config.groups.get(side)
        if group is None:
            return
        if hand not in self._clutch_frames:
            transform = self._transforms.get(
                "world", group.base_frame, self._feedback.get("source_ts"), 0.1, warn=False
            )
            if transform is None:
                return
            self._clutch_frames[hand] = np.linalg.inv(transform.to_matrix())
        matrix = self._clutch_frames[hand]
        pose = PoseStamped(
            position=matrix[:3, :3] @ output_msg.position.to_numpy() + matrix[:3, 3],
            orientation=(
                Rotation.from_matrix(matrix[:3, :3])
                * Rotation.from_quat(output_msg.orientation.to_list())
            ).as_quat(),
            frame_id=group.base_frame,
            ts=output_msg.ts,
        )
        super()._publish_msg(hand, pose)

    @rpc
    def operator_action(self, action: str, session: str, epoch: int) -> None:
        super().operator_action(action, session, epoch)
        if action == "hold":
            self._coordinator.set_estop(True)
        elif action == "resume":
            self._coordinator.set_estop(False)

    def _setup_routes(self) -> None:
        super()._setup_routes()
        assert self._web_server is not None

        @self._web_server.app.post("/spatial/jog")
        def jog(command: Jog) -> dict[str, bool]:
            with self._lock:
                if (
                    command.session != self._operator_session
                    or command.epoch != self._control_epoch
                ):
                    raise HTTPException(409, "Control session changed")
                if not self._joint_feedback_ready(0):
                    raise HTTPException(409, "Robot feedback unavailable")
                allowed = {j for group in self.config.groups.values() for j in group.joint_names}
                if command.joint not in allowed:
                    raise HTTPException(400, "Joint is not an advertised arm control")
                self.operator_action("hold", command.session, command.epoch)
                try:
                    self._servo.set_joint(command.joint, command.value)
                except ValueError as error:
                    raise HTTPException(400, str(error)) from error
            return {"accepted": True}
