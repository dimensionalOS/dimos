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

"""Transport adaptation for the existing R1 Pro coordinator and planning groups."""

import math
import threading
from typing import Any

from reactivex.disposable import Disposable

from dimos.control.coordinator import ControlCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.galaxea.r1pro.config import R1PRO_PLANAR_BASE
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS, coordinator_name
from dimos.simulation.behavior.r1pro_model import MODEL_JOINTS, upper_body_limits
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def ordered_state(message: JointState, names: tuple[str, ...]) -> JointState:
    if len(set(message.name)) != len(message.name):
        raise ValueError("Duplicate joint names")
    if any(
        len(values) != len(message.name)
        for values in (message.position, message.velocity, message.effort)
    ):
        raise ValueError("Incomplete position/velocity/effort feedback")
    indices = [message.name.index(name) for name in names]
    arrays = [
        [float(values[i]) for i in indices]
        for values in (message.position, message.velocity, message.effort)
    ]
    if not all(math.isfinite(v) for values in arrays for v in values):
        raise ValueError("Nonfinite joint feedback")
    return JointState(
        ts=message.ts,
        frame_id="base_link",
        name=[coordinator_name(n) for n in names],
        position=arrays[0],
        velocity=arrays[1],
        effort=arrays[2],
    )


class BehaviorCoordinator(ControlCoordinator):
    """Resolve installed robot limits when deployed, never during blueprint discovery."""

    @rpc
    def start(self) -> None:
        self.config.hardware[0].limits = upper_body_limits()
        super().start()


class BehaviorR1ProBridge(Module):
    joint_state: In[JointState]
    odom: In[PoseStamped]
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    joint_command: Out[JointState]
    planning_joint_state: Out[JointState]
    goal: Out[PointStamped]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._odom: PoseStamped | None = None
        self._state: JointState | None = None
        self._limits = upper_body_limits()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.joint_state.subscribe(self._on_state)))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_command)))

    def _on_odom(self, message: PoseStamped) -> None:
        with self._lock:
            self._odom = message

    def _on_state(self, message: JointState) -> None:
        try:
            upper = ordered_state(message, UPPER_BODY_JOINTS)
            model = ordered_state(message, MODEL_JOINTS)
        except ValueError as error:
            logger.warning("Rejected simulator feedback", error=str(error))
            return
        self.motor_states.publish(upper)
        with self._lock:
            odom = self._odom
            self._state = upper
        if odom is None or abs(odom.ts - message.ts) > 0.25:
            return
        q = odom.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.planning_joint_state.publish(
            JointState(
                ts=message.ts,
                frame_id="world",
                name=[*R1PRO_PLANAR_BASE.joint_names, *model.name],
                position=[odom.position.x, odom.position.y, yaw, *model.position],
                velocity=[0.0, 0.0, 0.0, *model.velocity],
                effort=[0.0, 0.0, 0.0, *model.effort],
            )
        )

    def _on_command(self, message: MotorCommandArray) -> None:
        if len(message.q) != len(UPPER_BODY_JOINTS):
            logger.warning("Rejected incomplete coordinator command")
            return
        for value, low, high in zip(
            message.q, self._limits.position_lower, self._limits.position_upper, strict=True
        ):
            if not math.isfinite(value) or low is None or high is None or not low <= value <= high:
                logger.warning("Rejected out-of-range coordinator command")
                return
        self.joint_command.publish(
            JointState(name=list(UPPER_BODY_JOINTS), position=list(message.q))
        )

    @rpc
    def set_goal(self, x: float, y: float, z: float = 0.0) -> None:
        """Send a world-frame navigation goal without requiring a visualization client."""
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ValueError("Goal must be finite")
        self.goal.publish(PointStamped(x, y, z, frame_id="world"))

    @rpc
    def snapshot(self) -> dict[str, Any]:
        """Read measured feedback for development checks."""
        with self._lock:
            return {
                "position": None
                if self._odom is None
                else [self._odom.position.x, self._odom.position.y, self._odom.position.z],
                "yaw": None if self._odom is None else float(self._odom.orientation.euler[2]),
                "joints": {}
                if self._state is None
                else dict(zip(self._state.name, self._state.position, strict=True)),
            }
