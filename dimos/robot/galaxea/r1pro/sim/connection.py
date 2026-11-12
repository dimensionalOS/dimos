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

"""CC's R1 Pro command contract connected to sim2's physical servo.

Canonical jaw opening is 0..0.1 m. The simulation has two opposing fingers
with 0.050 m travel each; normalize at this boundary, preserving CC's API.
"""

import threading
from typing import Any

from dimos_lcm.sim_msgs import WorldState
from scipy.spatial.transform import Rotation

from dimos.control.connection.connection_module import ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_BODY_AXES, R1PRO_GRIPPER_OPENING_RANGE
from dimos.robot.galaxea.r1pro.connection import R1ProConnectionConfig
from dimos.robot.galaxea.r1pro.joylo import JAWS, UPPER
from dimos.robot.galaxea.r1pro.sim.model import FINGER_TRAVEL_M

OPENING_M = R1PRO_GRIPPER_OPENING_RANGE[1]


class R1ProSimConnection(ConnectionModule):
    config: R1ProConnectionConfig
    observations: In[WorldState]
    encoder: In[JointState]
    position_command: In[JointState]
    velocity_command: In[JointState]
    joint_state: Out[JointState]
    upper_command: Out[JointState]
    servo_left_gripper: Out[Float32]
    servo_right_gripper: Out[Float32]
    servo_velocity: Out[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._feedback_lock = threading.RLock()
        self._positions: dict[str, float] = {}

    def connect(self) -> None:
        """The simulation is owned by SimModule, not this command connection."""

    async def handle_position_command(self, sample: JointState) -> None:
        self.command(joint_state_to_values(sample, (Interface.POSITION,)))

    async def handle_velocity_command(self, sample: JointState) -> None:
        self.command(joint_state_to_values(sample, (Interface.VELOCITY,)))

    def write(self, values: Values) -> None:
        positions = {
            name: values[f"{name}/position"] for name in UPPER if f"{name}/position" in values
        }
        if positions:
            self.upper_command.publish(
                JointState(name=list(positions), position=list(positions.values()))
            )
        for name, port in zip(
            JAWS, (self.servo_left_gripper, self.servo_right_gripper), strict=True
        ):
            key = f"{name}/position"
            if key in values:
                port.publish(Float32(data=values[key] / OPENING_M))
        if any(f"{name}/velocity" in values for name in R1PRO_BODY_AXES):
            x, y, yaw = [values.get(f"{name}/velocity", 0.0) for name in R1PRO_BODY_AXES]
            self.servo_velocity.publish(Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw)))

    def halt(self) -> None:
        with self._feedback_lock:
            positions = self._positions.copy()
        values = {}
        for name, value in positions.items():
            key = f"{name}/position"
            bound = self.limits[key]
            assert bound.lo is not None and bound.hi is not None
            values[key] = min(bound.hi, max(bound.lo, value))
        self.write(values)
        self.servo_velocity.publish(Twist.zero())

    async def handle_observations(self, truth: WorldState) -> None:
        """Report measured chassis velocity in CC's body-frame convention."""
        body = next((b for b in truth.bodies if b.body_id == "r1pro/base_link"), None)
        if body is None:
            return
        q = body.pose.orientation
        to_body = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv()
        v, w = body.twist.linear, body.twist.angular
        linear = to_body.apply([v.x, v.y, v.z])
        angular = to_body.apply([w.x, w.y, w.z])
        self.joint_state.publish(
            JointState(
                ts=truth.header.time_ns / 1e9,
                frame_id=self.frame("joint_state"),
                name=list(R1PRO_BODY_AXES),
                velocity=[float(linear[0]), float(linear[1]), float(angular[2])],
            )
        )

    async def handle_encoder(self, sample: JointState) -> None:
        measured = dict(zip(sample.name, sample.position, strict=True))
        positions = {name: measured[name] for name in UPPER}
        for side, joint in zip(("left", "right"), JAWS, strict=True):
            aperture = (
                measured[f"r1pro/{side}_gripper_finger_joint1"]
                - measured[f"r1pro/{side}_gripper_finger_joint2"]
            )
            positions[joint] = max(
                0.0, min(OPENING_M, aperture / (2 * FINGER_TRAVEL_M) * OPENING_M)
            )
        with self._feedback_lock:
            self._positions = positions
        self.joint_state.publish(
            JointState(
                ts=sample.ts,
                frame_id=self.frame("joint_state"),
                name=list(positions),
                position=list(positions.values()),
            )
        )
