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

"""Example policy host for testing sim2's two independent data contracts.

Uses the existing GR00T task unchanged. This is a simulation demonstration,
not a replacement connection, coordinator, or hardware firmware emulator.
"""

import threading
import time
from typing import Any

from dimos_lcm.sim_msgs import Action, Actions, JointCommand
import numpy as np

from dimos.control.components import make_humanoid_joints
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.g1_groot_wbc_task.g1_groot_wbc_task import (
    G1_GROOT_KD,
    G1_GROOT_KP,
    G1GrootWBCTask,
    G1GrootWBCTaskConfig,
)
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.control_msgs.ControlValues import ControlValues
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.sim2.physics.module import SimulationSpec
from dimos.utils.data import LfsPath

JOINTS = tuple(n + "_joint" for n in make_humanoid_joints("g1"))
POLICY = G1GrootWBCTaskConfig(
    balance_onnx=LfsPath("groot/balance.onnx"),
    walk_onnx=LfsPath("groot/walk.onnx"),
    joint_names=list(JOINTS[:15]),
    all_joint_names=list(JOINTS),
    auto_arm=True,
    default_ramp_seconds=0,
    decimation=1,
)


class G1PolicyDemo(Module):
    _sim: SimulationSpec
    robot_feedback: In[ControlValues]
    cmd_vel: In[Twist]
    actions: Out[Actions]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._task = G1GrootWBCTask("groot_wbc", POLICY)
        self._lock = threading.RLock()
        self._velocity = np.zeros(3)
        self._until = 0.0
        self._identity = (-1, -1)
        self._last_action: Actions | None = None
        self._arm_targets = np.array(POLICY.default_positions_29[15:])
        self.inferences = 0

    @rpc
    def start(self) -> None:
        layout = self._sim.describe()
        self._revision = layout.revision
        self._run_id = self._sim.sample().run_id
        world = self._sim.describe_world()
        joints = {j.name: j for i in world.instances for j in i.description.joints}
        self._lower = np.array([joints[n].lower for n in JOINTS], dtype=float)
        self._upper = np.array([joints[n].upper for n in JOINTS], dtype=float)
        self._task.start()
        super().start()

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self.move(twist)

    @rpc
    def move(self, twist: Twist, duration: float = 0.3) -> None:
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not np.isfinite((*values, duration)).all() or duration <= 0:
            raise ValueError("finite velocity and positive duration required")
        with self._lock:
            self._velocity = np.clip(values, [-0.5, -0.3, -0.6], [0.5, 0.3, 0.6])
            self._until = time.monotonic() + duration

    @rpc
    def arm_joint(self, name: str, position: float) -> None:
        index = JOINTS.index(name)
        if index < 15 or not self._lower[index] <= position <= self._upper[index]:
            raise ValueError("select an arm joint and a position within its model limits")
        with self._lock:
            self._arm_targets[index - 15] = position

    async def handle_robot_feedback(self, message: ControlValues) -> None:
        action = self.compute(message)
        if action is not None:
            self.actions.publish(action)

    def compute(self, message: ControlValues) -> Actions | None:
        with self._lock:
            if message.source != self._run_id:
                return None
            identity = (message.epoch, message.sequence)
            if identity < self._identity:
                return None
            if identity == self._identity:
                return self._last_action  # retransmit without advancing policy history
            if message.epoch != self._identity[0]:
                self._task.reset_runtime_state(reactivate=True)
                self._velocity[:] = 0
                self._until = 0
                self._arm_targets[:] = POLICY.default_positions_29[15:]
            values = message.as_dict()
            now = values["sim/time"]
            velocity = self._velocity if time.monotonic() < self._until else np.zeros(3)
            self._task.set_velocity_command(
                float(velocity[0]), float(velocity[1]), float(velocity[2]), t_now=now
            )
            imu = [
                values["g1/imu/" + field]
                for field in ("qw", "qx", "qy", "qz", "wx", "wy", "wz", "ax", "ay", "az")
            ]
            state = CoordinatorState(
                joints=JointStateSnapshot(
                    joint_positions={n: values[n + "/position"] for n in JOINTS},
                    joint_velocities={n: values[n + "/velocity"] for n in JOINTS},
                    timestamp=message.source_ts,
                ),
                imu={
                    "g1": IMUState(
                        quaternion=(imu[0], imu[1], imu[2], imu[3]),
                        gyroscope=(imu[4], imu[5], imu[6]),
                        accelerometer=(imu[7], imu[8], imu[9]),
                    )
                },
                t_now=now,
                dt=0.02,
            )
            result = self._task.compute(state)
            if result is None or result.positions is None:
                return None
            positions = np.clip(
                np.r_[result.positions, self._arm_targets], self._lower, self._upper
            )
            command = JointCommand(
                n=29,
                names=list(JOINTS),
                position=positions.tolist(),
                velocity=[0.0] * 29,
                kp=G1_GROOT_KP,
                kd=G1_GROOT_KD,
                effort=[0.0] * 29,
            )
            payload = command.lcm_encode()
            self._last_action = Actions(
                run_id=self._run_id,
                model_revision=self._revision,
                epoch=message.epoch,
                tick=message.sequence,
                ttl=0.25,
                actions_length=1,
                actions=[Action(kind=Action.JOINT_COMMAND, data_length=len(payload), data=payload)],
            )
            self._identity = identity
            self.inferences += 1
            return self._last_action

    @rpc
    def status(self) -> dict[str, Any]:
        return {"inferences": self.inferences, **self._task.state_snapshot()}

    @rpc
    def stop(self) -> None:
        self._task.stop()
        super().stop()
