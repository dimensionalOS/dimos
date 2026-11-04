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

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import JointCommand, JointTarget
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
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.sim2.module import ControllerConfig
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


CONTROL = ControllerConfig("g1/groot", 20_000_000, JOINTS)


class G1PolicyConfig(ModuleConfig):
    control: ControllerConfig = CONTROL
    imu_source: str = "g1/imu"
    joint_source: str = "g1/encoders"


class G1PolicyModule(SharedConfigModule):
    config: G1PolicyConfig
    imu: In[Imu]
    joint_state: In[JointState]
    cmd_vel: In[Twist]
    joint_command: Out[JointCommand]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._task = G1GrootWBCTask("groot_wbc", POLICY)
        self._lock = threading.RLock()
        self._velocity = np.zeros(3)
        self._until = 0.0
        self._identity = (-1, -1)
        self._last_action: JointCommand | None = None
        self._joints: dict[tuple[int, int], JointState] = {}
        self._imus: dict[tuple[int, int], Imu] = {}
        self._clock_id: str | None = None
        self._arm_targets = np.array(POLICY.default_positions_29[15:])
        self.inferences = 0

    def config_receive(self, config_senders: ConfigSenders) -> None:
        joints = {
            joint.name: joint
            for _, config in config_senders
            for model in config.get("robot_models", {}).values()
            for joint in model.joints
        }
        self._lower = np.array([joints[n].lower for n in JOINTS], dtype=float)
        self._upper = np.array([joints[n].upper for n in JOINTS], dtype=float)

    @rpc
    def start(self) -> None:
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

    async def handle_joint_state(self, sample: JointState) -> None:
        self._receive(sample, self.config.joint_source)

    async def handle_imu(self, sample: Imu) -> None:
        self._receive(sample, self.config.imu_source)

    def _receive(self, sample: JointState | Imu, source: str) -> None:
        h = sample.sample_header
        if h is None or h.source_id != source or h.time_ns % self.config.control.period_ns:
            return
        with self._lock:
            if self._clock_id is not None and h.clock_id != self._clock_id:
                raise ValueError("controller clock changed; rebind")
            self._clock_id = h.clock_id
            identity = (h.epoch, h.time_ns)
            if identity < self._identity:
                return
            if identity == self._identity:
                if self._last_action is not None:
                    self.joint_command.publish(self._last_action)
                return
            if isinstance(sample, JointState):
                self._joints[identity] = sample
            else:
                self._imus[identity] = sample
            if identity in self._joints and identity in self._imus:
                action = self.compute(self._joints.pop(identity), self._imus.pop(identity))
                if action is not None:
                    self.joint_command.publish(action)
            # Bounded join; old samples can never satisfy a later boundary.
            self._joints = {k: v for k, v in self._joints.items() if k >= identity}
            self._imus = {k: v for k, v in self._imus.items() if k >= identity}

    def compute(self, joints: JointState, imu: Imu) -> JointCommand | None:
        with self._lock:
            h, ih = joints.sample_header, imu.sample_header
            if (
                h is None
                or ih is None
                or (h.clock_id, h.epoch, h.time_ns) != (ih.clock_id, ih.epoch, ih.time_ns)
            ):
                raise ValueError(
                    "policy requires joint and IMU measurements at the same acquisition"
                )
            if h.source_id != self.config.joint_source or ih.source_id != self.config.imu_source:
                raise ValueError("wrong configured measurement source")
            identity = (h.epoch, h.time_ns)
            if h.time_ns % self.config.control.period_ns or identity < self._identity:
                return None
            if identity == self._identity:
                return self._last_action
            if h.epoch != self._identity[0]:
                self._task.reset_runtime_state(reactivate=True)
                self._velocity[:] = 0
                self._until = 0
                self._arm_targets[:] = POLICY.default_positions_29[15:]
            now = h.time_ns / 1e9
            velocity = self._velocity if time.monotonic() < self._until else np.zeros(3)
            self._task.set_velocity_command(
                float(velocity[0]), float(velocity[1]), float(velocity[2]), t_now=now
            )
            if imu.orientation_covariance[0] < 0:
                raise ValueError(
                    "GR00T requires an attitude estimate or explicitly ideal orientation"
                )
            q, w, a = imu.orientation, imu.angular_velocity, imu.linear_acceleration
            state = CoordinatorState(
                joints=JointStateSnapshot(
                    joint_positions=dict(zip(joints.name, joints.position, strict=True)),
                    joint_velocities=dict(zip(joints.name, joints.velocity, strict=True)),
                    timestamp=now,
                ),
                imu={
                    "g1": IMUState(
                        quaternion=(q.w, q.x, q.y, q.z),
                        gyroscope=(w.x, w.y, w.z),
                        accelerometer=(a.x, a.y, a.z),
                    )
                },
                t_now=now,
                dt=self.config.control.period_ns / 1e9,
            )
            result = self._task.compute(state)
            if result is None or result.positions is None:
                return None
            positions = np.clip(
                np.r_[result.positions, self._arm_targets], self._lower, self._upper
            )
            self._last_action = JointCommand(
                header=SampleHeader(
                    clock_id=h.clock_id,
                    epoch=h.epoch,
                    time_ns=h.time_ns,
                    source_id=self.config.control.controller_id,
                    sequence=h.sequence,
                ),
                based_on_time_ns=h.time_ns,
                apply_at_ns=h.time_ns,
                valid_until_ns=h.time_ns + self.config.control.period_ns,
                targets_length=len(JOINTS),
                targets=[
                    JointTarget(
                        joint_id=n, position=float(q), velocity=0, kp=kp, kd=kd, feedforward=0
                    )
                    for n, q, kp, kd in zip(
                        JOINTS, positions, G1_GROOT_KP, G1_GROOT_KD, strict=True
                    )
                ],
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
