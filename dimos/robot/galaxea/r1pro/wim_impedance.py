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

"""Robot-side JoyLo connection for a WIM node in arm_impedance mode."""

from __future__ import annotations

from contextvars import ContextVar
import time
from typing import Any

from dimos_lcm.control_msgs import DynamicJointState
from reactivex.disposable import Disposable

from dimos.control.connection.connection_module import Values
from dimos.control.contract.convert import (
    dynamic_joint_state_from_values,
    dynamic_joint_state_to_values,
)
from dimos.core.stream import In
from dimos.robot.galaxea.r1pro.connection import _make_qos, _R1ProConnection, _ros_stamp_now
from dimos.robot.galaxea.r1pro.joylo import ARMS, TORSO

_command_stamp: ContextVar[tuple[int, int] | None] = ContextVar("wim_command_stamp", default=None)


class R1ProWimImpedanceConnection(_R1ProConnection):
    motor_command: In[DynamicJointState]

    def _setup_control_topics(self) -> None:
        from control_msgs.msg import DynamicJointState as RosMotor

        from dimos.protocol.pubsub.impl.rospubsub import RawROSTopic

        super()._setup_control_topics()
        self._arm_topics = [
            RawROSTopic(f"/wim/impedance/{side}_arm", RosMotor, qos=_make_qos())
            for side in ("left", "right")
        ]
        assert self._ros is not None and self._ros._node is not None
        deadline = time.monotonic() + 5
        while not all(self._ros._node.count_subscribers(t.topic) for t in self._arm_topics):
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    "Start the impedance-enabled WIM node before launching JoyLo impedance"
                )
            time.sleep(0.05)

    def _subscribe_commands(self) -> None:
        self.register_disposable(Disposable(self.motor_command.subscribe(self._receive_motor)))

    def _receive_motor(self, sample: DynamicJointState) -> None:
        stamp = sample.header.stamp
        age = time.time() - (stamp.sec + stamp.nsec * 1e-9)
        if 0 <= age < 0.1:
            token = _command_stamp.set((stamp.sec, stamp.nsec))
            try:
                self.command(dynamic_joint_state_to_values(sample))
            finally:
                _command_stamp.reset(token)

    def write(self, values: Values) -> None:
        from control_msgs.msg import DynamicJointState as RosMotor, InterfaceValue
        from sensor_msgs.msg import JointState as RosJointState

        ros = self._ros
        if ros is None:
            raise RuntimeError("WIM impedance connection is offline")
        stamp = _ros_stamp_now()
        original = _command_stamp.get()
        if original is not None:
            stamp.sec, stamp.nanosec = original
        packets: list[tuple[Any, Any]] = []
        arm_values = {k.rsplit("/", 1)[0] for k in values}
        for joints, topic in zip((ARMS[:7], ARMS[7:]), self._arm_topics, strict=True):
            if arm_values.isdisjoint(joints):
                continue
            rows = dynamic_joint_state_from_values(values, joints)
            packet = RosMotor()
            packet.header.stamp = stamp
            packet.joint_names = [joint.removeprefix("r1pro/") for joint in joints]
            packet.interface_values = [
                InterfaceValue(interface_names=row.interface_names, values=row.values)
                for row in rows.interface_values
            ]
            packets.append((topic, packet))
        if any(f"{joint}/position" in values for joint in TORSO):
            cmd = RosJointState()
            cmd.header.stamp = stamp
            cmd.name = [j.removeprefix("r1pro/") for j in TORSO]
            cmd.position = [values[f"{j}/position"] for j in TORSO]
            cmd.velocity = [self.config.tracking_speed] * len(TORSO)
            packets.append((self._cmd_torso_topic, cmd))
        for topic, packet in packets:
            ros.publish(topic, packet)
        super().write(
            {k: v for k, v in values.items() if k.rsplit("/", 1)[0] not in (*ARMS, *TORSO)}
        )

    def _send_joint_targets(self, q: list[float], stamp: Any) -> None:
        # halt() uses position-only arm rows to release the virtual spring.
        self.write(
            {f"{joint}/position": value for joint, value in zip((*TORSO, *ARMS), q, strict=True)}
        )
