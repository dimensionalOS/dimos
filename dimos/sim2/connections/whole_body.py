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

"""Whole-body device streams; SHM remains the current coordinator command path."""

from __future__ import annotations

import threading
import time
from typing import Any, Literal

from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.sim2.control.adapters import WholeBodyAdapter
from dimos.sim2.spec import RobotConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class WholeBodyConnectionConfig(ModuleConfig):
    definition: RobotConfig
    address: str
    robot_id: str
    rate_hz: float = 50.0
    command_source: Literal["coordinator", "stream"] = "coordinator"


class WholeBodyConnection(Module):
    config: WholeBodyConnectionConfig
    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._device = WholeBodyAdapter(
            address=self.config.address,
            dof=len(self.config.definition.joints),
            definition=self.config.definition,
        )
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._device.connect()
        if self.config.command_source == "stream":
            self.register_disposable(Disposable(self.motor_command.subscribe(self._command)))
        self._stop.clear()
        self._thread = threading.Thread(target=self._publish, daemon=True)
        self._thread.start()

    def _command(self, msg: MotorCommandArray) -> None:
        self._device.write_motor_commands(
            [
                MotorCommand(*row)
                for row in zip(
                    msg.q,
                    msg.dq,
                    msg.kp,
                    msg.kd,
                    msg.tau,
                    strict=True,
                )
            ]
        )

    def _publish(self) -> None:
        try:
            while not self._stop.is_set():
                before = time.monotonic()
                try:
                    frame = self._device.sample()
                except BlockingIOError:
                    # A missed read is not new feedback. Retry next period;
                    # the policy watchdog still detects sustained starvation.
                    self._stop.wait(1.0 / self.config.rate_hz)
                    continue
                v = frame.values
                ts = float(v["wall_time"][0])
                self.motor_states.publish(
                    JointState(
                        name=[j.name for j in self.config.definition.joints],
                        position=v["position"].tolist(),
                        velocity=v["velocity"].tolist(),
                        effort=v["effort"].tolist(),
                        ts=ts,
                    )
                )
                iq = v["imu_quaternion"]
                self.imu.publish(
                    Imu(
                        orientation=Quaternion(
                            float(iq[1]), float(iq[2]), float(iq[3]), float(iq[0])
                        ),
                        angular_velocity=Vector3(*v["imu_gyroscope"]),
                        linear_acceleration=Vector3(*v["imu_accelerometer"]),
                        frame_id=f"{self.config.robot_id}/imu",
                        ts=ts,
                    )
                )
                pos = Vector3(*v["root_position"])
                q = v["root_quaternion"]
                rot = Quaternion(float(q[1]), float(q[2]), float(q[3]), float(q[0]))
                self.odom.publish(
                    PoseStamped(position=pos, orientation=rot, frame_id="world", ts=ts)
                )
                self.tf.publish(
                    TFMessage(
                        Transform(
                            translation=pos,
                            rotation=rot,
                            frame_id="world",
                            child_frame_id=f"{self.config.robot_id}/{self.config.definition.root_body}",
                            ts=ts,
                        )
                    )
                )
                self._stop.wait(max(0, 1.0 / self.config.rate_hz - (time.monotonic() - before)))
        except Exception:
            if not self._stop.is_set():
                logger.exception("sim2 whole-body connection stopped")

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._device.disconnect()
        super().stop()
