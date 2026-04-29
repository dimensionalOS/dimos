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

"""G1 wholebody (Arch B) Module.

Wraps UnitreeG1LowLevelAdapter as a worker-isolated dimos Module. Owns the
DDS connection; the coordinator stays out of the DDS process.

Stream interface:
  - motor_states: Out[JointState]      29 motors, q/dq/tau in position/velocity/effort
  - imu:          Out[Imu]             quaternion + gyro + accel
  - motor_command: In[MotorCommandArray]  29-DOF q/dq/kp/kd/tau

mode_machine handling stays inside the adapter — never appears on the wire.
"""

from __future__ import annotations

import logging
import threading
from threading import Thread
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import MotorCommand
from dimos.hardware.whole_body.unitree.g1.adapter import UnitreeG1LowLevelAdapter
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray

logger = logging.getLogger(__name__)

_NUM_MOTORS = 29

# Motor index → joint name. Order matches UnitreeG1LowLevelAdapter docstring:
#   0-5:   left leg  (hip pitch/roll/yaw, knee, ankle pitch/roll)
#   6-11:  right leg
#   12-14: waist (yaw, roll, pitch — roll/pitch may be invalid on some variants)
#   15-21: left arm  (shoulder pitch/roll/yaw, elbow, wrist roll/pitch/yaw)
#   22-28: right arm
G1_JOINT_NAMES: list[str] = [
    "g1/left_hip_pitch",
    "g1/left_hip_roll",
    "g1/left_hip_yaw",
    "g1/left_knee",
    "g1/left_ankle_pitch",
    "g1/left_ankle_roll",
    "g1/right_hip_pitch",
    "g1/right_hip_roll",
    "g1/right_hip_yaw",
    "g1/right_knee",
    "g1/right_ankle_pitch",
    "g1/right_ankle_roll",
    "g1/waist_yaw",
    "g1/waist_roll",
    "g1/waist_pitch",
    "g1/left_shoulder_pitch",
    "g1/left_shoulder_roll",
    "g1/left_shoulder_yaw",
    "g1/left_elbow",
    "g1/left_wrist_roll",
    "g1/left_wrist_pitch",
    "g1/left_wrist_yaw",
    "g1/right_shoulder_pitch",
    "g1/right_shoulder_roll",
    "g1/right_shoulder_yaw",
    "g1/right_elbow",
    "g1/right_wrist_roll",
    "g1/right_wrist_pitch",
    "g1/right_wrist_yaw",
]
assert len(G1_JOINT_NAMES) == _NUM_MOTORS


class G1WholeBodyConnectionConfig(ModuleConfig):
    network_interface: str = Field(default="")
    release_sport_mode: bool = True
    publish_rate_hz: float = 500.0
    frame_id: str = "g1_pelvis"


class G1WholeBodyConnection(Module):
    """G1 humanoid Module — owns the DDS connection in its own worker.

    Mirrors GO2Connection's shape (sensors + actuation in one Module) but at
    the low-level joint layer instead of high-level Twist.
    """

    config: G1WholeBodyConnectionConfig

    motor_command: In[MotorCommandArray]
    motor_states: Out[JointState]
    imu: Out[Imu]

    connection: UnitreeG1LowLevelAdapter

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.connection = UnitreeG1LowLevelAdapter(
            network_interface=self.config.network_interface,
            release_sport_mode=self.config.release_sport_mode,
        )
        self._stop_event = threading.Event()
        self._publish_thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()

        if not self.connection.connect():
            raise RuntimeError("G1 low-level adapter failed to connect")

        self.register_disposable(Disposable(self.motor_command.subscribe(self._on_motor_command)))

        self._publish_thread = Thread(
            target=self._publish_loop,
            name="g1-wholebody-pump",
            daemon=True,
        )
        self._publish_thread.start()

        logger.info(
            f"G1WholeBodyConnection started (rate={self.config.publish_rate_hz}Hz, "
            f"release_sport_mode={self.config.release_sport_mode})"
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._publish_thread is not None and self._publish_thread.is_alive():
            self._publish_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            self._publish_thread = None

        if self.connection.is_connected():
            self.connection.disconnect()

        super().stop()

    def _publish_loop(self) -> None:
        period = 1.0 / float(self.config.publish_rate_hz)
        next_tick = time.perf_counter()
        frame_id = self.config.frame_id

        while not self._stop_event.is_set():
            states = self.connection.read_motor_states()
            imu_state = self.connection.read_imu()
            now = time.time()

            self.motor_states.publish(
                JointState(
                    ts=now,
                    frame_id=frame_id,
                    name=G1_JOINT_NAMES,
                    position=[s.q for s in states],
                    velocity=[s.dq for s in states],
                    effort=[s.tau for s in states],
                )
            )

            # Unitree IMU quaternion is (w, x, y, z); dimos Quaternion is (x, y, z, w).
            q = imu_state.quaternion
            g = imu_state.gyroscope
            a = imu_state.accelerometer
            self.imu.publish(
                Imu(
                    ts=now,
                    frame_id=frame_id,
                    orientation=Quaternion(q[1], q[2], q[3], q[0]),
                    angular_velocity=Vector3(g[0], g[1], g[2]),
                    linear_acceleration=Vector3(a[0], a[1], a[2]),
                )
            )

            next_tick += period
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()

    def _on_motor_command(self, msg: MotorCommandArray) -> None:
        if msg.num_joints != _NUM_MOTORS:
            logger.warning(f"Expected {_NUM_MOTORS} motor commands, got {msg.num_joints}; ignoring")
            return

        commands = [
            MotorCommand(
                q=msg.q[i],
                dq=msg.dq[i],
                kp=msg.kp[i],
                kd=msg.kd[i],
                tau=msg.tau[i],
            )
            for i in range(_NUM_MOTORS)
        ]
        self.connection.write_motor_commands(commands)


__all__ = ["G1_JOINT_NAMES", "G1WholeBodyConnection", "G1WholeBodyConnectionConfig"]
