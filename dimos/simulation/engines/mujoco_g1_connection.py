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

"""The Unitree G1 simulated by the blueprint's ``MujocoSimModule``, as a connection module."""

from __future__ import annotations

import threading
import time
from typing import Annotated, Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.control.contract.convert import joint_state_to_values
from dimos.control.contract.keys import Interface
from dimos.core.core import rpc
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.assets.model import RobotModel
from dimos.robot.unitree.g1.config import G1_JOINT_MODEL, G1_JOINT_NAMES
from dimos.simulation.engines.mujoco_shm import ManipShmReader, shm_key_from_path
from dimos.simulation.engines.spec import MujocoSimSpec

_NUM_MOTORS = len(G1_JOINT_NAMES)
# How long connect waits for the sim's shared memory to appear, then to update, in seconds.
_ATTACH_TIMEOUT_S = 30.0
_READY_TIMEOUT_S = 180.0
_POLL_S = 0.1

_Gain = Annotated[float, Field(ge=0.0, allow_inf_nan=False)]


class MujocoG1ConnectionConfig(ConnectionConfig):
    """The simulated G1's joints and motor gains, as on ``G1WholeBodyConnectionConfig``.

    Attributes:
        kp: Position gain of each of the 29 motors (N·m/rad), in joint order.
        kd: Damping gain of each motor (N·m·s/rad).
        publish_rate_hz: How many times a second the sim is checked for new readings.
        frame_id: Frame the joint state and IMU readings are stamped with.
    """

    source: str = "g1"
    model: RobotModel | None = G1_JOINT_MODEL
    joint_commands: dict[str, tuple[Interface, ...]] = {
        name: (Interface.POSITION,) for name in G1_JOINT_NAMES
    }
    kp: tuple[_Gain, ...] = Field(default=(40.0,) * _NUM_MOTORS, min_length=29, max_length=29)
    kd: tuple[_Gain, ...] = Field(default=(3.0,) * _NUM_MOTORS, min_length=29, max_length=29)
    publish_rate_hz: float = Field(default=500.0, gt=0, allow_inf_nan=False)
    frame_id: str = "g1_pelvis"


class MujocoG1Connection(ConnectionModule):
    """A G1 in the blueprint's MuJoCo scene, with the ports ``G1WholeBodyConnection`` has.

    All 29 joint targets (rad) come in together on ``position_command`` and go to
    the sim's PD controller with the configured gains. ``joint_state`` and ``imu``
    go out each time the sim steps. A halt drops stiffness and keeps damping.
    """

    config: MujocoG1ConnectionConfig

    position_command: In[JointState]
    joint_state: Out[JointState]
    imu: Out[Imu]

    _mujoco_sim: MujocoSimSpec

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._shm: ManipShmReader | None = None
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(
                self.position_command.subscribe(
                    lambda msg: self.command(joint_state_to_values(msg, (Interface.POSITION,)))
                )
            )
        )
        self._reader = threading.Thread(
            target=self._read_loop, daemon=True, name="MujocoG1Connection-read"
        )
        self._reader.start()

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if self._reader is not None and self._reader is not threading.current_thread():
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()
        shm, self._shm = self._shm, None
        if shm is not None:
            shm.cleanup()

    def connect(self) -> None:
        scene = self._mujoco_sim.scene_path()
        shm = ManipShmReader.wait_for_live(
            shm_key_from_path(scene),
            attach_timeout_s=_ATTACH_TIMEOUT_S,
            ready_timeout_s=_READY_TIMEOUT_S,
            poll_s=_POLL_S,
        )
        if shm is None:
            raise ConnectionError(f"the MuJoCo sim of {scene} never reported its joints")
        if shm.num_joints() != _NUM_MOTORS:
            shm.cleanup()
            raise ValueError(f"the sim of {scene} has {shm.num_joints()} joints, the G1 has 29")
        self._shm = shm

    def write(self, values: Values) -> None:
        keys = [f"{name}/{Interface.POSITION}" for name in G1_JOINT_NAMES]
        if not all(key in values for key in keys):
            raise ValueError("G1 requires a complete 29-joint position command")
        self._live().write_pd_tau_command(
            [values[key] for key in keys],
            list(self.config.kp),
            list(self.config.kd),
            [0.0] * _NUM_MOTORS,
        )

    def halt(self) -> None:
        shm = self._shm
        if shm is not None and shm.is_live():
            shm.write_pd_tau_command(
                shm.read_positions(_NUM_MOTORS),
                [0.0] * _NUM_MOTORS,
                list(self.config.kd),
                [0.0] * _NUM_MOTORS,
            )

    def _live(self) -> ManipShmReader:
        shm = self._shm
        if shm is None or not shm.is_live():
            raise ConnectionError("the MuJoCo sim stopped updating the G1's joints")
        return shm

    def _read_loop(self) -> None:
        period = 1.0 / self.config.publish_rate_hz
        frame_id = self.config.frame_id
        last_seq: int | None = None
        while not self._stop_reading.wait(period):
            try:
                shm = self._live()
                seq = shm.position_sequence()
                if seq == last_seq:
                    continue
                last_seq = seq
                (qw, qx, qy, qz), gyro, accel = shm.read_imu()
                now = time.time()
                self.joint_state.publish(
                    JointState(
                        ts=now,
                        frame_id=frame_id,
                        name=list(G1_JOINT_NAMES),
                        position=shm.read_positions(_NUM_MOTORS),
                        velocity=shm.read_velocities(_NUM_MOTORS),
                        effort=shm.read_efforts(_NUM_MOTORS),
                    )
                )
                self.imu.publish(
                    Imu(
                        orientation=Quaternion(qx, qy, qz, qw),
                        angular_velocity=Vector3(*gyro),
                        linear_acceleration=Vector3(*accel),
                        frame_id=frame_id,
                        ts=now,
                    )
                )
            except Exception as error:
                self._report(f"reading failed: {type(error).__name__}: {error}")
