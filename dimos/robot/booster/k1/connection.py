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

"""Booster SDK serial-joint connection for the 22-motor K1."""

import importlib
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field
from scipy.spatial.transform import Rotation

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.booster.k1.control_config import HOME, JOINTS, KD, KP, LOWER, UPPER
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class K1ConnectionConfig(ModuleConfig):
    network_interface: str
    domain_id: int = Field(default=0, ge=0, le=232)
    feedback_timeout: float = Field(default=0.1, gt=0, allow_inf_nan=False)


def decode_state(state: Any, timestamp: float) -> tuple[JointState, Imu]:
    motors = state.motor_state_serial
    if len(motors) != 22:
        raise ValueError(f"K1 requires 22 serial motors, received {len(motors)}")
    sensor = state.imu_state
    position = [m.q for m in motors]
    velocity = [m.dq for m in motors]
    effort = [m.tau_est for m in motors]
    if not np.isfinite(
        [*position, *velocity, *effort, *sensor.rpy, *sensor.gyro, *sensor.acc]
    ).all():
        raise ValueError("K1 feedback must be finite")
    return (
        JointState(
            name=list(JOINTS), position=position, velocity=velocity, effort=effort, ts=timestamp
        ),
        Imu(
            orientation=Quaternion(*Rotation.from_euler("xyz", sensor.rpy).as_quat()),
            angular_velocity=Vector3(*sensor.gyro),
            linear_acceleration=Vector3(*sensor.acc),
            frame_id="k1/imu",
            ts=timestamp,
        ),
    )


def encode_command(sdk: Any, command: MotorCommandArray) -> Any:
    arrays = (command.q, command.dq, command.kp, command.kd, command.tau)
    if command.num_joints != 22 or any(len(a) != 22 for a in arrays):
        raise ValueError("K1 commands require 22 serial motors")
    if not np.isfinite(arrays).all() or np.any(np.asarray(arrays[2:4]) < 0):
        raise ValueError("K1 commands must be finite with nonnegative gains")
    if np.any(np.asarray(command.q) < LOWER) or np.any(np.asarray(command.q) > UPPER):
        raise ValueError("K1 position target outside joint limits")
    msg = sdk.LowCmd()
    msg.cmd_type = sdk.LowCmdType.SERIAL
    motors = []
    for q, dq, kp, kd, tau in zip(*arrays, strict=True):
        motor = sdk.MotorCmd()
        motor.q, motor.dq, motor.kp, motor.kd, motor.tau = q, dq, kp, kd, tau
        motor.weight = 1.0
        motors.append(motor)
    msg.motor_cmd = motors
    return msg


class K1Connection(Module):
    """Starts in telemetry-only mode; activate() explicitly takes custom control."""

    config: K1ConnectionConfig
    joint_state: Out[JointState]
    imu: Out[Imu]
    motor_command: In[MotorCommandArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sdk: Any = None
        self._publisher: Any = None
        self._subscriber: Any = None
        self._client: Any = None
        self._command: Any = None
        self._state: JointState | None = None
        self._state_at = self._command_at = float("-inf")
        self._command_stamp = float("-inf")
        self._active = False
        self._error = ""
        self._lock = threading.RLock()
        self._done = threading.Event()
        self._thread: threading.Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        try:
            self._sdk = importlib.import_module("booster_robotics_sdk_python")
            if self._sdk.kJointCntK1 != 22:
                raise RuntimeError("SDK does not expose the supported 22-motor K1 layout")
            self._sdk.ChannelFactory.Instance().Init(
                self.config.domain_id, self.config.network_interface
            )
            self._client = self._sdk.B1LocoClient()
            self._client.Init()
            self._publisher = self._sdk.B1LowCmdPublisher()
            self._publisher.InitChannel()
            self._subscriber = self._sdk.B1LowStateSubscriber(self._on_state)
            self._subscriber.InitChannel()
            self._done.clear()
            self._thread = threading.Thread(target=self._run, name="k1-sdk", daemon=True)
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def _on_state(self, state: Any) -> None:
        try:
            joints, imu = decode_state(state, time.time())
            with self._lock:
                self._state, self._state_at = joints, time.monotonic()
            self.joint_state.publish(joints)
            self.imu.publish(imu)
        except Exception as exc:
            with self._lock:
                self._error = str(exc)
                self._state_at = float("-inf")
            logger.exception("K1 feedback rejected")

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        with self._lock:
            if self._sdk is None:
                return
            if (
                not np.isfinite(command.timestamp)
                or not 0 <= time.time() - command.timestamp <= self.config.feedback_timeout
            ):
                raise ValueError("K1 motor command must reference recent hardware feedback")
            if command.timestamp <= self._command_stamp:
                return
            self._command = encode_command(self._sdk, command)
            self._command_at = time.monotonic()
            self._command_stamp = command.timestamp

    def _change_mode(self, mode: Any) -> None:
        if self._client is None:
            raise RuntimeError("K1 connection has not started")
        result = self._client.ChangeMode(mode)
        if result != 0:
            raise RuntimeError(f"Booster ChangeMode({mode}) failed: {result}")

    @rpc
    def prepare(self) -> None:
        """Ask the vendor controller to enter its prepare posture."""
        with self._lock:
            if self._active:
                raise RuntimeError("Deactivate the K1 policy before vendor preparation")
            if self._sdk is None:
                raise RuntimeError("K1 connection has not started")
            self._change_mode(self._sdk.RobotMode.kPrepare)

    @rpc
    def activate(self) -> None:
        """Take custom control from a standing K1 near the policy's home posture."""
        with self._lock:
            if self._active:
                return
            self._check_fresh()
            if (
                self._state is None
                or np.max(np.abs(np.asarray(self._state.position) - HOME)) > 0.35
            ):
                raise RuntimeError(
                    "Prepare the standing K1 near its home posture before activation"
                )
            hold = MotorCommandArray(q=self._state.position, kp=list(KP), kd=list(KD))
            self._publisher.Write(encode_command(self._sdk, hold))
            self._change_mode(self._sdk.RobotMode.kCustom)
            self._active = True
            self._error = ""

    @rpc
    def deactivate(self) -> None:
        """Release owned custom control into the vendor damping mode."""
        with self._lock:
            if self._active:
                self._active = False
                self._change_mode(self._sdk.RobotMode.kDamping)

    def _check_fresh(self) -> None:
        if time.monotonic() - min(self._state_at, self._command_at) > self.config.feedback_timeout:
            raise RuntimeError("K1 feedback or policy command timed out")

    def _tick(self) -> None:
        with self._lock:
            if not self._active:
                return
            try:
                self._check_fresh()
                self._publisher.Write(self._command)
            except Exception as exc:
                self._error = str(exc)
                self.deactivate()
                raise

    def _run(self) -> None:
        while not self._done.wait(0.002):
            try:
                self._tick()
            except Exception:
                logger.exception("K1 custom control stopped")

    @rpc
    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "active": self._active,
                "error": self._error,
                "feedback_age": time.monotonic() - self._state_at,
                "command_age": time.monotonic() - self._command_at,
            }

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        try:
            self.deactivate()
        finally:
            for channel in (self._subscriber, self._publisher):
                if channel is not None:
                    channel.CloseChannel()
            self._subscriber = self._publisher = self._client = self._sdk = None
            super().stop()
