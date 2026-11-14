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

"""The DimOS side of a Feetech leader robot: follower targets at a fixed rate.

The motor layer lives in :mod:`dimos.robot.manipulators.feetech.arm` and the
robot's data in :mod:`dimos.robot.manipulators.feetech.robot`. Each reading
is published as targets for the follower joints the motors drive, kept inside
their limits, and each trigger as its jaw's opening. The motors are only
read: a Feetech connection takes no commands. While ``follow`` is off, no
targets are sent and the follower holds still.
"""

from __future__ import annotations

from abc import abstractmethod
import math
from pathlib import Path
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from reactivex.disposable import Disposable
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.manipulators.feetech.arm import FeetechArm, MockFeetechArm
from dimos.robot.manipulators.feetech.calibration import (
    FEETECH_CALIBRATION_ROOT,
    default_calibration_path,
    load_calibration,
)
from dimos.robot.manipulators.feetech.robot import FeetechRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
# The trajectory task treats a JointState in this frame as a streamed jog that
# it releases 0.4 s after the last message, so the follower is free once the
# leader stops.
GUARDED_JOINT_FRAME_ID = "dimos/guarded-joint"


class FeetechConnectionConfig(ConnectionConfig):
    """Settings for a Feetech connection.

    Attributes:
        source: The follower connection's name, which starts every published
            joint name, e.g. "arm" gives "arm/left_joint1".
        state_rate_hz: How often the leaders are read and published, in Hz.
        calibration_root: Directory holding one calibration folder per bus.
        filter_cutoff_hz: Cutoff of a low-pass on the joint targets, in Hz,
            to smooth hand tremor and encoder noise. None sends each reading
            as it is.
        mock_positions: Follower joint angles to publish when no serial port
            is given, in radians, e.g. {"left_joint1": 0.3}; joints left out
            read 0. None makes a serial port required.
    """

    source: str = "arm"
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    calibration_root: Path = FEETECH_CALIBRATION_ROOT
    filter_cutoff_hz: float | None = Field(default=None, gt=0, allow_inf_nan=False)
    mock_positions: dict[str, float] | None = None

    @model_validator(mode="after")
    def _read_only(self) -> Self:
        if self.joint_commands:
            raise ValueError("Feetech connections only read; they take no commands")
        return self


class FeetechConnection(ConnectionModule):
    """Own one Feetech leader robot and publish what its follower should do."""

    #: The robot this connection reads: its buses, motors and triggers.
    robot: ClassVar[FeetechRobot]
    config: FeetechConnectionConfig
    # Follower joint targets, for the follower coordinator's trajectory task.
    joint_state: Out[JointState]
    # Jaw openings, 1 = open, for the follower coordinator's gripper tasks.
    left_gripper_command: Out[Float32]
    right_gripper_command: Out[Float32]
    # Off stops the joint targets so the leaders can be moved while the
    # follower holds; on (the start state) sends them again.
    follow: In[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._arm: FeetechArm | MockFeetechArm | None = None
        self._following = True
        self._follow_lock = threading.Lock()
        self._last_read_error: str | None = None
        self._filtered: dict[str, float] = {}
        self._stop_reading = threading.Event()
        self._reader: threading.Thread | None = None

    @abstractmethod
    def serial_ports(self) -> dict[str, str]:
        """The serial port per bus name; a bus given "" or left out is not read."""

    def calibration_path(self, bus: str) -> Path:
        """Where one bus's calibration file is read from."""
        return default_calibration_path(bus, self.config.calibration_root)

    def connect(self) -> None:
        """Load each bus's calibration and open its serial port, or start the mock."""
        ports = {bus: port for bus, port in self.serial_ports().items() if port.strip()}
        arm: FeetechArm | MockFeetechArm
        if not ports:
            if self.config.mock_positions is None:
                raise ValueError("give a serial port for at least one Feetech bus")
            arm = MockFeetechArm(self.robot, self.config.mock_positions)
        else:
            calibrations = {
                bus: load_calibration(
                    self.calibration_path(bus), bus, self.robot.bus(bus).motor_names
                )
                for bus in ports
            }
            arm = FeetechArm(self.robot, ports, calibrations)
        arm.connect()
        self._arm = arm

    @rpc
    def start(self) -> None:
        try:
            # ConnectionModule.start builds command limits, and a connection
            # that takes no commands has none, so start the module directly.
            Module.start(self)
            self._session_id = uuid.uuid4().hex
            self.connect()
            with self._hardware_lock:
                self._connected = True
            self.register_disposable(Disposable(self.follow.subscribe(self._on_follow)))
            self._reader = threading.Thread(
                target=self._read_loop, daemon=True, name=f"{self.config.source}-read"
            )
            self._reader.start()
        except Exception:
            self.stop()
            raise

    @rpc
    def stop(self) -> None:
        self._stop_reading.set()
        if (
            self._reader is not None
            and self._reader.is_alive()
            and self._reader is not threading.current_thread()
        ):
            self._reader.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        try:
            super().stop()
        finally:
            with self._hardware_lock:
                arm, self._arm = self._arm, None
                if arm is not None:
                    arm.disconnect()

    def write(self, values: Values) -> None:
        raise RuntimeError("Feetech connections only read; they take no commands")

    def halt(self) -> None:
        """Nothing is ever commanded, so there is nothing to stop."""

    def publish_reading(self, readings: dict[str, dict[str, float]]) -> None:
        """Publish the follower targets and jaw openings for one reading.

        Args:
            readings: Motor angles in radians, by bus name, then motor name,
                as :meth:`FeetechArm.read` returns them.
        """
        with self._follow_lock:
            if self._following:
                self._publish_targets(readings)
        outputs = {"left": self.left_gripper_command, "right": self.right_gripper_command}
        for bus in self.robot.buses:
            if bus.trigger is not None and bus.name in readings:
                opening = bus.trigger.opening(readings[bus.name][bus.trigger.name])
                outputs[bus.name].publish(Float32(data=opening))

    def _publish_targets(self, readings: dict[str, dict[str, float]]) -> None:
        targets = {
            motor.joint: motor.target(readings[bus.name][motor.name])
            for bus in self.robot.buses
            if bus.name in readings
            for motor in bus.motors
        }
        cutoff = self.config.filter_cutoff_hz
        if cutoff is not None:
            # First-order low-pass, sampled once per read.
            period = 1.0 / self.config.state_rate_hz
            alpha = period / (1.0 / (math.tau * cutoff) + period)
            targets = {
                joint: self._filtered[joint] + alpha * (value - self._filtered[joint])
                if joint in self._filtered
                else value
                for joint, value in targets.items()
            }
            self._filtered.update(targets)
        self.joint_state.publish(
            JointState(
                ts=time.time(),
                frame_id=GUARDED_JOINT_FRAME_ID,
                name=[f"{self.config.source}/{joint}" for joint in targets],
                position=list(targets.values()),
            )
        )

    def _on_follow(self, msg: Bool) -> None:
        with self._follow_lock:
            if msg.data == self._following:
                return
            self._following = msg.data
            # A resumed stream starts at the leaders' reading, not the old pose.
            self._filtered.clear()
            if not msg.data:
                # A jog with no positions and zero speed tells the trajectory
                # task to hold every joint now, not 0.4 s after the last target.
                joints = [
                    f"{self.config.source}/{motor.joint}"
                    for bus in self.robot.buses
                    for motor in bus.motors
                ]
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        frame_id=GUARDED_JOINT_FRAME_ID,
                        name=joints,
                        velocity=[0.0] * len(joints),
                    )
                )
        logger.info("Leader targets %s", "resumed" if msg.data else "paused")

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    readings = self._arm.read()
            except Exception as error:
                # A failed read is skipped; the next one may succeed.
                message = f"{type(error).__name__}: {error}"
                if message != self._last_read_error:
                    self._report(f"read failed: {message}")
                    self._last_read_error = message
            else:
                self._last_read_error = None
                self.publish_reading(readings)
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
