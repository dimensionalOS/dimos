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

"""The DimOS side of a Feetech robot: publish its joint angles at a fixed rate.

The motor layer lives in :mod:`dimos.robot.manipulators.feetech.arm` and the
robot's data in :mod:`dimos.robot.manipulators.feetech.robot`. The motors are
only read: a Feetech connection takes no commands.
"""

from __future__ import annotations

from abc import abstractmethod
from pathlib import Path
import threading
import time
from typing import Any, ClassVar
import uuid

from pydantic import Field, model_validator
from typing_extensions import Self

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.connection.connection_module import ConnectionConfig, ConnectionModule, Values
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.manipulators.feetech.arm import FeetechArm, MockFeetechArm
from dimos.robot.manipulators.feetech.calibration import (
    FEETECH_CALIBRATION_ROOT,
    default_calibration_path,
    load_calibration,
)
from dimos.robot.manipulators.feetech.robot import FeetechRobot
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class FeetechConnectionConfig(ConnectionConfig):
    """Settings for a Feetech connection.

    Attributes:
        source: The first part of every published joint name, e.g. "leader"
            gives "leader/left_joint_1".
        state_rate_hz: How often the joint angles are read and published, in Hz.
        calibration_root: Directory holding one calibration folder per bus.
        mock_positions: Joint angles to publish when no serial port is given,
            in radians, keyed by joint name ("left_joint_1"); joints left out
            read 0. None makes a serial port required.
    """

    source: str = "leader"
    state_rate_hz: float = Field(default=100.0, gt=0, allow_inf_nan=False)
    calibration_root: Path = FEETECH_CALIBRATION_ROOT
    mock_positions: dict[str, float] | None = None

    @model_validator(mode="after")
    def _read_only(self) -> Self:
        if self.joint_commands or self.pd or self.base_range is not None:
            raise ValueError("Feetech connections only read; they take no commands")
        return self


class FeetechConnection(ConnectionModule):
    """Own one Feetech robot and publish its joint angles."""

    #: The robot this connection reads: its buses and their motors.
    robot: ClassVar[FeetechRobot]
    config: FeetechConnectionConfig
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._arm: FeetechArm | MockFeetechArm | None = None
        self._last_read_error: str | None = None
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
                bus: load_calibration(self.calibration_path(bus), bus, self.robot.bus(bus).motors)
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

    def _read_loop(self) -> None:
        period = 1.0 / self.config.state_rate_hz
        while not self._stop_reading.is_set():
            started = time.monotonic()
            try:
                with self._hardware_lock:
                    if self._arm is None or not self._connected:
                        return
                    positions = self._arm.read()
            except Exception as error:
                # A failed read is skipped; the next one may succeed.
                message = f"{type(error).__name__}: {error}"
                if message != self._last_read_error:
                    self._report(f"read failed: {message}")
                    self._last_read_error = message
            else:
                self._last_read_error = None
                self.joint_state.publish(
                    JointState(
                        ts=time.time(),
                        name=[f"{self.config.source}/{name}" for name in positions],
                        position=list(positions.values()),
                    )
                )
            self._stop_reading.wait(max(0.0, period - (time.monotonic() - started)))
