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

"""``ManipulatorAdapter`` for RoboLab's DROID robot, over the RoboLab bridge.

RoboLab runs the robot in Isaac Sim; its runner sends the joint state on ``state`` and
takes joint and gripper targets on ``cmd`` (see dimos.simulation.robolab.protocol). The
joints are RoboLab's seven Panda joints, then the Robotiq ``finger_joint`` in radians:
0 open, pi/4 closed.
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import zenoh

from dimos.hardware.manipulators.spec import ControlMode, ManipulatorInfo
from dimos.hardware.spec import JointLimits
from dimos.simulation.robolab.protocol import (
    ARM_DOF,
    DEFAULT_ENDPOINT,
    DEFAULT_PREFIX,
    GRIPPER_CLOSED_RAD,
    RobotState,
    open_session,
    pack_command,
    unpack,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# RoboLab starts Isaac and loads the task before its first state: minutes on a cold cache.
_FIRST_STATE_TIMEOUT_S = 600.0
# The runner repeats the state every 0.2 s while paused; this long without one is a stall.
_STALE_AFTER_S = 5.0
_STALE_STATE_ERROR = 1
# RoboLab's limits for the DROID Panda (Isaac's soft limits), rad.
_ARM_LOWER = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
_ARM_UPPER = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]
_ARM_VELOCITY = [2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61]


class RoboLabAdapter:
    """Reads RoboLab's joint state and sends it joint and gripper targets.

    ``address`` is the zenoh endpoint of RoboLab's runner.
    """

    def __init__(
        self,
        dof: int = ARM_DOF + 1,
        address: str | None = None,
        hardware_id: str | None = None,
        prefix: str = DEFAULT_PREFIX,
        **_: Any,
    ) -> None:
        if dof != ARM_DOF + 1:
            raise ValueError(f"sim_robolab drives 7 arm joints and the gripper (8), got {dof}")
        self._dof = dof
        self._endpoint = str(address) if address else DEFAULT_ENDPOINT
        self._prefix = prefix
        self._hardware_id = hardware_id
        self._session: zenoh.Session | None = None
        self._cmd_pub: Any = None
        self._lock = threading.Lock()
        self._first_state = threading.Event()
        self._state: RobotState | None = None
        self._state_wall = 0.0
        self._velocity = np.zeros(ARM_DOF)
        self._seq = 0
        self._sent: list[float] | None = None
        self._connected = False
        self._servos_enabled = False
        self._control_mode = ControlMode.POSITION
        self._error_code = 0
        self._error_message = ""

    def _on_state(self, sample: zenoh.Sample) -> None:
        state = RobotState.decode(unpack(sample))
        with self._lock:
            previous = self._state
            if previous is not None and state.sim_time > previous.sim_time:
                self._velocity = (state.q - previous.q) / (state.sim_time - previous.sim_time)
            elif previous is not None and state.paused:
                self._velocity = np.zeros(ARM_DOF)
            self._state = state
            self._state_wall = time.monotonic()
        self._first_state.set()

    def connect(self) -> bool:
        self._session = open_session(self._endpoint)
        self._session.declare_subscriber(f"{self._prefix}/state", self._on_state)
        self._cmd_pub = self._session.declare_publisher(f"{self._prefix}/cmd")
        if not self._first_state.wait(_FIRST_STATE_TIMEOUT_S):
            logger.error("No state from RoboLab", endpoint=self._endpoint, prefix=self._prefix)
            self.disconnect()
            return False
        self._connected = True
        self._servos_enabled = True
        self._error_code = 0
        self._error_message = ""
        logger.info("RoboLabAdapter connected", endpoint=self._endpoint, prefix=self._prefix)
        return True

    def disconnect(self) -> None:
        session, self._session = self._session, None
        self._cmd_pub = None
        self._connected = False
        if session is not None:
            session.close()

    def _live_state(self) -> RobotState | None:
        with self._lock:
            state, age = self._state, time.monotonic() - self._state_wall
        if not self._connected or state is None:
            return None
        if age > _STALE_AFTER_S:
            if self._error_code != _STALE_STATE_ERROR:
                self._error_code = _STALE_STATE_ERROR
                self._error_message = "RoboLab stopped sending its state"
                logger.error(self._error_message, endpoint=self._endpoint, age_s=round(age, 1))
            return None
        return state

    def is_connected(self) -> bool:
        return self._live_state() is not None

    def has_live_state(self) -> bool:
        """Gates ConnectedHardware reads and commands on fresh sim state."""
        return self.is_connected()

    def activate(self) -> bool:
        return self.write_enable(True)

    def deactivate(self) -> bool:
        self.write_stop()
        return self.write_enable(False)

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="RoboLab",
            model="DROID (Franka Panda + Robotiq 2F-85)",
            dof=self._dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_dof(self) -> int:
        return self._dof

    def get_limits(self) -> JointLimits:
        """The Panda's limits, then finger_joint: 0 open, pi/4 closed."""
        return JointLimits(
            position_lower=[*_ARM_LOWER, 0.0],
            position_upper=[*_ARM_UPPER, GRIPPER_CLOSED_RAD],
            velocity_max=[*_ARM_VELOCITY, 0.0],
        )

    def set_control_mode(self, mode: ControlMode) -> bool:
        # Joint targets either way: RoboLab steps toward the latest one at 15 Hz.
        if mode not in (ControlMode.POSITION, ControlMode.SERVO_POSITION):
            return False
        self._control_mode = mode
        return True

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def read_joint_positions(self) -> list[float]:
        with self._lock:
            state = self._state
        if state is None:
            return [0.0] * self._dof
        return [*state.q.tolist(), state.gripper * GRIPPER_CLOSED_RAD]

    def read_joint_velocities(self) -> list[float]:
        with self._lock:
            velocity = self._velocity.copy()
        return [*velocity.tolist(), 0.0]

    def read_joint_efforts(self) -> list[float]:
        return [0.0] * self._dof

    def read_state(self) -> dict[str, int]:
        is_moving = any(abs(v) > 1e-4 for v in self.read_joint_velocities())
        return {"state": 1 if is_moving else 0, "mode": list(ControlMode).index(self._control_mode)}

    def read_error(self) -> tuple[int, str]:
        return self._error_code, self._error_message

    def write_joint_positions(self, positions: list[float], velocity: float = 1.0) -> bool:
        """Send the seven arm targets and finger_joint (rad); unchanged targets are not resent."""
        if not self._servos_enabled or len(positions) != self._dof:
            return False
        if self._live_state() is None or self._cmd_pub is None:
            return False
        target = [float(p) for p in positions]
        if self._sent is not None and np.allclose(target, self._sent, atol=1e-6):
            return True
        self._seq += 1
        gripper = min(1.0, max(0.0, target[ARM_DOF] / GRIPPER_CLOSED_RAD))
        self._cmd_pub.put(pack_command(self._seq, target[:ARM_DOF], gripper))
        self._sent = target
        return True

    def write_joint_velocities(self, velocities: list[float]) -> bool:
        return False

    def write_joint_efforts(self, efforts: list[float]) -> bool:
        return False

    def write_stop(self) -> bool:
        """Hold where the robot is."""
        state = self._live_state()
        if state is None:
            return False
        hold = [*state.q.tolist(), (self._sent or [0.0] * self._dof)[ARM_DOF]]
        return self.write_joint_positions(hold)

    def write_enable(self, enable: bool) -> bool:
        self._servos_enabled = enable
        return True

    def read_enabled(self) -> bool:
        return self._servos_enabled

    def write_clear_errors(self) -> bool:
        self._error_code = 0
        self._error_message = ""
        return True

    def read_cartesian_position(self) -> dict[str, float] | None:
        return None

    def write_cartesian_position(self, pose: dict[str, float], velocity: float = 1.0) -> bool:
        return False

    def read_force_torque(self) -> list[float] | None:
        return None
