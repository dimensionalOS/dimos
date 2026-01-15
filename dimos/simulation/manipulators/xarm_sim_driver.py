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

"""MuJoCo-native xArm simulation driver and SDK wrapper."""

import logging
import math
from typing import Any

from dimos.hardware.manipulators.base import (
    BaseManipulatorDriver,
    StandardMotionComponent,
    StandardServoComponent,
    StandardStatusComponent,
)
from dimos.hardware.manipulators.base.sdk_interface import BaseManipulatorSDK, ManipulatorInfo
from dimos.simulation.manipulators import MujocoSimBackend

logger = logging.getLogger(__name__)


class XArmSimSDKWrapper(BaseManipulatorSDK):
    """SDK wrapper for xArm simulation using the MuJoCo simulation backend."""

    def __init__(self) -> None:
        self.logger = logging.getLogger(self.__class__.__name__)
        self.native_sdk: MujocoSimBackend | None = None
        self.dof = 7
        self._connected = False
        self._servos_enabled = False
        self._mode = 0
        self._state = 0

    def connect(self, config: dict[str, Any]) -> bool:
        """Connect to the MuJoCo simulation backend."""
        try:
            robot = config.get("robot")
            if not robot:
                raise ValueError("robot is required for MuJoCo simulation loading")
            config_path = config.get("config_path")
            headless = bool(config.get("headless", False))

            self.logger.info("Connecting to MuJoCo Sim...")
            self.native_sdk = MujocoSimBackend(
                robot=robot,
                config_path=config_path,
                headless=headless,
            )
            self.native_sdk.connect()

            if self.native_sdk.connected:
                self._connected = True
                self._servos_enabled = True
                self._state = 0
                self._mode = 0
                self.dof = int(config.get("dof", self.native_sdk.num_joints))
                self.logger.info(f"Successfully connected to MuJoCo Sim (DOF: {self.dof})")
                return True

            self.logger.error("Failed to connect to XArm Sim")
            return False
        except Exception as exc:
            self.logger.error(f"Sim connection failed: {exc}")
            return False

    def disconnect(self) -> None:
        """Disconnect from simulation."""
        if self.native_sdk:
            try:
                self.native_sdk.disconnect()
            finally:
                self._connected = False
                self.native_sdk = None

    def is_connected(self) -> bool:
        return bool(self._connected and self.native_sdk and self.native_sdk.connected)

    def get_joint_positions(self) -> list[float]:
        return self.native_sdk.joint_positions[: self.dof]

    def get_joint_velocities(self) -> list[float]:
        return self.native_sdk.joint_velocities[: self.dof]

    def get_joint_efforts(self) -> list[float]:
        return self.native_sdk.joint_efforts[: self.dof]

    def set_joint_positions(
        self,
        positions: list[float],
        _velocity: float = 1.0,
        _acceleration: float = 1.0,
        _wait: bool = False,
    ) -> bool:
        _ = _velocity
        _ = _acceleration
        _ = _wait
        if not self._servos_enabled:
            return False
        self._mode = 0
        self.native_sdk.set_joint_position_targets(positions[: self.dof])
        return True

    def set_joint_velocities(self, velocities: list[float]) -> bool:
        if not self._servos_enabled:
            return False
        self._mode = 1
        dt = 1.0 / self.native_sdk.control_frequency
        current = self.native_sdk.joint_positions
        targets = [current[i] + velocities[i] * dt for i in range(min(len(velocities), self.dof))]
        self.native_sdk.set_joint_position_targets(targets)
        return True

    def set_joint_efforts(self, efforts: list[float]) -> bool:
        self.logger.warning("Torque control not supported in MuJoCo Sim bridge")
        _ = efforts
        return False

    def stop_motion(self) -> bool:
        self.native_sdk.hold_current_position()
        self._state = 0
        return True

    def enable_servos(self) -> bool:
        self._servos_enabled = True
        self._state = 0
        return True

    def disable_servos(self) -> bool:
        self._servos_enabled = False
        return True

    def are_servos_enabled(self) -> bool:
        return self._servos_enabled

    def get_robot_state(self) -> dict[str, Any]:
        is_moving = any(abs(v) > 1e-4 for v in self.native_sdk.joint_velocities[: self.dof])
        self._state = 1 if is_moving else 0
        return {
            "state": self._state,
            "mode": self._mode,
            "error_code": 0,
            "warn_code": 0,
            "is_moving": is_moving,
            "cmd_num": 0,
        }

    def get_error_code(self) -> int:
        return 0

    def get_error_message(self) -> str:
        return ""

    def clear_errors(self) -> bool:
        self._state = 0
        return True

    def emergency_stop(self) -> bool:
        self._state = 3
        self.native_sdk.hold_current_position()
        return True

    def get_info(self) -> ManipulatorInfo:
        return ManipulatorInfo(
            vendor="UFACTORY",
            model=f"xArm{self.dof}",
            dof=self.dof,
            firmware_version=None,
            serial_number=None,
        )

    def get_joint_limits(self) -> tuple[list[float], list[float]]:
        if self.dof == 7:
            lower_deg = [-360, -118, -360, -233, -360, -97, -360]
            upper_deg = [360, 118, 360, 11, 360, 180, 360]
        elif self.dof == 6:
            lower_deg = [-360, -118, -225, -11, -360, -97]
            upper_deg = [360, 118, 11, 225, 360, 180]
        else:
            lower_deg = [-360, -118, -225, -97, -360]
            upper_deg = [360, 118, 11, 180, 360]

        lower_rad = [math.radians(d) for d in lower_deg[: self.dof]]
        upper_rad = [math.radians(d) for d in upper_deg[: self.dof]]
        return (lower_rad, upper_rad)

    def get_velocity_limits(self) -> list[float]:
        max_vel_rad = math.radians(180.0)
        return [max_vel_rad] * self.dof

    def get_acceleration_limits(self) -> list[float]:
        max_acc_rad = math.radians(1145.0)
        return [max_acc_rad] * self.dof


class XArmSimDriver(BaseManipulatorDriver):
    """xArm driver backed by the MuJoCo simulation bridge."""

    def __init__(self, **kwargs: Any) -> None:
        config: dict[str, Any] = kwargs.pop("config", {})

        driver_params = [
            "dof",
            "has_gripper",
            "has_force_torque",
            "control_rate",
            "monitor_rate",
            "robot",
            "config_path",
            "headless",
        ]
        for param in driver_params:
            if param in kwargs:
                config[param] = kwargs.pop(param)

        logger.info(f"Initializing XArmSimDriver with config: {config}")

        sdk = XArmSimSDKWrapper()
        sdk.dof = int(config.get("dof", sdk.dof))
        components = [
            StandardMotionComponent(sdk),
            StandardServoComponent(sdk),
            StandardStatusComponent(sdk),
        ]

        kwargs.pop("sdk", None)
        kwargs.pop("components", None)
        kwargs.pop("name", None)

        super().__init__(
            sdk=sdk,
            components=components,
            config=config,
            name="XArmSimDriver",
            **kwargs,
        )

        logger.info("XArmSimDriver initialized successfully")


def get_blueprint() -> dict[str, Any]:
    return {
        "name": "XArmSimDriver",
        "class": XArmSimDriver,
        "config": {
            "dof": 7,
            "has_gripper": False,
            "has_force_torque": False,
            "control_rate": 100,
            "monitor_rate": 10,
            "robot": None,
            "config_path": None,
            "headless": False,
        },
        "inputs": {
            "joint_position_command": "JointCommand",
            "joint_velocity_command": "JointCommand",
        },
        "outputs": {
            "joint_state": "JointState",
            "robot_state": "RobotState",
        },
    }


xarm_sim_driver = XArmSimDriver.blueprint

__all__ = [
    "XArmSimDriver",
    "XArmSimSDKWrapper",
    "get_blueprint",
    "xarm_sim_driver",
]
