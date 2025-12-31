# Copyright 2025 Dimensional Inc.
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

"""
State Query Component for XArmDriver.

Provides RPC methods for querying robot state including:
- Joint state
- Robot state
- Cartesian position
- Firmware version
"""

import threading
from typing import TYPE_CHECKING, Any

from dimos.core import rpc
from dimos.msgs.sensor_msgs import JointState, RobotState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from xarm.wrapper import XArmAPI

logger = setup_logger()


class StateQueryComponent:
    """
    Component providing state query RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    - self._joint_state_lock: threading.Lock
    - self._joint_states_: Optional[JointState]
    - self._robot_state_: Optional[RobotState]
    """

    # Type hints for attributes expected from parent class
    arm: "XArmAPI"
    config: Any  # Config dict accessed as object (dict with attribute access)
    _joint_state_lock: threading.Lock
    _joint_states_: JointState | None
    _robot_state_: RobotState | None

    @rpc
    def get_joint_state(self) -> JointState | None:
        """
        Get the current joint state (RPC method).

        Returns:
            Current JointState or None
        """
        with self._joint_state_lock:
            return self._joint_states_

    @rpc
    def get_robot_state(self) -> RobotState | None:
        """
        Get the current robot state (RPC method).

        Returns:
            Current RobotState or None
        """
        with self._joint_state_lock:
            return self._robot_state_

    @rpc
    def get_position(self) -> tuple[int, list[float] | None]:
        """
        Get TCP position [x, y, z, roll, pitch, yaw].

        Returns:
            Tuple of (code, position)
        """
        try:
            code, position = self.arm.get_position(is_radian=self.config.is_radian)
            return (code, list(position) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_position failed: {e}")
            return (-1, None)

    @rpc
    def get_version(self) -> tuple[int, str | None]:
        """Get firmware version."""
        try:
            code, version = self.arm.get_version()
            return (code, version if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_servo_angle(self) -> tuple[int, list[float] | None]:
        """Get joint angles."""
        try:
            code, angles = self.arm.get_servo_angle(is_radian=self.config.is_radian)
            return (code, list(angles) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_servo_angle failed: {e}")
            return (-1, None)

    @rpc
    def get_position_aa(self) -> tuple[int, list[float] | None]:
        """Get TCP position in axis-angle format."""
        try:
            code, position = self.arm.get_position_aa(is_radian=self.config.is_radian)
            return (code, list(position) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_position_aa failed: {e}")
            return (-1, None)

    # =========================================================================
    # Robot State Queries
    # =========================================================================

    @rpc
    def get_state(self) -> tuple[int, int | None]:
        """Get robot state (0=ready, 3=pause, 4=stop)."""
        try:
            code, state = self.arm.get_state()
            return (code, state if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_cmdnum(self) -> tuple[int, int | None]:
        """Get command queue length."""
        try:
            code, cmdnum = self.arm.get_cmdnum()
            return (code, cmdnum if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_err_warn_code(self) -> tuple[int, list[int] | None]:
        """Get error and warning codes."""
        try:
            err_warn = [0, 0]
            code = self.arm.get_err_warn_code(err_warn)
            return (code, err_warn if code == 0 else None)
        except Exception:
            return (-1, None)

    # =========================================================================
    # Force/Torque Sensor Queries
    # =========================================================================

    @rpc
    def get_ft_sensor_data(self) -> tuple[int, list[float] | None]:
        """Get force/torque sensor data [fx, fy, fz, tx, ty, tz]."""
        try:
            code, ft_data = self.arm.get_ft_sensor_data()
            return (code, list(ft_data) if code == 0 else None)
        except Exception as e:
            logger.error(f"get_ft_sensor_data failed: {e}")
            return (-1, None)

    @rpc
    def get_ft_sensor_error(self) -> tuple[int, int | None]:
        """Get FT sensor error code."""
        try:
            code, error = self.arm.get_ft_sensor_error()
            return (code, error if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_ft_sensor_mode(self) -> tuple[int, int | None]:
        """Get FT sensor application mode."""
        try:
            code, mode = self.arm.get_ft_sensor_app_get()
            return (code, mode if code == 0 else None)
        except Exception:
            return (-1, None)
