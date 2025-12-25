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
Motion Control Component for XArmDriver.

Provides RPC methods for motion control operations including:
- Joint position control
- Joint velocity control
- Cartesian position control
- Home positioning
"""

import math
import threading
from typing import TYPE_CHECKING, Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from xarm.wrapper import XArmAPI

logger = setup_logger()


class MotionControlComponent:
    """
    Component providing motion control RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    - self._joint_cmd_lock: threading.Lock
    - self._joint_cmd_: Optional[list[float]]
    """

    # Type hints for attributes expected from parent class
    arm: "XArmAPI"
    config: Any  # Config dict accessed as object (dict with attribute access)
    _joint_cmd_lock: threading.Lock
    _joint_cmd_: list[float] | None

    @rpc
    def set_joint_angles(self, angles: list[float]) -> tuple[int, str]:
        """
        Set joint angles (RPC method).

        Args:
            angles: List of joint angles (in radians if is_radian=True)

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_servo_angle_j(angles=angles, is_radian=self.config.is_radian)
            msg = "Success" if code == 0 else f"Error code: {code}"
            return (code, msg)
        except Exception as e:
            logger.error(f"set_joint_angles failed: {e}")
            return (-1, str(e))

    @rpc
    def set_joint_velocities(self, velocities: list[float]) -> tuple[int, str]:
        """
        Set joint velocities (RPC method).
        Note: Requires velocity control mode.

        Args:
            velocities: List of joint velocities (rad/s)

        Returns:
            Tuple of (code, message)
        """
        try:
            # For velocity control, you would use vc_set_joint_velocity
            # This requires mode 4 (joint velocity control)
            code = self.arm.vc_set_joint_velocity(
                speeds=velocities, is_radian=self.config.is_radian
            )
            msg = "Success" if code == 0 else f"Error code: {code}"
            return (code, msg)
        except Exception as e:
            logger.error(f"set_joint_velocities failed: {e}")
            return (-1, str(e))

    @rpc
    def set_position(self, position: list[float], wait: bool = False) -> tuple[int, str]:
        """
        Set TCP position [x, y, z, roll, pitch, yaw].

        Args:
            position: Target position
            wait: Wait for motion to complete

        Returns:
            Tuple of (code, message)
        """
        try:
            code = self.arm.set_position(*position, is_radian=self.config.is_radian, wait=wait)
            return (code, "Success" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def move_gohome(self, wait: bool = False) -> tuple[int, str]:
        """Move to home position."""
        try:
            code = self.arm.move_gohome(wait=wait, is_radian=self.config.is_radian)
            return (code, "Moving home" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_joint_command(self, positions: list[float]) -> tuple[int, str]:
        """
        Manually set the joint command (for testing).
        This updates the shared joint_cmd that the control loop reads.

        Args:
            positions: List of joint positions in radians

        Returns:
            Tuple of (code, message)
        """
        try:
            if len(positions) != self.config.num_joints:
                return (-1, f"Expected {self.config.num_joints} positions, got {len(positions)}")

            with self._joint_cmd_lock:
                self._joint_cmd_ = list(positions)

            logger.info(f"✓ Joint command set: {[f'{math.degrees(p):.2f}°' for p in positions]}")
            return (0, "Joint command updated")
        except Exception as e:
            return (-1, str(e))
