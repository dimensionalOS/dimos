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
Gripper Control Component for XArmDriver.

Provides RPC methods for controlling various grippers:
- Standard xArm gripper
- Bio gripper
- Vacuum gripper
- Robotiq gripper
"""

from typing import TYPE_CHECKING, Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from xarm.wrapper import XArmAPI

logger = setup_logger()


class GripperControlComponent:
    """
    Component providing gripper control RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    """

    # Type hints for attributes expected from parent class
    arm: "XArmAPI"
    config: Any  # Config dict accessed as object (dict with attribute access)

    # =========================================================================
    # Standard xArm Gripper
    # =========================================================================

    @rpc
    def set_gripper_enable(self, enable: int) -> tuple[int, str]:
        """Enable/disable gripper."""
        try:
            code = self.arm.set_gripper_enable(enable)
            return (
                code,
                f"Gripper {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_gripper_mode(self, mode: int) -> tuple[int, str]:
        """Set gripper mode (0=location mode, 1=speed mode, 2=current mode)."""
        try:
            code = self.arm.set_gripper_mode(mode)
            return (code, f"Gripper mode set to {mode}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_gripper_speed(self, speed: float) -> tuple[int, str]:
        """Set gripper speed (r/min)."""
        try:
            code = self.arm.set_gripper_speed(speed)
            return (code, f"Gripper speed set to {speed}" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_gripper_position(
        self,
        position: float,
        wait: bool = False,
        speed: float | None = None,
        timeout: float | None = None,
    ) -> tuple[int, str]:
        """
        Set gripper position.

        Args:
            position: Target position (0-850)
            wait: Wait for completion
            speed: Optional speed override
            timeout: Optional timeout for wait
        """
        try:
            code = self.arm.set_gripper_position(position, wait=wait, speed=speed, timeout=timeout)
            return (
                code,
                f"Gripper position set to {position}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def get_gripper_position(self) -> tuple[int, float | None]:
        """Get current gripper position."""
        try:
            code, position = self.arm.get_gripper_position()
            return (code, position if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_gripper_err_code(self) -> tuple[int, int | None]:
        """Get gripper error code."""
        try:
            code, err = self.arm.get_gripper_err_code()
            return (code, err if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def clean_gripper_error(self) -> tuple[int, str]:
        """Clear gripper error."""
        try:
            code = self.arm.clean_gripper_error()
            return (code, "Gripper error cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Bio Gripper
    # =========================================================================

    @rpc
    def set_bio_gripper_enable(self, enable: int, wait: bool = True) -> tuple[int, str]:
        """Enable/disable bio gripper."""
        try:
            code = self.arm.set_bio_gripper_enable(enable, wait=wait)
            return (
                code,
                f"Bio gripper {'enabled' if enable else 'disabled'}"
                if code == 0
                else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def set_bio_gripper_speed(self, speed: int) -> tuple[int, str]:
        """Set bio gripper speed (1-100)."""
        try:
            code = self.arm.set_bio_gripper_speed(speed)
            return (
                code,
                f"Bio gripper speed set to {speed}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def open_bio_gripper(
        self, speed: int = 0, wait: bool = True, timeout: float = 5
    ) -> tuple[int, str]:
        """Open bio gripper."""
        try:
            code = self.arm.open_bio_gripper(speed=speed, wait=wait, timeout=timeout)
            return (code, "Bio gripper opened" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def close_bio_gripper(
        self, speed: int = 0, wait: bool = True, timeout: float = 5
    ) -> tuple[int, str]:
        """Close bio gripper."""
        try:
            code = self.arm.close_bio_gripper(speed=speed, wait=wait, timeout=timeout)
            return (code, "Bio gripper closed" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def get_bio_gripper_status(self) -> tuple[int, int | None]:
        """Get bio gripper status."""
        try:
            code, status = self.arm.get_bio_gripper_status()
            return (code, status if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_bio_gripper_error(self) -> tuple[int, int | None]:
        """Get bio gripper error code."""
        try:
            code, error = self.arm.get_bio_gripper_error()
            return (code, error if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def clean_bio_gripper_error(self) -> tuple[int, str]:
        """Clear bio gripper error."""
        try:
            code = self.arm.clean_bio_gripper_error()
            return (code, "Bio gripper error cleared" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    # =========================================================================
    # Vacuum Gripper
    # =========================================================================

    @rpc
    def set_vacuum_gripper(self, on: int) -> tuple[int, str]:
        """Turn vacuum gripper on/off (0=off, 1=on)."""
        try:
            code = self.arm.set_vacuum_gripper(on)
            return (
                code,
                f"Vacuum gripper {'on' if on else 'off'}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def get_vacuum_gripper(self) -> tuple[int, int | None]:
        """Get vacuum gripper state."""
        try:
            code, state = self.arm.get_vacuum_gripper()
            return (code, state if code == 0 else None)
        except Exception:
            return (-1, None)

    # =========================================================================
    # Robotiq Gripper
    # =========================================================================

    @rpc
    def robotiq_reset(self) -> tuple[int, str]:
        """Reset Robotiq gripper."""
        try:
            code = self.arm.robotiq_reset()
            return (code, "Robotiq gripper reset" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def robotiq_set_activate(self, wait: bool = True, timeout: float = 3) -> tuple[int, str]:
        """Activate Robotiq gripper."""
        try:
            code = self.arm.robotiq_set_activate(wait=wait, timeout=timeout)
            return (code, "Robotiq gripper activated" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def robotiq_set_position(
        self,
        position: int,
        speed: int = 0xFF,
        force: int = 0xFF,
        wait: bool = True,
        timeout: float = 5,
    ) -> tuple[int, str]:
        """
        Set Robotiq gripper position.

        Args:
            position: Target position (0-255, 0=open, 255=closed)
            speed: Gripper speed (0-255)
            force: Gripper force (0-255)
            wait: Wait for completion
            timeout: Timeout for wait
        """
        try:
            code = self.arm.robotiq_set_position(
                position, speed=speed, force=force, wait=wait, timeout=timeout
            )
            return (
                code,
                f"Robotiq position set to {position}" if code == 0 else f"Error code: {code}",
            )
        except Exception as e:
            return (-1, str(e))

    @rpc
    def robotiq_open(
        self, speed: int = 0xFF, force: int = 0xFF, wait: bool = True, timeout: float = 5
    ) -> tuple[int, str]:
        """Open Robotiq gripper."""
        try:
            code = self.arm.robotiq_open(speed=speed, force=force, wait=wait, timeout=timeout)
            return (code, "Robotiq gripper opened" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def robotiq_close(
        self, speed: int = 0xFF, force: int = 0xFF, wait: bool = True, timeout: float = 5
    ) -> tuple[int, str]:
        """Close Robotiq gripper."""
        try:
            code = self.arm.robotiq_close(speed=speed, force=force, wait=wait, timeout=timeout)
            return (code, "Robotiq gripper closed" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def robotiq_get_status(self) -> tuple[int, dict[str, Any] | None]:
        """Get Robotiq gripper status."""
        try:
            ret = self.arm.robotiq_get_status()
            if isinstance(ret, tuple) and len(ret) >= 2:
                code = ret[0]
                if code == 0:
                    # Return status as dict if successful
                    status = {
                        "gOBJ": ret[1] if len(ret) > 1 else None,  # Object detection status
                        "gSTA": ret[2] if len(ret) > 2 else None,  # Gripper status
                        "gGTO": ret[3] if len(ret) > 3 else None,  # Go to requested position
                        "gACT": ret[4] if len(ret) > 4 else None,  # Activation status
                        "kFLT": ret[5] if len(ret) > 5 else None,  # Fault status
                        "gFLT": ret[6] if len(ret) > 6 else None,  # Fault status
                        "gPR": ret[7] if len(ret) > 7 else None,  # Requested position echo
                        "gPO": ret[8] if len(ret) > 8 else None,  # Actual position
                        "gCU": ret[9] if len(ret) > 9 else None,  # Current
                    }
                    return (code, status)
                return (code, None)
            return (-1, None)
        except Exception as e:
            logger.error(f"robotiq_get_status failed: {e}")
            return (-1, None)

    # =========================================================================
    # Lite6 Gripper
    # =========================================================================

    @rpc
    def open_lite6_gripper(self) -> tuple[int, str]:
        """Open Lite6 gripper."""
        try:
            code = self.arm.open_lite6_gripper()
            return (code, "Lite6 gripper opened" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def close_lite6_gripper(self) -> tuple[int, str]:
        """Close Lite6 gripper."""
        try:
            code = self.arm.close_lite6_gripper()
            return (code, "Lite6 gripper closed" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))

    @rpc
    def stop_lite6_gripper(self) -> tuple[int, str]:
        """Stop Lite6 gripper."""
        try:
            code = self.arm.stop_lite6_gripper()
            return (code, "Lite6 gripper stopped" if code == 0 else f"Error code: {code}")
        except Exception as e:
            return (-1, str(e))
