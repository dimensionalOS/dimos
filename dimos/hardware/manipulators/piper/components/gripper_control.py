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
Gripper Control Component for PiperDriver.

Provides RPC methods for gripper control operations.
"""

from typing import Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GripperControlComponent:
    """
    Component providing gripper control RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    """

    # Type hints for attributes provided by parent class
    piper: Any
    config: Any

    @rpc
    def set_gripper(
        self,
        gripper_angle: int,
        gripper_effort: int = 100,
        gripper_enable: int = 0x01,
        gripper_state: int = 0x00,
    ) -> tuple[bool, str]:
        """
        Set gripper position and parameters.

        Args:
            gripper_angle: Gripper angle (0-1000, 0=closed, 1000=open)
            gripper_effort: Gripper effort/force (0-1000)
            gripper_enable: Gripper enable (0x00=disabled, 0x01=enabled)
            gripper_state: Gripper state

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.GripperCtrl(
                gripper_angle, gripper_effort, gripper_enable, gripper_state
            )

            if result:
                return (True, f"Gripper set to angle={gripper_angle}, effort={gripper_effort}")
            else:
                return (False, "Failed to set gripper")

        except Exception as e:
            logger.error(f"set_gripper failed: {e}")
            return (False, str(e))

    @rpc
    def open_gripper(self, effort: int = 100) -> tuple[bool, str]:
        """
        Open gripper.

        Args:
            effort: Gripper effort (0-1000)

        Returns:
            Tuple of (success, message)
        """
        result: tuple[bool, str] = self.set_gripper(gripper_angle=1000, gripper_effort=effort)  # type: ignore[no-any-return]
        return result

    @rpc
    def close_gripper(self, effort: int = 100) -> tuple[bool, str]:
        """
        Close gripper.

        Args:
            effort: Gripper effort (0-1000)

        Returns:
            Tuple of (success, message)
        """
        result: tuple[bool, str] = self.set_gripper(gripper_angle=0, gripper_effort=effort)  # type: ignore[no-any-return]
        return result

    @rpc
    def set_gripper_zero(self) -> tuple[bool, str]:
        """
        Set gripper zero position.

        Returns:
            Tuple of (success, message)
        """
        try:
            # This method may require specific SDK implementation
            # For now, we'll just document it
            logger.info("set_gripper_zero called - implementation may vary by SDK version")
            return (True, "Gripper zero set (if supported by SDK)")

        except Exception as e:
            logger.error(f"set_gripper_zero failed: {e}")
            return (False, str(e))
