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
Kinematics Component for XArmDriver.

Provides RPC methods for kinematic calculations including:
- Forward kinematics
- Inverse kinematics
"""

from typing import TYPE_CHECKING, Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from xarm.wrapper import XArmAPI

logger = setup_logger()


class KinematicsComponent:
    """
    Component providing kinematics RPC methods for XArmDriver.

    This component assumes the parent class has:
    - self.arm: XArmAPI instance
    - self.config: XArmDriverConfig instance
    """

    # Type hints for attributes expected from parent class
    arm: "XArmAPI"
    config: Any  # Config dict accessed as object (dict with attribute access)

    @rpc
    def get_inverse_kinematics(self, pose: list[float]) -> tuple[int, list[float] | None]:
        """
        Compute inverse kinematics.

        Args:
            pose: [x, y, z, roll, pitch, yaw]

        Returns:
            Tuple of (code, joint_angles)
        """
        try:
            code, angles = self.arm.get_inverse_kinematics(
                pose, input_is_radian=self.config.is_radian, return_is_radian=self.config.is_radian
            )
            return (code, list(angles) if code == 0 else None)
        except Exception:
            return (-1, None)

    @rpc
    def get_forward_kinematics(self, angles: list[float]) -> tuple[int, list[float] | None]:
        """
        Compute forward kinematics.

        Args:
            angles: Joint angles

        Returns:
            Tuple of (code, pose)
        """
        try:
            code, pose = self.arm.get_forward_kinematics(
                angles,
                input_is_radian=self.config.is_radian,
                return_is_radian=self.config.is_radian,
            )
            return (code, list(pose) if code == 0 else None)
        except Exception:
            return (-1, None)
