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
Kinematics Component for PiperDriver.

Provides RPC methods for kinematic calculations including:
- Forward kinematics
"""

from typing import Any

from dimos.core import rpc
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class KinematicsComponent:
    """
    Component providing kinematics RPC methods for PiperDriver.

    This component assumes the parent class has:
    - self.piper: C_PiperInterface_V2 instance
    - self.config: PiperDriverConfig instance
    - PIPER_TO_RAD: conversion constant (0.001 degrees → radians)
    """

    # Type hints for attributes provided by parent class
    piper: Any
    config: Any

    @rpc
    def get_forward_kinematics(
        self, mode: str = "feedback"
    ) -> tuple[bool, dict[str, float] | None]:
        """
        Compute forward kinematics.

        Args:
            mode: "feedback" for current joint angles, "control" for commanded angles

        Returns:
            Tuple of (success, pose_dict) with keys: x, y, z, rx, ry, rz
        """
        try:
            fk_result = self.piper.GetFK(mode=mode)

            if fk_result is not None:
                # Convert from Piper units
                pose_dict = {
                    "x": fk_result[0] * 0.001,  # 0.001 mm → mm
                    "y": fk_result[1] * 0.001,
                    "z": fk_result[2] * 0.001,
                    "rx": fk_result[3] * 0.001 * (3.14159 / 180.0),  # → rad
                    "ry": fk_result[4] * 0.001 * (3.14159 / 180.0),
                    "rz": fk_result[5] * 0.001 * (3.14159 / 180.0),
                }
                return (True, pose_dict)
            else:
                return (False, None)

        except Exception as e:
            logger.error(f"get_forward_kinematics failed: {e}")
            return (False, None)

    @rpc
    def enable_fk_calculation(self) -> tuple[bool, str]:
        """
        Enable forward kinematics calculation.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.EnableFkCal()

            if result:
                return (True, "FK calculation enabled")
            else:
                return (False, "Failed to enable FK calculation")

        except Exception as e:
            logger.error(f"enable_fk_calculation failed: {e}")
            return (False, str(e))

    @rpc
    def disable_fk_calculation(self) -> tuple[bool, str]:
        """
        Disable forward kinematics calculation.

        Returns:
            Tuple of (success, message)
        """
        try:
            result = self.piper.DisableFkCal()

            if result:
                return (True, "FK calculation disabled")
            else:
                return (False, "Failed to disable FK calculation")

        except Exception as e:
            logger.error(f"disable_fk_calculation failed: {e}")
            return (False, str(e))
