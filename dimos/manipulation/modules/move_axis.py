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
Move Axis Module

Provides directional movement capabilities for robotic manipulation.
This module exposes RPC methods for moving the end-effector in specific
directions (up, down, left, right, forward, backward) by specified distances.
"""

import logging

from dimos.core import Module, rpc
from dimos.core.rpc_client import RpcCall

logger = logging.getLogger(__name__)


class MoveAxisModule(Module):
    """Module for directional end-effector movement.

    This module provides high-level directional movement commands that use
    the ManipulationModule's get_ee_pose and move_to_pose methods to move
    the end-effector in specific directions by specified distances.

    RPC Dependencies:
        - ManipulationModule.get_ee_pose: Get current end-effector pose
        - ManipulationModule.move_to_pose: Move to target pose
    """

    # Declare RPC dependencies
    rpc_calls: list[str] = [
        "ManipulationModule.get_ee_pose",
        "ManipulationModule.move_to_pose",
    ]

    def __init__(self) -> None:
        super().__init__()
        self._get_ee_pose: RpcCall | None = None
        self._move_to_pose: RpcCall | None = None

    @rpc
    def set_ManipulationModule_get_ee_pose(self, callable: RpcCall) -> None:
        """Wire get_ee_pose RPC from ManipulationModule."""
        self._get_ee_pose = callable
        self._get_ee_pose.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ManipulationModule_move_to_pose(self, callable: RpcCall) -> None:
        """Wire move_to_pose RPC from ManipulationModule."""
        self._move_to_pose = callable
        self._move_to_pose.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def start(self) -> None:
        """Start the move axis module."""
        super().start()
        logger.info("MoveAxisModule started")

    @rpc
    def stop(self) -> None:
        """Stop the move axis module."""
        super().stop()
        logger.info("MoveAxisModule stopped")

    # =========================================================================
    # Directional Movement RPC Methods
    # =========================================================================

    @rpc
    def move_up(self, distance: float = 0.05) -> bool:
        """Move end-effector up (positive Z direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(0.0, 0.0, distance)

    @rpc
    def move_down(self, distance: float = 0.05) -> bool:
        """Move end-effector down (negative Z direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(0.0, 0.0, -distance)

    @rpc
    def move_left(self, distance: float = 0.05) -> bool:
        """Move end-effector left (positive Y direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(0.0, distance, 0.0)

    @rpc
    def move_right(self, distance: float = 0.05) -> bool:
        """Move end-effector right (negative Y direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(0.0, -distance, 0.0)

    @rpc
    def move_forward(self, distance: float = 0.05) -> bool:
        """Move end-effector forward (positive X direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(distance, 0.0, 0.0)

    @rpc
    def move_backward(self, distance: float = 0.05) -> bool:
        """Move end-effector backward (negative X direction).

        Args:
            distance: Distance to move in meters (default: 0.05m = 5cm)

        Returns:
            True if movement successful, False otherwise
        """
        return self._move_relative(-distance, 0.0, 0.0)

    def _move_relative(self, dx: float, dy: float, dz: float) -> bool:
        """Move end-effector by relative offset.

        Args:
            dx: X offset in meters
            dy: Y offset in meters
            dz: Z offset in meters

        Returns:
            True if movement successful, False otherwise
        """
        if self._get_ee_pose is None or self._move_to_pose is None:
            logger.error("ManipulationModule not connected")
            return False

        try:
            # Get current pose [x, y, z, roll, pitch, yaw]
            current_pose = self._get_ee_pose()
            if current_pose is None:
                logger.error("Could not get current end-effector pose")
                return False

            x, y, z, roll, pitch, yaw = current_pose

            # Calculate new pose
            new_x = x + dx
            new_y = y + dy
            new_z = z + dz

            logger.info(
                f"Moving from ({x:.3f}, {y:.3f}, {z:.3f}) "
                f"to ({new_x:.3f}, {new_y:.3f}, {new_z:.3f})"
            )

            # Execute movement (keep same orientation)
            success = self._move_to_pose(new_x, new_y, new_z, roll, pitch, yaw)
            return bool(success)

        except Exception as e:
            logger.error(f"Error in relative movement: {e}")
            return False


# Expose blueprint for declarative composition
move_axis_module = MoveAxisModule.blueprint

__all__ = ["MoveAxisModule", "move_axis_module"]
