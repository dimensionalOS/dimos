#!/usr/bin/env python3

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
Recovery server for handling stuck detection and recovery behaviors.
"""

from dimos.msgs.geometry_msgs import PoseStamped
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import get_distance

logger = setup_logger()


class RecoveryServer:
    """
    Recovery server for detecting stuck situations and executing recovery behaviors.

    Currently implements stuck detection based on time without significant movement.
    Will be extended with actual recovery behaviors in the future.
    """

    def __init__(
        self,
        position_threshold: float = 0.2,
        stuck_duration: float = 3.0,
    ) -> None:
        """Initialize the recovery server.

        Args:
            position_threshold: Minimum distance to travel to reset stuck timer (meters)
            stuck_duration: Time duration without significant movement to consider stuck (seconds)
        """
        self.position_threshold = position_threshold
        self.stuck_duration = stuck_duration

        # Store last position that exceeded threshold
        self.last_moved_pose = None
        self.last_moved_time = None
        self.current_odom = None

        logger.info(
            f"RecoveryServer initialized with position_threshold={position_threshold}, "
            f"stuck_duration={stuck_duration}"
        )

    def update_odom(self, odom: PoseStamped) -> None:
        """Update the odometry data for stuck detection.

        Args:
            odom: Current robot odometry with timestamp
        """
        if odom is None:
            return

        # Store current odom for checking stuck
        self.current_odom = odom  # type: ignore[assignment]

        # Initialize on first update
        if self.last_moved_pose is None:
            self.last_moved_pose = odom  # type: ignore[assignment]
            self.last_moved_time = odom.ts  # type: ignore[assignment]
            return

        # Calculate distance from the reference position (last significant movement)
        distance = get_distance(odom, self.last_moved_pose)

        # If robot has moved significantly from the reference, update reference
        if distance > self.position_threshold:
            self.last_moved_pose = odom
            self.last_moved_time = odom.ts

    def check_stuck(self) -> bool:
        """Check if the robot is stuck based on time without movement.

        Returns:
            True if robot appears to be stuck, False otherwise
        """
        if self.last_moved_time is None:
            return False

        # Need current odom to check
        if self.current_odom is None:
            return False

        # Calculate time since last significant movement
        current_time = self.current_odom.ts
        time_since_movement = current_time - self.last_moved_time

        # Check if stuck based on duration without movement
        is_stuck = time_since_movement > self.stuck_duration

        if is_stuck:
            logger.warning(
                f"Robot appears stuck! No movement for {time_since_movement:.1f} seconds"
            )

        return is_stuck

    def reset(self) -> None:
        """Reset the recovery server state."""
        self.last_moved_pose = None
        self.last_moved_time = None
        self.current_odom = None
        logger.debug("RecoveryServer reset")
