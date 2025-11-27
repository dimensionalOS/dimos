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
Visual navigation skills for robot interaction.

This module provides skills for visual navigation, including following humans
and navigating to specific objects using computer vision.
"""

import logging
import threading
import time

from pydantic import Field

from dimos.perception.visual_servoing import VisualServoing  # type: ignore[import-untyped]
from dimos.skills.skills import AbstractRobotSkill
from dimos.types.vector import Vector
from dimos.utils.logging_config import setup_logger

logger = setup_logger(level=logging.DEBUG)


class FollowHuman(AbstractRobotSkill):
    """
    A skill that makes the robot follow a human using visual servoing continuously.

    This skill uses the robot's person tracking stream to follow a human
    while maintaining a specified distance. It will keep following the human
    until the timeout is reached or the skill is stopped. Don't use this skill
    if you want to navigate to a specific person, use NavigateTo instead.
    """

    distance: float = Field(
        1.5, description="Desired distance to maintain from the person in meters"
    )
    timeout: float = Field(20.0, description="Maximum time to follow the person in seconds")
    point: tuple[int, int] | None = Field(
        None, description="Optional point to start tracking (x,y pixel coordinates)"
    )

    def __init__(self, robot=None, **data) -> None:  # type: ignore[no-untyped-def]
        super().__init__(robot=robot, **data)
        self._stop_event = threading.Event()
        self._visual_servoing = None

    def __call__(self):  # type: ignore[no-untyped-def]
        """
        Start following a human using visual servoing.

        Returns:
            bool: True if successful, False otherwise
        """
        super().__call__()  # type: ignore[no-untyped-call]

        if (
            not hasattr(self._robot, "person_tracking_stream")
            or self._robot.person_tracking_stream is None
        ):
            logger.error("Robot does not have a person tracking stream")
            return False

        # Stop any existing operation
        self.stop()
        self._stop_event.clear()

        success = False

        try:
            # Initialize visual servoing
            self._visual_servoing = VisualServoing(
                tracking_stream=self._robot.person_tracking_stream
            )

            logger.warning(f"Following human for {self.timeout} seconds...")
            start_time = time.time()

            # Start tracking
            track_success = self._visual_servoing.start_tracking(  # type: ignore[attr-defined]
                point=self.point, desired_distance=self.distance
            )

            if not track_success:
                logger.error("Failed to start tracking")
                return False

            # Main follow loop
            while (
                self._visual_servoing.running  # type: ignore[attr-defined]
                and time.time() - start_time < self.timeout
                and not self._stop_event.is_set()
            ):
                output = self._visual_servoing.updateTracking()  # type: ignore[attr-defined]
                x_vel = output.get("linear_vel")
                z_vel = output.get("angular_vel")
                logger.debug(f"Following human: x_vel: {x_vel}, z_vel: {z_vel}")
                self._robot.move(Vector(x_vel, 0, z_vel))  # type: ignore[arg-type, attr-defined]
                time.sleep(0.05)

            # If we completed the full timeout duration, consider it success
            if time.time() - start_time >= self.timeout:
                success = True
                logger.info("Human following completed successfully")
            elif self._stop_event.is_set():
                logger.info("Human following stopped externally")
            else:
                logger.info("Human following stopped due to tracking loss")

            return success

        except Exception as e:
            logger.error(f"Error in follow human: {e}")
            return False
        finally:
            # Clean up
            if self._visual_servoing:
                self._visual_servoing.stop_tracking()
                self._visual_servoing = None

    def stop(self) -> bool:
        """
        Stop the human following process.

        Returns:
            bool: True if stopped, False if it wasn't running
        """
        if self._visual_servoing is not None:
            logger.info("Stopping FollowHuman skill")
            self._stop_event.set()

            # Clean up visual servoing if it exists
            self._visual_servoing.stop_tracking()
            self._visual_servoing = None

            return True
        return False
