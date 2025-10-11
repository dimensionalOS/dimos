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
Unitree skill container for the new agents2 framework.
Dynamically generates skills from UNITREE_WEBRTC_CONTROLS list.
"""

from __future__ import annotations

import datetime
import time
from typing import TYPE_CHECKING, Optional

from dimos.core import Module
from dimos.core.core import rpc
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Reducer, Stream
from dimos.utils.logging_config import setup_logger
from dimos.robot.unitree_webrtc.unitree_skills import UNITREE_WEBRTC_CONTROLS
from go2_webrtc_driver.constants import RTC_TOPIC

if TYPE_CHECKING:
    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_skill_container")


class UnitreeSkillContainer(Module):
    """Container for Unitree Go2 robot skills using the new framework."""

    def __init__(self, robot: Optional[UnitreeGo2] = None):
        """Initialize the skill container with robot reference.

        Args:
            robot: The UnitreeGo2 robot instance
        """
        super().__init__()
        self._robot = robot

        # Dynamically generate skills from UNITREE_WEBRTC_CONTROLS
        self._generate_unitree_skills()

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        # TODO: Do I need to clean up dynamic skills?
        super().stop()

    def _generate_unitree_skills(self):
        """Dynamically generate skills from the UNITREE_WEBRTC_CONTROLS list."""
        logger.info(f"Generating {len(UNITREE_WEBRTC_CONTROLS)} dynamic Unitree skills")

        for name, api_id, description in UNITREE_WEBRTC_CONTROLS:
            if name not in ["Reverse", "Spin"]:  # Exclude reverse and spin as in original
                # Convert CamelCase to snake_case for method name
                skill_name = self._convert_to_snake_case(name)
                self._create_dynamic_skill(skill_name, api_id, description, name)

    def _convert_to_snake_case(self, name: str) -> str:
        """Convert CamelCase to snake_case.

        Examples:
            StandUp -> stand_up
            RecoveryStand -> recovery_stand
            FrontFlip -> front_flip
        """
        result = []
        for i, char in enumerate(name):
            if i > 0 and char.isupper():
                result.append("_")
            result.append(char.lower())
        return "".join(result)

    def _create_dynamic_skill(
        self, skill_name: str, api_id: int, description: str, original_name: str
    ):
        """Create a dynamic skill method with the @skill decorator.

        Args:
            skill_name: Snake_case name for the method
            api_id: The API command ID
            description: Human-readable description
            original_name: Original CamelCase name for display
        """

        # Define the skill function
        def dynamic_skill_func(self) -> str:
            """Dynamic skill function."""
            return self._execute_sport_command(api_id, original_name)

        # Set the function's metadata
        dynamic_skill_func.__name__ = skill_name
        dynamic_skill_func.__doc__ = description

        # Apply the @skill decorator
        decorated_skill = skill()(dynamic_skill_func)

        # Bind the method to the instance
        bound_method = decorated_skill.__get__(self, self.__class__)

        # Add it as an attribute
        setattr(self, skill_name, bound_method)

        logger.debug(f"Generated skill: {skill_name} (API ID: {api_id})")

    # ========== Explicit Skills ==========

    @skill()
    def move(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            move(**args)

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds)
        """
        if self._robot is None:
            return "Error: Robot not connected"

        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self._robot.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill()
    def wait(self, seconds: float) -> str:
        """Wait for a specified amount of time.

        Args:
            seconds: Seconds to wait
        """
        time.sleep(seconds)
        return f"Wait completed with length={seconds}s"

    @skill(stream=Stream.passive, reducer=Reducer.latest)
    def current_time(self):
        """Provides current time implicitly, don't call this skill directly."""
        print("Starting current_time skill")
        while True:
            yield str(datetime.datetime.now())
            time.sleep(1)

    @skill()
    def speak(self, text: str):
        """Speak text out loud through the robot's speakers."""
        return f"This is being said aloud: {text}"

    # ========== Helper Methods ==========

    def _execute_sport_command(self, api_id: int, name: str) -> str:
        """Execute a sport command through WebRTC interface.

        Args:
            api_id: The API command ID
            name: Human-readable name of the command
        """
        if self._robot is None:
            return f"Error: Robot not connected (cannot execute {name})"

        try:
            result = self._robot.connection.publish_request(
                RTC_TOPIC["SPORT_MOD"], {"api_id": api_id}
            )
            message = f"{name} command executed successfully (id={api_id})"
            logger.info(message)
            return message
        except Exception as e:
            error_msg = f"Failed to execute {name}: {e}"
            logger.error(error_msg)
            return error_msg
