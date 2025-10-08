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
Unitree G1 skill container for the new agents2 framework.
Dynamically generates skills for G1 humanoid robot including arm controls and movement modes.
"""

from __future__ import annotations

import datetime
import time
from typing import TYPE_CHECKING, Optional, Union

from dimos.msgs.geometry_msgs import Twist, TwistStamped, Vector3
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Reducer, Stream
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.unitree_webrtc.unitree_g1 import UnitreeG1
    from dimos.robot.unitree_webrtc.unitree_go2 import UnitreeGo2

logger = setup_logger("dimos.robot.unitree_webrtc.unitree_g1_skill_container")

# G1 Arm Actions - all use api_id 7106 on topic "rt/api/arm/request"
G1_ARM_CONTROLS = [
    ("Handshake", 27, "Perform a handshake gesture with the right hand."),
    ("HighFive", 18, "Give a high five with the right hand."),
    ("Hug", 19, "Perform a hugging gesture with both arms."),
    ("HighWave", 26, "Wave with the hand raised high."),
    ("Clap", 17, "Clap hands together."),
    ("FaceWave", 25, "Wave near the face level."),
    ("LeftKiss", 12, "Blow a kiss with the left hand."),
    ("ArmHeart", 20, "Make a heart shape with both arms overhead."),
    ("RightHeart", 21, "Make a heart gesture with the right hand."),
    ("HandsUp", 15, "Raise both hands up in the air."),
    ("XRay", 24, "Hold arms in an X-ray pose position."),
    ("RightHandUp", 23, "Raise only the right hand up."),
    ("Reject", 22, "Make a rejection or 'no' gesture."),
    ("CancelAction", 99, "Cancel any current arm action and return hands to neutral position."),
]

# G1 Movement Modes - all use api_id 7101 on topic "rt/api/sport/request"
G1_MODE_CONTROLS = [
    ("WalkMode", 500, "Switch to normal walking mode."),
    ("WalkControlWaist", 501, "Switch to walking mode with waist control."),
    ("RunMode", 801, "Switch to running mode."),
]


class UnitreeG1SkillContainer(UnitreeSkillContainer):
    """Container for Unitree G1 humanoid robot skills.

    Inherits all Go2 skills and adds G1-specific arm controls and movement modes.
    """

    def __init__(self, robot: Optional[Union[UnitreeG1, UnitreeGo2]] = None):
        """Initialize the skill container with robot reference.

        Args:
            robot: The UnitreeG1 or UnitreeGo2 robot instance
        """
        # Initialize parent class to get all base Unitree skills
        super().__init__(robot)

        # Add G1-specific skills on top
        self._generate_arm_skills()
        self._generate_mode_skills()

    def _generate_arm_skills(self):
        """Dynamically generate arm control skills from G1_ARM_CONTROLS list."""
        logger.info(f"Generating {len(G1_ARM_CONTROLS)} G1 arm control skills")

        for name, data_value, description in G1_ARM_CONTROLS:
            skill_name = self._convert_to_snake_case(name)
            self._create_arm_skill(skill_name, data_value, description, name)

    def _generate_mode_skills(self):
        """Dynamically generate movement mode skills from G1_MODE_CONTROLS list."""
        logger.info(f"Generating {len(G1_MODE_CONTROLS)} G1 movement mode skills")

        for name, data_value, description in G1_MODE_CONTROLS:
            skill_name = self._convert_to_snake_case(name)
            self._create_mode_skill(skill_name, data_value, description, name)

    def _create_arm_skill(
        self, skill_name: str, data_value: int, description: str, original_name: str
    ):
        """Create a dynamic arm control skill method with the @skill decorator.

        Args:
            skill_name: Snake_case name for the method
            data_value: The arm action data value
            description: Human-readable description
            original_name: Original CamelCase name for display
        """

        def dynamic_skill_func(self) -> str:
            """Dynamic arm skill function."""
            return self._execute_arm_command(data_value, original_name)

        # Set the function's metadata
        dynamic_skill_func.__name__ = skill_name
        dynamic_skill_func.__doc__ = description

        # Apply the @skill decorator
        decorated_skill = skill()(dynamic_skill_func)

        # Bind the method to the instance
        bound_method = decorated_skill.__get__(self, self.__class__)

        # Add it as an attribute
        setattr(self, skill_name, bound_method)

        logger.debug(f"Generated arm skill: {skill_name} (data={data_value})")

    def _create_mode_skill(
        self, skill_name: str, data_value: int, description: str, original_name: str
    ):
        """Create a dynamic movement mode skill method with the @skill decorator.

        Args:
            skill_name: Snake_case name for the method
            data_value: The mode data value
            description: Human-readable description
            original_name: Original CamelCase name for display
        """

        def dynamic_skill_func(self) -> str:
            """Dynamic mode skill function."""
            return self._execute_mode_command(data_value, original_name)

        # Set the function's metadata
        dynamic_skill_func.__name__ = skill_name
        dynamic_skill_func.__doc__ = description

        # Apply the @skill decorator
        decorated_skill = skill()(dynamic_skill_func)

        # Bind the method to the instance
        bound_method = decorated_skill.__get__(self, self.__class__)

        # Add it as an attribute
        setattr(self, skill_name, bound_method)

        logger.debug(f"Generated mode skill: {skill_name} (data={data_value})")

    # ========== Override Skills for G1 ==========

    @skill()
    def move(self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0) -> str:
        """Move the robot using direct velocity commands (G1 version with TwistStamped).

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds)
        """
        if self._robot is None:
            return "Error: Robot not connected"

        # G1 uses TwistStamped instead of Twist
        twist_stamped = TwistStamped(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self._robot.move(twist_stamped, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    # ========== Helper Methods ==========

    def _execute_arm_command(self, data_value: int, name: str) -> str:
        """Execute an arm command through WebRTC interface.

        Args:
            data_value: The arm action data value
            name: Human-readable name of the command
        """
        if self._robot is None:
            return f"Error: Robot not connected (cannot execute {name})"

        try:
            result = self._robot.connection.publish_request(
                "rt/api/arm/request", {"api_id": 7106, "parameter": {"data": data_value}}
            )
            message = f"G1 arm action {name} executed successfully (data={data_value})"
            logger.info(message)
            return message
        except Exception as e:
            error_msg = f"Failed to execute G1 arm action {name}: {e}"
            logger.error(error_msg)
            return error_msg

    def _execute_mode_command(self, data_value: int, name: str) -> str:
        """Execute a movement mode command through WebRTC interface.

        Args:
            data_value: The mode data value
            name: Human-readable name of the command
        """
        if self._robot is None:
            return f"Error: Robot not connected (cannot execute {name})"

        try:
            result = self._robot.connection.publish_request(
                "rt/api/sport/request", {"api_id": 7101, "parameter": {"data": data_value}}
            )
            message = f"G1 mode {name} activated successfully (data={data_value})"
            logger.info(message)
            return message
        except Exception as e:
            error_msg = f"Failed to execute G1 mode {name}: {e}"
            logger.error(error_msg)
            return error_msg
