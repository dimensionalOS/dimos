# Copyright 2025-2026 Dimensional Inc.
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
Unitree G1 skill container for the new agents framework.
Dynamically generates skills for G1 humanoid robot including arm controls and movement modes.
"""

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.connection_spec import G1ConnectionSpec
from dimos.robot.unitree.g1.effectors.high_level.commands import (
    ARM_API_ID,
    ARM_COMMANDS,
    ARM_COMMANDS_DOC,
    ARM_TOPIC,
    MODE_API_ID,
    MODE_COMMANDS,
    MODE_COMMANDS_DOC,
    MODE_TOPIC,
    execute_g1_command,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class UnitreeG1SkillContainer(Module):
    _connection: G1ConnectionSpec

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
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

        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        if not self._connection.move(twist, duration=duration):
            return "Movement command was rejected."
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        """Execute a supported stock G1 arm gesture."""
        return execute_g1_command(
            self._connection.publish_request,
            ARM_COMMANDS,
            ARM_API_ID,
            ARM_TOPIC,
            command_name,
            logger=logger,
        )

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        """Select a supported stock G1 movement mode."""
        return execute_g1_command(
            self._connection.publish_request,
            MODE_COMMANDS,
            MODE_API_ID,
            MODE_TOPIC,
            command_name,
            logger=logger,
        )


UnitreeG1SkillContainer.execute_arm_command.__doc__ = (
    "Execute a supported stock G1 arm gesture.\n\n" + ARM_COMMANDS_DOC
)
UnitreeG1SkillContainer.execute_mode_command.__doc__ = (
    "Select a supported stock G1 movement mode.\n\n" + MODE_COMMANDS_DOC
)
