#!/usr/bin/env python3
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

"""G1 MuJoCo-specific skill container and agentic skill bundle.

The legacy ``UnitreeG1SkillContainer`` references ``G1Connection`` RPC calls
which only exist when the *hardware* connection module is deployed.  In MuJoCo
simulation the connection module is ``G1SimConnection``, so we provide a
dedicated container that wires to the correct RPC endpoints.
"""

import difflib

from dimos.agents.annotation import skill
from dimos.agents.skills.navigation import NavigationSkillContainer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.agents.skills.speak_skill import SpeakSkill
from dimos.agents.web_human_input import WebInput
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.g1.legacy.skill_container import (
    _ARM_COMMANDS,
    _MODE_COMMANDS,
)
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1MujocoSkillContainer(Module):
    """Skill container for G1 MuJoCo simulation.

    Wires to ``G1SimConnection.move`` / ``G1SimConnection.publish_request``
    instead of the hardware ``G1Connection`` used in the legacy container.
    Arm and mode commands are forwarded but are no-ops in the simulator.
    """

    rpc_calls: list[str] = [
        "G1SimConnection.move",
        "G1SimConnection.publish_request",
    ]

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
        move_rpc = self.get_rpc_calls("G1SimConnection.move")
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        move_rpc(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        return self._execute_g1_command(_ARM_COMMANDS, 7106, "rt/api/arm/request", command_name)

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        return self._execute_g1_command(_MODE_COMMANDS, 7101, "rt/api/sport/request", command_name)

    def _execute_g1_command(
        self,
        command_dict: dict[str, tuple[int, str]],
        api_id: int,
        topic: str,
        command_name: str,
    ) -> str:
        publish_request_rpc = self.get_rpc_calls("G1SimConnection.publish_request")

        if command_name not in command_dict:
            suggestions = difflib.get_close_matches(
                command_name, command_dict.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = command_dict[command_name]

        try:
            publish_request_rpc(topic, {"api_id": api_id, "parameter": {"data": id_}})
            return f"'{command_name}' command executed successfully."
        except Exception as e:
            logger.error(f"Failed to execute {command_name}: {e}")
            return "Failed to execute the command."


# Copy docstrings from the legacy container definitions
_arm_commands = "\n".join(
    [f'- "{name}": {description}' for name, (_, description) in _ARM_COMMANDS.items()]
)

G1MujocoSkillContainer.execute_arm_command.__doc__ = f"""Execute a Unitree G1 arm command.

Example usage:

    execute_arm_command("ArmHeart")

Here are all the command names and what they do.

{_arm_commands}
"""

_mode_commands = "\n".join(
    [f'- "{name}": {description}' for name, (_, description) in _MODE_COMMANDS.items()]
)

G1MujocoSkillContainer.execute_mode_command.__doc__ = f"""Execute a Unitree G1 mode command.

Example usage:

    execute_mode_command("RunMode")

Here are all the command names and what they do.

{_mode_commands}
"""

g1_mujoco_skills = G1MujocoSkillContainer.blueprint

_mujoco_agentic_skills = autoconnect(
    NavigationSkillContainer.blueprint(),
    PersonFollowSkillContainer.blueprint(camera_info=MujocoConnection.camera_info_static),
    g1_mujoco_skills(),
    WebInput.blueprint(),
    SpeakSkill.blueprint(),
)

__all__ = ["G1MujocoSkillContainer", "_mujoco_agentic_skills", "g1_mujoco_skills"]
