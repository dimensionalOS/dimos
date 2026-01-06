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
Unitree G1 skill container for the new agents framework.
Dynamically generates skills for G1 humanoid robot including arm controls and movement modes.
"""

import difflib

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.protocol.skill.skill import skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

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

_ARM_COMMANDS: dict[str, tuple[int, str]] = {
    name: (id_, description) for name, id_, description in G1_ARM_CONTROLS
}

_MODE_COMMANDS: dict[str, tuple[int, str]] = {
    name: (id_, description) for name, id_, description in G1_MODE_CONTROLS
}


class UnitreeG1SkillContainer(SkillModule):
    rpc_calls: list[str] = [
        "G1ConnectionModule.move",
        "G1ConnectionModule.publish_request",
    ]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

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

        move_rpc = self.get_rpc_calls("G1ConnectionModule.move")
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        move_rpc(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill()
    def execute_arm_command(self, command_name: str) -> str:
        return self._execute_g1_command(_ARM_COMMANDS, 7106, command_name)

    @skill()
    def execute_mode_command(self, command_name: str) -> str:
        return self._execute_g1_command(_MODE_COMMANDS, 7101, command_name)

    def _execute_g1_command(
        self, command_dict: dict[str, tuple[int, str]], api_id: int, command_name: str
    ) -> str:
        publish_request_rpc = self.get_rpc_calls("G1ConnectionModule.publish_request")

        if command_name not in command_dict:
            suggestions = difflib.get_close_matches(
                command_name, command_dict.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = command_dict[command_name]

        try:
            publish_request_rpc(
                "rt/api/sport/request", {"api_id": api_id, "parameter": {"data": id_}}
            )
            return f"'{command_name}' command executed successfully."
        except Exception as e:
            logger.error(f"Failed to execute {command_name}: {e}")
            return "Failed to execute the command."


_arm_commands = "\n".join(
    [f'- "{name}": {description}' for name, (_, description) in _ARM_COMMANDS.items()]
)

UnitreeG1SkillContainer.execute_arm_command.__doc__ = f"""Execute a Unitree G1 arm command.

Example usage:

    execute_arm_command("ArmHeart")

Here are all the command names and what they do.

{_arm_commands}
"""

_mode_commands = "\n".join(
    [f'- "{name}": {description}' for name, (_, description) in _MODE_COMMANDS.items()]
)

UnitreeG1SkillContainer.execute_mode_command.__doc__ = f"""Execute a Unitree G1 mode command.

Example usage:

    execute_mode_command("RunMode")

Here are all the command names and what they do.

{_mode_commands}
"""

g1_skills = UnitreeG1SkillContainer.blueprint

__all__ = ["UnitreeG1SkillContainer", "g1_skills"]
