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

import difflib
import math
from collections.abc import Callable
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs import Twist, Vector3
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

_MIN_SPEED_MPS = 0.05
_MAX_SPEED_MPS = 0.5
_DEFAULT_YAW_RATE_RADPS = 0.8
_MAX_STAIR_STEPS = 30


class UnitreeG1SkillContainer(Module):
    rpc_calls: list[str] = [
        "G1ConnectionBase.move",
        "G1ConnectionBase.publish_request",
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

        move_rpc = self.get_rpc_calls("G1ConnectionBase.move")
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        move_rpc(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        return self._execute_g1_command(_ARM_COMMANDS, 7106, "rt/api/arm/request", command_name)

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        return self._execute_g1_command(_MODE_COMMANDS, 7101, "rt/api/sport/request", command_name)

    @skill
    def navigate_3d(self, x: float, y: float, z: float, speed: float = 0.25) -> str:
        """Navigate in 3D by first yaw-aligning, then moving in the XY plane, then handling elevation.

        This helper uses the existing low-level `move` RPC and is designed to work in both
        simulation and real robot stacks. Elevation (`z`) is handled by a dedicated
        stair-climbing sequence to keep behavior explicit and safe.

        Args:
            x: Forward target offset in meters in the robot frame.
            y: Left target offset in meters in the robot frame.
            z: Vertical target offset in meters. Positive means moving up.
            speed: Nominal linear speed in m/s used to convert distance into duration.
        """
        move_rpc = self.get_rpc_calls("G1ConnectionBase.move")
        safe_speed = min(max(speed, _MIN_SPEED_MPS), _MAX_SPEED_MPS)

        # Step 1: yaw align to the 2D target direction.
        yaw = 0.0 if (abs(x) < 1e-3 and abs(y) < 1e-3) else float(math.atan2(y, x))
        if abs(yaw) > 1e-2:
            yaw_rate = math.copysign(_DEFAULT_YAW_RATE_RADPS, yaw)
            move_rpc(
                Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(0.0, 0.0, yaw_rate)),
                duration=abs(yaw) / _DEFAULT_YAW_RATE_RADPS,
            )

        # Step 2: move in XY.
        planar_distance = (x**2 + y**2) ** 0.5
        if planar_distance > 1e-3:
            duration = planar_distance / safe_speed
            move_rpc(
                Twist(linear=Vector3(safe_speed if x >= 0 else -safe_speed, 0.0, 0.0), angular=Vector3()),
                duration=duration,
            )

        # Step 3: if height change requested, run stair traversal routine.
        if abs(z) > 1e-3:
            self._run_stair_sequence(move_rpc, z, safe_speed=safe_speed)

        return (
            f"3D navigation completed to offset ({x:.2f}, {y:.2f}, {z:.2f}) "
            f"with nominal speed {safe_speed:.2f} m/s."
        )

    @skill
    def climb_stairs(self, steps: int, step_height: float = 0.17, speed: float = 0.2) -> str:
        """Climb or descend stairs in simulation-safe incremental segments.

        Args:
            steps: Number of steps to traverse. Positive climbs up; negative goes down.
            step_height: Height of one stair step in meters.
            speed: Forward speed in m/s for each stair segment.
        """
        if steps == 0:
            return "No stair traversal requested because steps=0."
        if abs(steps) > _MAX_STAIR_STEPS:
            return f"Refusing stair traversal: steps exceeds safety limit of {_MAX_STAIR_STEPS}."
        if not 0.05 <= step_height <= 0.25:
            return "Refusing stair traversal: step_height must be within [0.05, 0.25] meters."

        move_rpc = self.get_rpc_calls("G1ConnectionBase.move")
        safe_speed = min(max(speed, _MIN_SPEED_MPS), _MAX_SPEED_MPS)
        self._run_stair_sequence(move_rpc, z=steps * step_height, safe_speed=safe_speed)
        direction = "up" if steps > 0 else "down"
        return f"Completed stair traversal {direction} for {abs(steps)} steps."

    def _run_stair_sequence(self, move_rpc: Callable[..., Any], z: float, safe_speed: float) -> None:
        # Segment elevation motion into short chunks for smoother simulation and easier interruption.
        step_count = max(1, int(round(abs(z) / 0.17)))
        segment_distance = max(abs(z) * 1.2 / step_count, 0.25)
        signed_speed = safe_speed if z >= 0 else -safe_speed
        for _ in range(step_count):
            move_rpc(
                Twist(linear=Vector3(signed_speed, 0.0, 0.0), angular=Vector3()),
                duration=segment_distance / max(safe_speed, _MIN_SPEED_MPS),
            )

    def _execute_g1_command(
        self,
        command_dict: dict[str, tuple[int, str]],
        api_id: int,
        topic: str,
        command_name: str,
    ) -> str:
        publish_request_rpc = self.get_rpc_calls("G1ConnectionBase.publish_request")

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
