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

"""G1 high-level control via WebRTC connection."""

import difflib
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.robot.unitree.g1.effectors.high_level.high_level_spec import HighLevelG1Spec
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

_ARM_COMMANDS_DOC = "\n".join(f'- "{name}": {desc}' for name, (_, desc) in _ARM_COMMANDS.items())
_MODE_COMMANDS_DOC = "\n".join(f'- "{name}": {desc}' for name, (_, desc) in _MODE_COMMANDS.items())


class G1HighLevelWebRtcConfig(ModuleConfig):
    ip: str | None = None
    connection_mode: str = "ai"


class G1HighLevelWebRtc(Module, HighLevelG1Spec):
    """G1 high-level control module using WebRTC transport.

    Wraps :class:`UnitreeWebRTCConnection` and exposes the
    :class:`HighLevelG1Spec` interface plus LLM-callable skills for
    arm gestures, movement modes, and velocity control.
    """

    cmd_vel: In[Twist]
    config: G1HighLevelWebRtcConfig

    connection: UnitreeWebRTCConnection | None

    def __init__(self, *args: Any, g: GlobalConfig = global_config, **kwargs: Any) -> None:
        super().__init__(*args, g=g, **kwargs)
        self._global_config = g

    # lifecycle

    @rpc
    def start(self) -> None:
        super().start()
        assert self.config.ip is not None, "ip must be set in G1HighLevelWebRtcConfig"
        self.connection = UnitreeWebRTCConnection(self.config.ip, self.config.connection_mode)
        self.connection.start()
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

    @rpc
    def stop(self) -> None:
        if self.connection is not None:
            self.connection.stop()
        super().stop()

    # HighLevelG1Spec

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        assert self.connection is not None
        return self.connection.move(twist, duration)

    @rpc
    def get_state(self) -> str:
        if self.connection is None:
            return "Not connected"
        return "Connected (WebRTC)"

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[str, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)  # type: ignore[no-any-return]

    @rpc
    def stand_up(self) -> bool:
        assert self.connection is not None
        return self.connection.standup()

    @rpc
    def lie_down(self) -> bool:
        assert self.connection is not None
        return self.connection.liedown()

    # skills (LLM-callable)

    @skill
    def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            move_velocity(**args)

        Args:
            x: Forward velocity (m/s)
            y: Left/right velocity (m/s)
            yaw: Rotational velocity (rad/s)
            duration: How long to move (seconds)
        """
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        """Execute a Unitree G1 arm command."""
        return self._execute_g1_command(_ARM_COMMANDS, 7106, "rt/api/arm/request", command_name)

    execute_arm_command.__doc__ = f"""Execute a Unitree G1 arm command.

        Example usage:

            execute_arm_command("ArmHeart")

        Here are all the command names and what they do.

        {_ARM_COMMANDS_DOC}
        """

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        """Execute a Unitree G1 mode command."""
        return self._execute_g1_command(_MODE_COMMANDS, 7101, "rt/api/sport/request", command_name)

    execute_mode_command.__doc__ = f"""Execute a Unitree G1 mode command.

        Example usage:

            execute_mode_command("RunMode")

        Here are all the command names and what they do.

        {_MODE_COMMANDS_DOC}
        """

    # private helpers

    def _execute_g1_command(
        self,
        command_dict: dict[str, tuple[int, str]],
        api_id: int,
        topic: str,
        command_name: str,
    ) -> str:
        if command_name not in command_dict:
            suggestions = difflib.get_close_matches(
                command_name, command_dict.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = command_dict[command_name]

        try:
            self.publish_request(topic, {"api_id": api_id, "parameter": {"data": id_}})
            return f"'{command_name}' command executed successfully."
        except Exception as e:
            logger.error(f"Failed to execute {command_name}: {e}")
            return "Failed to execute the command."


__all__ = ["G1HighLevelWebRtc", "G1HighLevelWebRtcConfig"]
