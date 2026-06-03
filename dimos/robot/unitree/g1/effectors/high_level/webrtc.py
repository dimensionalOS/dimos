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
from dimos.robot.unitree.g1.effectors.high_level.high_level_spec import HighLevelG1Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1HighLevelWebRtcConfig(ModuleConfig):
    ip: str | None = None
    connection_mode: str = "ai"


class G1HighLevelWebRtc(Module, HighLevelG1Spec):
    """G1 high-level control module using WebRTC transport."""

    cmd_vel: In[Twist]
    config: G1HighLevelWebRtcConfig

    connection: UnitreeWebRTCConnection | None

    def __init__(self, *args: Any, g: GlobalConfig = global_config, **kwargs: Any) -> None:
        super().__init__(*args, g=g, **kwargs)
        self._global_config = g

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

    @skill
    def move_velocity(
        self, x: float, y: float = 0.0, yaw: float = 0.0, duration: float = 0.0
    ) -> str:
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions.

        Example call:
            args = { "x": 0.5, "y": 0.0, "yaw": 0.0, "duration": 2.0 }
            move_velocity(**args)
        """
        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self.move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill
    def execute_arm_command(self, command_name: str) -> str:
        """Execute a Unitree G1 arm command."""
        return execute_g1_command(
            self.publish_request, ARM_COMMANDS, ARM_API_ID, ARM_TOPIC, command_name, logger=logger
        )

    execute_arm_command.__doc__ = f"""Execute a Unitree G1 arm command.

        Example usage:

            execute_arm_command("ArmHeart")

        Here are all the command names and what they do.

        {ARM_COMMANDS_DOC}
        """

    @skill
    def execute_mode_command(self, command_name: str) -> str:
        """Execute a Unitree G1 mode command."""
        return execute_g1_command(
            self.publish_request,
            MODE_COMMANDS,
            MODE_API_ID,
            MODE_TOPIC,
            command_name,
            logger=logger,
        )

    execute_mode_command.__doc__ = f"""Execute a Unitree G1 mode command.

        Example usage:

            execute_mode_command("RunMode")

        Here are all the command names and what they do.

        {MODE_COMMANDS_DOC}
        """


__all__ = ["G1HighLevelWebRtc", "G1HighLevelWebRtcConfig"]
