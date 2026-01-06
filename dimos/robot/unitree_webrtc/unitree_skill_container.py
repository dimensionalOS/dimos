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

from __future__ import annotations

import datetime
import difflib
import time
from typing import TYPE_CHECKING

from unitree_webrtc_connect.constants import RTC_TOPIC  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Reducer, Stream
from dimos.robot.unitree_webrtc.unitree_skills import UNITREE_WEBRTC_CONTROLS
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.rpc_client import RpcCall

logger = setup_logger()


_UNITREE_COMMANDS = {
    name: (id_, description)
    for name, id_, description in UNITREE_WEBRTC_CONTROLS
    if name not in ["Reverse", "Spin"]
}


class UnitreeSkillContainer(SkillModule):
    """Container for Unitree Go2 robot skills using the new framework."""

    _move: RpcCall | None = None
    _publish_request: RpcCall | None = None

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def set_ConnectionModule_move(self, callable: RpcCall) -> None:
        self._move = callable
        self._move.set_rpc(self.rpc)  # type: ignore[arg-type]

    @rpc
    def set_ConnectionModule_publish_request(self, callable: RpcCall) -> None:
        self._publish_request = callable
        self._publish_request.set_rpc(self.rpc)  # type: ignore[arg-type]

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
        if self._move is None:
            return "Error: Robot not connected"

        twist = Twist(linear=Vector3(x, y, 0), angular=Vector3(0, 0, yaw))
        self._move(twist, duration=duration)
        return f"Started moving with velocity=({x}, {y}, {yaw}) for {duration} seconds"

    @skill()
    def wait(self, seconds: float) -> str:
        """Wait for a specified amount of time.

        Args:
            seconds: Seconds to wait
        """
        time.sleep(seconds)
        return f"Wait completed with length={seconds}s"

    @skill(stream=Stream.passive, reducer=Reducer.latest, hide_skill=True)  # type: ignore[arg-type]
    def current_time(self):  # type: ignore[no-untyped-def]
        """Provides current time implicitly, don't call this skill directly."""
        print("Starting current_time skill")
        while True:
            yield str(datetime.datetime.now())
            time.sleep(1)

    @skill()
    def execute_sport_command(self, command_name: str) -> str:
        if self._publish_request is None:
            return f"Error: Robot not connected (cannot execute {command_name})"

        if command_name not in _UNITREE_COMMANDS:
            suggestions = difflib.get_close_matches(
                command_name, _UNITREE_COMMANDS.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = _UNITREE_COMMANDS[command_name]

        try:
            self._publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": id_})
            return f"'{command_name}' command executed successfully."
        except Exception as e:
            logger.error(f"Failed to execute {command_name}: {e}")
            return "Failed to execute the command."


_commands = "\n".join(
    [f'- "{name}": {description}' for name, (_, description) in _UNITREE_COMMANDS.items()]
)

UnitreeSkillContainer.execute_sport_command.__doc__ = f"""Execute a Unitree sport command.

Example usage:

    execute_sport_command("FrontPounce")

Here are all the command names and what they do.

{_commands}
"""


unitree_skills = UnitreeSkillContainer.blueprint

__all__ = ["UnitreeSkillContainer", "unitree_skills"]
