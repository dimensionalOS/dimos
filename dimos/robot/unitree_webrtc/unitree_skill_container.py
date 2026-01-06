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

from __future__ import annotations

import datetime
import difflib
import math
import time

from unitree_webrtc_connect.constants import RTC_TOPIC

from dimos.core.core import rpc
from dimos.core.skill_module import SkillModule
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation.base import NavigationState
from dimos.protocol.skill.skill import skill
from dimos.protocol.skill.type import Reducer, Stream
from dimos.robot.unitree_webrtc.unitree_skills import UNITREE_WEBRTC_CONTROLS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


_UNITREE_COMMANDS = {
    name: (id_, description)
    for name, id_, description in UNITREE_WEBRTC_CONTROLS
    if name not in ["Reverse", "Spin"]
}


class UnitreeSkillContainer(SkillModule):
    """Container for Unitree Go2 robot skills using the new framework."""

    rpc_calls: list[str] = [
        "NavigationInterface.set_goal",
        "NavigationInterface.get_state",
        "NavigationInterface.is_goal_reached",
        "NavigationInterface.cancel_goal",
        "GO2Connection.publish_request",
    ]

    @rpc
    def start(self) -> None:
        super().start()
        # Initialize TF early so it can start receiving transforms.
        _ = self.tf

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill()
    def relative_move(self, forward: float = 0.0, left: float = 0.0, degrees: float = 0.0) -> str:
        """Move the robot relative to its current position.

        The `degrees` arguments refers to the rotation the robot should be at the end, relative to its current rotation.

        Example calls:

            # Move to a point that's 2 meters forward and 1 to the right.
            relative_move(forward=2, left=-1, degrees=0)

            # Move back 1 meter, while still facing the same direction.
            relative_move(forward=-1, left=0, degrees=0)

            # Rotate 90 degrees to the right (in place)
            relative_move(forward=0, left=0, degrees=-90)

            # Move 3 meters left, and face that direction
            relative_move(forward=0, left=3, degrees=90)
        """

        tf = self.tf.get("world", "base_link")
        if tf is None:
            return "Failed to get the position of the robot."

        try:
            set_goal_rpc, get_state_rpc, is_goal_reached_rpc = self.get_rpc_calls(
                "NavigationInterface.set_goal",
                "NavigationInterface.get_state",
                "NavigationInterface.is_goal_reached",
            )
        except Exception:
            logger.error("Navigation module not connected properly")
            return "Failed to connect to navigation module."

        # TODO: Improve this. This is not a nice way to do it. I should
        # subscribe to arrival/cancellation events instead.

        set_goal_rpc(self._generate_new_goal(tf.to_pose(), forward, left, degrees))

        time.sleep(1.0)

        start_time = time.monotonic()
        timeout = 100.0
        while get_state_rpc() == NavigationState.FOLLOWING_PATH:
            if time.monotonic() - start_time > timeout:
                return "Navigation timed out"
            time.sleep(0.1)

        time.sleep(1.0)

        if not is_goal_reached_rpc():
            return "Navigation was cancelled or failed"
        else:
            return "Navigation goal reached"

    def _generate_new_goal(
        self, current_pose: PoseStamped, forward: float, left: float, degrees: float
    ) -> PoseStamped:
        local_offset = Vector3(forward, left, 0)
        global_offset = current_pose.orientation.rotate_vector(local_offset)
        goal_position = current_pose.position + global_offset

        current_euler = current_pose.orientation.to_euler()
        goal_yaw = current_euler.yaw + math.radians(degrees)
        goal_euler = Vector3(current_euler.roll, current_euler.pitch, goal_yaw)
        goal_orientation = Quaternion.from_euler(goal_euler)

        return PoseStamped(position=goal_position, orientation=goal_orientation)

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
        try:
            publish_request = self.get_rpc_calls("GO2Connection.publish_request")
        except Exception:
            logger.error("GO2Connection not connected properly")
            return "Failed to connect to GO2Connection."

        if command_name not in _UNITREE_COMMANDS:
            suggestions = difflib.get_close_matches(
                command_name, _UNITREE_COMMANDS.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = _UNITREE_COMMANDS[command_name]

        try:
            publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": id_})
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
