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

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.navigation.base import NavigationState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


UNITREE_WEBRTC_CONTROLS: list[tuple[str, int, str]] = [
    # ("Damp", 1001, "Lowers the robot to the ground fully."),
    (
        "BalanceStand",
        1002,
        "Activates a mode that maintains the robot in a balanced standing position.",
    ),
    (
        "StandUp",
        1004,
        "Commands the robot to transition from a sitting or prone position to a standing posture.",
    ),
    (
        "StandDown",
        1005,
        "Instructs the robot to move from a standing position to a sitting or prone posture.",
    ),
    (
        "RecoveryStand",
        1006,
        "Recovers the robot to a state from which it can take more commands. Useful to run after multiple dynamic commands like front flips, Must run after skills like sit and jump and standup.",
    ),
    ("Sit", 1009, "Commands the robot to sit down from a standing or moving stance."),
    (
        "RiseSit",
        1010,
        "Commands the robot to rise back to a standing position from a sitting posture.",
    ),
    (
        "SwitchGait",
        1011,
        "Switches the robot's walking pattern or style dynamically, suitable for different terrains or speeds.",
    ),
    ("Trigger", 1012, "Triggers a specific action or custom routine programmed into the robot."),
    (
        "BodyHeight",
        1013,
        "Adjusts the height of the robot's body from the ground, useful for navigating various obstacles.",
    ),
    (
        "FootRaiseHeight",
        1014,
        "Controls how high the robot lifts its feet during movement, which can be adjusted for different surfaces.",
    ),
    (
        "SpeedLevel",
        1015,
        "Sets or adjusts the speed at which the robot moves, with various levels available for different operational needs.",
    ),
    (
        "Hello",
        1016,
        "Performs a greeting action, which could involve a wave or other friendly gesture.",
    ),
    ("Stretch", 1017, "Engages the robot in a stretching routine."),
    (
        "TrajectoryFollow",
        1018,
        "Directs the robot to follow a predefined trajectory, which could involve complex paths or maneuvers.",
    ),
    (
        "ContinuousGait",
        1019,
        "Enables a mode for continuous walking or running, ideal for long-distance travel.",
    ),
    ("Content", 1020, "To display or trigger when the robot is happy."),
    ("Wallow", 1021, "The robot falls onto its back and rolls around."),
    (
        "Dance1",
        1022,
        "Performs a predefined dance routine 1, programmed for entertainment or demonstration.",
    ),
    ("Dance2", 1023, "Performs another variant of a predefined dance routine 2."),
    ("GetBodyHeight", 1024, "Retrieves the current height of the robot's body from the ground."),
    (
        "GetFootRaiseHeight",
        1025,
        "Retrieves the current height at which the robot's feet are being raised during movement.",
    ),
    (
        "GetSpeedLevel",
        1026,
        "Retrieves the current speed level setting of the robot.",
    ),
    (
        "SwitchJoystick",
        1027,
        "Switches the robot's control mode to respond to joystick input for manual operation.",
    ),
    (
        "Pose",
        1028,
        "Commands the robot to assume a specific pose or posture as predefined in its programming.",
    ),
    ("Scrape", 1029, "The robot performs a scraping motion."),
    (
        "FrontFlip",
        1030,
        "Commands the robot to perform a front flip, showcasing its agility and dynamic movement capabilities.",
    ),
    (
        "FrontJump",
        1031,
        "Instructs the robot to jump forward, demonstrating its explosive movement capabilities.",
    ),
    (
        "FrontPounce",
        1032,
        "Commands the robot to perform a pouncing motion forward.",
    ),
    (
        "WiggleHips",
        1033,
        "The robot performs a hip wiggling motion, often used for entertainment or demonstration purposes.",
    ),
    (
        "GetState",
        1034,
        "Retrieves the current operational state of the robot, including its mode, position, and status.",
    ),
    (
        "EconomicGait",
        1035,
        "Engages a more energy-efficient walking or running mode to conserve battery life.",
    ),
    ("FingerHeart", 1036, "Performs a finger heart gesture while on its hind legs."),
    (
        "Handstand",
        1301,
        "Commands the robot to perform a handstand, demonstrating balance and control.",
    ),
    (
        "CrossStep",
        1302,
        "Commands the robot to perform cross-step movements.",
    ),
    (
        "OnesidedStep",
        1303,
        "Commands the robot to perform one-sided step movements.",
    ),
    ("Bound", 1304, "Commands the robot to perform bounding movements."),
    ("MoonWalk", 1305, "Commands the robot to perform a moonwalk motion."),
    ("LeftFlip", 1042, "Executes a flip towards the left side."),
    ("RightFlip", 1043, "Performs a flip towards the right side."),
    ("Backflip", 1044, "Executes a backflip, a complex and dynamic maneuver."),
]


_UNITREE_COMMANDS = {
    name: (id_, description)
    for name, id_, description in UNITREE_WEBRTC_CONTROLS
    if name not in ["Reverse", "Spin"]
}


class UnitreeSkillContainer(Module):
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

    @skill
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
        forward, left, degrees = float(forward), float(left), float(degrees)

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

    @skill
    def wait(self, seconds: float) -> str:
        """Wait for a specified amount of time.

        Args:
            seconds: Seconds to wait
        """
        time.sleep(seconds)
        return f"Wait completed with length={seconds}s"

    @skill
    def current_time(self) -> str:
        """Provides current time."""
        return str(datetime.datetime.now())

    @skill
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
