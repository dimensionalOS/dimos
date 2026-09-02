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
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.robot.unitree.go2.connection_spec import GO2ConnectionSpec
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


def _goal_pose(
    current: PoseStamped, x: float, y: float, degrees: float | None, relative: bool
) -> PoseStamped:
    """Where move_to sends the robot, in the world frame."""
    euler = current.orientation.to_euler()
    if relative:
        position = current.position + current.orientation.rotate_vector(Vector3(x, y, 0))
        yaw = euler.yaw + math.radians(degrees or 0.0)
    else:
        position = Vector3(x, y, current.position.z)
        yaw = euler.yaw if degrees is None else math.radians(degrees)
    orientation = Quaternion.from_euler(Vector3(euler.roll, euler.pitch, yaw))
    return PoseStamped(position=position, orientation=orientation, frame_id="world")


def _pose_text(pose: PoseStamped) -> str:
    yaw = math.degrees(pose.orientation.to_euler().yaw)
    return f"x={pose.position.x:.2f} y={pose.position.y:.2f} heading={yaw:.0f}deg"


class UnitreeSkillContainer(Module):
    """Container for Unitree Go2 robot skills using the new framework."""

    _navigation: NavigationInterfaceSpec
    _connection: GO2ConnectionSpec

    tf: In[TFMessage]

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill
    def move_to(
        self, x: float = 0.0, y: float = 0.0, degrees: float | None = None, relative: bool = False
    ) -> str:
        """Move to a position and wait until the robot arrives or gives up.

        By default x, y are world coordinates in meters: the frame odometry and the
        lidar readouts use, +x east and +y north. degrees is the world heading to
        face on arrival (0 = +x, 90 = +y).

        With relative=True, x is forward and y is left in meters from where the
        robot stands now, and degrees is a turn relative to its current heading.

        Example calls:

            move_to(x=3.2, y=-0.5)                   # go to world (3.2, -0.5)
            move_to(x=3.2, y=-0.5, degrees=90)       # ...and end up facing north
            move_to(x=2, y=-1, relative=True)        # 2 m forward, 1 m right
            move_to(x=-1, relative=True)             # back up 1 m
            move_to(degrees=-90, relative=True)      # turn right 90 degrees in place
            move_to(y=3, degrees=90, relative=True)  # 3 m left, and face that way

        Returns the outcome and where the robot ended up, in world coordinates.
        """
        x, y = float(x), float(y)
        degrees = None if degrees is None else float(degrees)

        tf = self.tfbuffer.get("world", "base_link")
        if tf is None:
            return "Failed to get the position of the robot."

        goal = _goal_pose(tf.to_pose(), x, y, degrees, relative)
        self._navigation.set_goal(goal)
        outcome = self._wait_for_goal()

        tf = self.tfbuffer.get("world", "base_link")
        now = "unknown" if tf is None else _pose_text(tf.to_pose())
        return f"{outcome}. Robot is at {now}; goal was {_pose_text(goal)}."

    def _wait_for_goal(self, timeout: float = 100.0, settle: float = 2.0) -> str:
        """Block until the planner arrives, gives up, or `timeout` passes.

        The planner drops out of FOLLOWING_PATH for a moment every time it
        replans, so a pause only counts as the end once it has lasted `settle`
        seconds without the goal being reached.
        """
        # TODO: Improve this. This is not a nice way to do it. I should
        # subscribe to arrival/cancellation events instead.
        time.sleep(1.0)

        deadline = time.monotonic() + timeout
        idle_since: float | None = None
        while time.monotonic() < deadline:
            if self._navigation.is_goal_reached():
                return "Navigation goal reached"
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH:
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > settle:
                return "Navigation was cancelled or failed"
            time.sleep(0.1)
        return "Navigation timed out"

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
        if command_name not in _UNITREE_COMMANDS:
            suggestions = difflib.get_close_matches(
                command_name, _UNITREE_COMMANDS.keys(), n=3, cutoff=0.6
            )
            return f"There's no '{command_name}' command. Did you mean: {suggestions}"

        id_, _ = _UNITREE_COMMANDS[command_name]

        try:
            self._connection.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": id_})
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
