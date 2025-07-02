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

from typing import TYPE_CHECKING, List, Optional, Tuple, Union
import time
from pydantic import Field

if TYPE_CHECKING:
    from dimos.robot.robot import Robot, MockRobot
else:
    Robot = "Robot"
    MockRobot = "MockRobot"

from dimos.skills.skills import AbstractRobotSkill, AbstractSkill, SkillLibrary
from dimos.types.constants import Colors
from dimos.types.vector import Vector
from go2_webrtc_driver.constants import RTC_TOPIC, SPORT_CMD

# Module-level constant for Unitree WebRTC control definitions
UNITREE_WEBRTC_CONTROLS: List[Tuple[str, int, str]] = [
    ("Damp", 1001, "Lowers the robot to the ground fully."),
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
        "Recovers the robot to a state from which it can take more commands. Useful to run after multiple dynamic commands like front flips.",
    ),
    (
        "Euler",
        1007,
        "Adjusts the robot's orientation using Euler angles, providing precise control over its rotation.",
    ),
    # ("Move", 1008, "Move the robot using velocity commands."),  # Handled separately
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

# region MyUnitreeSkills


class MyUnitreeSkills(SkillLibrary):
    """My Unitree Skills for WebRTC interface."""

    def __init__(self, robot: Optional[Robot] = None):
        super().__init__()
        self._robot: Robot = None

        # Add dynamic skills to this class
        dynamic_skills = self.create_skills_live()
        self.register_skills(dynamic_skills)

    @classmethod
    def register_skills(cls, skill_classes: Union["AbstractSkill", list["AbstractSkill"]]):
        """Add multiple skill classes as class attributes.

        Args:
            skill_classes: List of skill classes to add
        """
        if not isinstance(skill_classes, list):
            skill_classes = [skill_classes]

        for skill_class in skill_classes:
            # Add to the class as a skill
            setattr(cls, skill_class.__name__, skill_class)

    def initialize_skills(self):
        for skill_class in self.get_class_skills():
            self.create_instance(skill_class.__name__, robot=self._robot)

        # Refresh the class skills
        self.refresh_class_skills()

    def create_skills_live(self) -> List[AbstractRobotSkill]:
        # ================================================
        # Procedurally created skills
        # ================================================
        class BaseUnitreeSkill(AbstractRobotSkill):
            """Base skill for dynamic skill creation."""

            def __call__(self):
                string = f"{Colors.GREEN_PRINT_COLOR}This is a base skill, created for the specific skill: {self._app_id}{Colors.RESET_COLOR}"
                print(string)
                super().__call__()
                if self._app_id is None:
                    raise RuntimeError(
                        f"{Colors.RED_PRINT_COLOR}"
                        f"No App ID provided to {self.__class__.__name__} Skill"
                        f"{Colors.RESET_COLOR}"
                    )
                else:
                    # Use WebRTC publish_request interface through the robot's webrtc_connection
                    result = self._robot.webrtc_connection.publish_request(
                        RTC_TOPIC["SPORT_MOD"], {"api_id": self._app_id}
                    )
                    string = f"{Colors.GREEN_PRINT_COLOR}{self.__class__.__name__} was successful: id={self._app_id}{Colors.RESET_COLOR}"
                    print(string)
                    return string

        skills_classes = []
        for name, app_id, description in UNITREE_WEBRTC_CONTROLS:
            if name not in ["Reverse", "Spin"]:  # Exclude reverse and spin skills
                skill_class = type(
                    name,  # Name of the class
                    (BaseUnitreeSkill,),  # Base classes
                    {"__doc__": description, "_app_id": app_id},
                )
                skills_classes.append(skill_class)

        return skills_classes

    # region Class-based Skills

    class Move(AbstractRobotSkill):
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions."""

        x: float = Field(..., description="Forward velocity (m/s).")
        y: float = Field(default=0.0, description="Left/right velocity (m/s)")
        yaw: float = Field(default=0.0, description="Rotational velocity (rad/s)")
        duration: float = Field(default=0.0, description="How long to move (seconds).")

        def __call__(self):
            return self._robot.move(Vector(self.x, self.y, self.yaw), duration=self.duration)

    class Wait(AbstractSkill):
        """Wait for a specified amount of time."""

        seconds: float = Field(..., description="Seconds to wait")

        def __call__(self):
            time.sleep(self.seconds)
            return f"Wait completed with length={self.seconds}s"

    # endregion


# endregion
