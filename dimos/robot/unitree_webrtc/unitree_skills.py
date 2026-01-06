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

import time
from typing import TYPE_CHECKING

from pydantic import Field

if TYPE_CHECKING:
    from dimos.robot.robot import MockRobot, Robot  # type: ignore[attr-defined]
else:
    Robot = "Robot"
    MockRobot = "MockRobot"

from unitree_webrtc_connect.constants import RTC_TOPIC  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.skills.skills import AbstractRobotSkill, AbstractSkill, SkillLibrary
from dimos.types.constants import Colors

# Module-level constant for Unitree Go2 WebRTC control definitions
UNITREE_WEBRTC_CONTROLS: list[tuple[str, int, str]] = [
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

# Module-level constants for Unitree G1 WebRTC control definitions
# G1 Arm Actions - all use api_id 7106 on topic "rt/api/arm/request"
G1_ARM_CONTROLS: list[tuple[str, int, str]] = [
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
G1_MODE_CONTROLS: list[tuple[str, int, str]] = [
    ("WalkMode", 500, "Switch to normal walking mode."),
    ("WalkControlWaist", 501, "Switch to walking mode with waist control."),
    ("RunMode", 801, "Switch to running mode."),
]

# region MyUnitreeSkills


class MyUnitreeSkills(SkillLibrary):
    """My Unitree Skills for WebRTC interface."""

    def __init__(self, robot: Robot | None = None, robot_type: str = "go2") -> None:
        """Initialize Unitree skills library.

        Args:
            robot: Optional robot instance
            robot_type: Type of robot ("go2" or "g1"), defaults to "go2"
        """
        super().__init__()
        self._robot: Robot = None  # type: ignore[assignment]
        self.robot_type = robot_type.lower()

        if self.robot_type not in ["go2", "g1"]:
            raise ValueError(f"Unsupported robot type: {robot_type}. Must be 'go2' or 'g1'")

        # Add dynamic skills to this class based on robot type
        dynamic_skills = self.create_skills_live()
        self.register_skills(dynamic_skills)  # type: ignore[arg-type]

    @classmethod
    def register_skills(cls, skill_classes: AbstractSkill | list[AbstractSkill]) -> None:
        """Add multiple skill classes as class attributes.

        Args:
            skill_classes: List of skill classes to add
        """
        if not isinstance(skill_classes, list):
            skill_classes = [skill_classes]

        for skill_class in skill_classes:
            # Add to the class as a skill
            setattr(cls, skill_class.__name__, skill_class)  # type: ignore[attr-defined]

    def initialize_skills(self) -> None:
        for skill_class in self.get_class_skills():
            self.create_instance(skill_class.__name__, robot=self._robot)  # type: ignore[attr-defined]

        # Refresh the class skills
        self.refresh_class_skills()

    def create_skills_live(self) -> list[AbstractRobotSkill]:
        # ================================================
        # Procedurally created skills
        # ================================================
        class BaseUnitreeSkill(AbstractRobotSkill):
            """Base skill for dynamic skill creation."""

            def __call__(self) -> str:
                super().__call__()  # type: ignore[no-untyped-call]

                # For Go2: Simple api_id based call
                if hasattr(self, "_app_id"):
                    string = f"{Colors.GREEN_PRINT_COLOR}Executing Go2 skill: {self.__class__.__name__} with api_id={self._app_id}{Colors.RESET_COLOR}"
                    print(string)
                    self._robot.connection.publish_request(  # type: ignore[attr-defined]
                        RTC_TOPIC["SPORT_MOD"], {"api_id": self._app_id}
                    )
                    return f"{self.__class__.__name__} executed successfully"

                # For G1: Fixed api_id with parameter data
                elif hasattr(self, "_data_value"):
                    string = f"{Colors.GREEN_PRINT_COLOR}Executing G1 skill: {self.__class__.__name__} with data={self._data_value}{Colors.RESET_COLOR}"
                    print(string)
                    self._robot.connection.publish_request(  # type: ignore[attr-defined]
                        self._topic,  # type: ignore[attr-defined]
                        {"api_id": self._api_id, "parameter": {"data": self._data_value}},  # type: ignore[attr-defined]
                    )
                    return f"{self.__class__.__name__} executed successfully"
                else:
                    raise RuntimeError(
                        f"Skill {self.__class__.__name__} missing required attributes"
                    )

        skills_classes = []

        if self.robot_type == "g1":
            # Create G1 arm skills
            for name, data_value, description in G1_ARM_CONTROLS:
                skill_class = type(
                    name,
                    (BaseUnitreeSkill,),
                    {
                        "__doc__": description,
                        "_topic": "rt/api/arm/request",
                        "_api_id": 7106,
                        "_data_value": data_value,
                    },
                )
                skills_classes.append(skill_class)

            # Create G1 mode skills
            for name, data_value, description in G1_MODE_CONTROLS:
                skill_class = type(
                    name,
                    (BaseUnitreeSkill,),
                    {
                        "__doc__": description,
                        "_topic": "rt/api/sport/request",
                        "_api_id": 7101,
                        "_data_value": data_value,
                    },
                )
                skills_classes.append(skill_class)
        else:
            # Go2 skills (existing code)
            for name, app_id, description in UNITREE_WEBRTC_CONTROLS:
                if name not in ["Reverse", "Spin"]:  # Exclude reverse and spin skills
                    skill_class = type(
                        name, (BaseUnitreeSkill,), {"__doc__": description, "_app_id": app_id}
                    )
                    skills_classes.append(skill_class)

        return skills_classes  # type: ignore[return-value]

    # region Class-based Skills

    class Move(AbstractRobotSkill):
        """Move the robot using direct velocity commands. Determine duration required based on user distance instructions."""

        x: float = Field(..., description="Forward velocity (m/s).")
        y: float = Field(default=0.0, description="Left/right velocity (m/s)")
        yaw: float = Field(default=0.0, description="Rotational velocity (rad/s)")
        duration: float = Field(default=0.0, description="How long to move (seconds).")

        def __call__(self) -> str:
            self._robot.move(  # type: ignore[attr-defined]
                Twist(linear=Vector3(self.x, self.y, 0.0), angular=Vector3(0.0, 0.0, self.yaw)),
                duration=self.duration,
            )
            return f"started moving with velocity={self.x}, {self.y}, {self.yaw} for {self.duration} seconds"

    class Wait(AbstractSkill):
        """Wait for a specified amount of time."""

        seconds: float = Field(..., description="Seconds to wait")

        def __call__(self) -> str:
            time.sleep(self.seconds)
            return f"Wait completed with length={self.seconds}s"

    # endregion


# endregion
