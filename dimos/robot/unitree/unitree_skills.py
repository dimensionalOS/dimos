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

from typing import TYPE_CHECKING, List, Optional, Tuple, Type, Union
import time
from pydantic import Field

if TYPE_CHECKING:
    from dimos.robot.robot import Robot, MockRobot
else:
    Robot = 'Robot'
    MockRobot = 'MockRobot'

from dimos.skills.skills import AbstractRobotSkill, AbstractSkill, SkillLibrary
from dimos.types.constants import Colors
from inspect import signature, Parameter
from typing import Callable, Any, get_type_hints

# Module-level constant for Unitree ROS control definitions
UNITREE_ROS_CONTROLS: List[Tuple[str, int, str]] = [
    ("Damp", 1001,
     "Lowers the robot to the ground fully."
    ),
    ("BalanceStand", 1002,
     "Activates a mode that maintains the robot in a balanced standing position."
    ),
    ("StopMove", 1003,
     "Immediately stops all ongoing movement commands to the robot, bringing it to a stationary position."
    ),
    ("StandUp", 1004,
     "Commands the robot to transition from a sitting or prone position to a standing posture."
    ),
    ("StandDown", 1005,
     "Instructs the robot to move from a standing position to a sitting or prone posture."
    ),
    ("RecoveryStand", 1006,
     "Recovers the robot to a state from which it can take more commands. Useful to run after multiple dynamic commands like front flips."
    ),
    ("Euler", 1007,
     "Adjusts the robot's orientation using Euler angles, providing precise control over its rotation."
    ),
    # ("Move", 1008, "Move the robot using velocity commands."),  # Intentionally omitted
    ("Sit", 1009,
     "Commands the robot to sit down from a standing or moving stance."),
    ("RiseSit", 1010,
     "Commands the robot to rise back to a standing position from a sitting posture."
    ),
    ("SwitchGait", 1011,
     "Switches the robot's walking pattern or style dynamically, suitable for different terrains or speeds."
    ),
    ("Trigger", 1012,
     "Triggers a specific action or custom routine programmed into the robot."),
    ("BodyHeight", 1013,
     "Adjusts the height of the robot's body from the ground, useful for navigating various obstacles."
    ),
    ("FootRaiseHeight", 1014,
     "Controls how high the robot lifts its feet during movement, which can be adjusted for different surfaces."
    ),
    ("SpeedLevel", 1015,
     "Sets or adjusts the speed at which the robot moves, with various levels available for different operational needs."
    ),
    ("Hello", 1016,
     "Performs a greeting action, which could involve a wave or other friendly gesture."
    ),
    ("Stretch", 1017,
     "Engages the robot in a stretching routine."
    ),
    ("TrajectoryFollow", 1018,
     "Directs the robot to follow a predefined trajectory, which could involve complex paths or maneuvers."
    ),
    ("ContinuousGait", 1019,
     "Enables a mode for continuous walking or running, ideal for long-distance travel."
    ),
    ("Content", 1020,
     "To display or trigger when the robot is happy."
    ),
    ("Wallow", 1021,
     "The robot falls onto its back and rolls around."
    ),
    ("Dance1", 1022,
     "Performs a predefined dance routine 1, programmed for entertainment or demonstration."
    ),
    ("Dance2", 1023,
     "Performs another variant of a predefined dance routine 2."),
    ("GetBodyHeight", 1024,
     "Retrieves the current height of the robot's body from the ground."),
    ("GetFootRaiseHeight", 1025,
     "Retrieves the current height at which the robot's feet are being raised during movement."
    ),
    ("GetSpeedLevel", 1026,
     "Returns the current speed level at which the robot is operating."),
    ("SwitchJoystick", 1027,
     "Toggles the control mode to joystick input, allowing for manual direction of the robot's movements."
    ),
    ("Pose", 1028,
     "Directs the robot to take a specific pose or stance, which could be used for tasks or performances."
    ),
    ("Scrape", 1029,
     "Robot falls to its hind legs and makes scraping motions with its front legs."
    ),
    ("FrontFlip", 1030,
     "Executes a front flip, a complex and dynamic maneuver."
    ),
    ("FrontJump", 1031,
     "Commands the robot to perform a forward jump."
    ),
    ("FrontPounce", 1032,
     "Initiates a pouncing movement forward, mimicking animal-like pouncing behavior."
    ),
    ("WiggleHips", 1033,
     "Causes the robot to wiggle its hips."
    ),
    ("GetState", 1034,
     "Retrieves the current operational state of the robot, including status reports or diagnostic information."
    ),
    ("EconomicGait", 1035,
     "Engages a more energy-efficient walking or running mode to conserve battery life."
    ),
    ("FingerHeart", 1036,
     "Performs a finger heart gesture while on its hind legs."
    ),
    ("Handstand", 1301,
     "Commands the robot to perform a handstand, demonstrating balance and control."
    ),
    ("CrossStep", 1302,
     "Engages the robot in a cross-stepping routine, useful for complex locomotion or dance moves."
    ),
    ("OnesidedStep", 1303,
     "Commands the robot to perform a stepping motion that predominantly uses one side."
    ),
    ("Bound", 1304,
     "Initiates a bounding motion, similar to a light, repetitive hopping or leaping."
    ),
    ("LeadFollow", 1045,
     "Engages follow-the-leader behavior, where the robot follows a designated leader or follows a signal."
    ),
    ("LeftFlip", 1042,
     "Executes a flip towards the left side."
    ),
    ("RightFlip", 1043,
     "Performs a flip towards the right side."
    ),
    ("Backflip", 1044,
     "Executes a backflip, a complex and dynamic maneuver."
    )
]

# region MyUnitreeSkills

class MyUnitreeSkills(SkillLibrary):
    """My Unitree Skills."""

    _robot: Optional[Robot] = None

    @classmethod
    def register_skills(cls, skill_classes: Union['AbstractSkill', list['AbstractSkill']]):
        """Add multiple skill classes as class attributes.
        
        Args:
            skill_classes: List of skill classes to add
        """
        if isinstance(skill_classes, list):
            for skill_class in skill_classes:
                setattr(cls, skill_class.__name__, skill_class)
        else:
            setattr(cls, skill_classes.__name__, skill_classes)

    def __init__(self, robot: Optional[Robot] = None):
        super().__init__()
        self._robot: Robot = None

        # Add dynamic skills to this class
        self.register_skills(self.create_skills_live())

        if robot is not None:
            self._robot = robot
            self.initialize_skills()

    def initialize_skills(self):
        # Create the skills and add them to the list of skills
        self.register_skills(self.create_skills_live())

        # Provide the robot instance to each skill
        for skill_class in self:
            print(f"{Colors.GREEN_PRINT_COLOR}Creating instance for skill: {skill_class}{Colors.RESET_COLOR}")
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
                        f"{Colors.RESET_COLOR}")
                else:
                    self._robot.webrtc_req(api_id=self._app_id)
                    string = f"{Colors.GREEN_PRINT_COLOR}{self.__class__.__name__} was successful: id={self._app_id}{Colors.RESET_COLOR}"
                    print(string)
                    return string

        skills_classes = []
        for name, app_id, description in UNITREE_ROS_CONTROLS:
            skill_class = type(
                name,  # Name of the class
                (BaseUnitreeSkill,),  # Base classes
                {
                    '__doc__': description,
                    '_app_id': app_id
                })
            skills_classes.append(skill_class)

        return skills_classes

    # region Class-based Skills
    
    class Move(AbstractRobotSkill):
        """Move the robot forward using distance commands."""

        distance: float = Field(..., description="Distance to move in meters")

        def __call__(self):
            super().__call__()
            return self._robot.move(distance=self.distance)

    class Reverse(AbstractRobotSkill):
        """Reverse the robot using distance commands."""

        distance: float = Field(..., description="Distance to reverse in meters")
        
        def __call__(self):
            super().__call__()
            return self._robot.reverse(distance=self.distance)

    class SpinLeft(AbstractRobotSkill):
        """Spin the robot left using degree commands."""

        degrees: float = Field(..., description="Distance to spin left in degrees")

        def __call__(self):
            super().__call__()
            return self._robot.spin(degrees=self.degrees)  # Spinning left is positive degrees

    class SpinRight(AbstractRobotSkill):
        """Spin the robot right using degree commands."""

        degrees: float = Field(..., description="Distance to spin right in degrees")

        def __call__(self):
            super().__call__()
            return self._robot.spin(degrees=-self.degrees)  # Spinning right is negative degrees

    class MoveVel(AbstractRobotSkill):
        """Move the robot using direct velocity commands."""

        x: float = Field(..., description="Forward/backward velocity (m/s). Negative for reverse, positive for forward")
        y: float = Field(..., description="Left/right velocity (m/s)")
        yaw: float = Field(..., description="Rotational velocity (rad/s)")
        duration: float = Field(..., description="How long to move (seconds). If 0, command is continuous")

        def __call__(self):
            super().__call__()
            return self._robot.move_vel(x=self.x, y=self.y, yaw=self.yaw, duration=self.duration)

    class Wait(AbstractRobotSkill):
        """Wait for a specified amount of time."""

        seconds: float = Field(..., description="Seconds to wait")

        def __call__(self):
            super().__call__()
            return time.sleep(self.seconds)

    class FollowHuman(AbstractRobotSkill):
        """Follow a human using a camera."""

        def __call__(self):
            super().__call__()
            return self._robot.follow_human()

    class NavigateToObject(AbstractRobotSkill):
        """Navigate to an object using a camera."""

        object_name: str = Field(..., description="Object to navigate to")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(robot=robot, **data)

        def __call__(self):
            super().__call__()
            return self._robot.navigate_to(object_name=self.object_name)