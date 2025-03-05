from typing import List, Optional, Tuple, Type
import time
from pydantic import Field
from dimos.robot.robot import Robot
from dimos.robot.skills import AbstractSkill

# Module-level constant for Unitree ROS control definitions
UNITREE_ROS_CONTROLS: List[Tuple[str, int, str]] = [
    ("Damp", 1001, "Reduces the impact of vibrations and shocks on the robot's hardware during movement."),
    ("BalanceStand", 1002, "Activates a mode that maintains the robot in a balanced standing position, adjusting dynamically to slight movements or slopes."),
    ("StopMove", 1003, "Immediately stops all ongoing movement commands to the robot, bringing it to a stationary position."),
    ("StandUp", 1004, "Commands the robot to transition from a sitting or prone position to a standing posture."),
    ("StandDown", 1005, "Instructs the robot to move from a standing position to a sitting or prone posture."),
    ("RecoveryStand", 1006, "Engages a sequence to recover the robot to a standing position if it has fallen or been displaced."),
    ("Euler", 1007, "Adjusts the robot’s orientation using Euler angles, providing precise control over its rotation."),
    # ("Move", 1008, "Move the robot using velocity commands."),  # Intentionally omitted
    ("Sit", 1009, "Commands the robot to sit down from a standing or moving stance."),
    ("RiseSit", 1010, "Commands the robot to rise back to a standing position from a sitting posture."),
    ("SwitchGait", 1011, "Switches the robot's walking pattern or style dynamically, suitable for different terrains or speeds."),
    ("Trigger", 1012, "Triggers a specific action or custom routine programmed into the robot."),
    ("BodyHeight", 1013, "Adjusts the height of the robot’s body from the ground, useful for navigating various obstacles."),
    ("FootRaiseHeight", 1014, "Controls how high the robot lifts its feet during movement, which can be adjusted for different surfaces."),
    ("SpeedLevel", 1015, "Sets or adjusts the speed at which the robot moves, with various levels available for different operational needs."),
    ("Hello", 1016, "Performs a greeting action, which could involve a wave or other friendly gesture."),
    ("Stretch", 1017, "Engages the robot in a stretching routine to maintain or enhance mechanical flexibility."),
    ("TrajectoryFollow", 1018, "Directs the robot to follow a predefined trajectory, which could involve complex paths or maneuvers."),
    ("ContinuousGait", 1019, "Enables a mode for continuous walking or running, ideal for long-distance travel."),
    ("Content", 1020, "Displays or triggers content, which could involve audio-visual outputs or interactions."),
    ("Wallow", 1021, "Engages a playful or restful activity, possibly mimicking animal-like wallowing."),
    ("Dance1", 1022, "Performs a predefined dance routine 1, programmed for entertainment or demonstration."),
    ("Dance2", 1023, "Performs another variant of a predefined dance routine 2."),
    ("GetBodyHeight", 1024, "Retrieves the current height of the robot's body from the ground."),
    ("GetFootRaiseHeight", 1025, "Retrieves the current height at which the robot’s feet are being raised during movement."),
    ("GetSpeedLevel", 1026, "Returns the current speed level at which the robot is operating."),
    ("SwitchJoystick", 1027, "Toggles the control mode to joystick input, allowing for manual direction of the robot’s movements."),
    ("Pose", 1028, "Directs the robot to take a specific pose or stance, which could be used for tasks or performances."),
    ("Scrape", 1029, "Engages a scraping motion, possibly for clearing debris or interacting with the environment."),
    ("FrontFlip", 1030, "Executes a front flip, a complex acrobatic maneuver for demonstration or obstacle navigation."),
    ("FrontJump", 1031, "Commands the robot to perform a forward jump, useful for clearing obstacles or for show."),
    ("FrontPounce", 1032, "Initiates a pouncing movement forward, mimicking animal-like pouncing behavior."),
    ("WiggleHips", 1033, "Causes the robot to wiggle its hips, likely for a playful effect or to demonstrate agility."),
    ("GetState", 1034, "Retrieves the current operational state of the robot, including status reports or diagnostic information."),
    ("EconomicGait", 1035, "Engages a more energy-efficient walking or running mode to conserve battery life."),
    ("FingerHeart", 1036, "Performs a finger heart gesture, popular in some cultures as a friendly or loving gesture."),
    ("Handstand", 1301, "Commands the robot to perform a handstand, demonstrating balance and control."),
    ("CrossStep", 1302, "Engages the robot in a cross-stepping routine, useful for complex locomotion or dance moves."),
    ("OnesidedStep", 1303, "Commands the robot to perform a stepping motion that predominantly uses one side."),
    ("Bound", 1304, "Initiates a bounding motion, similar to a light, repetitive hopping or leaping."),
    ("LeadFollow", 1045, "Engages follow-the-leader behavior, where the robot follows a designated leader or follows a signal."),
    ("LeftFlip", 1042, "Executes a flip towards the left side, combining acrobatics with directional control."),
    ("RightFlip", 1043, "Performs a flip towards the right side, adding an acrobatic element to its repertoire."),
    ("Backflip", 1044, "Executes a backflip, a highly skilled maneuver for showing agility and control.")
]


class MyUnitreeSkills(AbstractSkill):
    """My Unitree Skills."""

    _robot: Optional[Robot] = None

    def __init__(self, robot: Optional[Robot] = None, **data):
        super().__init__(**data)
        self._robot: Robot = robot

        # Create the skills and add them to the list of skills
        self.add_skills(self.create_skills_live())
        nested_skills = self.get_nested_skills()
        self.set_list_of_skills(nested_skills)

        # Provide the robot instance to each skill
        for skill_class in nested_skills:
            print("\033[92mCreating instance for skill: {}\033[0m".format(skill_class))
            self.create_instance(skill_class.__name__, robot=robot)

    def create_skills_live(self) -> List[AbstractSkill]:
        # ================================================
        # Procedurally created skills
        # ================================================
        class BaseUnitreeSkill(AbstractSkill):
            """Base skill for dynamic skill creation."""
            _robot: Optional[Robot] = None

            def __init__(self, robot: Optional[Robot] = None, **data):
                super().__init__(**data)
                self._robot = robot

            def __call__(self):
                _GREEN_PRINT_COLOR = "\033[32m"
                _RESET_COLOR = "\033[0m"  
                string = f"{_GREEN_PRINT_COLOR}This is a base skill, created for the specific skill: {self._app_id}{_RESET_COLOR}"
                print(string)
                if self._robot is None:
                    raise RuntimeError("No Robot instance provided to {self.__class__.__name__} Skill")
                elif self._app_id is None:
                    raise RuntimeError("No App ID provided to {self.__class__.__name__} Skill")
                else:
                    self._robot.webrtc_req(api_id=self._app_id)
                    string = f"{_GREEN_PRINT_COLOR}{self.__class__.__name__} was successful: id={self._app_id}{_RESET_COLOR}"
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
                }
            )
            skills_classes.append(skill_class)
        
        return skills_classes
            
    class Move(AbstractSkill):
        """Move the robot forward using distance commands."""

        _robot: Robot = None
        _GREEN_PRINT_COLOR: str = "\033[32m"
        _RESET_COLOR: str = "\033[0m"

        distance: float = Field(..., description="Distance to move in meters")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._GREEN_PRINT_COLOR}Initializing Move Skill{self._RESET_COLOR}")
            self._robot = robot
            print(f"{self._GREEN_PRINT_COLOR}Move Skill Initialized with Robot: {self._robot}{self._RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to Move Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return self._robot.ros_control.move(distance=self.distance)
    
    class Reverse(AbstractSkill):
        """Reverse the robot using distance commands."""

        _robot: Robot = None
        _GREEN_PRINT_COLOR: str = "\033[32m"
        _RESET_COLOR: str = "\033[0m"

        distance: float = Field(..., description="Distance to reverse in meters")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._GREEN_PRINT_COLOR}Initializing Reverse Skill{self._RESET_COLOR}")
            self._robot = robot
            print(f"{self._GREEN_PRINT_COLOR}Reverse Skill Initialized with Robot: {self._robot}{self._RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to Reverse Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return self._robot.ros_control.reverse(distance=self.distance)

    class SpinLeft(AbstractSkill):
        """Spin the robot left using degree commands."""

        _robot: Robot = None
        _GREEN_PRINT_COLOR: str = "\033[32m"
        _RESET_COLOR: str = "\033[0m"

        degrees: float = Field(..., description="Distance to spin left in degrees")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._GREEN_PRINT_COLOR}Initializing SpinLeft Skill{self._RESET_COLOR}")
            self._robot = robot
            print(f"{self._GREEN_PRINT_COLOR}SpinLeft Skill Initialized with Robot: {self._robot}{self._RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to SpinLeft Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return self._robot.ros_control.spin(degrees=self.degrees) # Spinning left is positive degrees
            
    class SpinRight(AbstractSkill):
        """Spin the robot right using degree commands."""

        _robot: Robot = None
        _GREEN_PRINT_COLOR: str = "\033[32m"
        _RESET_COLOR: str = "\033[0m"

        degrees: float = Field(..., description="Distance to spin right in degrees")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._GREEN_PRINT_COLOR}Initializing SpinRight Skill{self._RESET_COLOR}")
            self._robot = robot
            print(f"{self._GREEN_PRINT_COLOR}SpinRight Skill Initialized with Robot: {self._robot}{self._RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to SpinRight Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return self._robot.ros_control.spin(degrees=-self.degrees) # Spinning right is negative degrees

    class Wait(AbstractSkill):
        """Wait for a specified amount of time."""
        
        _robot: Robot = None
        _GREEN_PRINT_COLOR: str = "\033[32m"
        _RESET_COLOR: str = "\033[0m"

        seconds: float = Field(..., description="Seconds to wait")

        def __init__(self, robot: Optional[Robot] = None, **data):
            super().__init__(**data)
            print(f"{self._GREEN_PRINT_COLOR}Initializing Wait Skill{self._RESET_COLOR}")
            self._robot = robot
            print(f"{self._GREEN_PRINT_COLOR}Wait Skill Initialized with Robot: {self._robot}{self._RESET_COLOR}")

        def __call__(self):
            if self._robot is None:
                raise RuntimeError("No Robot instance provided to SpinRight Skill")
            elif self._robot.ros_control is None:
                raise RuntimeError("No ROS control interface available for movement")
            else:
                return time.sleep(self.seconds)
                

            
        
            
   