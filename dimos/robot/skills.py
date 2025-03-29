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

import logging
from typing import Any, List, Optional, TYPE_CHECKING
from pydantic import BaseModel
from openai import pydantic_function_tool
if TYPE_CHECKING:
    from dimos.robot.robot import Robot
else:
    Robot = 'Robot'

# Configure logging for the module
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

class SkillRegistry:
    def __init__(self):
        self.skills: list["AbstractSkill"] = []

    def register_skill(self, skill: "AbstractSkill"):
        self.skills.append(skill)

    def get_skills(self) -> list["AbstractSkill"]:
        return self.skills


class AbstractSkill(BaseModel):

    _instances: dict[str, dict] = {} 

    _skill_registry: SkillRegistry = SkillRegistry()

    def __init__(self, *args, robot: Optional[Robot] = None, **kwargs):
        print("Initializing AbstractSkill Class")
        super().__init__(*args, **kwargs)
        self._instances = {}
        self._list_of_skills = []  # Initialize the list of skills
        self._robot = None
        print(f"Instances: {self._instances}")
        
        # Handle Robot() reference if AbstractSkill() is instantiated standalone with robot=Robot()
        if robot is not None:
            self.set_robot(robot)
    
    def set_robot(self, robot: Robot) -> None:
        """Set the robot reference for this skills instance and register skills with robot.
        
        Args:
            robot: The robot instance to associate with these skills.
        """
        self._robot = robot
        # Register skills with robot if not already registered
        if robot.get_skills() is None:
            robot.skills = self  # Establish bidirectional connection
    
    def create_instance(self, name, **kwargs):
        # Key based only on the name
        key = name
        
        print(f"Preparing to create instance with name: {name} and args: {kwargs}")

        if key not in self._instances:
            # Instead of creating an instance, store the args for later use
            self._instances[key] = kwargs
            print(f"Stored args for later instance creation: {name} with args: {kwargs}")

    def call_function(self, name, **args):
        # Get the stored args if available; otherwise, use an empty dict
        stored_args = self._instances.get(name, {})

        # Merge the arguments with priority given to stored arguments
        complete_args = {**args, **stored_args}

        try:
            # Dynamically get the class from the module or current script
            skill_class = getattr(self, name, None)
            if skill_class is None:
                for skill in self._skill_registry.get_skills():
                    if name == skill.__name__:
                        skill_class = skill
                        break
                if skill_class is None:
                    raise ValueError(f"Skill class not found: {name}")

            # Initialize the instance with the merged arguments
            instance = skill_class(**complete_args)
            print(f"Instance created and function called for: {name} with args: {complete_args}")
            
            # Call the instance directly
            return instance()
        except Exception as e:
            print(f"Error running function {name}: {e}")
            return f"Error running function {name}: {e}"

    def clone(self) -> "AbstractSkill":
        return AbstractSkill()

    # ==== Tools ====
    def set_list_of_skills(self, list_of_skills: list["AbstractSkill"]):
        self._list_of_skills = list_of_skills

    def get_tools(self) -> Any:
        tools_json = self.get_list_of_skills_as_json(list_of_skills=self._list_of_skills)
        # print(f"Tools JSON: {tools_json}")
        return tools_json

    def get_nested_skills(self) -> list["AbstractSkill"]:
        nested_skills = []
        for attr_name in dir(self):
            # Skip dunder attributes that cause issues
            if attr_name.startswith("__"):
                continue
            try:
                attr = getattr(self, attr_name)
            except AttributeError:
                continue
            if isinstance(attr, type) and issubclass(attr, AbstractSkill) and attr is not AbstractSkill:
                nested_skills.append(attr)

        for skill in self._skill_registry.get_skills():
            nested_skills.append(skill)
        
        return nested_skills
    
    def add_skill(self, skill: "AbstractSkill"):
        self._skill_registry.register_skill(skill)

    def add_skills(self, skills: list["AbstractSkill"]):
        for skill in skills:
            self._skill_registry.register_skill(skill)

    def get_list_of_skills_as_json(self, list_of_skills: list["AbstractSkill"]) -> list[str]:
        return list(map(pydantic_function_tool, list_of_skills))

class AbstractRobotSkill(AbstractSkill):

    _robot: Robot = None
    _GREEN_PRINT_COLOR: str = "\033[32m"
    _RESET_COLOR: str = "\033[0m"
    
    def __init__(self, *args, robot: Optional[Robot] = None, **kwargs):
        super().__init__(*args, robot=robot, **kwargs)
        self._robot = robot
        print(f"{self._GREEN_PRINT_COLOR}Robot Skill Initialized with Robot: {robot}{self._RESET_COLOR}")

    def __call__(self):
        if self._robot is None:
            raise RuntimeError(f"No Robot instance provided to Robot Skill: {self.__class__.__name__}")