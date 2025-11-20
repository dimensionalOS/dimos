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
from typing import Any, Optional
from pydantic import BaseModel
from openai import pydantic_function_tool

from dimos.types.constants import Colors

# Configure logging for the module
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

# region SkillLibrary

class SkillLibrary:

    # ==== Flat Skill Library ====

    def __init__(self):
        self.registered_skills: list["AbstractSkill"] = []
        self.class_skills: list["AbstractSkill"] = []
        self._running_skills = {}  # {skill_name: (instance, subscription)}

        self.init()
        
    def init(self):
        # Collect all skills from the parent class and update self.skills
        self.refresh_class_skills()

        # Temporary
        self.registered_skills = self.class_skills.copy()

    def get_class_skills(self) -> list["AbstractSkill"]:
        """Extract all AbstractSkill subclasses from a class.
            
        Returns:
            List of skill classes found within the class
        """
        skills = []
        
        # Loop through all attributes of the class
        for attr_name in dir(self.__class__):
            # Skip special/dunder attributes
            if attr_name.startswith('__'):
                continue
                
            try:
                attr = getattr(self.__class__, attr_name)
                
                # Check if it's a class and inherits from AbstractSkill
                if isinstance(attr, type) and issubclass(attr, AbstractSkill) and attr is not AbstractSkill:
                    skills.append(attr)
            except (AttributeError, TypeError):
                # Skip attributes that can't be accessed or aren't classes
                continue
                
        return skills

    def refresh_class_skills(self):
        self.class_skills = self.get_class_skills()

    def add(self, skill: "AbstractSkill") -> None:
        if skill not in self.registered_skills:
            self.registered_skills.append(skill)

    def get(self) -> list["AbstractSkill"]:
        return self.registered_skills.copy()

    def remove(self, skill: "AbstractSkill") -> None:
        try:
            self.registered_skills.remove(skill)
        except ValueError:
            logger.warning(f"Attempted to remove non-existent skill: {skill}")

    def clear(self) -> None:
        self.registered_skills.clear()

    def __iter__(self):
        return iter(self.registered_skills)

    def __len__(self) -> int:
        return len(self.registered_skills)

    def __contains__(self, skill: "AbstractSkill") -> bool:
        return skill in self.registered_skills
    
    def __getitem__(self, index):
        return self.registered_skills[index]
    
    # ==== Calling a Function ====

    _instances: dict[str, dict] = {} 

    def create_instance(self, name, **kwargs):
        # Key based only on the name
        key = name
        
        print(f"Preparing to create instance with name: {name} and args: {kwargs}")

        if key not in self._instances:
            # Instead of creating an instance, store the args for later use
            self._instances[key] = kwargs
            print(f"Stored args for later instance creation: {name} with args: {kwargs}")

    def call(self, name, **args):
        # Get the stored args if available; otherwise, use an empty dict
        stored_args = self._instances.get(name, {})

        # Merge the arguments with priority given to stored arguments
        complete_args = {**args, **stored_args}

        # Dynamically get the class from the module or current script
        skill_class = getattr(self, name, None)
        if skill_class is None:
            for skill in self.get():
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
    
    # ==== Tools ====

    def get_tools(self) -> Any:
        tools_json = self.get_list_of_skills_as_json(list_of_skills=self.registered_skills)
        # print(f"{Colors.YELLOW_PRINT_COLOR}Tools JSON: {tools_json}{Colors.RESET_COLOR}")
        return tools_json
    
    def get_list_of_skills_as_json(self, list_of_skills: list["AbstractSkill"]) -> list[str]:
        return list(map(pydantic_function_tool, list_of_skills))
        
    
    def register_running_skill(self, name: str, instance: Any, subscription=None):
        """
        Register a running skill with its subscription.
        
        Args:
            name: Name of the skill (will be converted to lowercase)
            instance: Instance of the running skill
            subscription: Optional subscription associated with the skill
        """
        name = name.lower()
        self._running_skills[name] = (instance, subscription)
        logger.info(f"Registered running skill: {name}")
        
    def unregister_running_skill(self, name: str):
        """
        Unregister a running skill.
        
        Args:
            name: Name of the skill to remove (will be converted to lowercase)
        
        Returns:
            True if the skill was found and removed, False otherwise
        """
        name = name.lower()
        if name in self._running_skills:
            del self._running_skills[name]
            logger.info(f"Unregistered running skill: {name}")
            return True
        return False
        
    def get_running_skills(self):
        """
        Get all running skills.
        
        Returns:
            A dictionary of running skill names and their (instance, subscription) tuples
        """
        return self._running_skills.copy()
        
    def terminate_skill(self, name: str):
        """
        Terminate a running skill.
        
        Args:
            name: Name of the skill to terminate (will be converted to lowercase)
            
        Returns:
            A message indicating whether the skill was successfully terminated
        """
        name = name.lower()
        if name in self._running_skills:
            instance, subscription = self._running_skills[name]
            
            if hasattr(instance, 'stop') and callable(instance.stop):
                try:
                    result = instance.stop()
                    logger.info(f"Stopped skill: {name}")
                    return f"Successfully terminated skill: {name}"
                except Exception as e:
                    error_msg = f"Error stopping skill {name}: {e}"
                    logger.error(error_msg)
                    return error_msg
            
            elif subscription is not None:
                try:
                    if hasattr(subscription, 'dispose') and callable(subscription.dispose):
                        subscription.dispose()
                        logger.info(f"Disposed subscription for skill: {name}")
                        self.unregister_running_skill(name)
                        return f"Successfully terminated skill: {name}"
                except Exception as e:
                    error_msg = f"Error disposing subscription for skill {name}: {e}"
                    logger.error(error_msg)
                    return error_msg
                    
            return f"Skill {name} does not have a stop method or disposable subscription"
        else:
            return f"No running skill found with name: {name}"

# endregion SkillLibrary

# region AbstractSkill

class AbstractSkill(BaseModel):

    def __init__(self, *args, **kwargs):
        print("Initializing AbstractSkill Class")
        super().__init__(*args, **kwargs)
        self._instances = {}
        self._list_of_skills = []  # Initialize the list of skills
        print(f"Instances: {self._instances}")

    def clone(self) -> "AbstractSkill":
        return AbstractSkill()
        
    
    def register_as_running(self, name: str, skill_library: SkillLibrary, subscription=None):
        """
        Register this skill as running in the skill library.
        
        Args:
            name: Name of the skill (will be converted to lowercase)
            skill_library: The skill library to register with
            subscription: Optional subscription associated with the skill
        """
        skill_library.register_running_skill(name, self, subscription)
        
    def unregister_as_running(self, name: str, skill_library: SkillLibrary):
        """
        Unregister this skill from the skill library.
        
        Args:
            name: Name of the skill to remove (will be converted to lowercase)
            skill_library: The skill library to unregister from
        """
        skill_library.unregister_running_skill(name)

    # ==== Tools ====
    def get_tools(self) -> Any:
        tools_json = self.get_list_of_skills_as_json(list_of_skills=self._list_of_skills)
        # print(f"Tools JSON: {tools_json}")
        return tools_json

    def get_list_of_skills_as_json(self, list_of_skills: list["AbstractSkill"]) -> list[str]:
        return list(map(pydantic_function_tool, list_of_skills))

# endregion AbstractSkill

# region Abstract Robot Skill

from typing import Optional, TYPE_CHECKING
if TYPE_CHECKING:
    from dimos.robot.robot import Robot
else:
    Robot = 'Robot'

class AbstractRobotSkill(AbstractSkill):

    _robot: Robot = None
    
    def __init__(self, *args, robot: Optional[Robot] = None, **kwargs):
        super().__init__(*args, **kwargs)
        self._robot = robot
        print(f"{Colors.BLUE_PRINT_COLOR}"
              f"Robot Skill Initialized with Robot: {robot}"
              f"{Colors.RESET_COLOR}")

    def set_robot(self, robot: Robot) -> None:
        """Set the robot reference for this skills instance.
        
        Args:
            robot: The robot instance to associate with these skills.
        """
        self._robot = robot

    def __call__(self):
        if self._robot is None:
            raise RuntimeError(
                f"{Colors.RED_PRINT_COLOR}"
                f"No Robot instance provided to Robot Skill: {self.__class__.__name__}"
                f"{Colors.RESET_COLOR}")
        else:
            print(f"{Colors.BLUE_PRINT_COLOR}Robot Instance provided to Robot Skill: {self.__class__.__name__}{Colors.RESET_COLOR}")
        
# endregion Abstract Robot Skill
