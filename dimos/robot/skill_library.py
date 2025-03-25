import logging
from dimos.utils.logging_config import setup_logger

# Configure logging for the module
logger = setup_logger(__name__)

class SkillLibrary:
    def __init__(self):
        """Initializes the SkillLibrary with available skills."""
        # A dictionary to map skill names to their respective functions and descriptions
        self.skills = {
            "MoveX": {
                "function": self.MoveX,
                "description": "Moves the robot's arm along the X-axis by a specified distance."
            },
            "MoveY": {
                "function": self.MoveY,
                "description": "Moves the robot's arm along the Y-axis by a specified distance."
            },
            "GripArm": {
                "function": self.GripArm,
                "description": "Activates the robotic gripper to grasp an object."
            }
        }

    def describe_skill(self, skill_name):
        """Returns the description of a given skill.

        Args:
            skill_name (str): The name of the skill.

        Returns:
            str: A description of the skill, or an error message if the skill is not found.
        """
        skill = self.skills.get(skill_name)
        if skill:
            return skill["description"]
        else:
            return f"Skill '{skill_name}' not found."

    def describe_all_skills(self):
        """Returns descriptions of all available skills.

        Returns:
            str: A formatted string containing all skill names and their descriptions.
        """
        if not self.skills:
            return "No skills available."
        
        descriptions = "\n".join(
            f"{name}: {info['description']}" for name, info in self.skills.items()
        )
        return f"Available skills:\n{descriptions}"

    def call_skill(self, skill_name, *args, **kwargs):
        """Calls a specified skill with the provided arguments.

        Args:
            skill_name (str): The name of the skill to call.
            *args: Positional arguments for the skill function.
            **kwargs: Keyword arguments for the skill function.

        Returns:
            The result of the skill function, or an error message if the skill is not found.
        """
        skill = self.skills.get(skill_name)
        if skill:
            return skill["function"](*args, **kwargs)
        else:
            logger.error(f"Skill '{skill_name}' not found.")
            return f"Skill '{skill_name}' not found."

    def MoveX(self, distance):
        """Moves the robot's arm along the X-axis.

        Args:
            distance (float): The distance to move along the X-axis.

        Returns:
            str: A message indicating the movement.
        """
        logger.info(f"Moving along X-axis by {distance} units.")
        return f"Moved along X-axis by {distance} units."

    def MoveY(self, distance):
        """Moves the robot's arm along the Y-axis.

        Args:
            distance (float): The distance to move along the Y-axis.

        Returns:
            str: A message indicating the movement.
        """
        logger.info(f"Moving along Y-axis by {distance} units.")
        return f"Moved along Y-axis by {distance} units."

    def GripArm(self):
        """Activates the robotic gripper.

        Returns:
            str: A message indicating the gripper action.
        """
        logger.info("Gripping with robotic arm.")
        return "Gripped with robotic arm."

# Singleton instantiation will come at the robot class
# Possibly two classes: Robot skills abstract & reasoning skills / perception skills
# Reach skill will differ from object recognition skill (may need separate classes)
# NavStack.export to skill func as well as other skills will use the same export function / helper

