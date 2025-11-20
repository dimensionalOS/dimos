#
#
#

"""
Kill skill for terminating running skills.

This module provides a skill that can terminate other running skills,
particularly those running in separate threads like the monitor skill.
"""

import logging
from typing import Optional, Dict, Any, List
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill, SkillLibrary
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.kill_skill", level=logging.INFO)

class KillSkill(AbstractRobotSkill):
    """
    A skill that terminates other running skills.
    
    This skill can be used to stop long-running or background skills
    like the monitor skill. It uses the centralized process management
    in the SkillLibrary to track and terminate skills.
    """
    
    skill_name: str = Field(..., description="Name of the skill to terminate")
    
    def __init__(self, robot=None, **data):
        """
        Initialize the kill skill.
        
        Args:
            robot: The robot instance
            **data: Additional data for configuration
        """
        super().__init__(robot=robot, **data)
    
    def __call__(self):
        """
        Terminate the specified skill.
        
        Returns:
            A message indicating whether the skill was successfully terminated
        """
        super().__call__()
        
        skill_library = self._robot.get_skills()
        
        # Terminate the skill using the skill library
        return skill_library.terminate_skill(self.skill_name)
    
    @classmethod
    def list_running_skills(cls, skill_library: SkillLibrary) -> List[str]:
        """
        List all currently running skills.
        
        Args:
            skill_library: The skill library to get running skills from
            
        Returns:
            A list of names of running skills
        """
        return list(skill_library.get_running_skills().keys())


def get_running_skills(skill_library: SkillLibrary) -> Dict[str, tuple]:
    """
    Get all running skills from the skill library.
    
    Args:
        skill_library: The skill library to get running skills from
        
    Returns:
        A dictionary of running skill names and their (instance, subscription) tuples
    """
    return skill_library.get_running_skills()

def terminate_skill(name: str, skill_library: SkillLibrary) -> str:
    """
    Terminate a running skill using the skill library.
    
    Args:
        name: Name of the skill to terminate
        skill_library: The skill library to terminate the skill from
        
    Returns:
        A message indicating whether the skill was successfully terminated
    """
    return skill_library.terminate_skill(name)
