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

from dimos.robot.skills import AbstractRobotSkill
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.kill_skill", level=logging.INFO)

RUNNING_SKILLS_REGISTRY = {}

class KillSkill(AbstractRobotSkill):
    """
    A skill that terminates other running skills.
    
    This skill can be used to stop long-running or background skills
    like the monitor skill.
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
        
        skill_name = self.skill_name.lower()
        
        if skill_name in RUNNING_SKILLS_REGISTRY:
            skill_instance = RUNNING_SKILLS_REGISTRY[skill_name]
            
            if hasattr(skill_instance, 'stop') and callable(skill_instance.stop):
                try:
                    result = skill_instance.stop()
                    logger.info(f"Stopped skill: {skill_name}")
                    return f"Successfully terminated skill: {skill_name}"
                except Exception as e:
                    error_msg = f"Error stopping skill {skill_name}: {e}"
                    logger.error(error_msg)
                    return error_msg
            
            elif hasattr(skill_instance, 'stop_monitoring') and callable(skill_instance.stop_monitoring):
                try:
                    result = skill_instance.stop_monitoring()
                    logger.info(f"Stopped skill: {skill_name} using stop_monitoring()")
                    return f"Successfully terminated skill: {skill_name}"
                except Exception as e:
                    error_msg = f"Error stopping skill {skill_name}: {e}"
                    logger.error(error_msg)
                    return error_msg
            else:
                error_msg = f"Skill {skill_name} does not have a stop or stop_monitoring method"
                logger.error(error_msg)
                return error_msg
        else:
            return f"No running skill found with name: {skill_name}"
    
    @classmethod
    def list_running_skills(cls) -> List[str]:
        """
        List all currently running skills.
        
        Returns:
            A list of names of running skills
        """
        return list(RUNNING_SKILLS_REGISTRY.keys())

def register_running_skill(name: str, instance: Any):
    """
    Register a running skill in the global registry.
    
    Args:
        name: Name of the skill (will be converted to lowercase)
        instance: Instance of the running skill
    """
    RUNNING_SKILLS_REGISTRY[name.lower()] = instance
    logger.info(f"Registered running skill: {name}")

def unregister_running_skill(name: str):
    """
    Remove a skill from the global registry.
    
    Args:
        name: Name of the skill to remove (will be converted to lowercase)
    
    Returns:
        True if the skill was found and removed, False otherwise
    """
    name = name.lower()
    if name in RUNNING_SKILLS_REGISTRY:
        del RUNNING_SKILLS_REGISTRY[name]
        logger.info(f"Unregistered running skill: {name}")
        return True
    return False

def get_running_skills():
    """
    Get a list of all currently running skills.
    
    Returns:
        A dictionary of running skill names and their instances
    """
    return RUNNING_SKILLS_REGISTRY.copy()
