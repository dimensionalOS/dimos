#
#
#

"""
Monitor skill for an agent.

This module provides a compatibility wrapper around ObserveStream skill
for backward compatibility with tests.
"""

import logging
from typing import Optional, Any, Dict
from pydantic import Field

from dimos.skills.skills import AbstractRobotSkill
from dimos.skills.observe_stream import ObserveStream
from dimos.agents.agent import LLMAgent
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.skills.monitor_skill")

class MonitorSkill(ObserveStream):
    """
    A compatibility wrapper around ObserveStream for backward compatibility.
    
    This class inherits from ObserveStream and provides the same functionality
    but with a different name for backward compatibility with tests.
    """
    
    def __init__(self, robot=None, claude_agent: Optional[LLMAgent] = None, **data):
        """
        Initialize the MonitorSkill.
        
        Args:
            robot: The robot instance
            claude_agent: The agent to send queries to (renamed from agent for compatibility)
            **data: Additional data for configuration
        """
        agent = claude_agent
        super().__init__(robot=robot, agent=agent, **data)
        logger.info("MonitorSkill initialized (compatibility wrapper around ObserveStream)")
