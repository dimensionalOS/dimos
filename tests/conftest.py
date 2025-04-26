#
#
#

"""
Pytest configuration file for the DIMOS project.

This file contains fixtures and configuration for pytest tests.
"""

import os
import sys
import pytest
from unittest import mock

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import tests.test_header


@pytest.fixture
def skill_library():
    """Fixture for a mock skill library."""
    from dimos.skills.skills import SkillLibrary
    
    skill_lib = SkillLibrary()
    return skill_lib


@pytest.fixture
def mock_robot():
    """Fixture for a mock robot."""
    robot = mock.MagicMock()
    robot.get_video_stream.return_value = mock.MagicMock()
    return robot


@pytest.fixture
def mock_agent():
    """Fixture for a mock agent."""
    agent = mock.MagicMock()
    agent.query.return_value = "Mock agent response"
    return agent


@pytest.fixture
def mock_video_frame():
    """Fixture for a mock video frame."""
    import numpy as np
    
    frame = np.zeros((100, 100, 3), dtype=np.uint8)
    
    frame[30:70, 30:70, 0] = 255  # Red square
    
    return frame
