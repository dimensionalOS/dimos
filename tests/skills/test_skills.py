#
#
#

"""Tests for the skills module in the dimos package."""

import pytest
from unittest import mock

import tests.test_header

from dimos.skills.skills import AbstractRobotSkill, SkillLibrary


def test_abstract_robot_skill():
    """Test that the AbstractRobotSkill class can be subclassed."""
    class TestSkill(AbstractRobotSkill):
        def __init__(self, robot=None, **data):
            super().__init__(robot=robot, **data)
        
        def __call__(self, *args, **kwargs):
            return "TestSkill called"
    
    skill = TestSkill()
    assert isinstance(skill, AbstractRobotSkill)
    assert skill() == "TestSkill called"


def test_skill_library_creation():
    """Test that a SkillLibrary can be created."""
    skill_lib = SkillLibrary()
    assert skill_lib is not None
    assert hasattr(skill_lib, "add")
    assert hasattr(skill_lib, "create_instance")
    assert hasattr(skill_lib, "get_skill_instance")


def test_skill_library_add_skill():
    """Test that skills can be added to a SkillLibrary."""
    class TestSkill(AbstractRobotSkill):
        def __init__(self, robot=None, **data):
            super().__init__(robot=robot, **data)
        
        def __call__(self, *args, **kwargs):
            return "TestSkill called"
    
    skill_lib = SkillLibrary()
    skill_lib.add(TestSkill)
    
    assert "TestSkill" in skill_lib.available_skills
    assert skill_lib.available_skills["TestSkill"] == TestSkill


def test_skill_library_create_instance():
    """Test that skill instances can be created from a SkillLibrary."""
    class TestSkill(AbstractRobotSkill):
        def __init__(self, robot=None, test_param=None, **data):
            super().__init__(robot=robot, **data)
            self.test_param = test_param
        
        def __call__(self, *args, **kwargs):
            return f"TestSkill called with {self.test_param}"
    
    skill_lib = SkillLibrary()
    skill_lib.add(TestSkill)
    
    mock_robot = mock.MagicMock()
    
    instance = skill_lib.create_instance("TestSkill", robot=mock_robot, test_param="test_value")
    
    assert instance is not None
    assert isinstance(instance, TestSkill)
    assert instance.robot == mock_robot
    assert instance.test_param == "test_value"
    assert instance() == "TestSkill called with test_value"


def test_skill_library_get_skill_instance():
    """Test that skill instances can be retrieved from a SkillLibrary."""
    class TestSkill(AbstractRobotSkill):
        def __init__(self, robot=None, **data):
            super().__init__(robot=robot, **data)
        
        def __call__(self, *args, **kwargs):
            return "TestSkill called"
    
    skill_lib = SkillLibrary()
    skill_lib.add(TestSkill)
    
    skill_lib.create_instance("TestSkill")
    
    instance = skill_lib.get_skill_instance("TestSkill")
    
    assert instance is not None
    assert isinstance(instance, TestSkill)
    assert instance() == "TestSkill called"


def test_skill_library_get_nonexistent_skill():
    """Test that getting a nonexistent skill raises an error."""
    skill_lib = SkillLibrary()
    
    with pytest.raises(ValueError):
        skill_lib.get_skill_instance("NonexistentSkill")


def test_skill_library_create_nonexistent_skill():
    """Test that creating a nonexistent skill raises an error."""
    skill_lib = SkillLibrary()
    
    with pytest.raises(ValueError):
        skill_lib.create_instance("NonexistentSkill")
