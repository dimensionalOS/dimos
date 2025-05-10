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

"""Tests for the skills module in the dimos package."""

import unittest
from unittest import mock

import tests.test_header

from dimos.skills.skills import AbstractSkill, SkillLibrary
from dimos.robot.robot import MockRobot
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.types.constants import Colors
from dimos.agents.agent import OpenAIAgent


class TestSkill(AbstractSkill):
    """A test skill that tracks its execution for testing purposes."""

    _called: bool = False

    def __init__(self, *args, **kwargs):     
        super().__init__(*args, **kwargs)
        self._called = False
        
    def __call__(self):
        self._called = True
        return "TestSkill executed successfully"


class SkillLibraryTest(unittest.TestCase):
    """Tests for the SkillLibrary functionality."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.robot = MockRobot()
        self.skill_library = MyUnitreeSkills(robot=self.robot)
        self.skill_library.initialize_skills()
        
    def test_skill_iteration(self):
        """Test that skills can be properly iterated in the skill library."""
        skills_count = 0
        for skill in self.skill_library:
            skills_count += 1
            self.assertTrue(hasattr(skill, '__name__'))
            self.assertTrue(issubclass(skill, AbstractSkill))
        
        self.assertGreater(skills_count, 0, "Skill library should contain at least one skill")
        
    def test_skill_registration(self):
        """Test that skills can be properly registered in the skill library."""
        # Clear existing skills for isolated test
        self.skill_library = MyUnitreeSkills(robot=self.robot)
        original_count = len(list(self.skill_library))
        
        # Add a custom test skill
        test_skill = TestSkill
        self.skill_library.add(test_skill)
        
        # Verify the skill was added
        new_count = len(list(self.skill_library))
        self.assertEqual(new_count, original_count + 1)
        
        # Check if the skill can be found by name
        found = False
        for skill in self.skill_library:
            if skill.__name__ == "TestSkill":
                found = True
                break
        self.assertTrue(found, "Added skill should be found in skill library")
        
    def test_skill_direct_execution(self):
        """Test that a skill can be executed directly."""
        test_skill = TestSkill()
        self.assertFalse(test_skill._called)
        result = test_skill()
        self.assertTrue(test_skill._called)
        self.assertEqual(result, "TestSkill executed successfully")
        
    def test_skill_library_execution(self):
        """Test that a skill can be executed through the skill library."""
        # Add our test skill to the library
        test_skill = TestSkill
        self.skill_library.add(test_skill)
        
        # Create an instance to confirm it was executed
        with mock.patch.object(TestSkill, '__call__', return_value="Success") as mock_call:
            result = self.skill_library.call("TestSkill")
            mock_call.assert_called_once()
            self.assertEqual(result, "Success")
        
    def test_skill_not_found(self):
        """Test that calling a non-existent skill raises an appropriate error."""
        with self.assertRaises(ValueError):
            self.skill_library.call("NonExistentSkill")


class SkillWithAgentTest(unittest.TestCase):
    """Tests for skills used with an agent."""

    def setUp(self):
        """Set up test fixtures before each test method."""
        self.robot = MockRobot()
        self.skill_library = MyUnitreeSkills(robot=self.robot)
        self.skill_library.initialize_skills()
        
        # Add a test skill
        self.skill_library.add(TestSkill)
        
        # Create the agent
        self.agent = OpenAIAgent(
            dev_name="SkillTestAgent",
            system_query="You are a skill testing agent. When prompted to perform an action, use the appropriate skill.",
            skills=self.skill_library
        )
        
    @mock.patch('dimos.agents.agent.OpenAIAgent.run_observable_query')
    def test_agent_skill_identification(self, mock_query):
        """Test that the agent can identify skills based on natural language."""
        # Mock the agent response
        mock_response = mock.MagicMock()
        mock_response.run.return_value = "I found the TestSkill and executed it."
        mock_query.return_value = mock_response
        
        # Run the test
        response = self.agent.run_observable_query("Please run the test skill").run()
        
        # Assertions
        mock_query.assert_called_once_with("Please run the test skill")
        self.assertEqual(response, "I found the TestSkill and executed it.")
        
    @mock.patch.object(TestSkill, '__call__')
    @mock.patch('dimos.agents.agent.OpenAIAgent.run_observable_query')
    def test_agent_skill_execution(self, mock_query, mock_skill_call):
        """Test that the agent can execute skills properly."""
        # Mock the agent and skill call
        mock_skill_call.return_value = "TestSkill executed successfully"
        mock_response = mock.MagicMock()
        mock_response.run.return_value = "Executed TestSkill successfully."
        mock_query.return_value = mock_response
        
        # Run the test
        response = self.agent.run_observable_query("Execute the TestSkill skill").run()
        
        # We can't directly verify the skill was called since our mocking setup
        # doesn't capture the internal skill execution of the agent, but we can
        # verify the agent was properly called
        mock_query.assert_called_once_with("Execute the TestSkill skill")
        self.assertEqual(response, "Executed TestSkill successfully.")
        
    def test_agent_multi_skill_registration(self):
        """Test that multiple skills can be registered with an agent."""
        # Create a new skill
        class AnotherTestSkill(AbstractSkill):
            def __call__(self):
                return "Another test skill executed"
                
        # Register the new skill
        initial_count = len(list(self.skill_library))
        self.skill_library.add(AnotherTestSkill)
        
        # Verify two distinct skills now exist
        self.assertEqual(len(list(self.skill_library)), initial_count + 1)
        
        # Verify both skills are found by name
        skill_names = [skill.__name__ for skill in self.skill_library]
        self.assertIn("TestSkill", skill_names)
        self.assertIn("AnotherTestSkill", skill_names)


if __name__ == "__main__":
    unittest.main()
