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

"""Complete mock test for CerebrasAgent with proper MockRobot setup."""

import unittest
from unittest import mock
import tests.test_header
from typing import Optional

from dimos.robot.robot import MockRobot
from dimos.robot.unitree.unitree_skills import MyUnitreeSkills
from dimos.agents.cerebras_agent import CerebrasAgent, CerebrasResponseMessage
from dimos.skills.skills import AbstractSkill
from pydantic import Field


class MockSkill(AbstractSkill):
    """A mock skill for testing."""
    
    execution_count: int = Field(default=0, description="Number of times skill has been executed")
    last_params: Optional[dict] = Field(default=None, description="Last parameters passed to skill")
    
    # Class-level tracking to persist across instances for testing
    _class_execution_count: int = 0
    _class_last_params: Optional[dict] = None

    def __call__(self, **kwargs):
        # Update instance values
        self.execution_count += 1
        self.last_params = kwargs
        
        # Update class-level tracking for tests
        MockSkill._class_execution_count += 1
        MockSkill._class_last_params = kwargs
        
        return f"MockSkill executed with params: {kwargs}"
    
    @classmethod
    def reset_tracking(cls):
        """Reset class-level tracking for clean tests."""
        cls._class_execution_count = 0
        cls._class_last_params = None


class TestCerebrasAgentMock(unittest.TestCase):
    """Complete mock test suite for CerebrasAgent with MockRobot."""

    def setUp(self):
        """Set up test fixtures with proper MockRobot integration."""
        # Reset MockSkill tracking for clean tests
        MockSkill.reset_tracking()
        
        # Create mock robot (properly initialized)
        self.robot = MockRobot()
        
        # Create skill library with the robot
        self.skill_library = MyUnitreeSkills(robot=self.robot)
        self.skill_library.initialize_skills()
        
        # Add our custom mock skill
        self.mock_skill = MockSkill
        self.skill_library.add(self.mock_skill)
        
        # Create skill instance with robot reference
        self.skill_library.create_instance("MockSkill", robot=self.robot)
        
        # Create agent with the skill library
        self.agent = CerebrasAgent(
            dev_name="MockTestAgent",
            system_query="You are a test agent. Use the MockSkill when asked.",
            skills=self.skill_library,
        )

    def get_skill_instance(self, name):
        """Helper method to create skill instances since get_instance doesn't exist."""
        # Get the stored args if available; otherwise, use an empty dict
        stored_args = self.skill_library._instances.get(name, {})

        # Find the skill class
        skill_class = None
        for skill in self.skill_library.get():
            if name == skill.__name__:
                skill_class = skill
                break
        
        if skill_class is None:
            raise ValueError(f"Skill class not found: {name}")

        # Initialize the instance with the stored arguments
        return skill_class(**stored_args)

    @mock.patch("cerebras.cloud.sdk.Cerebras")
    def test_complete_interaction_with_mockrobot(self, mock_cerebras_client):
        """Test a complete interaction flow with MockRobot and real agent logic."""
        # Create mock Cerebras client
        mock_client_instance = mock.MagicMock()
        mock_cerebras_client.return_value = mock_client_instance
        
        # Create mock response for tool call
        mock_tool_call = mock.MagicMock()
        mock_tool_call.id = "call_123"
        mock_tool_call.function.name = "MockSkill"
        mock_tool_call.function.arguments = '{"param1": "test_value"}'
        
        # First response with tool call
        mock_response_1 = mock.MagicMock()
        mock_response_1.choices = [mock.MagicMock()]
        mock_response_1.choices[0].message.content = "I'll execute the MockSkill"
        mock_response_1.choices[0].message.tool_calls = [mock_tool_call]
        
        # Second response after tool execution
        mock_response_2 = mock.MagicMock()
        mock_response_2.choices = [mock.MagicMock()]
        mock_response_2.choices[0].message.content = "Task completed successfully!"
        mock_response_2.choices[0].message.tool_calls = None
        
        # Set up client to return our mock responses
        mock_client_instance.chat.completions.create.side_effect = [
            mock_response_1,
            mock_response_2
        ]
        
        # Replace the client in our agent
        self.agent.client = mock_client_instance
        
        # Execute the test
        response = self.agent.run_observable_query("Execute MockSkill with param1=test_value").run()
        
        # Verify the response
        self.assertEqual(response, "Task completed successfully!")
        
        # Verify the skill was executed with correct parameters
        skill_instance = self.get_skill_instance("MockSkill")
        self.assertEqual(MockSkill._class_execution_count, 1)
        self.assertEqual(MockSkill._class_last_params, {"param1": "test_value"})

    def test_mockrobot_integration(self):
        """Test that MockRobot is properly integrated with skills."""
        # Verify robot is properly set up
        self.assertIsInstance(self.robot, MockRobot)
        self.assertIsNotNone(self.robot.skill_library)
        
        # Verify skill library has the robot reference
        self.assertEqual(self.skill_library._robot, self.robot)
        
        # Verify skill instance can be created
        skill_instance = self.get_skill_instance("MockSkill")
        self.assertIsInstance(skill_instance, MockSkill)

    @mock.patch("cerebras.cloud.sdk.Cerebras")
    def test_multi_tooling_with_mockrobot(self, mock_cerebras_client):
        """Test multi-tooling capability with MockRobot."""
        # Setup mock client
        mock_client_instance = mock.MagicMock()
        mock_cerebras_client.return_value = mock_client_instance
        
        # Create tool calls for multi-tooling test
        mock_tool_call_1 = mock.MagicMock()
        mock_tool_call_1.id = "call_1"
        mock_tool_call_1.function.name = "MockSkill"
        mock_tool_call_1.function.arguments = '{"step": 1}'
        
        mock_tool_call_2 = mock.MagicMock()
        mock_tool_call_2.id = "call_2"
        mock_tool_call_2.function.name = "MockSkill"
        mock_tool_call_2.function.arguments = '{"step": 2}'
        
        # Response sequence for multi-tooling
        responses = [
            # First response: tool call 1
            self._create_mock_response("Executing step 1", [mock_tool_call_1]),
            # Second response: tool call 2  
            self._create_mock_response("Executing step 2", [mock_tool_call_2]),
            # Final response: no more tools
            self._create_mock_response("All steps completed!", None)
        ]
        
        mock_client_instance.chat.completions.create.side_effect = responses
        self.agent.client = mock_client_instance
        
        # Execute multi-tool test
        response = self.agent.run_observable_query("Execute MockSkill in multiple steps").run()
        
        # Verify multi-tooling worked
        self.assertEqual(response, "All steps completed!")
        skill_instance = self.get_skill_instance("MockSkill")
        self.assertEqual(MockSkill._class_execution_count, 2)

    @mock.patch("cerebras.cloud.sdk.Cerebras")
    def test_infinite_loop_protection(self, mock_cerebras_client):
        """Test that infinite loop protection works."""
        # Setup mock client
        mock_client_instance = mock.MagicMock()
        mock_cerebras_client.return_value = mock_client_instance
        
        # Create a tool call that would loop infinitely
        mock_tool_call = mock.MagicMock()
        mock_tool_call.id = "call_loop"
        mock_tool_call.function.name = "MockSkill"
        mock_tool_call.function.arguments = '{"loop": "forever"}'
        
        # Create response that always has tool calls (would loop forever)
        loop_response = self._create_mock_response("Looping...", [mock_tool_call])
        
        # Set up client to always return the looping response
        mock_client_instance.chat.completions.create.return_value = loop_response
        self.agent.client = mock_client_instance
        
        # Execute test - should stop due to max iterations
        response = self.agent.run_observable_query("Start infinite loop").run()
        
        # Verify it stopped with the safety message
        self.assertEqual(response, "Tool execution stopped due to maximum iteration limit.")

    def _create_mock_response(self, content, tool_calls):
        """Helper to create mock Cerebras responses."""
        mock_response = mock.MagicMock()
        mock_response.choices = [mock.MagicMock()]
        mock_response.choices[0].message.content = content
        mock_response.choices[0].message.tool_calls = tool_calls
        return mock_response

    def test_cerebras_response_message_serialization(self):
        """Test that CerebrasResponseMessage serializes properly."""
        # Create a mock tool call
        mock_tool_call = mock.MagicMock()
        mock_tool_call.id = "call_123"
        mock_tool_call.function.name = "test_function"
        mock_tool_call.function.arguments = '{"test": "value"}'
        
        # Create CerebrasResponseMessage
        response_msg = CerebrasResponseMessage(
            content="Test content",
            tool_calls=[mock_tool_call]
        )
        
        # Test serialization
        serialized = response_msg.to_dict()
        self.assertEqual(serialized["role"], "assistant")
        self.assertEqual(serialized["content"], "Test content")
        self.assertEqual(len(serialized["tool_calls"]), 1)
        self.assertEqual(serialized["tool_calls"][0]["id"], "call_123")
        
        # Test that it inherits from dict
        self.assertIsInstance(response_msg, dict)

    @mock.patch("cerebras.cloud.sdk.Cerebras")
    def test_error_handling_with_mockrobot(self, mock_cerebras_client):
        """Test error handling in the agent with MockRobot."""
        # Setup mock client to raise an exception
        mock_client_instance = mock.MagicMock()
        mock_cerebras_client.return_value = mock_client_instance
        mock_client_instance.chat.completions.create.side_effect = Exception("API Error")
        
        self.agent.client = mock_client_instance
        
        # Test error handling
        with self.assertRaises(Exception) as context:
            self.agent.run_observable_query("Execute MockSkill").run()
        
        self.assertEqual(str(context.exception), "API Error")

    def test_skill_library_integration(self):
        """Test that skill library is properly integrated."""
        # Verify skill library setup
        self.assertIsNotNone(self.agent.skill_library)
        self.assertEqual(self.agent.skill_library, self.skill_library)
        
        # Verify tools are available
        tools = self.skill_library.get_tools()
        self.assertIsNotNone(tools)
        self.assertGreater(len(tools), 0)
        
        # Verify MockSkill is in the tools
        skill_names = [tool["function"]["name"] for tool in tools]
        self.assertIn("MockSkill", skill_names)


if __name__ == "__main__":
    unittest.main() 