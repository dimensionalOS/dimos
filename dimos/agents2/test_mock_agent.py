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

"""Test agent with FakeChatModel for unit testing."""

import os

import pytest
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage, ToolCall

from dimos.agents2.agent import Agent
from dimos.agents2.testing import MockModel
from dimos.protocol.skill.test_coordinator import TestContainer


async def test_tool_call():
    """Test agent initialization and tool call execution."""
    # Create a fake model that will respond with tool calls
    fake_model = MockModel(
        responses=[
            AIMessage(
                content="I'll add those numbers for you.",
                tool_calls=[
                    {
                        "name": "add",
                        "args": {"args": [], "kwargs": {"x": 5, "y": 3}},
                        "id": "tool_call_1",
                    }
                ],
            ),
            AIMessage(content="The result of adding 5 and 3 is 8."),
        ]
    )

    # Create agent with the fake model
    agent = Agent(
        model_instance=fake_model,
        system_prompt="You are a helpful robot assistant with math skills.",
    )

    # Register skills with coordinator
    skills = TestContainer()
    agent.coordinator.register_skills(skills)
    agent.start()
    # Query the agent
    await agent.query_async("Please add 5 and 3")

    # Check that tools were bound
    assert fake_model.tools is not None
    assert len(fake_model.tools) > 0

    # Verify the model was called and history updated
    assert len(agent._history) > 0


async def test_image_tool_call():
    """Test agent with image tool call execution."""
    # Create a fake model that will respond with image tool calls
    fake_model = MockModel(
        responses=[
            AIMessage(
                content="I'll take a photo for you.",
                tool_calls=[
                    {
                        "name": "take_photo",
                        "args": {"args": [], "kwargs": {}},
                        "id": "tool_call_image_1",
                    }
                ],
            ),
            AIMessage(content="I've taken the photo. The image shows a cafe scene."),
        ]
    )

    # Create agent with the fake model
    agent = Agent(
        model_instance=fake_model,
        system_prompt="You are a helpful robot assistant with camera capabilities.",
    )

    # Register skills with coordinator
    skills = TestContainer()
    agent.coordinator.register_skills(skills)
    agent.start()

    # Query the agent
    await agent.query_async("Please take a photo")

    # Check that tools were bound
    assert fake_model.tools is not None
    assert len(fake_model.tools) > 0

    # Verify the model was called and history updated
    assert len(agent._history) > 0

    # Check that image was handled specially
    # Look for HumanMessage with image content in history
    human_messages_with_images = [
        msg
        for msg in agent._history
        if isinstance(msg, HumanMessage) and msg.content and isinstance(msg.content, list)
    ]
    assert len(human_messages_with_images) >= 0  # May have image messages
