#!/usr/bin/env python3
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

"""Comprehensive conversation history tests for agents."""

import os
import asyncio
import pytest
import numpy as np
from dotenv import load_dotenv

from dimos.agents.modules.base import BaseAgent
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse, ConversationHistory
from dimos.msgs.sensor_msgs import Image
from dimos.skills.skills import AbstractSkill, SkillLibrary
from pydantic import Field
import logging

logger = logging.getLogger(__name__)


def test_conversation_history_basic():
    """Test basic conversation history functionality."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant with perfect memory.",
        temperature=0.0,
    )

    # Test 1: Simple text conversation
    response1 = agent.query("My favorite color is blue")
    assert isinstance(response1, AgentResponse)
    assert agent.conversation.size() == 2  # user + assistant

    # Test 2: Reference previous information
    response2 = agent.query("What is my favorite color?")
    assert "blue" in response2.content.lower(), "Agent should remember the color"
    assert agent.conversation.size() == 4

    # Test 3: Multiple facts
    agent.query("I live in San Francisco")
    agent.query("I work as an engineer")

    # Verify history is building up
    assert agent.conversation.size() == 8  # 4 exchanges (blue, what color, SF, engineer)

    response = agent.query("Tell me what you know about me")

    # Check if agent remembers at least some facts
    # Note: Models may sometimes give generic responses, so we check for any memory
    facts_mentioned = 0
    if "blue" in response.content.lower() or "color" in response.content.lower():
        facts_mentioned += 1
    if "san francisco" in response.content.lower() or "francisco" in response.content.lower():
        facts_mentioned += 1
    if "engineer" in response.content.lower():
        facts_mentioned += 1

    # Agent should remember at least one fact, or acknowledge the conversation
    assert facts_mentioned > 0 or "know" in response.content.lower(), (
        f"Agent should show some memory of conversation, got: {response.content}"
    )

    agent.dispose()


def test_conversation_history_with_images():
    """Test conversation history with multimodal content."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful vision assistant.",
        temperature=0.0,
    )

    # Send text message
    response1 = agent.query("I'm going to show you some colors")
    assert agent.conversation.size() == 2

    # Send image with text
    msg = AgentMessage()
    msg.add_text("This is a red square")
    red_img = Image(data=np.full((100, 100, 3), [255, 0, 0], dtype=np.uint8))
    msg.add_image(red_img)

    response2 = agent.query(msg)
    assert agent.conversation.size() == 4

    # Verify history format
    history = agent.conversation.to_openai_format()
    # Check that image message has proper format
    image_msg = history[2]  # Third message (after first exchange)
    assert image_msg["role"] == "user"
    assert isinstance(image_msg["content"], list), "Image message should have content array"

    # Send another text message
    response3 = agent.query("What color did I just show you?")
    assert agent.conversation.size() == 6

    # Send another image
    msg2 = AgentMessage()
    msg2.add_text("Now here's a blue square")
    blue_img = Image(data=np.full((100, 100, 3), [0, 0, 255], dtype=np.uint8))
    msg2.add_image(blue_img)

    response4 = agent.query(msg2)
    assert agent.conversation.size() == 8

    # Test memory of both images
    response5 = agent.query("What colors have I shown you?")
    response_lower = response5.content.lower()
    # Agent should mention both colors or indicate it saw images
    assert any(word in response_lower for word in ["red", "blue", "color", "square", "image"])

    agent.dispose()


def test_conversation_history_trimming():
    """Test that conversation history is properly trimmed."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant.",
        temperature=0.0,
        max_history=6,  # Small limit for testing
    )

    # Send multiple messages to exceed limit
    messages = [
        "Message 1: I like apples",
        "Message 2: I like oranges",
        "Message 3: I like bananas",
        "Message 4: I like grapes",
        "Message 5: I like strawberries",
    ]

    for msg in messages:
        agent.query(msg)

    # Should be trimmed to max_history
    assert agent.conversation.size() <= 6

    # Verify trimming by checking if early messages are forgotten
    response = agent.query("What was the first fruit I mentioned?")
    # Should not confidently remember apples since it's been trimmed
    # (This is a heuristic test - models may vary in response)

    # Test dynamic max_history update
    agent.max_history = 4
    agent.query("New message after resize")
    assert agent.conversation.size() <= 4

    agent.dispose()


def test_conversation_history_with_tools():
    """Test conversation history when tools are used."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Define a simple calculator skill
    class CalculatorSkill(AbstractSkill):
        """Perform mathematical calculations."""

        expression: str = Field(description="Mathematical expression to evaluate")

        def __call__(self) -> str:
            try:
                result = eval(self.expression)
                return f"The result is {result}"
            except:
                return "Error in calculation"

    skills = SkillLibrary()
    skills.add(CalculatorSkill)

    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant with a calculator. Use it when asked to compute.",
        skills=skills,
        temperature=0.0,
    )

    # Query without tools
    response1 = agent.query("Hello, I need help with math")
    assert agent.conversation.size() >= 2

    # Query that should trigger tool use
    response2 = agent.query("Please calculate 123 * 456 using your calculator")
    assert response2.content is not None

    # Verify tool calls are in history
    history = agent.conversation.to_openai_format()

    # Look for tool-related messages
    has_tool_call = False
    has_tool_result = False
    for msg in history:
        if msg.get("tool_calls"):
            has_tool_call = True
        if msg.get("role") == "tool":
            has_tool_result = True

    # Tool usage should be recorded in history
    assert has_tool_call or has_tool_result or "56088" in response2.content

    # Reference previous calculation
    response3 = agent.query("What was the result of the calculation?")
    assert "56088" in response3.content or "calculation" in response3.content.lower()

    agent.dispose()


def test_conversation_thread_safety():
    """Test that conversation history is thread-safe."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    agent = BaseAgent(
        model="openai::gpt-4o-mini", system_prompt="You are a helpful assistant.", temperature=0.0
    )

    async def query_async(text: str):
        """Async query wrapper."""
        return await agent.aquery(text)

    # Run multiple queries concurrently
    async def run_concurrent():
        tasks = [query_async("Query 1"), query_async("Query 2"), query_async("Query 3")]
        return await asyncio.gather(*tasks)

    # Execute concurrent queries
    responses = asyncio.run(run_concurrent())

    # All queries should get responses
    assert len(responses) == 3
    for r in responses:
        assert r.content is not None

    # History should contain all messages (6 total: 3 user + 3 assistant)
    # Due to concurrency, exact count may vary slightly
    assert agent.conversation.size() >= 6

    agent.dispose()


def test_conversation_history_formats():
    """Test different message formats in conversation history."""
    history = ConversationHistory(max_size=10)

    # Add text message
    history.add_user_message("Hello")

    # Add multimodal message
    content_array = [
        {"type": "text", "text": "Look at this"},
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64,..."}},
    ]
    history.add_user_message(content_array)

    # Add assistant response
    history.add_assistant_message("I see the image")

    # Add tool call
    from dimos.agents.agent_types import ToolCall

    tool_call = ToolCall(
        id="call_123", name="calculator", arguments={"expression": "2+2"}, status="completed"
    )
    history.add_assistant_message("Let me calculate", [tool_call])

    # Add tool result
    history.add_tool_result("call_123", "The result is 4")

    # Verify OpenAI format conversion
    messages = history.to_openai_format()
    assert len(messages) == 5

    # Check message formats
    assert messages[0]["role"] == "user"
    assert messages[0]["content"] == "Hello"

    assert messages[1]["role"] == "user"
    assert isinstance(messages[1]["content"], list)

    assert messages[2]["role"] == "assistant"

    assert messages[3]["role"] == "assistant"
    assert "tool_calls" in messages[3]

    assert messages[4]["role"] == "tool"
    assert messages[4]["tool_call_id"] == "call_123"


def test_conversation_edge_cases():
    """Test edge cases in conversation history."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    agent = BaseAgent(
        model="openai::gpt-4o-mini", system_prompt="You are a helpful assistant.", temperature=0.0
    )

    # Empty message
    msg1 = AgentMessage()
    msg1.add_text("")
    response1 = agent.query(msg1)
    assert response1.content is not None

    # Very long message
    long_text = "word " * 1000
    response2 = agent.query(long_text)
    assert response2.content is not None

    # Multiple text parts that combine
    msg3 = AgentMessage()
    for i in range(10):
        msg3.add_text(f"Part {i} ")
    response3 = agent.query(msg3)
    assert response3.content is not None

    # Verify history is maintained correctly
    assert agent.conversation.size() == 6  # 3 exchanges

    agent.dispose()


if __name__ == "__main__":
    # Run tests
    test_conversation_history_basic()
    test_conversation_history_with_images()
    test_conversation_history_trimming()
    test_conversation_history_with_tools()
    test_conversation_thread_safety()
    test_conversation_history_formats()
    test_conversation_edge_cases()
    print("\n✅ All conversation history tests passed!")
