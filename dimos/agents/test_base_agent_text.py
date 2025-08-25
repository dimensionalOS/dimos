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

"""Test BaseAgent text functionality."""

import pytest
import asyncio
import os
from dotenv import load_dotenv

from dimos.agents.modules.base import BaseAgent
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse
from dimos import core
from dimos.core import Module, Out, In, rpc
from dimos.protocol import pubsub


class QuerySender(Module):
    """Module to send test queries."""

    message_out: Out[AgentMessage] = None  # New AgentMessage output

    @rpc
    def send_query(self, query: str):
        """Send a query as AgentMessage."""
        msg = AgentMessage()
        msg.add_text(query)
        self.message_out.publish(msg)

    @rpc
    def send_message(self, message: AgentMessage):
        """Send an AgentMessage."""
        self.message_out.publish(message)


class ResponseCollector(Module):
    """Module to collect responses."""

    response_in: In[AgentResponse] = None

    def __init__(self):
        super().__init__()
        self.responses = []

    @rpc
    def start(self):
        """Start collecting."""
        self.response_in.subscribe(self._on_response)

    def _on_response(self, msg):
        self.responses.append(msg)

    @rpc
    def get_responses(self):
        """Get collected responses."""
        return self.responses


@pytest.mark.tofix
def test_base_agent_direct_text():
    """Test BaseAgent direct text usage."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Create agent
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant. Answer in 10 words or less.",
        temperature=0.0,
        seed=42,  # Fixed seed for deterministic results
    )

    # Test simple query with string (backward compatibility)
    response = agent.query("What is 2+2?")
    print(f"\n[Test] Query: 'What is 2+2?' -> Response: '{response.content}'")
    assert response.content is not None
    assert "4" in response.content or "four" in response.content.lower(), (
        f"Expected '4' or 'four' in response, got: {response.content}"
    )

    # Test with AgentMessage
    msg = AgentMessage()
    msg.add_text("What is 3+3?")
    response = agent.query(msg)
    print(f"[Test] Query: 'What is 3+3?' -> Response: '{response.content}'")
    assert response.content is not None
    assert "6" in response.content or "six" in response.content.lower(), (
        f"Expected '6' or 'six' in response"
    )

    # Test conversation history
    response = agent.query("What was my previous question?")
    print(f"[Test] Query: 'What was my previous question?' -> Response: '{response.content}'")
    assert response.content is not None
    # The agent should reference one of the previous questions
    # It might say "2+2" or "3+3" depending on interpretation of "previous"
    assert (
        "2+2" in response.content or "3+3" in response.content or "What is" in response.content
    ), f"Expected reference to a previous question, got: {response.content}"

    # Clean up
    agent.dispose()


@pytest.mark.tofix
@pytest.mark.asyncio
async def test_base_agent_async_text():
    """Test BaseAgent async text usage."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Create agent
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant.",
        temperature=0.0,
        seed=42,
    )

    # Test async query with string
    response = await agent.aquery("What is the capital of France?")
    assert response.content is not None
    assert "Paris" in response.content, f"Expected 'Paris' in response"

    # Test async query with AgentMessage
    msg = AgentMessage()
    msg.add_text("What is the capital of Germany?")
    response = await agent.aquery(msg)
    assert response.content is not None
    assert "Berlin" in response.content, f"Expected 'Berlin' in response"

    # Clean up
    agent.dispose()


@pytest.mark.tofix
@pytest.mark.module
@pytest.mark.asyncio
async def test_base_agent_module_text():
    """Test BaseAgentModule with text via DimOS."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    pubsub.lcm.autoconf()
    dimos = core.start(4)

    try:
        # Deploy modules
        sender = dimos.deploy(QuerySender)
        agent = dimos.deploy(
            BaseAgentModule,
            model="openai::gpt-4o-mini",
            system_prompt="You are a helpful assistant. Answer concisely.",
        )
        collector = dimos.deploy(ResponseCollector)

        # Configure transports
        sender.message_out.transport = core.pLCMTransport("/test/messages")
        agent.response_out.transport = core.pLCMTransport("/test/responses")

        # Connect modules
        agent.message_in.connect(sender.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()

        # Wait for initialization
        await asyncio.sleep(1)

        # Test queries
        sender.send_query("What is 2+2?")
        await asyncio.sleep(3)

        responses = collector.get_responses()
        assert len(responses) > 0, "Should have received a response"
        resp = responses[0]
        assert isinstance(resp, AgentResponse), "Expected AgentResponse object"
        assert "4" in resp.content or "four" in resp.content.lower(), (
            f"Expected '4' or 'four' in response, got: {resp.content}"
        )

        # Test another query
        sender.send_query("What color is the sky?")
        await asyncio.sleep(3)

        responses = collector.get_responses()
        assert len(responses) >= 2, "Should have at least two responses"
        resp = responses[1]
        assert isinstance(resp, AgentResponse), "Expected AgentResponse object"
        assert "blue" in resp.content.lower(), f"Expected 'blue' in response"

        # Test conversation history
        sender.send_query("What was my first question?")
        await asyncio.sleep(3)

        responses = collector.get_responses()
        assert len(responses) >= 3, "Should have at least three responses"
        resp = responses[2]
        assert isinstance(resp, AgentResponse), "Expected AgentResponse object"
        assert "2+2" in resp.content or "2" in resp.content, f"Expected reference to first question"

        # Stop modules
        agent.stop()

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.parametrize(
    "model,provider",
    [
        ("openai::gpt-4o-mini", "openai"),
        ("anthropic::claude-3-haiku-20240307", "anthropic"),
        ("cerebras::llama-3.3-70b", "cerebras"),
    ],
)
@pytest.mark.tofix
def test_base_agent_providers(model, provider):
    """Test BaseAgent with different providers."""
    load_dotenv()

    # Check for API key
    api_key_map = {
        "openai": "OPENAI_API_KEY",
        "anthropic": "ANTHROPIC_API_KEY",
        "cerebras": "CEREBRAS_API_KEY",
    }

    if not os.getenv(api_key_map[provider]):
        pytest.skip(f"No {api_key_map[provider]} found")

    # Create agent
    agent = BaseAgent(
        model=model,
        system_prompt="You are a helpful assistant. Answer in 10 words or less.",
        temperature=0.0,
        seed=42,
    )

    # Test query with AgentMessage
    msg = AgentMessage()
    msg.add_text("What is the capital of France?")
    response = agent.query(msg)
    assert response.content is not None
    assert "Paris" in response.content, f"Expected 'Paris' in response from {provider}"

    # Clean up
    agent.dispose()


@pytest.mark.tofix
def test_base_agent_memory():
    """Test BaseAgent with memory/RAG."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Create agent
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant. Use the provided context when answering.",
        temperature=0.0,
        rag_threshold=0.3,
        seed=42,
    )

    # Add context to memory
    agent.memory.add_vector("doc1", "The DimOS framework is designed for building robotic systems.")
    agent.memory.add_vector(
        "doc2", "Robots using DimOS can perform navigation and manipulation tasks."
    )

    # Test RAG retrieval with AgentMessage
    msg = AgentMessage()
    msg.add_text("What is DimOS?")
    response = agent.query(msg)
    assert response.content is not None
    assert "framework" in response.content.lower() or "robotic" in response.content.lower(), (
        f"Expected context about DimOS in response"
    )

    # Clean up
    agent.dispose()


class MockAgent(BaseAgent):
    """Mock agent for testing without API calls."""

    def __init__(self, **kwargs):
        # Don't call super().__init__ to avoid gateway initialization
        from dimos.agents.agent_types import ConversationHistory

        self.model = kwargs.get("model", "mock::test")
        self.system_prompt = kwargs.get("system_prompt", "Mock agent")
        self.conversation = ConversationHistory(max_size=20)
        self._supports_vision = False
        self.response_subject = None  # Simplified

    async def _process_query_async(self, query: str, base64_image=None):
        """Mock response."""
        if "2+2" in query:
            return "The answer is 4"
        elif "capital" in query and "France" in query:
            return "The capital of France is Paris"
        elif "color" in query and "sky" in query:
            return "The sky is blue"
        elif "previous" in query:
            history = self.conversation.to_openai_format()
            if len(history) >= 2:
                # Get the second to last item (the last user query before this one)
                for i in range(len(history) - 2, -1, -1):
                    if history[i]["role"] == "user":
                        return f"Your previous question was: {history[i]['content']}"
            return "No previous questions"
        else:
            return f"Mock response to: {query}"

    def query(self, message) -> AgentResponse:
        """Mock synchronous query."""
        # Convert to text if AgentMessage
        if isinstance(message, AgentMessage):
            text = message.get_combined_text()
        else:
            text = message

        # Update conversation history
        self.conversation.add_user_message(text)
        response = asyncio.run(self._process_query_async(text))
        self.conversation.add_assistant_message(response)
        return AgentResponse(content=response)

    async def aquery(self, message) -> AgentResponse:
        """Mock async query."""
        # Convert to text if AgentMessage
        if isinstance(message, AgentMessage):
            text = message.get_combined_text()
        else:
            text = message

        self.conversation.add_user_message(text)
        response = await self._process_query_async(text)
        self.conversation.add_assistant_message(response)
        return AgentResponse(content=response)

    def dispose(self):
        """Mock dispose."""
        pass


@pytest.mark.tofix
def test_mock_agent():
    """Test mock agent for CI without API keys."""
    # Create mock agent
    agent = MockAgent(model="mock::test", system_prompt="Mock assistant")

    # Test simple query
    response = agent.query("What is 2+2?")
    assert isinstance(response, AgentResponse), "Expected AgentResponse object"
    assert "4" in response.content

    # Test conversation history
    response = agent.query("What was my previous question?")
    assert isinstance(response, AgentResponse), "Expected AgentResponse object"
    assert "2+2" in response.content

    # Test other queries
    response = agent.query("What is the capital of France?")
    assert isinstance(response, AgentResponse), "Expected AgentResponse object"
    assert "Paris" in response.content

    response = agent.query("What color is the sky?")
    assert isinstance(response, AgentResponse), "Expected AgentResponse object"
    assert "blue" in response.content.lower()

    # Clean up
    agent.dispose()


@pytest.mark.tofix
def test_base_agent_conversation_history():
    """Test that conversation history is properly maintained."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Create agent
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant.",
        temperature=0.0,
        seed=42,
    )

    # Test 1: Simple conversation
    response1 = agent.query("My name is Alice")
    assert isinstance(response1, AgentResponse)

    # Check conversation history has both messages
    assert agent.conversation.size() == 2
    history = agent.conversation.to_openai_format()
    assert history[0]["role"] == "user"
    assert history[0]["content"] == "My name is Alice"
    assert history[1]["role"] == "assistant"

    # Test 2: Reference previous context
    response2 = agent.query("What is my name?")
    assert "Alice" in response2.content, f"Agent should remember the name"

    # Conversation history should now have 4 messages
    assert agent.conversation.size() == 4

    # Test 3: Multiple text parts in AgentMessage
    msg = AgentMessage()
    msg.add_text("Calculate")
    msg.add_text("the sum of")
    msg.add_text("5 + 3")

    response3 = agent.query(msg)
    assert "8" in response3.content or "eight" in response3.content.lower()

    # Check the combined text was stored correctly
    assert agent.conversation.size() == 6
    history = agent.conversation.to_openai_format()
    assert history[4]["role"] == "user"
    assert history[4]["content"] == "Calculate the sum of 5 + 3"

    # Test 4: History trimming (set low limit)
    agent.max_history = 4
    response4 = agent.query("What was my first message?")

    # Conversation history should be trimmed to 4 messages
    assert agent.conversation.size() == 4
    # First messages should be gone
    history = agent.conversation.to_openai_format()
    assert "Alice" not in history[0]["content"]

    # Clean up
    agent.dispose()


@pytest.mark.tofix
def test_base_agent_history_with_tools():
    """Test conversation history with tool calls."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    from dimos.skills.skills import AbstractSkill, SkillLibrary
    from pydantic import Field

    class CalculatorSkill(AbstractSkill):
        """Perform calculations."""

        expression: str = Field(description="Mathematical expression")

        def __call__(self) -> str:
            try:
                result = eval(self.expression)
                return f"The result is {result}"
            except:
                return "Error in calculation"

    # Create agent with calculator skill
    skills = SkillLibrary()
    skills.add(CalculatorSkill)

    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant with a calculator. Use the calculator tool when asked to compute something.",
        skills=skills,
        temperature=0.0,
        seed=42,
    )

    # Make a query that should trigger tool use
    response = agent.query("Please calculate 42 * 17 using the calculator tool")

    # Check response
    assert isinstance(response, AgentResponse)
    assert "714" in response.content, f"Expected 714 in response, got: {response.content}"

    # Check tool calls were made
    if response.tool_calls:
        assert len(response.tool_calls) > 0
        assert response.tool_calls[0].name == "CalculatorSkill"
        assert response.tool_calls[0].status == "completed"

    # Check history structure
    # If tools were called, we should have more messages
    if response.tool_calls and len(response.tool_calls) > 0:
        assert agent.conversation.size() >= 3, (
            f"Expected at least 3 messages in history when tools are used, got {agent.conversation.size()}"
        )

        # Find the assistant message with tool calls
        history = agent.conversation.to_openai_format()
        tool_msg_found = False
        tool_result_found = False

        for msg in history:
            if msg.get("role") == "assistant" and msg.get("tool_calls"):
                tool_msg_found = True
            if msg.get("role") == "tool":
                tool_result_found = True
                assert "result" in msg.get("content", "").lower()

        assert tool_msg_found, "Tool call message should be in history when tools were used"
        assert tool_result_found, "Tool result should be in history when tools were used"
    else:
        # No tools used, just verify we have user and assistant messages
        assert agent.conversation.size() >= 2, (
            f"Expected at least 2 messages in history, got {agent.conversation.size()}"
        )
        # The model solved it without using the tool - that's also acceptable
        print("Note: Model solved without using the calculator tool")

    # Clean up
    agent.dispose()


if __name__ == "__main__":
    test_base_agent_direct_text()
    asyncio.run(test_base_agent_async_text())
    asyncio.run(test_base_agent_module_text())
    test_base_agent_memory()
    test_mock_agent()
    test_base_agent_conversation_history()
    test_base_agent_history_with_tools()
    print("\n✅ All text tests passed!")
    test_base_agent_direct_text()
    asyncio.run(test_base_agent_async_text())
    asyncio.run(test_base_agent_module_text())
    test_base_agent_memory()
    test_mock_agent()
    print("\n✅ All text tests passed!")
