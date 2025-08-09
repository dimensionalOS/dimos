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

"""Test simple agent module with string input/output."""

import asyncio
import os
import pytest
from dotenv import load_dotenv

from dimos import core
from dimos.core import Module, Out, In, rpc
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse
from dimos.protocol import pubsub


class QuerySender(Module):
    """Module to send test queries."""

    message_out: Out[AgentMessage] = None

    @rpc
    def send_query(self, query: str):
        """Send a query."""
        msg = AgentMessage()
        msg.add_text(query)
        self.message_out.publish(msg)


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

    def _on_response(self, response: AgentResponse):
        """Handle response."""
        self.responses.append(response)

    @rpc
    def get_responses(self) -> list:
        """Get collected responses."""
        return self.responses

    @rpc
    def clear(self):
        """Clear responses."""
        self.responses = []


@pytest.mark.module
@pytest.mark.asyncio
@pytest.mark.parametrize(
    "model,provider",
    [
        ("openai::gpt-4o-mini", "OpenAI"),
        ("anthropic::claude-3-haiku-20240307", "Claude"),
        ("cerebras::llama3.1-8b", "Cerebras"),
        ("qwen::qwen-turbo", "Qwen"),
    ],
)
async def test_simple_agent_module(model, provider):
    """Test simple agent module with different providers."""
    load_dotenv()

    # Skip if no API key
    if provider == "OpenAI" and not os.getenv("OPENAI_API_KEY"):
        pytest.skip(f"No OpenAI API key found")
    elif provider == "Claude" and not os.getenv("ANTHROPIC_API_KEY"):
        pytest.skip(f"No Anthropic API key found")
    elif provider == "Cerebras" and not os.getenv("CEREBRAS_API_KEY"):
        pytest.skip(f"No Cerebras API key found")
    elif provider == "Qwen" and not os.getenv("ALIBABA_API_KEY"):
        pytest.skip(f"No Qwen API key found")

    pubsub.lcm.autoconf()

    # Start Dask cluster
    dimos = core.start(3)

    try:
        # Deploy modules
        sender = dimos.deploy(QuerySender)
        agent = dimos.deploy(
            BaseAgentModule,
            model=model,
            system_prompt=f"You are a helpful {provider} assistant. Keep responses brief.",
        )
        collector = dimos.deploy(ResponseCollector)

        # Configure transports
        sender.message_out.transport = core.pLCMTransport(f"/test/{provider}/messages")
        agent.response_out.transport = core.pLCMTransport(f"/test/{provider}/responses")

        # Connect modules
        agent.message_in.connect(sender.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()

        await asyncio.sleep(1)

        # Test simple math
        sender.send_query("What is 2+2?")
        await asyncio.sleep(5)

        responses = collector.get_responses()
        assert len(responses) > 0, f"{provider} should respond"
        assert isinstance(responses[0], AgentResponse), "Expected AgentResponse object"
        assert "4" in responses[0].content, f"{provider} should calculate correctly"

        # Test brief response
        collector.clear()
        sender.send_query("Name one color.")
        await asyncio.sleep(5)

        responses = collector.get_responses()
        assert len(responses) > 0, f"{provider} should respond"
        assert isinstance(responses[0], AgentResponse), "Expected AgentResponse object"
        assert len(responses[0].content) < 200, f"{provider} should give brief response"

        # Stop modules
        agent.stop()

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.module
@pytest.mark.asyncio
async def test_mock_agent_module():
    """Test agent module with mock responses (no API needed)."""
    pubsub.lcm.autoconf()

    class MockAgentModule(Module):
        """Mock agent for testing."""

        message_in: In[AgentMessage] = None
        response_out: Out[AgentResponse] = None

        @rpc
        def start(self):
            self.message_in.subscribe(self._handle_message)

        def _handle_message(self, msg: AgentMessage):
            query = msg.get_combined_text()
            if "2+2" in query:
                self.response_out.publish(AgentResponse(content="4"))
            elif "color" in query.lower():
                self.response_out.publish(AgentResponse(content="Blue"))
            else:
                self.response_out.publish(AgentResponse(content=f"Mock response to: {query}"))

    dimos = core.start(2)

    try:
        # Deploy
        agent = dimos.deploy(MockAgentModule)
        collector = dimos.deploy(ResponseCollector)

        # Configure
        agent.message_in.transport = core.pLCMTransport("/mock/messages")
        agent.response_out.transport = core.pLCMTransport("/mock/response")

        # Connect
        collector.response_in.connect(agent.response_out)

        # Start
        agent.start()
        collector.start()

        await asyncio.sleep(1)

        # Test - use a simple query sender
        sender = dimos.deploy(QuerySender)
        sender.message_out.transport = core.pLCMTransport("/mock/messages")
        agent.message_in.connect(sender.message_out)

        await asyncio.sleep(1)

        sender.send_query("What is 2+2?")
        await asyncio.sleep(1)

        responses = collector.get_responses()
        assert len(responses) == 1
        assert isinstance(responses[0], AgentResponse), "Expected AgentResponse object"
        assert responses[0].content == "4"

    finally:
        dimos.close()
        dimos.shutdown()


if __name__ == "__main__":
    asyncio.run(test_mock_agent_module())
