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

"""Test agent module with proper module connections."""

import asyncio
import os
import pytest
import threading
import time
from dotenv import load_dotenv

from dimos import core
from dimos.core import Module, Out, In, rpc
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse
from dimos.protocol import pubsub


# Test query sender module
class QuerySender(Module):
    """Module to send test queries."""

    message_out: Out[AgentMessage] = None

    def __init__(self):
        super().__init__()

    @rpc
    def send_query(self, query: str):
        """Send a query."""
        print(f"Sending query: {query}")
        msg = AgentMessage()
        msg.add_text(query)
        self.message_out.publish(msg)


# Test response collector module
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

    def _on_response(self, msg: AgentResponse):
        print(f"Received response: {msg.content if msg.content else msg}")
        self.responses.append(msg)

    @rpc
    def get_responses(self):
        """Get collected responses."""
        return self.responses


@pytest.mark.tofix
@pytest.mark.module
@pytest.mark.asyncio
async def test_agent_module_connections():
    """Test agent module with proper connections."""
    load_dotenv()
    pubsub.lcm.autoconf()

    # Start Dask
    dimos = core.start(4)

    try:
        # Deploy modules
        sender = dimos.deploy(QuerySender)
        agent = dimos.deploy(
            BaseAgentModule,
            model="openai::gpt-4o-mini",
            system_prompt="You are a helpful assistant. Answer in 10 words or less.",
        )
        collector = dimos.deploy(ResponseCollector)

        # Configure transports
        sender.message_out.transport = core.pLCMTransport("/messages")
        agent.response_out.transport = core.pLCMTransport("/responses")

        # Connect modules
        agent.message_in.connect(sender.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()

        # Wait for initialization
        await asyncio.sleep(1)

        # Test 1: Simple query
        print("\n=== Test 1: Simple Query ===")
        sender.send_query("What is 2+2?")

        await asyncio.sleep(5)  # Increased wait time for API response

        responses = collector.get_responses()
        assert len(responses) > 0, "Should have received a response"
        assert isinstance(responses[0], AgentResponse), "Expected AgentResponse object"
        assert "4" in responses[0].content or "four" in responses[0].content.lower(), (
            "Should calculate correctly"
        )

        # Test 2: Another query
        print("\n=== Test 2: Another Query ===")
        sender.send_query("What color is the sky?")

        await asyncio.sleep(5)  # Increased wait time

        responses = collector.get_responses()
        assert len(responses) >= 2, "Should have at least two responses"
        assert isinstance(responses[1], AgentResponse), "Expected AgentResponse object"
        assert "blue" in responses[1].content.lower(), "Should mention blue"

        # Test 3: Multiple queries
        print("\n=== Test 3: Multiple Queries ===")
        queries = ["Count from 1 to 3", "Name a fruit", "What is Python?"]

        for q in queries:
            sender.send_query(q)
            await asyncio.sleep(2)  # Give more time between queries

        await asyncio.sleep(8)  # More time for multiple queries

        responses = collector.get_responses()
        assert len(responses) >= 4, f"Should have at least 4 responses, got {len(responses)}"

        # Stop modules
        agent.stop()

        print("\n=== All tests passed! ===")

    finally:
        dimos.close()
        dimos.shutdown()


if __name__ == "__main__":
    asyncio.run(test_agent_module_connections())
