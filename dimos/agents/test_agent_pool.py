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

"""Test agent pool module."""

import asyncio
import os
import pytest
from dotenv import load_dotenv

from dimos import core
from dimos.core import Module, Out, In, rpc
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.protocol import pubsub


class PoolRouter(Module):
    """Simple router for agent pool."""

    query_in: In[dict] = None
    agent1_out: Out[str] = None
    agent2_out: Out[str] = None
    agent3_out: Out[str] = None

    @rpc
    def start(self):
        self.query_in.subscribe(self._route)

    def _route(self, msg: dict):
        agent_id = msg.get("agent_id", "agent1")
        query = msg.get("query", "")

        if agent_id == "agent1" and self.agent1_out:
            self.agent1_out.publish(query)
        elif agent_id == "agent2" and self.agent2_out:
            self.agent2_out.publish(query)
        elif agent_id == "agent3" and self.agent3_out:
            self.agent3_out.publish(query)
        elif agent_id == "all":
            # Broadcast to all
            if self.agent1_out:
                self.agent1_out.publish(query)
            if self.agent2_out:
                self.agent2_out.publish(query)
            if self.agent3_out:
                self.agent3_out.publish(query)


class PoolAggregator(Module):
    """Aggregate responses from pool."""

    agent1_in: In[str] = None
    agent2_in: In[str] = None
    agent3_in: In[str] = None
    response_out: Out[dict] = None

    @rpc
    def start(self):
        if self.agent1_in:
            self.agent1_in.subscribe(lambda r: self._handle_response("agent1", r))
        if self.agent2_in:
            self.agent2_in.subscribe(lambda r: self._handle_response("agent2", r))
        if self.agent3_in:
            self.agent3_in.subscribe(lambda r: self._handle_response("agent3", r))

    def _handle_response(self, agent_id: str, response: str):
        if self.response_out:
            self.response_out.publish({"agent_id": agent_id, "response": response})


class PoolController(Module):
    """Controller for pool testing."""

    query_out: Out[dict] = None

    @rpc
    def send_to_agent(self, agent_id: str, query: str):
        self.query_out.publish({"agent_id": agent_id, "query": query})

    @rpc
    def broadcast(self, query: str):
        self.query_out.publish({"agent_id": "all", "query": query})


class PoolCollector(Module):
    """Collect pool responses."""

    response_in: In[dict] = None

    def __init__(self):
        super().__init__()
        self.responses = []

    @rpc
    def start(self):
        self.response_in.subscribe(lambda r: self.responses.append(r))

    @rpc
    def get_responses(self) -> list:
        return self.responses

    @rpc
    def get_by_agent(self, agent_id: str) -> list:
        return [r for r in self.responses if r.get("agent_id") == agent_id]


@pytest.mark.skip("Skipping pool tests for now")
@pytest.mark.module
@pytest.mark.asyncio
async def test_agent_pool():
    """Test agent pool with multiple agents."""
    load_dotenv()
    pubsub.lcm.autoconf()

    # Check for at least one API key
    has_api_key = any(
        [os.getenv("OPENAI_API_KEY"), os.getenv("ANTHROPIC_API_KEY"), os.getenv("CEREBRAS_API_KEY")]
    )

    if not has_api_key:
        pytest.skip("No API keys found for testing")

    dimos = core.start(7)

    try:
        # Deploy three agents with different configs
        agents = []
        models = []

        if os.getenv("CEREBRAS_API_KEY"):
            agent1 = dimos.deploy(
                BaseAgentModule,
                model="cerebras::llama3.1-8b",
                system_prompt="You are agent1. Be very brief.",
            )
            agents.append(agent1)
            models.append("agent1")

        if os.getenv("OPENAI_API_KEY"):
            agent2 = dimos.deploy(
                BaseAgentModule,
                model="openai::gpt-4o-mini",
                system_prompt="You are agent2. Be helpful.",
            )
            agents.append(agent2)
            models.append("agent2")

        if os.getenv("CEREBRAS_API_KEY") and len(agents) < 3:
            agent3 = dimos.deploy(
                BaseAgentModule,
                model="cerebras::llama3.1-8b",
                system_prompt="You are agent3. Be creative.",
            )
            agents.append(agent3)
            models.append("agent3")

        if len(agents) < 2:
            pytest.skip("Need at least 2 working agents for pool test")

        # Deploy router, aggregator, controller, collector
        router = dimos.deploy(PoolRouter)
        aggregator = dimos.deploy(PoolAggregator)
        controller = dimos.deploy(PoolController)
        collector = dimos.deploy(PoolCollector)

        # Configure transports
        controller.query_out.transport = core.pLCMTransport("/pool/queries")
        aggregator.response_out.transport = core.pLCMTransport("/pool/responses")

        # Configure agent transports and connections
        if len(agents) > 0:
            router.agent1_out.transport = core.pLCMTransport("/pool/agent1/query")
            agents[0].response_out.transport = core.pLCMTransport("/pool/agent1/response")
            agents[0].query_in.connect(router.agent1_out)
            aggregator.agent1_in.connect(agents[0].response_out)

        if len(agents) > 1:
            router.agent2_out.transport = core.pLCMTransport("/pool/agent2/query")
            agents[1].response_out.transport = core.pLCMTransport("/pool/agent2/response")
            agents[1].query_in.connect(router.agent2_out)
            aggregator.agent2_in.connect(agents[1].response_out)

        if len(agents) > 2:
            router.agent3_out.transport = core.pLCMTransport("/pool/agent3/query")
            agents[2].response_out.transport = core.pLCMTransport("/pool/agent3/response")
            agents[2].query_in.connect(router.agent3_out)
            aggregator.agent3_in.connect(agents[2].response_out)

        # Connect router and collector
        router.query_in.connect(controller.query_out)
        collector.response_in.connect(aggregator.response_out)

        # Start all modules
        for agent in agents:
            agent.start()
        router.start()
        aggregator.start()
        collector.start()

        await asyncio.sleep(3)

        # Test direct routing
        for i, model_id in enumerate(models[:2]):  # Test first 2 agents
            controller.send_to_agent(model_id, f"Say hello from {model_id}")
            await asyncio.sleep(0.5)

        await asyncio.sleep(6)

        responses = collector.get_responses()
        print(f"Got {len(responses)} responses from direct routing")
        assert len(responses) >= len(models[:2]), (
            f"Should get responses from at least {len(models[:2])} agents"
        )

        # Test broadcast
        collector.responses.clear()
        controller.broadcast("What is 1+1?")

        await asyncio.sleep(6)

        responses = collector.get_responses()
        print(f"Got {len(responses)} responses from broadcast (expected {len(agents)})")
        # Allow for some agents to be slow
        assert len(responses) >= min(2, len(agents)), (
            f"Should get response from at least {min(2, len(agents))} agents"
        )

        # Check all agents responded
        agent_ids = {r["agent_id"] for r in responses}
        assert len(agent_ids) >= 2, "Multiple agents should respond"

        # Stop all agents
        for agent in agents:
            agent.stop()

    finally:
        dimos.close()
        dimos.shutdown()


@pytest.mark.skip("Skipping pool tests for now")
@pytest.mark.module
@pytest.mark.asyncio
async def test_mock_agent_pool():
    """Test agent pool with mock agents."""
    pubsub.lcm.autoconf()

    class MockPoolAgent(Module):
        """Mock agent for pool testing."""

        query_in: In[str] = None
        response_out: Out[str] = None

        def __init__(self, agent_id: str):
            super().__init__()
            self.agent_id = agent_id

        @rpc
        def start(self):
            self.query_in.subscribe(self._handle_query)

        def _handle_query(self, query: str):
            if "1+1" in query:
                self.response_out.publish(f"{self.agent_id}: The answer is 2")
            else:
                self.response_out.publish(f"{self.agent_id}: {query}")

    dimos = core.start(6)

    try:
        # Deploy mock agents
        agent1 = dimos.deploy(MockPoolAgent, agent_id="fast")
        agent2 = dimos.deploy(MockPoolAgent, agent_id="smart")
        agent3 = dimos.deploy(MockPoolAgent, agent_id="creative")

        # Deploy infrastructure
        router = dimos.deploy(PoolRouter)
        aggregator = dimos.deploy(PoolAggregator)
        collector = dimos.deploy(PoolCollector)

        # Configure all transports
        router.query_in.transport = core.pLCMTransport("/mock/pool/queries")
        router.agent1_out.transport = core.pLCMTransport("/mock/pool/agent1/q")
        router.agent2_out.transport = core.pLCMTransport("/mock/pool/agent2/q")
        router.agent3_out.transport = core.pLCMTransport("/mock/pool/agent3/q")

        agent1.response_out.transport = core.pLCMTransport("/mock/pool/agent1/r")
        agent2.response_out.transport = core.pLCMTransport("/mock/pool/agent2/r")
        agent3.response_out.transport = core.pLCMTransport("/mock/pool/agent3/r")

        aggregator.response_out.transport = core.pLCMTransport("/mock/pool/responses")

        # Connect everything
        agent1.query_in.connect(router.agent1_out)
        agent2.query_in.connect(router.agent2_out)
        agent3.query_in.connect(router.agent3_out)

        aggregator.agent1_in.connect(agent1.response_out)
        aggregator.agent2_in.connect(agent2.response_out)
        aggregator.agent3_in.connect(agent3.response_out)

        collector.response_in.connect(aggregator.response_out)

        # Start all
        agent1.start()
        agent2.start()
        agent3.start()
        router.start()
        aggregator.start()
        collector.start()

        await asyncio.sleep(0.5)

        # Test routing
        router.query_in.transport.publish({"agent_id": "agent1", "query": "Hello"})
        router.query_in.transport.publish({"agent_id": "agent2", "query": "Hi"})

        await asyncio.sleep(0.5)

        responses = collector.get_responses()
        assert len(responses) == 2
        assert any("fast" in r["response"] for r in responses)
        assert any("smart" in r["response"] for r in responses)

        # Test broadcast
        collector.responses.clear()
        router.query_in.transport.publish({"agent_id": "all", "query": "What is 1+1?"})

        await asyncio.sleep(0.5)

        responses = collector.get_responses()
        assert len(responses) == 3
        assert all("2" in r["response"] for r in responses)

    finally:
        dimos.close()
        dimos.shutdown()


if __name__ == "__main__":
    asyncio.run(test_mock_agent_pool())
