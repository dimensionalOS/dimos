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

"""Production test for BaseAgent tool handling functionality."""

import asyncio
import os

import pytest
from dotenv import load_dotenv

from dimos import core
from dimos.agents.agent_message import AgentMessage
from dimos.agents.agent_types import AgentResponse
from dimos.agents.modules.base import BaseAgent
from dimos.agents.modules.base_agent import BaseAgentModule
from dimos.core import In, Module, Out, rpc
from dimos.protocol import pubsub
from dimos.protocol.skill import SkillContainer, SkillCoordinator, skill
from dimos.utils.logging_config import setup_logger

logger = setup_logger("test_agent_tools")


# Test Skills
class TestSkills(SkillContainer):
    # description="Mathematical expression to evaluate"
    @skill()
    def calculate(self, expression: str) -> str:
        try:
            # Simple evaluation for testing
            result = eval(expression)
            return f"The result is {result}"
        except Exception as e:
            return f"Error calculating: {str(e)}"

    # "Location to get weather for (e.g. 'London', 'New York')"
    @skill()
    def weather(self, location: str) -> str:
        # Mock weather response
        return f"The weather in {location} is sunny with a temperature of 72°F"

    # destination: str = Field(description="Destination to navigate to")
    # speed: float = Field(default=1.0, description="Navigation speed in m/s")
    @skill()
    def navigation(self, destination: str, speed: float) -> str:
        # In real implementation, this would start navigation
        # For now, simulate blocking behavior
        import time

        time.sleep(0.5)  # Simulate some processing
        return f"Navigation to {destination} completed successfully"


# Module for testing tool execution
class ToolTestController(Module):
    """Controller that sends queries to agent."""

    message_out: Out[AgentMessage] = None

    @rpc
    def send_query(self, query: str):
        msg = AgentMessage()
        msg.add_text(query)
        self.message_out.publish(msg)


class ResponseCollector(Module):
    """Collect agent responses."""

    response_in: In[AgentResponse] = None

    def __init__(self):
        super().__init__()
        self.responses = []

    @rpc
    def start(self):
        logger.info("ResponseCollector starting subscription")
        self.response_in.subscribe(self._on_response)
        logger.info("ResponseCollector subscription active")

    def _on_response(self, response):
        logger.info(f"ResponseCollector received response #{len(self.responses) + 1}: {response}")
        self.responses.append(response)

    @rpc
    def get_responses(self):
        return self.responses


@pytest.mark.module
@pytest.mark.asyncio
async def test_agent_module_with_tools():
    """Test BaseAgentModule with tool execution."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    pubsub.lcm.autoconf()
    dimos = core.start(4)

    try:
        # Create skill library
        skill_library = SkillCoordinator()
        skill_library.register_skills(TestSkills())

        # Deploy modules
        controller = dimos.deploy(ToolTestController)
        controller.message_out.transport = core.pLCMTransport("/tools/messages")

        agent = dimos.deploy(
            BaseAgentModule,
            model="openai::gpt-4o-mini",
            system_prompt="You are a helpful assistant with access to calculation, weather, and navigation tools. When asked about weather, you MUST use the WeatherSkill tool - it provides mock weather data for testing. When asked to navigate somewhere, you MUST use the NavigationSkill tool. Always use the appropriate tool when available.",
            skills=skill_library,
            temperature=0.0,
            memory=False,
        )
        agent.response_out.transport = core.pLCMTransport("/tools/responses")

        collector = dimos.deploy(ResponseCollector)

        # Connect modules
        agent.message_in.connect(controller.message_out)
        collector.response_in.connect(agent.response_out)

        # Start modules
        agent.start()
        collector.start()

        # Wait for initialization
        await asyncio.sleep(1)

        # Test 1: Calculation (fast tool)
        logger.info("\n=== Test 1: Calculation Tool ===")
        controller.send_query("Use the calculate tool to compute 42 * 17")
        await asyncio.sleep(5)  # Give more time for the response

        responses = collector.get_responses()
        logger.info(f"Got {len(responses)} responses after first query")
        assert len(responses) >= 1, (
            f"Should have received at least one response, got {len(responses)}"
        )

        response = responses[-1]
        logger.info(f"Response: {response}")

        # Verify the calculation result
        assert isinstance(response, AgentResponse), "Expected AgentResponse object"
        assert "714" in response.content, f"Expected '714' in response, got: {response.content}"

        # Test 2: Weather query (fast tool)
        logger.info("\n=== Test 2: Weather Tool ===")
        controller.send_query("What's the weather in New York?")
        await asyncio.sleep(5)  # Give more time for the second response

        responses = collector.get_responses()
        assert len(responses) >= 2, "Should have received at least two responses"

        response = responses[-1]
        logger.info(f"Response: {response}")

        # Verify weather details
        assert isinstance(response, AgentResponse), "Expected AgentResponse object"
        assert "new york" in response.content.lower(), f"Expected 'New York' in response"
        assert "72" in response.content, f"Expected temperature '72' in response"
        assert "sunny" in response.content.lower(), f"Expected 'sunny' in response"

        # Test 3: Navigation (potentially long-running)
        logger.info("\n=== Test 3: Navigation Tool ===")
        controller.send_query("Use the NavigationSkill to navigate to the kitchen")
        await asyncio.sleep(6)  # Give more time for navigation tool to complete

        responses = collector.get_responses()
        logger.info(f"Total responses collected: {len(responses)}")
        for i, r in enumerate(responses):
            logger.info(f"  Response {i + 1}: {r.content[:50]}...")
        assert len(responses) >= 3, (
            f"Should have received at least three responses, got {len(responses)}"
        )

        response = responses[-1]
        logger.info(f"Response: {response}")

        # Verify navigation response
        assert isinstance(response, AgentResponse), "Expected AgentResponse object"
        assert "kitchen" in response.content.lower(), "Expected 'kitchen' in response"

        # Check if NavigationSkill was called
        if response.tool_calls is not None and len(response.tool_calls) > 0:
            # Tool was called - verify it
            assert any(tc.name == "navigation" for tc in response.tool_calls), (
                "Expected navigation to be called"
            )
            logger.info("✓ navigation was called")
        else:
            # Tool wasn't called - just verify response mentions navigation
            logger.info("Note: NavigationSkill was not called, agent gave instructions instead")

        # Stop agent
        agent.stop()

        # Print summary
        logger.info("\n=== Test Summary ===")
        all_responses = collector.get_responses()
        for i, resp in enumerate(all_responses):
            logger.info(
                f"Response {i + 1}: {resp.content if isinstance(resp, AgentResponse) else resp}"
            )

    finally:
        dimos.close()
        dimos.shutdown()


def test_base_agent_direct_tools():
    """Test BaseAgent direct usage with tools."""
    load_dotenv()

    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("No OPENAI_API_KEY found")

    # Create skill library
    skill_library = SkillCoordinator()
    skill_library.register_skills(TestSkills())

    print(skill_library.get_tools())

    # Create agent with skills
    agent = BaseAgent(
        model="openai::gpt-4o-mini",
        system_prompt="You are a helpful assistant with access to a calculator tool. When asked to calculate something, you should use the CalculateSkill tool.",
        skills=skill_library,
        temperature=0.0,
        memory=False,
    )

    # Test calculation with explicit tool request
    logger.info("\n=== Direct Test 1: Calculation Tool ===")
    response = agent.query("Calculate 144**0.5 using the 'calculate' tool")

    logger.info(f"Response content: {response.content}")
    logger.info(f"Tool calls: {response.tool_calls}")

    assert response.content is not None
    assert "12" in response.content or "twelve" in response.content.lower(), (
        f"Expected '12' in response, got: {response.content}"
    )

    # Verify tool was called OR answer is correct
    assert response.tool_calls is not None
    assert len(response.tool_calls) > 0, "Expected at least one tool call"
    assert response.tool_calls[0].name == "calculate", (
        f"Expected calculate, got: {response.tool_calls[0].name}"
    )
    assert response.tool_calls[0].status == "completed", (
        f"Expected completed status, got: {response.tool_calls[0].status}"
    )
    logger.info("✓ Tool was called successfully")

    # Test weather tool
    logger.info("\n=== Direct Test 2: Weather Tool ===")
    response2 = agent.query("Use the 'weather' function to check the weather in London")

    logger.info(f"Response content: {response2.content}")
    logger.info(f"Tool calls: {response2.tool_calls}")

    assert response2.content is not None
    assert "london" in response2.content.lower(), f"Expected 'London' in response"
    assert "72" in response2.content, f"Expected temperature '72' in response"
    assert "sunny" in response2.content.lower(), f"Expected 'sunny' in response"

    # Verify tool was called
    if response2.tool_calls is not None:
        assert len(response2.tool_calls) > 0, "Expected at least one tool call"
        assert response2.tool_calls[0].name == "weather", (
            f"Expected weather, got: {response2.tool_calls[0].name}"
        )
        logger.info("✓ Weather tool was called successfully")
    else:
        logger.warning("Weather tool was not called - agent answered directly")

    # Clean up
    agent.dispose()
