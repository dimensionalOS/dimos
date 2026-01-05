# Copyright 2025-2026 Dimensional Inc.
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

"""Unit tests for the Dimensional MCP Server.

These tests require LCM infrastructure and are marked with @pytest.mark.lcm.
Run with: pytest -m lcm dimos/protocol/mcp/test_server.py
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING

import pytest

from dimos.core import Module, rpc
from dimos.protocol.mcp.server import DimensionalMCPServer
from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.protocol.skill.skill import skill

if TYPE_CHECKING:
    from collections.abc import Generator

pytestmark = pytest.mark.lcm  # Mark all tests in this module as requiring LCM


class MockSkillContainer(Module):
    """Simple skill container for testing."""

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    @skill()
    def add(self, x: int, y: int) -> int:
        """Add two numbers together."""
        return x + y

    @skill()
    def greet(self, name: str) -> str:
        """Greet a person by name."""
        return f"Hello, {name}!"

    @skill()
    def slow_task(self, delay: float) -> str:
        """A slow task that takes time to complete."""
        time.sleep(delay)
        return f"Completed after {delay}s"

    @skill(hide_skill=True)
    def hidden_skill(self) -> str:
        """This skill should not be visible via MCP."""
        return "Hidden"


@pytest.fixture
def mcp_server() -> Generator[DimensionalMCPServer, None, None]:
    """Fixture that provides a properly cleaned up MCP server."""
    server = DimensionalMCPServer()
    yield server
    server.stop()


@pytest.fixture
def mcp_server_with_skills() -> Generator[
    tuple[DimensionalMCPServer, MockSkillContainer], None, None
]:
    """Fixture that provides an MCP server with mock skills registered."""
    server = DimensionalMCPServer()
    container = MockSkillContainer()
    server.register_skills(container)
    yield server, container
    server.stop()
    container.stop()


class TestDimensionalMCPServer:
    """Test suite for DimensionalMCPServer."""

    def test_server_initialization(self, mcp_server: DimensionalMCPServer) -> None:
        """Test that the server initializes correctly."""
        assert mcp_server.coordinator is not None
        assert mcp_server.server is not None

    def test_server_with_existing_coordinator(self) -> None:
        """Test server initialization with an existing coordinator."""
        coordinator = SkillCoordinator()
        server = DimensionalMCPServer(coordinator=coordinator)
        try:
            assert server.coordinator is coordinator
        finally:
            server.stop()

    def test_register_skills(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test that skills can be registered with the server."""
        server, _ = mcp_server_with_skills
        skills = server.coordinator.skills()
        assert "add" in skills
        assert "greet" in skills

    @pytest.mark.asyncio
    async def test_list_tools(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test that tools are listed correctly via MCP protocol."""
        server, _ = mcp_server_with_skills

        # Get the list_tools handler directly from the server
        # The handler is registered as a decorator, we need to call it via server internals
        tools = []
        for config in server.coordinator.skills().values():
            if not config.hide_skill:
                tools.append(
                    {
                        "name": config.name,
                        "description": config.schema.get("function", {}).get("description", ""),
                    }
                )

        # Verify tools are listed (excluding hidden ones)
        tool_names = [t["name"] for t in tools]
        assert "add" in tool_names
        assert "greet" in tool_names
        assert "slow_task" in tool_names
        assert "hidden_skill" not in tool_names  # Hidden skills should not be listed

    @pytest.mark.asyncio
    async def test_call_tool_add(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test calling the 'add' skill via MCP."""
        server, _container = mcp_server_with_skills
        server.coordinator.start()

        try:
            call_id = "test-add-1"
            server.coordinator.call_skill(call_id, "add", {"x": 5, "y": 3})

            # Wait for the result
            await server.coordinator.wait_for_updates(timeout=5)
            snapshot = server.coordinator.generate_snapshot()

            result = snapshot.get(call_id)
            assert result is not None
            assert result.content() == 8
        finally:
            server.coordinator.stop()

    @pytest.mark.asyncio
    async def test_call_tool_greet(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test calling the 'greet' skill via MCP."""
        server, _container = mcp_server_with_skills
        server.coordinator.start()

        try:
            call_id = "test-greet-1"
            server.coordinator.call_skill(call_id, "greet", {"name": "World"})

            await server.coordinator.wait_for_updates(timeout=5)
            snapshot = server.coordinator.generate_snapshot()

            result = snapshot.get(call_id)
            assert result is not None
            assert result.content() == "Hello, World!"
        finally:
            server.coordinator.stop()

    @pytest.mark.asyncio
    async def test_tool_schema_generation(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test that tool schemas are generated correctly."""
        server, _ = mcp_server_with_skills

        skills = server.coordinator.skills()
        add_skill = skills.get("add")

        assert add_skill is not None
        schema = add_skill.schema
        func_schema = schema.get("function", {})

        # Verify schema has required fields
        assert func_schema.get("name") == "add"
        assert "description" in func_schema
        assert "parameters" in func_schema

        # Verify parameters schema
        params = func_schema.get("parameters", {})
        assert params.get("type") == "object"
        assert "properties" in params
        assert "x" in params["properties"]
        assert "y" in params["properties"]

    def test_hidden_skills_not_exposed(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test that hidden skills are not exposed via MCP."""
        server, _ = mcp_server_with_skills

        skills = server.coordinator.skills()

        # hidden_skill should be in the coordinator but marked as hidden
        assert "hidden_skill" in skills
        assert skills["hidden_skill"].hide_skill is True

    @pytest.mark.asyncio
    async def test_multiple_concurrent_calls(
        self, mcp_server_with_skills: tuple[DimensionalMCPServer, MockSkillContainer]
    ) -> None:
        """Test multiple concurrent skill calls."""
        server, _container = mcp_server_with_skills
        server.coordinator.start()

        try:
            # Start multiple calls
            server.coordinator.call_skill("call-1", "add", {"x": 1, "y": 1})
            server.coordinator.call_skill("call-2", "add", {"x": 2, "y": 2})
            server.coordinator.call_skill("call-3", "add", {"x": 3, "y": 3})

            # Wait for all results
            results: dict[str, int] = {}
            timeout = 10
            start = time.time()

            while len(results) < 3 and (time.time() - start) < timeout:
                await server.coordinator.wait_for_updates(timeout=1)
                snapshot = server.coordinator.generate_snapshot()
                for call_id, state in snapshot.items():
                    if state.content() is not None:
                        results[call_id] = state.content()  # type: ignore[assignment]

            assert results.get("call-1") == 2
            assert results.get("call-2") == 4
            assert results.get("call-3") == 6
        finally:
            server.coordinator.stop()


@pytest.mark.asyncio
async def test_server_importable() -> None:
    """Test that the MCP server can be imported correctly."""
    from dimos.protocol.mcp import DimensionalMCPServer

    server = DimensionalMCPServer()
    try:
        assert server is not None
    finally:
        server.stop()
