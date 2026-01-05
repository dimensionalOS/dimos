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

"""MCP Server exposing DimOS skills as tools to Claude Code and other MCP clients."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any
import uuid

from mcp.server import Server
from mcp.server.stdio import stdio_server
from mcp.types import TextContent, Tool

from dimos.protocol.skill.coordinator import SkillCoordinator
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.protocol.skill.skill import SkillContainer

logger = setup_logger()


class DimensionalMCPServer:
    """MCP Server exposing DimOS skills as tools.

    This server bridges the MCP protocol with DimOS's SkillCoordinator,
    allowing external AI agents like Claude Code to control robots.

    Example:
        >>> from dimos.protocol.mcp import DimensionalMCPServer
        >>> server = DimensionalMCPServer()
        >>> server.register_skills(robot.skill_library)
        >>> await server.run()
    """

    def __init__(self, coordinator: SkillCoordinator | None = None):
        """Initialize the MCP server.

        Args:
            coordinator: Optional SkillCoordinator to use. If not provided,
                        a new one will be created.
        """
        self.coordinator = coordinator if coordinator is not None else SkillCoordinator()
        self.server = Server("dimensional")
        self._setup_handlers()

    def _setup_handlers(self) -> None:
        """Set up MCP protocol handlers."""

        @self.server.list_tools()
        async def list_tools() -> list[Tool]:
            """List all available skills as MCP tools."""
            tools = []
            for config in self.coordinator.skills().values():
                if config.hide_skill:
                    continue

                schema = config.schema
                func_schema = schema.get("function", {})

                tools.append(
                    Tool(
                        name=config.name,
                        description=func_schema.get("description", ""),
                        inputSchema=func_schema.get(
                            "parameters", {"type": "object", "properties": {}}
                        ),
                    )
                )
            return tools

        @self.server.call_tool()
        async def call_tool(name: str, arguments: dict[str, Any] | None) -> list[TextContent]:
            """Execute a skill and return the result."""
            call_id = str(uuid.uuid4())
            args = arguments or {}

            logger.info(f"MCP: Calling skill '{name}' with args: {args}")

            # Check if skill exists
            skill_config = self.coordinator.get_skill_config(name)
            if not skill_config:
                return [TextContent(type="text", text=f"Error: Skill '{name}' not found")]

            # Execute the skill
            self.coordinator.call_skill(call_id, name, args)

            # Wait for skill completion
            try:
                await self.coordinator.wait_for_updates(timeout=60)
            except Exception as e:
                logger.error(f"MCP: Error waiting for skill updates: {e}")
                return [TextContent(type="text", text=f"Error: {e}")]

            # Get the result
            snapshot = self.coordinator.generate_snapshot()
            result = snapshot.get(call_id)

            if result is None:
                return [TextContent(type="text", text="Skill executed but no result returned")]

            content = result.content()
            if content is None:
                content = "Skill completed successfully"

            return [TextContent(type="text", text=str(content))]

    def register_skills(self, container: SkillContainer) -> None:
        """Register a skill container with the coordinator.

        Args:
            container: SkillContainer with skills to expose via MCP.
        """
        self.coordinator.register_skills(container)

    async def run(self) -> None:
        """Run the MCP server using stdio transport.

        This starts the coordinator and runs the MCP server,
        listening for requests on stdin and responding on stdout.
        """
        logger.info("Starting Dimensional MCP Server...")
        self.coordinator.start()

        try:
            async with stdio_server() as (read_stream, write_stream):
                logger.info("MCP Server ready, listening for requests...")
                await self.server.run(
                    read_stream,
                    write_stream,
                    self.server.create_initialization_options(),
                )
        finally:
            logger.info("Shutting down MCP Server...")
            self.coordinator.stop()

    def stop(self) -> None:
        """Stop the MCP server and coordinator."""
        self.coordinator.stop()
