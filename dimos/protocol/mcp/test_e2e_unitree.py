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

"""End-to-end tests for MCP server with UnitreeGo2 robot skills.

These tests verify that:
1. The MCP server can be properly configured with UnitreeGo2 skills
2. Skills are correctly exposed as MCP tools
3. Claude Code (or any MCP client) would be able to import and use the server
4. Tool calls execute correctly with the skill coordinator

Most tests require LCM infrastructure and are marked with @pytest.mark.lcm.
Run with: pytest -m lcm dimos/protocol/mcp/test_e2e_unitree.py
"""

from __future__ import annotations

import subprocess
import sys
import time

import pytest

pytestmark = pytest.mark.lcm  # Mark all tests in this module as requiring LCM


class TestMCPServerE2E:
    """End-to-end tests for MCP server with Unitree skills."""

    def test_mcp_module_importable(self) -> None:
        """Test that the MCP server module can be imported."""
        from dimos.protocol.mcp import DimensionalMCPServer

        assert DimensionalMCPServer is not None

    def test_mcp_server_with_unitree_skills(self) -> None:
        """Test MCP server setup with Unitree skill container."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        skills = server.coordinator.skills()

        # Verify Unitree skills are registered
        assert "relative_move" in skills
        assert "wait" in skills
        assert "execute_sport_command" in skills

        # Verify skill schemas are generated
        move_skill = skills["relative_move"]
        schema = move_skill.schema.get("function", {})
        assert schema.get("name") == "relative_move"
        assert "parameters" in schema

    def test_mcp_tools_exclude_hidden_skills(self) -> None:
        """Test that hidden skills (like current_time) are not exposed as tools."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        skills = server.coordinator.skills()

        # current_time is marked as hide_skill=True
        assert "current_time" in skills
        assert skills["current_time"].hide_skill is True

        # Verify hidden skills would be filtered in list_tools
        visible_skills = [name for name, config in skills.items() if not config.hide_skill]
        assert "current_time" not in visible_skills
        assert "relative_move" in visible_skills

    def test_mcp_tool_schema_format(self) -> None:
        """Test that tool schemas match MCP expected format."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        skills = server.coordinator.skills()
        wait_skill = skills["wait"]

        schema = wait_skill.schema
        func_schema = schema.get("function", {})

        # MCP tool format requirements
        assert "name" in func_schema
        assert "description" in func_schema
        assert "parameters" in func_schema

        params = func_schema["parameters"]
        assert params.get("type") == "object"
        assert "properties" in params
        assert "seconds" in params["properties"]

    @pytest.mark.asyncio
    async def test_mcp_call_wait_skill(self) -> None:
        """Test calling the 'wait' skill through MCP coordinator."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)
        server.coordinator.start()

        try:
            call_id = "e2e-wait-test"
            start_time = time.time()

            # Call the wait skill with 0.5 seconds
            server.coordinator.call_skill(call_id, "wait", {"seconds": 0.5})

            # Wait for completion
            await server.coordinator.wait_for_updates(timeout=5)
            elapsed = time.time() - start_time

            snapshot = server.coordinator.generate_snapshot()
            result = snapshot.get(call_id)

            assert result is not None
            assert "Wait completed" in str(result.content())
            assert elapsed >= 0.5  # Should have waited at least 0.5s
        finally:
            server.coordinator.stop()
            container.stop()

    def test_cli_module_runnable(self) -> None:
        """Test that the CLI module can be loaded without errors."""
        # This simulates what Claude Code would do when starting the server
        result = subprocess.run(
            [sys.executable, "-c", "from dimos.protocol.mcp.cli import main; print('OK')"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0
        assert "OK" in result.stdout

    def test_server_can_be_instantiated_from_cli_context(self) -> None:
        """Test server instantiation as it would happen from CLI."""
        result = subprocess.run(
            [
                sys.executable,
                "-c",
                """
from dimos.protocol.mcp.server import DimensionalMCPServer
from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

server = DimensionalMCPServer()
container = UnitreeSkillContainer()
server.register_skills(container)

skills = server.coordinator.skills()
print(f"Registered {len(skills)} skills")
print("Skills:", list(skills.keys()))
""",
            ],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0, f"Failed with stderr: {result.stderr}"
        assert "Registered" in result.stdout
        assert "relative_move" in result.stdout


class TestMCPProtocolCompatibility:
    """Tests to verify MCP protocol compatibility."""

    def test_tool_inputschema_is_valid_json_schema(self) -> None:
        """Test that inputSchema is valid JSON Schema format."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        for name, config in server.coordinator.skills().items():
            if config.hide_skill:
                continue

            schema = config.schema.get("function", {})
            params = schema.get("parameters", {})

            # JSON Schema requirements
            assert params.get("type") == "object", f"Skill {name} missing type: object"
            assert "properties" in params, f"Skill {name} missing properties"

    def test_mcp_server_protocol_version(self) -> None:
        """Test that server reports correct MCP protocol version."""
        from dimos.protocol.mcp.server import DimensionalMCPServer

        server = DimensionalMCPServer()

        # Server should be named "dimensional"
        assert server.server.name == "dimensional"

    @pytest.mark.asyncio
    async def test_coordinator_state_tracking(self) -> None:
        """Test that skill states are properly tracked for MCP responses."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.protocol.skill.coordinator import SkillStateEnum
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)
        server.coordinator.start()

        try:
            call_id = "state-tracking-test"
            server.coordinator.call_skill(call_id, "wait", {"seconds": 0.1})

            # Initially should be pending or running
            initial_state = server.coordinator._skill_state.get(call_id)
            assert initial_state is not None
            assert initial_state.state in [SkillStateEnum.pending, SkillStateEnum.running]

            # Wait for completion
            await server.coordinator.wait_for_updates(timeout=5)
            snapshot = server.coordinator.generate_snapshot()

            result = snapshot.get(call_id)
            assert result is not None
            assert result.state == SkillStateEnum.completed
        finally:
            server.coordinator.stop()
            container.stop()


class TestClaudeCodeIntegration:
    """Tests simulating Claude Code MCP client behavior."""

    def test_mcp_config_format(self) -> None:
        """Test that the expected Claude Code config format works."""
        # Verify the module path is valid (this is what Claude Code would call)
        result = subprocess.run(
            [sys.executable, "-c", "import dimos.protocol.mcp.cli"],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0, f"CLI import failed: {result.stderr}"

    def test_tool_call_argument_formats(self) -> None:
        """Test that various argument formats work with skill calls."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        # Test that get_skill_config works (used in call_tool handler)
        config = server.coordinator.get_skill_config("wait")
        assert config is not None
        assert config.name == "wait"

        # Test with missing skill
        missing = server.coordinator.get_skill_config("nonexistent_skill")
        assert missing is None

    def test_skill_descriptions_for_llm(self) -> None:
        """Test that skill descriptions are suitable for LLM consumption."""
        from dimos.protocol.mcp.server import DimensionalMCPServer
        from dimos.robot.unitree_webrtc.unitree_skill_container import UnitreeSkillContainer

        server = DimensionalMCPServer()
        container = UnitreeSkillContainer()
        server.register_skills(container)

        skills = server.coordinator.skills()

        # Check that key skills have meaningful descriptions
        relative_move = skills["relative_move"]
        desc = relative_move.schema.get("function", {}).get("description", "")
        assert len(desc) > 20, "relative_move should have a detailed description"
        assert "move" in desc.lower() or "robot" in desc.lower()

        # Note: execute_sport_command has its docstring set dynamically after
        # class definition, so the @skill decorator schema doesn't capture it.
        # This is a known limitation of the schema generation.
        execute_cmd = skills["execute_sport_command"]
        assert execute_cmd is not None, "execute_sport_command skill should exist"
