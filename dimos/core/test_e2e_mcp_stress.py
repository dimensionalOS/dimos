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

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
import time

import pytest
import requests
from typer.testing import CliRunner

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.blueprints import autoconnect
from dimos.core.daemon import health_check
from dimos.core.global_config import global_config
from dimos.core.run_registry import (
    RunEntry,
    cleanup_stale,
    list_runs,
)
from dimos.core.tests.stress_test_module import StressTestModule
from dimos.robot.cli.dimos import main

MCP_PORT = 9990  # non-default to avoid conflicts


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def _ci_env(monkeypatch):
    monkeypatch.setenv("CI", "1")


@pytest.fixture(autouse=True)
def _clean_registry(tmp_path, monkeypatch):
    import dimos.core.run_registry as _reg

    test_dir = tmp_path / "runs"
    test_dir.mkdir()
    monkeypatch.setattr(_reg, "REGISTRY_DIR", test_dir)
    yield test_dir


@pytest.fixture()
def mcp_blueprint():
    """Build a StressTestModule + McpServer blueprint. Stops on teardown."""
    global_config.update(viewer_backend="none", n_workers=1)
    bp = autoconnect(
        StressTestModule.blueprint(),
        McpServer.blueprint(),
    )
    coord = bp.build()
    yield coord
    coord.stop()


@pytest.fixture()
def mcp_entry(mcp_blueprint, tmp_path):
    """Create registry entry for the running blueprint."""
    log_dir = tmp_path / "stress-logs"
    entry = RunEntry(
        run_id=f"stress-{datetime.now(timezone.utc).strftime('%H%M%S%f')}",
        pid=os.getpid(),
        blueprint="stress-test",
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir=str(log_dir),
        cli_args=["stress-test"],
        config_overrides={"n_workers": 1},
    )
    entry.save()
    yield entry
    entry.remove()


def _mcp_call(method: str, params: dict | None = None, port: int = MCP_PORT) -> dict:
    """Send a JSON-RPC request to MCP server."""
    payload: dict = {"jsonrpc": "2.0", "id": 1, "method": method}
    if params:
        payload["params"] = params
    resp = requests.post(f"http://localhost:{port}/mcp", json=payload, timeout=10)
    resp.raise_for_status()
    return resp.json()


def _wait_for_mcp(port: int = MCP_PORT, timeout: float = 10.0) -> bool:
    """Poll until MCP server responds or timeout."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            resp = requests.post(
                f"http://localhost:{port}/mcp",
                json={"jsonrpc": "2.0", "id": 1, "method": "initialize"},
                timeout=2,
            )
            if resp.status_code == 200:
                return True
        except requests.ConnectionError:
            pass
        time.sleep(0.5)
    return False


def _wait_for_mcp_down(port: int = MCP_PORT, timeout: float = 10.0) -> bool:
    """Poll until MCP server stops responding."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            requests.post(
                f"http://localhost:{port}/mcp",
                json={"jsonrpc": "2.0", "id": 1, "method": "initialize"},
                timeout=1,
            )
        except (requests.ConnectionError, requests.ReadTimeout):
            return True
        time.sleep(0.5)
    return False


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


@pytest.mark.slow
class TestMCPLifecycle:
    """MCP server lifecycle: start → respond → stop → dead."""

    def test_mcp_responds_after_build(self, mcp_blueprint):
        """After blueprint.build(), MCP should accept requests."""
        assert _wait_for_mcp(), "MCP server did not start"

        result = _mcp_call("initialize")
        assert result["result"]["serverInfo"]["name"] == "dimensional"

    def test_tools_list_includes_stress_skills(self, mcp_blueprint):
        """tools/list should include echo, ping, slow, info from StressTestModule."""
        assert _wait_for_mcp()

        result = _mcp_call("tools/list")
        tool_names = {t["name"] for t in result["result"]["tools"]}
        assert "echo" in tool_names
        assert "ping" in tool_names
        assert "slow" in tool_names
        assert "info" in tool_names

    def test_echo_tool(self, mcp_blueprint):
        """Call echo tool — should return the message."""
        assert _wait_for_mcp()

        result = _mcp_call(
            "tools/call",
            {
                "name": "echo",
                "arguments": {"message": "hello from stress test"},
            },
        )
        text = result["result"]["content"][0]["text"]
        assert text == "hello from stress test"

    def test_ping_tool(self, mcp_blueprint):
        """Call ping tool — should return 'pong'."""
        assert _wait_for_mcp()

        result = _mcp_call("tools/call", {"name": "ping", "arguments": {}})
        text = result["result"]["content"][0]["text"]
        assert text == "pong"

    def test_info_tool_returns_pid(self, mcp_blueprint):
        """info tool should return process info."""
        assert _wait_for_mcp()

        result = _mcp_call("tools/call", {"name": "info", "arguments": {}})
        text = result["result"]["content"][0]["text"]
        assert "pid=" in text

    def test_dimos_status_method(self, mcp_blueprint):
        """dimos/status should return module and skill info."""
        assert _wait_for_mcp()

        result = _mcp_call("dimos/status")
        data = result["result"]
        assert "pid" in data
        assert "modules" in data
        assert "skills" in data
        assert "StressTestModule" in data["modules"]

    def test_dimos_list_modules_method(self, mcp_blueprint):
        """dimos/list_modules should group skills by module."""
        assert _wait_for_mcp()

        result = _mcp_call("dimos/list_modules")
        modules = result["result"]["modules"]
        assert "StressTestModule" in modules
        assert "echo" in modules["StressTestModule"]

    def test_mcp_dead_after_stop(self):
        """After coordinator.stop(), MCP should stop responding."""
        global_config.update(viewer_backend="none", n_workers=1)
        bp = autoconnect(StressTestModule.blueprint(), McpServer.blueprint())
        coord = bp.build()
        assert _wait_for_mcp(), "MCP server did not start"

        coord.stop()
        assert _wait_for_mcp_down(), "MCP server did not stop"


@pytest.mark.slow
class TestDaemonMCPRecovery:
    """Test MCP recovery after daemon crashes and restarts."""

    def test_restart_after_clean_stop(self):
        """Stop then start again — MCP should come back."""
        global_config.update(viewer_backend="none", n_workers=1)

        # First run
        bp1 = autoconnect(StressTestModule.blueprint(), McpServer.blueprint())
        coord1 = bp1.build()
        assert _wait_for_mcp(), "First MCP start failed"
        coord1.stop()
        assert _wait_for_mcp_down(), "MCP didn't stop"

        # Second run — should work without port conflicts
        bp2 = autoconnect(StressTestModule.blueprint(), McpServer.blueprint())
        coord2 = bp2.build()
        assert _wait_for_mcp(), "Second MCP start failed (port conflict?)"
        coord2.stop()

    def test_registry_cleanup_after_stop(self, mcp_blueprint, mcp_entry):
        """Registry entry should be removable after stop."""
        assert health_check(mcp_blueprint)

        runs = list_runs(alive_only=True)
        assert len(runs) == 1

        mcp_blueprint.stop()
        mcp_entry.remove()

        runs = list_runs(alive_only=True)
        assert len(runs) == 0

    def test_stale_cleanup_after_crash(self):
        """Stale entries from crashed processes should be cleaned up."""
        stale = RunEntry(
            run_id="crashed-mcp-test",
            pid=99999999,
            blueprint="stress-test",
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir="/tmp/ghost",
            cli_args=[],
            config_overrides={},
        )
        stale.save()
        assert len(list_runs(alive_only=False)) == 1

        removed = cleanup_stale()
        assert removed == 1
        assert len(list_runs(alive_only=False)) == 0


@pytest.mark.slow
class TestMCPCLICommands:
    """Test dimos mcp CLI commands against real MCP server."""

    def test_cli_list_tools(self, mcp_blueprint):
        """dimos mcp list-tools should output JSON with tools."""
        assert _wait_for_mcp()

        result = CliRunner().invoke(main, ["mcp", "list-tools"])
        assert result.exit_code == 0
        tools = json.loads(result.output)
        tool_names = {t["name"] for t in tools}
        assert "echo" in tool_names
        assert "ping" in tool_names

    def test_cli_call_echo(self, mcp_blueprint):
        """dimos mcp call echo --arg message=hello should return hello."""
        assert _wait_for_mcp()

        result = CliRunner().invoke(main, ["mcp", "call", "echo", "--arg", "message=hello"])
        assert result.exit_code == 0
        assert "hello" in result.output

    def test_cli_mcp_status(self, mcp_blueprint):
        """dimos mcp status should return JSON with modules."""
        assert _wait_for_mcp()

        result = CliRunner().invoke(main, ["mcp", "status"])
        assert result.exit_code == 0
        data = json.loads(result.output)
        assert "StressTestModule" in data["modules"]

    def test_cli_mcp_modules(self, mcp_blueprint):
        """dimos mcp modules should show module-skill mapping."""
        assert _wait_for_mcp()

        result = CliRunner().invoke(main, ["mcp", "modules"])
        assert result.exit_code == 0
        data = json.loads(result.output)
        assert "StressTestModule" in data["modules"]
        assert "echo" in data["modules"]["StressTestModule"]

    def test_cli_no_server_error(self):
        """dimos mcp list-tools with no server should exit with error."""
        result = CliRunner().invoke(main, ["mcp", "list-tools"])
        assert result.exit_code == 1
        assert "no running" in result.output.lower() or "error" in result.output.lower()


@pytest.mark.slow
class TestMCPErrorHandling:
    """Test MCP error handling and edge cases."""

    def test_call_nonexistent_tool(self, mcp_blueprint):
        """Calling a tool that doesn't exist should return an error message."""
        assert _wait_for_mcp()

        result = _mcp_call(
            "tools/call",
            {
                "name": "nonexistent_tool_xyz",
                "arguments": {},
            },
        )
        text = result["result"]["content"][0]["text"]
        assert "not found" in text.lower()

    def test_call_tool_wrong_args(self, mcp_blueprint):
        """Calling a tool with wrong argument types should still return."""
        assert _wait_for_mcp()

        # echo expects string, send nothing — should still work or error gracefully
        result = _mcp_call(
            "tools/call",
            {
                "name": "echo",
                "arguments": {},
            },
        )
        # Should not crash — either returns error or empty result
        assert "result" in result or "error" in result

    def test_unknown_jsonrpc_method(self, mcp_blueprint):
        """Unknown JSON-RPC method should return error."""
        assert _wait_for_mcp()

        result = _mcp_call("nonexistent/method")
        assert "error" in result
        assert result["error"]["code"] == -32601

    def test_mcp_handles_malformed_json(self, mcp_blueprint):
        """MCP should handle malformed JSON gracefully."""
        assert _wait_for_mcp()

        resp = requests.post(
            f"http://localhost:{MCP_PORT}/mcp",
            data=b"not json{{{",
            headers={"Content-Type": "application/json"},
            timeout=5,
        )
        assert resp.status_code == 400

    def test_rapid_tool_calls(self, mcp_blueprint):
        """Fire 20 rapid echo calls — all should succeed."""
        assert _wait_for_mcp()

        for i in range(20):
            result = _mcp_call(
                "tools/call",
                {
                    "name": "echo",
                    "arguments": {"message": f"rapid-{i}"},
                },
            )
            text = result["result"]["content"][0]["text"]
            assert text == f"rapid-{i}", f"Mismatch on call {i}"

    def test_cli_call_tool_wrong_arg_format(self, mcp_blueprint):
        """dimos mcp call with bad --arg format should error."""
        assert _wait_for_mcp()

        result = CliRunner().invoke(main, ["mcp", "call", "echo", "--arg", "no_equals_sign"])
        assert result.exit_code == 1
        assert "key=value" in result.output


@pytest.mark.slow
class TestMCPRapidRestart:
    """Test rapid stop/start cycles."""

    def test_three_restart_cycles(self):
        """Start → stop → start 3 times — no port conflicts."""
        global_config.update(viewer_backend="none", n_workers=1)

        for cycle in range(3):
            bp = autoconnect(StressTestModule.blueprint(), McpServer.blueprint())
            coord = bp.build()
            assert _wait_for_mcp(timeout=15), f"MCP failed to start on cycle {cycle}"

            result = _mcp_call("tools/call", {"name": "ping", "arguments": {}})
            assert result["result"]["content"][0]["text"] == "pong"

            coord.stop()
            assert _wait_for_mcp_down(timeout=10), f"MCP failed to stop on cycle {cycle}"
