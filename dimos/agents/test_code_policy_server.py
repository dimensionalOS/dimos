# Copyright 2026 Dimensional Inc.
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

import asyncio
import logging
from pathlib import Path

from mcp import Client
import pytest

from dimos.agents.code_policy_core import CodePolicySessionConfig, FrozenMemoryEnvironment
from dimos.agents.code_policy_server import CodePolicyMcpServer
from dimos.memory2.store.sqlite import SqliteStore


def _config(tmp_path: Path) -> CodePolicySessionConfig:
    source = tmp_path / "source.db"
    derived = tmp_path / "derived.db"
    with SqliteStore(path=str(source)) as store:
        store.stream("messages", str).append("visible", ts=1.0)
    with SqliteStore(path=str(derived)) as store:
        store.stream("global_map", str).append("map", ts=1.0)
    return CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(source),
            derived_recording_path=str(derived),
            memory_cutoff_timestamp=1.0,
        )
    )


@pytest.mark.asyncio
async def test_server_exposes_exactly_one_persistent_python_tool(
    tmp_path: Path, caplog: pytest.LogCaptureFixture
) -> None:
    server = CodePolicyMcpServer(_config(tmp_path))
    with caplog.at_level(logging.INFO):
        server.start()
        try:
            async with Client(server.mcp_url) as client:
                tools = await client.list_tools()
                assert [tool.name for tool in tools.tools] == ["python_exec"]
                first = await client.call_tool("python_exec", {"code": "items = [1]\nitems"})
                second = await client.call_tool("python_exec", {"code": "items.append(2)\nitems"})
                assert "[1]" in first.content[0].text
                assert "[1, 2]" in second.content[0].text
                assert server.session.execution_count == 2
        finally:
            await asyncio.to_thread(server.stop)
    assert "StreamableHTTP session manager started" not in caplog.text
    assert "Terminating session" not in caplog.text


@pytest.mark.asyncio
async def test_kernel_start_failure_is_returned_as_tool_text(tmp_path: Path) -> None:
    config = CodePolicySessionConfig(
        environment=FrozenMemoryEnvironment(
            recording_path=str(tmp_path / "missing.db"),
            derived_recording_path=str(tmp_path / "also-missing.db"),
            memory_cutoff_timestamp=1.0,
        )
    )
    server = CodePolicyMcpServer(config)
    server.start()
    try:
        async with Client(server.mcp_url) as client:
            result = await client.call_tool("python_exec", {"code": "1 + 1"})
            assert "failed to start" in result.content[0].text
    finally:
        await asyncio.to_thread(server.stop)
