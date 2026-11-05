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

import os
from types import SimpleNamespace

import pytest

from dimos.agents.mcp.constants import DEFAULT_MCP_TIMEOUT, mcp_url
from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.core.run_registry import RunEntry


def test_call_sends_config_timeout_unless_overridden(monkeypatch: pytest.MonkeyPatch) -> None:
    posted: list[object] = []

    def test_post(*args: object, **kwargs: object) -> SimpleNamespace:
        posted.append(kwargs["timeout"])
        return SimpleNamespace(raise_for_status=lambda: None, json=lambda: {"result": {}})

    monkeypatch.setattr("dimos.agents.mcp.mcp_adapter.requests.post", test_post)

    McpAdapter(url="http://127.0.0.1:9/mcp").call("initialize")
    McpAdapter(url="http://127.0.0.1:9/mcp", timeout=7).call("initialize")

    assert posted == [DEFAULT_MCP_TIMEOUT, 7]


def test_adapter_uses_the_endpoint_persisted_by_the_running_server(mocker, tmp_path) -> None:
    mocker.patch("dimos.core.run_registry.REGISTRY_DIR", tmp_path)
    entry = RunEntry(
        run_id="test-mcp-endpoint",
        pid=1,
        blueprint="test",
        started_at="now",
        log_dir="logs",
        mcp_url="http://127.0.0.1:12345/mcp",
    )
    entry.save()
    post = mocker.patch("dimos.agents.mcp.mcp_adapter.requests.post")
    post.return_value.json.return_value = {"result": {}}

    McpAdapter.from_run_entry(RunEntry.load(entry.registry_path), timeout=7).initialize()

    assert post.call_args.args == ("http://127.0.0.1:12345/mcp",)
    assert post.call_args.kwargs["timeout"] == 7


def test_discovery_picks_the_newest_run_that_serves_mcp(mocker, tmp_path) -> None:
    mocker.patch("dimos.core.run_registry.REGISTRY_DIR", tmp_path)
    for run_id, url in (
        ("20260101-000001-aaaa-old", "http://127.0.0.1:9991/mcp"),
        ("20260101-000002-bbbb-new", "http://127.0.0.1:9992/mcp"),
        ("20260101-000003-cccc-no-mcp", None),
    ):
        RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint="t",
            started_at="now",
            log_dir="l",
            mcp_url=url,
        ).save()

    assert McpAdapter.from_run_entry().url == "http://127.0.0.1:9992/mcp"


@pytest.mark.parametrize(
    ("host", "expected"),
    [
        ("0.0.0.0", "http://localhost:9992/mcp"),
        ("::", "http://localhost:9992/mcp"),
        ("::1", "http://[::1]:9992/mcp"),
        ("192.0.2.10", "http://192.0.2.10:9992/mcp"),
    ],
)
def test_mcp_endpoint_is_connectable_for_bind_addresses(host: str, expected: str) -> None:
    assert mcp_url(host, 9992) == expected
