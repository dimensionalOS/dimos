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

from types import SimpleNamespace

import pytest

from dimos.agents.mcp.mcp_adapter import McpAdapter
from dimos.core.global_config import global_config


def test_call_sends_config_timeout_unless_overridden(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(global_config, "mcp_timeout", 11)
    posted: list[object] = []

    def test_post(*args: object, **kwargs: object) -> SimpleNamespace:
        posted.append(kwargs["timeout"])
        return SimpleNamespace(raise_for_status=lambda: None, json=lambda: {"result": {}})

    monkeypatch.setattr("dimos.agents.mcp.mcp_adapter.requests.post", test_post)

    McpAdapter(url="http://127.0.0.1:9/mcp").call("initialize")
    McpAdapter(url="http://127.0.0.1:9/mcp", timeout=7).call("initialize")

    assert posted == [11, 7]
