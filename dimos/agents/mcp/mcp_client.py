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

import time
from typing import Any
import uuid

import requests

from dimos.utils.sequential_ids import SequentialIds

_RETRY_INTERVAL_S = 1.0


class McpClient:
    """JSON-RPC over HTTP to an MCP server. Connects on first request."""

    def __init__(self, url: str) -> None:
        self.url = url
        self._session = requests.Session()
        self._seq_ids = SequentialIds()

    def request(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        body: dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": self._seq_ids.next(),
            "method": method,
        }
        if params is not None:
            body["params"] = params

        resp = self._session.post(self.url, json=body, timeout=120.0)
        resp.raise_for_status()
        data = resp.json()

        if "error" in data:
            raise RuntimeError(f"MCP error {data['error']['code']}: {data['error']['message']}")

        result: dict[str, Any] = data.get("result")
        return result

    def call_tool(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
        return self.request(
            "tools/call",
            {
                "name": name,
                "arguments": arguments,
                "_meta": {"progressToken": str(uuid.uuid4())},
            },
        )

    def list_tools(self, timeout: float = 60.0) -> list[dict[str, Any]]:
        """The server's tools, waiting up to *timeout* seconds for it to come up."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                self.request("initialize")
                break
            except requests.ConnectionError:
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"Failed to fetch tools from MCP server {self.url}")
                time.sleep(_RETRY_INTERVAL_S)

        tools: list[dict[str, Any]] = self.request("tools/list").get("tools", [])
        return tools

    def close(self) -> None:
        self._session.close()
