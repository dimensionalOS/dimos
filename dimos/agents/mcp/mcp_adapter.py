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

"""Lightweight MCP JSON-RPC client adapter.

``McpAdapter`` provides a typed Python interface to a running MCP server.
It is used by:

* The ``dimos mcp`` CLI commands
* Integration / e2e tests
* Any code that needs to talk to a local MCP server

Usage::

    adapter = McpAdapter("http://localhost:9990/mcp")
    adapter.wait_for_ready(timeout=10)
    tools = adapter.list_tools()
    result = adapter.call_tool("echo", {"message": "hi"})
"""

from __future__ import annotations

import time
from typing import Any
import uuid

import requests

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_TIMEOUT = 30


class McpError(Exception):
    """Raised when the MCP server returns a JSON-RPC error."""

    def __init__(self, message: str, code: int | None = None) -> None:
        self.code = code
        super().__init__(message)


class McpAdapter:
    """Thin JSON-RPC client for a running MCP server."""

    def __init__(self, url: str | None = None, timeout: int = DEFAULT_TIMEOUT) -> None:
        if url is None:
            from dimos.core.global_config import global_config

            url = f"http://localhost:{global_config.mcp_port}/mcp"
        self.url = url
        self.timeout = timeout

    # ------------------------------------------------------------------
    # Low-level JSON-RPC
    # ------------------------------------------------------------------

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict[str, Any]:
        """Send a JSON-RPC request and return the parsed response.

        Raises ``requests.ConnectionError`` if the server is unreachable.
        """
        payload: dict[str, Any] = {
            "jsonrpc": "2.0",
            "id": str(uuid.uuid4()),
            "method": method,
        }
        if params:
            payload["params"] = params

        resp = requests.post(self.url, json=payload, timeout=self.timeout)
        try:
            resp.raise_for_status()
        except requests.HTTPError as e:
            raise McpError(f"HTTP {resp.status_code}: {e}") from e
        return resp.json()  # type: ignore[no-any-return]

    # ------------------------------------------------------------------
    # MCP standard methods
    # ------------------------------------------------------------------

    def initialize(self) -> dict[str, Any]:
        """Send ``initialize`` and return server info."""
        return self.call("initialize")

    def list_tools(self) -> list[dict[str, Any]]:
        """Return the list of available tools."""
        result = self._unwrap(self.call("tools/list"))
        return result.get("tools", [])  # type: ignore[no-any-return]

    def call_tool(self, name: str, arguments: dict[str, Any] | None = None) -> dict[str, Any]:
        """Call a tool by name and return the result dict."""
        return self._unwrap(self.call("tools/call", {"name": name, "arguments": arguments or {}}))

    def call_tool_text(self, name: str, arguments: dict[str, Any] | None = None) -> str:
        """Call a tool and return just the first text content item."""
        result = self.call_tool(name, arguments)
        content = result.get("content", [])
        if not content:
            return ""
        return content[0].get("text", str(content[0]))  # type: ignore[no-any-return]

    # ------------------------------------------------------------------
    # Readiness probes
    # ------------------------------------------------------------------

    def wait_for_ready(self, timeout: float = 10.0, interval: float = 0.5) -> bool:
        """Poll until the MCP server responds, or return False on timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                resp = requests.post(
                    self.url,
                    json={"jsonrpc": "2.0", "id": "probe", "method": "initialize"},
                    timeout=2,
                )
                if resp.status_code == 200:
                    return True
            except requests.ConnectionError:
                pass
            time.sleep(interval)
        return False

    def wait_for_down(self, timeout: float = 10.0, interval: float = 0.5) -> bool:
        """Poll until the MCP server stops responding."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                requests.post(
                    self.url,
                    json={"jsonrpc": "2.0", "id": "probe", "method": "initialize"},
                    timeout=1,
                )
            except (requests.ConnectionError, requests.ReadTimeout):
                return True
            time.sleep(interval)
        return False

    # ------------------------------------------------------------------
    # Class methods for discovery
    # ------------------------------------------------------------------

    @classmethod
    def from_run_entry(cls, entry: Any | None = None, timeout: int = DEFAULT_TIMEOUT) -> McpAdapter:
        """Create an adapter from a RunEntry, or discover the latest one.

        Falls back to the default URL if no entry is found.
        """
        if entry is None:
            from dimos.core.run_registry import list_runs

            runs = list_runs(alive_only=True)
            entry = runs[0] if runs else None

        if entry is not None and hasattr(entry, "mcp_url") and entry.mcp_url:
            return cls(url=entry.mcp_url, timeout=timeout)

        # Fall back to default URL using GlobalConfig port
        from dimos.core.global_config import global_config

        url = f"http://localhost:{global_config.mcp_port}/mcp"
        return cls(url=url, timeout=timeout)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _unwrap(response: dict[str, Any]) -> dict[str, Any]:
        """Extract the ``result`` from a JSON-RPC response, raising on error."""
        if "error" in response:
            err = response["error"]
            msg = err.get("message", str(err)) if isinstance(err, dict) else str(err)
            raise McpError(msg, code=err.get("code") if isinstance(err, dict) else None)
        return response.get("result", {})  # type: ignore[no-any-return]

    def __repr__(self) -> str:
        return f"McpAdapter(url={self.url!r})"
