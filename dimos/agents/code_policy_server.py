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

"""In-process, one-tool MCP server for a :class:`CodePolicySession`."""

from __future__ import annotations

import asyncio
import logging
import socket
import threading
import time

from mcp.server.mcpserver import MCPServer
import uvicorn

from dimos.agents.code_policy_core import (
    MAX_EXECUTION_TIMEOUT_S,
    CodePolicySession,
    CodePolicySessionConfig,
)

PYTHON_EXEC_DESCRIPTION = """Execute Python in a persistent trusted, unsandboxed session.

Imports, functions, and variables persist between calls. The runtime environment
determines which globals are available.
"""

_NOISY_MCP_TRANSPORT_LOGGERS = (
    "mcp.server.streamable_http",
    "mcp.server.streamable_http_manager",
)


class CodePolicyMcpServer:
    """Own the CodePolicy session and an official MCP HTTP server in one process."""

    def __init__(
        self,
        config: CodePolicySessionConfig,
        *,
        host: str = "127.0.0.1",
    ) -> None:
        self.host = host
        self.port = 0
        self.session = CodePolicySession(config)
        self.mcp = MCPServer(name="dimos-code-policy", version="1.0.0")

        @self.mcp.tool(
            name="python_exec",
            description=PYTHON_EXEC_DESCRIPTION,
            structured_output=False,
        )
        async def python_exec(code: str, timeout_s: float = MAX_EXECUTION_TIMEOUT_S) -> str:
            return await asyncio.to_thread(self.session.python_exec, code, timeout_s)

        self.app = self.mcp.streamable_http_app(
            streamable_http_path="/mcp",
            json_response=True,
            stateless_http=True,
            host=host,
        )
        self._server: uvicorn.Server | None = None
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    @property
    def mcp_url(self) -> str:
        if self.port == 0:
            raise RuntimeError("CodePolicy MCP server is not running")
        return f"http://{self.host}:{self.port}/mcp"

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("CodePolicy MCP server is already running")
        for logger_name in _NOISY_MCP_TRANSPORT_LOGGERS:
            logging.getLogger(logger_name).setLevel(logging.WARNING)
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, 0))
        sock.listen(128)
        self.port = int(sock.getsockname()[1])
        self._socket = sock
        self.session.start()
        server = uvicorn.Server(uvicorn.Config(self.app, log_level="warning", access_log=False))
        self._server = server

        def serve() -> None:
            asyncio.run(server.serve(sockets=[sock]))

        thread = threading.Thread(
            target=serve,
            name=f"code-policy-mcp-{self.port}",
            daemon=True,
        )
        self._thread = thread
        thread.start()
        deadline = time.monotonic() + 10
        while not server.started and thread.is_alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not server.started:
            self.stop()
            raise TimeoutError("CodePolicy MCP server did not start")

    def stop(self) -> None:
        server, thread = self._server, self._thread
        self._server = None
        self._thread = None
        if server is not None:
            server.should_exit = True
        if thread is not None:
            thread.join(timeout=5)
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        self.session.stop()
        self.port = 0

    def __enter__(self) -> CodePolicyMcpServer:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()
