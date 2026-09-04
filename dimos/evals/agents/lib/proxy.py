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

"""Wire capture for agents that run as a subprocess and cannot take an
``http_client``: a local endpoint that forwards to the provider and records
every request/response pair whole."""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
from typing import Any

import httpx

from dimos.agents.llm_trace import tracing_http_client


class RecordingProxy:
    """``with RecordingProxy(raw_dir, upstream) as url:`` — a request to
    ``url/<path>`` is sent to ``upstream/<path>`` through
    :func:`~dimos.agents.llm_trace.tracing_http_client`, so each one becomes a
    ``<seq>-request.json`` / ``-response.json`` pair under *raw_dir* (auth
    header dropped). A reply is relayed whole once it has arrived, so a
    streamed response reaches the caller in one piece."""

    def __init__(self, raw_dir: Path, upstream: str) -> None:
        self.raw_dir = raw_dir
        self.upstream = upstream.rstrip("/")
        self._server: ThreadingHTTPServer | None = None

    def __enter__(self) -> str:
        client = tracing_http_client(self.raw_dir, timeout=httpx.Timeout(600.0))
        upstream = self.upstream

        class Forward(BaseHTTPRequestHandler):
            def do_POST(self) -> None:
                body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
                # hop-by-hop headers belong to this connection, not the upstream one
                skip = {
                    "host",
                    "content-length",
                    "connection",
                    "accept-encoding",
                    "transfer-encoding",
                }
                headers = {k: v for k, v in self.headers.items() if k.lower() not in skip}
                reply = client.request(
                    self.command, upstream + self.path, content=body, headers=headers
                )
                self.send_response(reply.status_code)
                self.send_header(
                    "Content-Type", reply.headers.get("content-type", "application/json")
                )
                self.send_header("Content-Length", str(len(reply.content)))
                self.end_headers()
                self.wfile.write(reply.content)

            def log_message(self, format: str, *args: Any) -> None:
                return None

        self._server = ThreadingHTTPServer(("127.0.0.1", 0), Forward)
        threading.Thread(target=self._server.serve_forever, name="llm-proxy", daemon=True).start()
        return f"http://127.0.0.1:{self._server.server_port}"

    def __exit__(self, *exc: object) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
