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

"""Forward an external agent's model HTTP requests and record requests and responses."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import subprocess
import sys
from typing import Any

from dimos.agents.llm_trace import tracing_http_client


def _serve(raw_dir: Path, upstream: str) -> None:
    """Own forwarding threads and sockets in a process that can be stopped."""
    with tracing_http_client(raw_dir, timeout=600.0) as client:

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

        with ThreadingHTTPServer(("127.0.0.1", 0), Forward) as server:
            print(f"http://127.0.0.1:{server.server_port}", flush=True)
            server.serve_forever()


@contextmanager
def model_trace_proxy(raw_dir: Path, upstream: str) -> Iterator[str]:
    """Yield a local provider URL; record each complete HTTP exchange in raw_dir.

    Exiting stops the server process, including any requests blocked upstream.
    A subprocess also works inside the eval module's daemon worker.
    """
    with subprocess.Popen(
        [sys.executable, "-m", __name__, str(raw_dir), upstream.rstrip("/")],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        text=True,
    ) as process:
        assert process.stdout is not None
        try:
            with process.stdout:
                url = process.stdout.readline().strip()
                if not url:
                    status = process.wait()
                    raise RuntimeError(
                        f"Model trace proxy exited before startup with status {status}"
                    )
            yield url
        finally:
            process.terminate()
            try:
                # Bound cleanup even if the worker fails to respond to SIGTERM.
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


if __name__ == "__main__":
    _serve(Path(sys.argv[1]), sys.argv[2])
