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

"""desktop.py against a fake Desktop: asking its shell tool, then waiting for the session to end."""

from __future__ import annotations

import asyncio
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import threading
from typing import Any

import pytest

from dimos.gateway import desktop


@pytest.fixture
def fake_desktop(monkeypatch: pytest.MonkeyPatch) -> Iterator[list[Any]]:
    seen: list[Any] = []
    polls = {"n": 0}

    class Handler(BaseHTTPRequestHandler):
        def answer(self, value: Any) -> None:
            body = json.dumps(value).encode()
            self.send_response(200)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self) -> None:
            seen.append(json.loads(self.rfile.read(int(self.headers["content-length"]))))
            self.answer({"id": "sh-7", "status": "pending"})

        def do_GET(self) -> None:
            seen.append(self.path)
            polls["n"] += 1
            self.answer({"status": "running" if polls["n"] < 2 else "succeeded", "commands": []})

        def log_message(self, *args: Any) -> None:
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    monkeypatch.setenv("DESKTOP_URL", f"http://127.0.0.1:{server.server_address[1]}/")
    yield seen
    server.shutdown()


def test_asks_desktop_then_waits_for_the_end(fake_desktop: list[Any]) -> None:
    commands = [{"run": "uv sync", "note": "Install"}]
    assert asyncio.run(desktop.request_shell("Title", "Words", commands, "launcher")) == "sh-7"
    assert fake_desktop[0] == {
        "title": "Title",
        "message": "Words",
        "commands": commands,
        "app": "launcher",
    }
    assert asyncio.run(desktop.wait_shell("sh-7"))["status"] == "succeeded"
    assert fake_desktop[1:] == ["/api/desktop/shell/sh-7?wait=25"] * 2


def test_no_desktop_is_its_own_error() -> None:
    # conftest points DESKTOP_URL at a port that refuses
    with pytest.raises(desktop.DesktopUnavailableError):
        asyncio.run(desktop.request_shell("t", "", [{"run": "true"}]))
