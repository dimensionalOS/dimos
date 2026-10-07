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

from fastapi import FastAPI
from fastapi.testclient import TestClient
import pytest

from dimos.gateway.loopback import LoopbackOnly, allowed


@pytest.mark.parametrize(
    "host, origin, ok",
    [
        ("localhost", None, True),
        ("127.0.0.1:5557", None, True),
        ("[::1]:5557", None, True),
        ("127.0.0.1:5557", "http://127.0.0.1:5557", True),
        ("evil.example:5557", None, False),
        ("127.0.0.1.evil.example", None, False),
        (None, None, False),
        ("127.0.0.1:5557", "https://evil.example", False),
        ("127.0.0.1:5557", "http://127.0.0.1:5555", False),
        ("localhost", "null", False),
    ],
)
def test_allowed(host: str | None, origin: str | None, ok: bool) -> None:
    assert allowed(host, origin) is ok


def test_middleware_refuses_other_hosts_and_origins() -> None:
    app = FastAPI()

    @app.get("/healthz")
    def healthz() -> str:
        return "ok"

    client = TestClient(LoopbackOnly(app), base_url="http://127.0.0.1:5557")
    assert client.get("/healthz").status_code == 200
    assert client.get("/healthz", headers={"host": "evil.example"}).status_code == 403
    assert client.get("/healthz", headers={"origin": "https://evil.example"}).status_code == 403


def test_port_comes_from_desktops_config(tmp_path, monkeypatch) -> None:
    from dimos.gateway import main

    monkeypatch.setenv("DIMOS_HOME", str(tmp_path))
    assert main.configured_port() == main.DEFAULT_PORT
    (tmp_path / "config.yaml").write_text("dimos_gateway:\n  port: 6123\n")
    assert main.configured_port() == 6123


def test_a_held_port_is_taken_but_not_healthy() -> None:
    import socket

    from dimos.gateway import main

    with socket.socket() as listener:
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        port = listener.getsockname()[1]
        assert main.taken(port)
        assert not main.healthy(port, timeout=0.3)
