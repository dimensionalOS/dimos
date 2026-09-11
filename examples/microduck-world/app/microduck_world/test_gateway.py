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

import gzip
import socket
from collections.abc import Iterator

import httpx
import pytest
from microduck_world.gateway import GatewayConfig, create_app, rewrite_info
from microduck_world.udp_forwarder import UdpForwarder
from starlette.testclient import TestClient


@pytest.fixture
def config() -> GatewayConfig:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    return GatewayConfig(
        bind_host="127.0.0.1",
        port=port,
        public_origin=f"https://world.test:{port}",
        upstream_url="http://127.0.0.1:7780",
        cert_file="unused.crt",
        key_file="unused.key",
    )


@pytest.fixture
def client(config: GatewayConfig, monkeypatch: pytest.MonkeyPatch) -> Iterator[TestClient]:
    async def request(self, method, url, **kwargs):
        return httpx.Response(
            200,
            request=httpx.Request(method, url),
            json={"wtUrl": "https://127.0.0.1:45678/viewer", "certHash": "relay-pin", "v": 5},
            headers={"access-control-allow-origin": "*"},
        )

    monkeypatch.setattr(httpx.AsyncClient, "request", request)
    with TestClient(create_app(config), base_url=config.public_origin) as client:
        yield client


def test_discovery_preserves_certificate_pin_and_uses_gateway(client, config):
    response = client.get("/api/info")
    assert response.json() == {
        "wtUrl": f"{config.public_origin}/viewer",
        "certHash": "relay-pin",
        "v": 5,
    }
    assert response.headers["cache-control"] == "no-store"
    assert "access-control-allow-origin" not in response.headers


@pytest.mark.parametrize(
    "headers",
    [
        {"origin": "https://unrelated.test"},
        {"sec-fetch-site": "cross-site"},
    ],
)
def test_foreign_websites_cannot_read_bootstrap(client, headers):
    assert client.get("/api/info", headers=headers).status_code == 403


def test_wrong_host_is_rejected(client):
    assert client.get("/api/info", headers={"host": "unrelated.test"}).status_code == 421


def test_gateway_is_read_only_http_and_strips_upstream_cors(client):
    assert client.post("/api/info").status_code == 405
    assert "access-control-allow-origin" not in client.get("/").headers


def test_discovery_cannot_redirect_udp_to_an_external_host(config):
    with pytest.raises(ValueError, match="non-loopback"):
        rewrite_info({"wtUrl": "https://203.0.113.1:443/viewer"}, config, UdpForwarder())


@pytest.mark.parametrize("host", ["0.0.0.0", "192.168.1.10", "203.0.113.1"])
def test_non_tailnet_bind_is_rejected(config, host):
    with pytest.raises(ValueError, match="Tailscale"):
        GatewayConfig.model_validate({**config.model_dump(), "bind_host": host})


@pytest.fixture
def offline_client(client, monkeypatch):
    async def offline(self, method, url, **kwargs):
        raise httpx.ConnectError("offline")

    monkeypatch.setattr(httpx.AsyncClient, "request", offline)
    return client


def test_browser_outage_page_retries_automatically(offline_client):
    response = offline_client.get("/", headers={"accept": "text/html"})
    assert response.status_code == 503
    assert response.headers["cache-control"] == "no-store"
    assert response.headers["retry-after"] == "3"
    assert '<meta http-equiv="refresh" content="3">' in response.text
    assert "You can leave this page open" in response.text


def test_api_outage_returns_retryable_json(offline_client):
    response = offline_client.get("/api/info")
    assert response.status_code == 503
    assert response.headers["cache-control"] == "no-store"
    assert response.headers["retry-after"] == "3"
    assert response.json() == {"status": "starting"}


@pytest.mark.parametrize(
    "stats,status",
    [({"robots": [{"id": "duck"}]}, 200), ({"robots": []}, 503), ({}, 503), ([], 503)],
)
def test_readiness_requires_a_connected_robot(client, monkeypatch, stats, status):
    async def reply(self, method, url, **kwargs):
        assert url.path == "/api/stats"
        return httpx.Response(200, json=stats)

    monkeypatch.setattr(httpx.AsyncClient, "request", reply)
    response = client.get("/healthz")
    assert response.status_code == status
    assert response.json() == {"status": "ready" if status == 200 else "starting"}
    assert response.headers["cache-control"] == "no-store"


@pytest.fixture
def visual_client(config, tmp_path):
    frontend = tmp_path / "frontend"
    assets = tmp_path / "world"
    frontend.mkdir()
    assets.mkdir()
    (frontend / "index.html").write_text("<title>World 3D</title>")
    (frontend / "app.js").write_text("console.log('world')")
    (tmp_path / "private.txt").write_text("private")
    (frontend / "escape.txt").symlink_to(tmp_path / "private.txt")
    (assets / "scene-0123456789abcdef0123.json").write_text('{"version":1}')
    (assets / "scene-0123456789abcdef0123.json.gz").write_bytes(gzip.compress(b'{"version":1}'))
    settings = config.model_copy(update={"frontend_dir": frontend, "world_assets_dir": assets})
    with TestClient(create_app(settings), base_url=config.public_origin) as client:
        yield client


def test_project_frontend_and_model_assets_keep_access_boundary(visual_client):
    assert "World 3D" in visual_client.get("/").text
    assert visual_client.get("/client/app.js").text == "console.log('world')"
    response = visual_client.get(
        "/world-assets/scene-0123456789abcdef0123.json", headers={"accept-encoding": "identity"}
    )
    assert response.json() == {"version": 1}
    assert "immutable" in response.headers["cache-control"]
    assert visual_client.get("/client/escape.txt").status_code == 404
    assert visual_client.get("/world-assets/private.txt").status_code == 404
    assert (
        visual_client.get("/client/app.js", headers={"origin": "https://foreign.test"}).status_code
        == 403
    )
    assert (
        visual_client.get(
            "/world-assets/scene-0123456789abcdef0123.json",
            headers={"sec-fetch-site": "cross-site"},
        ).status_code
        == 403
    )


def test_browser_receives_compressed_world_model(visual_client):
    response = visual_client.get("/world-assets/scene-0123456789abcdef0123.json")
    assert response.headers["content-encoding"] == "gzip"
    assert response.json() == {"version": 1}
    assert response.headers["vary"] == "accept-encoding"


def test_private_relay_routes_are_not_exposed(client):
    for path in ("/internal/robot-info", "/internal/assignments", "/internal%2Frobot-info"):
        assert client.get(path).status_code == 404


def test_lobby_rejects_cross_origin_and_oversized_posts(client):
    assert client.post("/api/lobby", headers={"origin": "https://evil.test"}).status_code == 403
    assert client.post("/api/lobby", content=b"x" * 1025).status_code == 413


def test_discovery_keeps_session_ticket(config):
    result = rewrite_info(
        {"wtUrl": "https://127.0.0.1:45678/viewer?ticket=example"}, config, UdpForwarder()
    )
    assert result["wtUrl"] == config.public_origin + "/viewer?ticket=example"
