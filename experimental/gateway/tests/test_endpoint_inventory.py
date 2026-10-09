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

from __future__ import annotations

import json
from pathlib import Path

from fastapi.testclient import TestClient

from experimental.gateway.server.app import create_app
from experimental.gateway.server.endpoints import ENDPOINTS
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.events import Bus
from experimental.gateway.utils.uploads import Uploads


def test_every_original_route_and_published_rpc_route_is_registered(
    server_home: Path, checkout: Path, fake_worker: list[str]
) -> None:
    old = json.loads(
        (Path(__file__).parent / "fixtures/endpoint_inventory_before.json").read_text()
    )
    expected = {(method.upper(), url) for method, url in old}
    expected |= {
        ("GET", "/dimos/openapi.json"),
        ("HEAD", "/dimos/openapi.json"),
        ("GET", "/dimos/rpc"),
        ("POST", "/dimos/rpc/call"),
    }
    bus = Bus()
    state = ServerState(
        checkout, bus, Uploads(checkout, bus, None, server_home / "upload.log", worker=fake_worker)
    )
    app = create_app(state, background=False)
    actual = {(method, route.path) for route in app.routes for method in route.methods}
    inventory = json.loads((ENDPOINTS / "inventory.json").read_text())
    declared = {(method, url) for method, url, filename in inventory}
    assert actual == declared == expected
    for method, url, filename in inventory:
        assert filename == (url.lstrip("/") or "index") + ".py"
        route = next(route for route in app.routes if route.path == url and method in route.methods)
        assert Path(route.endpoint.__code__.co_filename) == ENDPOINTS / filename
    with TestClient(app) as client:
        assert client.get("/dimos/healthz").text == "ok"
        assert client.get("/dimos/openapi.json").json()["info"]["version"] == "1.18.0"
        assert client.head("/dimos/openapi.json").status_code == 200
