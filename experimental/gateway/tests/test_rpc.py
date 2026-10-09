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

"""GET /dimos/rpc and POST /dimos/rpc/call against a fake dimos RPC bus: a coordinator whose one module is FakeConnection
below (its class imports here, so its signatures are read) and one whose class doesn't import."""

from __future__ import annotations

from collections.abc import Iterator
from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.core.coordination.module_coordinator import ModuleDescriptor
from experimental.gateway.server.app import create_app
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import events, skills
from experimental.gateway.utils.uploads import Uploads


def get_battery_soc(self: Any) -> float:
    """The battery's charge, in percent."""
    return 87.0


def set_speed(self: Any, speed: float, *, ramp: bool = False) -> str:
    """Caps the walking speed."""
    return f"speed {speed}"


def lifecycle(self: Any) -> None:
    pass


class FakeConnection:
    rpcs = {
        "get_battery_soc": get_battery_soc,
        "set_speed": set_speed,
        "start": lifecycle,
        "stop": lifecycle,
        "build": lifecycle,
    }


class FakeBus:
    def __init__(self, up: bool = True) -> None:
        self.up = up
        self.calls: list[tuple[str, list[Any], dict[str, Any]]] = []

    def call(self, name: str, args: list[Any], kwargs: dict[str, Any], timeout: float) -> Any:
        if not self.up:
            raise TimeoutError(f"RPC call to '{name}' timed out")
        if name == "Coordinator/list_modules":
            return [
                ModuleDescriptor(
                    "FakeConnection",
                    f"{__name__}.FakeConnection",
                    list(FakeConnection.rpcs),
                    "GO2Connection",
                ),
                ModuleDescriptor("Elsewhere", "nowhere.Elsewhere", ["start", "stop", "poke"], ""),
            ]
        self.calls.append((name, args, kwargs))
        if name == "GO2Connection/get_battery_soc":
            return 87.0
        if name == "GO2Connection/set_speed":
            if (args or [kwargs.get("speed")])[0] < 0:
                raise ValueError("speed can't be negative")
            return f"speed {(args or [kwargs.get('speed')])[0]}"
        return {"poked": object()}


RUN = skills.Run("20260101-120000-unitree-go2", "unitree-go2", None)


@pytest.fixture
def bus(monkeypatch: pytest.MonkeyPatch) -> FakeBus:
    fake = FakeBus()
    monkeypatch.setattr(skills, "bus", fake)
    monkeypatch.setattr(skills, "live_run", lambda: RUN)
    return fake


@pytest.fixture
def client(server_home: Path, checkout: Path, fake_worker: list[str]) -> Iterator[TestClient]:
    the_bus = events.Bus()
    uploads = Uploads(checkout, the_bus, None, server_home / "uploads.log", worker=fake_worker)
    with TestClient(
        create_app(ServerState(checkout, the_bus, uploads), background=False)
    ) as client:
        yield client


def test_lists_rpc_methods_without_lifecycle_ones(client: TestClient, bus: FakeBus) -> None:
    answer = client.get("/dimos/rpc").json()
    assert [(r["module"], r["method"]) for r in answer["rpcs"]] == [
        ("Elsewhere", "poke"),
        ("GO2Connection", "get_battery_soc"),
        ("GO2Connection", "set_speed"),
    ]
    speed = answer["rpcs"][2]
    assert speed["class"] == f"{__name__}.FakeConnection"
    assert speed["known"] is True
    assert speed["doc"] == "Caps the walking speed."
    assert speed["return_type"] == "str"
    assert [
        (p["name"], p["type"], p["default"], p["required"], p["kind"]) for p in speed["params"]
    ] == [
        ("speed", "float", None, True, "positional_or_keyword"),
        ("ramp", "bool", "False", False, "keyword_only"),
    ]
    assert answer["rpcs"][0]["known"] is False
    assert answer["run"] == {"runId": RUN.run_id, "blueprint": "unitree-go2"}


def test_nothing_running(client: TestClient, bus: FakeBus) -> None:
    bus.up = False
    assert client.get("/dimos/rpc").json() == {"rpcs": [], "run": None}
    response = client.post(
        "/dimos/rpc/call", json={"module": "GO2Connection", "method": "get_battery_soc"}
    )
    assert response.status_code == 409


def test_calls_over_module_rpc(client: TestClient, bus: FakeBus) -> None:
    battery = client.post(
        "/dimos/rpc/call", json={"module": "GO2Connection", "method": "get_battery_soc"}
    )
    assert battery.status_code == 200
    assert battery.json() == {
        "module": "GO2Connection",
        "method": "get_battery_soc",
        "runId": RUN.run_id,
        "blueprint": "unitree-go2",
        "ok": True,
        "result": 87.0,
        "text": "87.0",
    }
    by_name = client.post(
        "/dimos/rpc/call",
        json={
            "module": "GO2Connection",
            "method": "set_speed",
            "args": {"speed": 0.5, "ramp": True},
        },
    ).json()
    assert (by_name["ok"], by_name["text"]) == (True, "speed 0.5")
    by_position = client.post(
        "/dimos/rpc/call", json={"module": "GO2Connection", "method": "set_speed", "args": [0.3]}
    ).json()
    assert by_position["result"] == "speed 0.3"
    assert bus.calls == [
        ("GO2Connection/get_battery_soc", [], {}),
        ("GO2Connection/set_speed", [], {"speed": 0.5, "ramp": True}),
        ("GO2Connection/set_speed", [0.3], {}),
    ]
    raised = client.post(
        "/dimos/rpc/call", json={"module": "GO2Connection", "method": "set_speed", "args": [-1]}
    ).json()
    assert raised["ok"] is False and "speed can't be negative" in raised["text"]
    # a module whose class doesn't import is called unchecked; a non-JSON answer comes back as its repr
    poked = client.post(
        "/dimos/rpc/call", json={"module": "Elsewhere", "method": "poke", "args": {"x": 1}}
    ).json()
    assert poked["ok"] is True and isinstance(poked["result"], str) and "poked" in poked["result"]


@pytest.mark.parametrize("method", ["start", "stop", "build"])
def test_lifecycle_methods_are_refused(client: TestClient, bus: FakeBus, method: str) -> None:
    response = client.post("/dimos/rpc/call", json={"module": "GO2Connection", "method": method})
    assert response.status_code == 400
    assert "lifecycle" in response.json()["error"]
    assert bus.calls == []


def test_bad_calls_never_reach_the_module(client: TestClient, bus: FakeBus) -> None:
    def refused(body: dict[str, Any]) -> tuple[int, str]:
        response = client.post("/dimos/rpc/call", json=body)
        return response.status_code, response.json()["error"]

    status, error = refused({"module": "GO2Connection", "method": "fly"})
    assert status == 404 and "get_battery_soc, set_speed" in error
    status, error = refused({"module": "Nope", "method": "fly"})
    assert status == 404 and "no module Nope" in error
    status, error = refused({"module": "GO2Connection", "method": "set_speed"})
    assert status == 400 and "missing speed" in error
    status, error = refused(
        {"module": "GO2Connection", "method": "set_speed", "args": {"speed": 1, "fast": 1}}
    )
    assert status == 400 and "doesn't take fast" in error
    status, error = refused({"module": "GO2Connection", "method": "get_battery_soc", "args": [1]})
    assert status == 400 and "at most 0" in error
    assert bus.calls == []
