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

import json
from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
import pytest
import yaml

from dimos.core.global_config import GlobalConfig
from dimos.robot.all_blueprints import all_blueprints
from experimental.gateway import overrides as ov, robots, store
from experimental.gateway.state import API_VERSION, State

ARG = {
    "default": None,
    "description": None,
    "required": False,
    "base": False,
    "json_compatible": True,
}
CONFIG = {
    "name": "demo",
    "modules": [
        {
            "module": "camera",
            "class": "x.Camera",
            "args": [
                {**ARG, "name": "fps", "type": "int", "default": 30, "value": 10},
                {**ARG, "name": "api_key", "type": "str", "value": "real"},
            ],
        }
    ],
}


@pytest.fixture
def introspected(state: State, monkeypatch: pytest.MonkeyPatch) -> list[Any]:
    calls: list[Any] = []

    async def fake(args: list[str], stdin: Any = None) -> Any:
        calls.append((args, stdin))
        if args[0] == "config":
            return CONFIG
        if args[0] == "blueprint":
            return {"name": args[1], "file": None, "line": None, "modules": []}
        bad = [a for a in stdin or [] if "fps=fast" in a]
        return {"args": [{"tokens": [a], "error": "not an int"} for a in bad]}

    monkeypatch.setattr(state, "introspected", fake)
    return calls


def test_server_routes(client: TestClient, checkout: Path) -> None:
    assert client.get("/healthz").text == "ok"
    info = client.get("/info").json()
    assert {
        k: info[k] for k in ("dir", "found", "installed", "version", "api", "commit", "tag")
    } == {
        "dir": str(checkout),
        "found": True,
        "installed": True,
        "version": "0.0.14",
        "api": API_VERSION,
        "commit": None,
        "tag": None,
    }
    assert info["python"]
    assert info["paths"]["dimosDir"] == str(checkout)
    assert info["server"]["zenohNamespace"] == "ns"


def test_openapi_lists_the_routes(client: TestClient) -> None:
    routes = client.get("/openapi.json").json()["paths"]
    assert {"/runs", "/skills", "/catalog"} <= set(routes)
    slow = {path for path, methods in routes.items() if methods.get("get", {}).get("x-slow")}
    assert slow == {"/blueprints/{name}"}
    assert not any(path.startswith("/discovery") for path in routes)


def test_global_config_is_saved_in_the_gateways_own_state(client: TestClient, home: Path) -> None:
    answer = client.put("/global-config", json={"overrides": {"robot_ip": "10.0.0.2"}})
    assert answer.status_code == 200
    assert answer.json()["overrides"] == {"robot_ip": "10.0.0.2"}
    saved = json.loads((home / "state" / "gateway" / "settings.json").read_text())
    assert saved["global_config"] == {"robot_ip": "10.0.0.2"}
    shown = client.get("/global-config").json()
    assert "robot_ip" in shown["schema"]["properties"]
    assert shown["defaults"]["rerun_open"] == "none"


def test_global_config_refuses_what_dimos_would(client: TestClient) -> None:
    unknown = client.put("/global-config", json={"overrides": {"robot_iq": "x"}})
    assert unknown.status_code == 400
    assert "robot_iq" in unknown.json()["error"]
    wrong = client.put("/global-config", json={"overrides": {"n_workers": "many"}})
    assert wrong.status_code == 400


def test_a_secret_is_hidden_and_kept_when_sent_back(client: TestClient) -> None:
    client.put("/global-config", json={"overrides": {"typesafe_api_key": "s3cret"}})
    assert client.get("/global-config").json()["overrides"] == {"typesafe_api_key": ov.HIDDEN}
    client.put(
        "/global-config", json={"overrides": {"typesafe_api_key": ov.HIDDEN, "n_workers": 2}}
    )
    assert store.global_config_overrides() == {"typesafe_api_key": "s3cret", "n_workers": 2}


def test_module_config_round_trip(client: TestClient, introspected: list[Any]) -> None:
    put = client.put(
        "/blueprints/demo/config", json={"overrides": {"camera": {"fps": 5, "api_key": "k"}}}
    )
    assert put.status_code == 200
    assert store.module_config("demo") == {"camera": {"api_key": "k", "fps": 5}}
    shown = client.get("/blueprints/demo").json()["config"]
    assert shown["overrides"] == {"camera": {"api_key": ov.HIDDEN, "fps": 5}}
    args = {a["name"]: a for a in shown["modules"][0]["args"]}
    assert args["api_key"]["secret"] and args["api_key"]["value"] == ov.HIDDEN
    refused = client.put("/blueprints/demo/config", json={"overrides": {"camera": {"fps": "fast"}}})
    assert refused.status_code == 400
    assert store.module_config("demo") == {"camera": {"api_key": "k", "fps": 5}}


def test_a_bad_blueprint_name_is_refused(client: TestClient) -> None:
    assert client.get("/blueprints/-x").status_code == 400


def test_robots_come_from_the_yaml_with_global_config_reflected(client: TestClient) -> None:
    answer = client.get("/robots").json()
    go2 = answer["robots"]["go2"]
    assert go2["blueprints"]["unitree-go2"]["registered"]
    settings = go2["blueprints"]["unitree-go2"]["recommended_config"]
    assert any(s["kind"] == "pick" for s in settings)


def test_annotations_name_registered_blueprints_and_real_fields() -> None:
    source = robots.load()
    for robot_id, robot in source["robots"].items():
        for name, blueprint in robot["blueprints"].items():
            assert name in all_blueprints, f"{robot_id} lists {name}, which isn't registered"
            settings = [
                *blueprint.get("recommended_config", []),
                *robot.get("defaults", {}).get("recommended_config", []),
            ]
            for setting in settings:
                if "global" in setting:
                    assert setting["global"] in GlobalConfig.model_fields


def test_dimos_yaml_declares_this_api_and_its_endpoints(client: TestClient) -> None:
    manifest = yaml.safe_load((Path(__file__).parents[3] / "dimos.yaml").read_text())
    assert manifest["api"]["version"] == API_VERSION
    routes = client.get("/openapi.json").json()["paths"]
    served = {
        f"{method.upper()} {path.lstrip('/')}" for path, ops in routes.items() for method in ops
    }
    declared = {f"{e['method']} {e['path']}" for e in manifest["provides"]["endpoints"]}
    assert declared == served


def test_blueprint_args_are_read_as_dimos_run_would(
    client: TestClient, state: State, monkeypatch: pytest.MonkeyPatch
) -> None:
    async def fake(args: list[str], stdin: Any = None) -> Any:
        assert args == ["args", "demo"]
        return {
            "name": "demo",
            "args": [{"tokens": [a], "target": "fps", "error": None} for a in stdin],
        }

    monkeypatch.setattr(state, "introspected", fake)
    checked = client.post("/blueprints/demo/args", json={"args": ["--fps=5"]})
    assert checked.status_code == 200, checked.text
    assert checked.json()["args"] == [{"tokens": ["--fps=5"], "target": "fps", "error": None}]
    assert client.post("/blueprints/-x/args", json={"args": []}).status_code == 400
