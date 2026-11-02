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

import os
from pathlib import Path
from typing import Any

from fastapi.testclient import TestClient
import pytest
import yaml

from dimos.core.global_config import GlobalConfig
from dimos.core.run_registry import RunEntry
from dimos.robot.all_blueprints import all_blueprints
from experimental.gateway import annotations as annotation_file, constants, launches
from experimental.gateway.__main__ import serve
from experimental.gateway.state import API_VERSION, ApiError, State

ARG = {
    "default": None,
    "description": None,
    "required": False,
    "secret": False,
    "base": False,
    "json_compatible": True,
}
CONFIG = {
    "name": "demo",
    "global": {"n_workers": {**ARG, "type": "int", "default": 2}},
    "modules": {
        "camera": {
            "class": "x.Camera",
            "args": {
                "fps": {**ARG, "type": "int", "default": 30},
                "api_key": {**ARG, "type": "str", "secret": True},
            },
        }
    },
}
REMOVED = [
    ("get", "/blueprints"),
    ("get", "/blueprints/demo"),
    ("post", "/blueprints/demo/args"),
    ("put", "/blueprints/demo/config"),
    ("get", "/robots"),
    ("get", "/global-config"),
    ("get", "/skills"),
    ("post", "/skills/call"),
    ("get", "/runs/skills"),
    ("get", "/catalog/skills"),
    ("get", "/runs/latest/log"),
    ("get", "/runs"),
    ("post", "/runs"),
    ("post", "/runs/stop"),
    ("post", "/runs/restart"),
    ("get", "/runs/rpcs"),
    ("get", "/runs/logs"),
    ("get", "/runs/log_hist"),
    ("get", "/config/global"),
    ("put", "/config/global"),
    ("get", "/config/blueprints/demo"),
    ("put", "/config/blueprints/demo"),
]


@pytest.fixture
def introspected(state: State, monkeypatch: pytest.MonkeyPatch) -> list[Any]:
    calls: list[Any] = []
    state.listed = [{"name": "demo", "kind": "builtin"}]

    async def fake(args: list[str], stdin: Any = None) -> Any:
        calls.append((args, stdin))
        if args[0] == "config":
            return CONFIG
        if args[0] == "blueprint":
            return {"name": args[1], "file": "x.py", "line": 3, "modules": []}
        raise AssertionError(f"unexpected introspection {args}")

    monkeypatch.setattr(state, "introspected", fake)
    return calls


def test_server_routes(client: TestClient, checkout: Path) -> None:
    assert client.get("/healthz").text == "ok"
    info = client.get("/info").json()
    assert {
        key: info[key] for key in ("dir", "found", "installed", "version", "api", "commit", "tag")
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
    assert {"/active", "/active/modules", "/catalog/rpcs", "/catalog", "/catalog/config"} <= set(
        routes
    )
    slow = {path for path, methods in routes.items() if methods.get("get", {}).get("x-slow")}
    assert slow == {
        "/catalog/blueprints/{name}",
        "/catalog/blueprints/{name}/config",
        "/catalog/modules/{name}",
        "/catalog/modules/{name}/config",
    }
    assert not any(
        path.startswith(("/blueprints", "/skills", "/robots", "/global-config")) for path in routes
    )


@pytest.mark.parametrize(("method", "path"), REMOVED)
def test_the_old_routes_are_gone(client: TestClient, method: str, path: str) -> None:
    assert getattr(client, method)(path).status_code in (404, 405)


def test_global_config_is_its_schema_defaults_and_secret_names(client: TestClient) -> None:
    shown = client.get("/catalog/config").json()
    assert set(shown) == {"schema", "defaults", "secrets"}
    assert "robot_ip" in shown["schema"]["properties"]
    assert all(launches.is_secret_name(key) for key in shown["secrets"])


def test_a_blueprints_config_schema_is_global_and_module_fields(
    client: TestClient, introspected: list[Any]
) -> None:
    schema = client.get("/catalog/blueprints/demo/config").json()
    assert schema["global"]["n_workers"]["default"] == 2
    assert schema["modules"]["camera"]["args"]["api_key"]["secret"]


def test_a_blueprint_running_from_another_checkout_is_read_from_that_one(
    client: TestClient, state: State, home: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    branch = home / "dimos_branch"
    (branch / ".venv" / "bin").mkdir(parents=True)
    (branch / "pyproject.toml").write_text('[project]\nname = "dimos"\n')
    state.listed = [{"name": "demo", "kind": "builtin"}]
    read_from: list[Any] = []

    async def fake(args: list[str], stdin: Any = None, source: Any = None) -> Any:
        read_from.append(source and source.checkout)
        if source is not None and "broken" in str(source.run_id):
            raise ApiError(500, "ImportError: no get_by_name")
        if args[0] == "config":
            return {**CONFIG, "name": args[1], "global": {} if source else CONFIG["global"]}
        stream = "lidar_with_robot" if source else "lidar"
        streams = [
            {"name": "lidar", "type": "x.PointCloud2", "direction": "in", "wire": stream}
            | {"topic": None}
        ]
        modules: list[dict[str, Any]] = [
            {"name": "mapper", "class": "x.Mapper", "doc": "", "summary": "", "streams": streams}
            | {"rpcs": [], "file": None, "line": None}
        ]
        return {"name": args[1], "file": None, "line": None, "modules": modules}

    monkeypatch.setattr(state, "introspected", fake)

    def wire_shown(name: str = "demo") -> tuple[str, Any, Any]:
        found = client.get(f"/catalog/blueprints/{name}?fields=modules,checkout,checkoutError")
        body = found.json()
        return body["modules"][0]["streams"][0]["wire"], body["checkout"], body["checkoutError"]

    def started_from(program: Path, run_id: str, blueprint: str = "demo") -> RunEntry:
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint,
            started_at="2026-01-01T12:00:00+00:00",
            log_dir="/tmp/x",
            original_argv=[str(program), "run", blueprint],
        )
        entry.save()
        return entry

    assert wire_shown() == ("lidar", None, None)
    # a blueprint only a branch has is unknown until it runs from that branch
    assert client.get("/catalog/blueprints/branch-only").status_code == 404

    from_gateway = started_from(state.dimos_dir / ".venv" / "bin" / "dimos", "20260101-120000-demo")
    assert wire_shown() == ("lidar", None, None)
    from_gateway.remove()
    started_from(branch / ".venv" / "bin" / "dimos", "20260101-120001-demo")
    assert wire_shown() == ("lidar_with_robot", str(branch), None)
    assert read_from[-1] == branch
    assert client.get("/catalog/blueprints/demo/config").json()["global"] == {}

    only = started_from(branch / ".venv" / "bin" / "dimos", "20260101-120003-only", "branch-only")
    assert wire_shown("branch-only") == ("lidar_with_robot", str(branch), None)
    assert client.get("/catalog/blueprints/branch-only/config").json()["name"] == "branch-only"
    only.remove()
    assert client.get("/catalog/blueprints/branch-only").status_code == 404

    started_from(branch / ".venv" / "bin" / "dimos", "20260101-120002-broken")
    wire, checkout, error = wire_shown()
    assert (wire, checkout) == ("lidar", None) and "ImportError" in error


def test_a_bad_blueprint_name_is_refused(client: TestClient) -> None:
    assert client.get("/catalog/blueprints/-x").status_code == 400


def test_robots_list_their_blueprints_which_carry_the_annotations(client: TestClient) -> None:
    go2 = client.get("/catalog/robots").json()["robots"]["go2"]
    assert "unitree-go2" in go2["blueprints"]
    found = client.get("/catalog/blueprints?fields=blueprints.name,blueprints.recommended_config")
    unitree_go2 = next(
        blueprint for blueprint in found.json()["blueprints"] if blueprint["name"] == "unitree-go2"
    )
    assert set(unitree_go2) == {"name", "recommended_config"}
    assert any(setting["kind"] == "pick" for setting in unitree_go2["recommended_config"])


def test_annotations_name_registered_blueprints_and_real_fields() -> None:
    source = annotation_file.load()
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


def test_every_registered_blueprint_is_annotated() -> None:
    source = annotation_file.load()
    annotated = {name for robot in source["robots"].values() for name in robot["blueprints"]}
    missing = sorted(
        name
        for name in all_blueprints
        if name not in annotated and not annotation_file.HIDDEN_NAME.search(name)
    )
    assert not missing, f"add these to experimental/gateway/annotations.yaml: {', '.join(missing)}"


def test_dimos_yaml_declares_this_api_and_its_endpoints(client: TestClient) -> None:
    manifest = yaml.safe_load((Path(__file__).parents[3] / "dimos.yaml").read_text())
    assert manifest["provides"]["info"]["version"] == API_VERSION
    routes = client.get("/openapi.json").json()["paths"]
    served = {
        f"{method.upper()} {path.lstrip('/')}" for path, ops in routes.items() for method in ops
    }
    declared = {
        f"{method.upper()} {path.lstrip('/')}"
        for path, ops in manifest["provides"]["paths"].items()
        for method in ops
    }
    assert declared == served


def test_serve_refuses_a_socket_path_too_long_to_bind(tmp_path: Path) -> None:
    socket_path = tmp_path / ("s" * constants.UNIX_SOCKET_PATH_LIMIT) / "dimos-gateway.sock"
    with pytest.raises(SystemExit, match="dimos-gateway.socket"):
        serve(socket_path, tmp_path, None, constants.DEFAULT_HOST, None, None, False)
    assert not socket_path.parent.exists()
