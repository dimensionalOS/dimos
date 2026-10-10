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

from collections.abc import Callable
from pathlib import Path
import sys
import time
from typing import Any

from fastapi.testclient import TestClient
import pytest

from experimental.gateway import discovery, introspect
from experimental.gateway.state import State

FAKE_SCAN = """
import json, os, sys
request = json.loads(sys.stdin.read() or "null") or {{}}
def emit(value):
    print("\\n{marker}" + json.dumps(value), flush=True)
names = request.get("blueprints")
if names is None:
    names = ["good", "crashes", "missing"]
    emit({{"kind": "names", "blueprints": names, "modules": ["cam"]}})
for name in names:
    emit({{"kind": "start", "name": name}})
    if name == "crashes":
        os._exit(3)
    ok = name == "good"
    emit({{"kind": "blueprint", "name": name, "ref": "dimos.robot.unitree.go2.x:" + name, "builtin": True,
          "importable": ok, "optional_dependency": False, "import_error": None if ok else "ModuleNotFoundError: x",
          "import_traceback": None, "missing_module": None if ok else "unitree_sdk2py", "doc": "",
          "modules": [{{"name": "cam", "class": "x.Cam", "module": "cam", "streams": []}}] if ok else []}})
for name in request.get("modules", ["cam"]):
    emit({{"kind": "start", "name": "module:" + name}})
    emit({{"kind": "module", "name": name, "class": "x.Cam", "doc": "A camera.", "inputs": [], "outputs": [],
          "skills": [{{"name": "snap", "doc": "", "params": []}}]}})
emit({{"kind": "end"}})
"""


def until(check: Callable[[], bool], seconds: float = 10.0) -> None:
    deadline = time.monotonic() + seconds
    while not check():
        assert time.monotonic() < deadline, "timed out"
        time.sleep(0.05)


@pytest.fixture
def scanned(state: State, tmp_path: Path) -> State:
    script = tmp_path / "fake_scan.py"
    script.write_text(FAKE_SCAN.format(marker=introspect.MARKER))
    state.scanner.command = [sys.executable, str(script)]
    return state


def test_discovery_scans_on_demand_and_survives_a_crash(client: TestClient, scanned: State) -> None:
    (scanned.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\n[project.optional-dependencies]\nunitree-dds = ["unitree-sdk2py"]\n'
    )
    assert scanned.scanner.task is None
    client.get("/dimos/discovery")
    until(lambda: client.get("/dimos/discovery").json()["state"] == "done")
    status = client.get("/dimos/discovery").json()
    assert (status["blueprints_done"], status["importable"], status["not_importable"]) == (3, 1, 2)
    found = {b["name"]: b for b in client.get("/dimos/discovery/blueprints").json()["blueprints"]}
    assert found["good"]["robot"] == "go2"
    assert "crashed" in found["crashes"]["import_error"]
    assert found["missing"]["suggested_extras"] == ["unitree-dds"]
    catalog = client.get("/dimos/catalog").json()
    assert [b["name"] for b in catalog["blueprints"]] == ["good"]
    assert catalog["modules"][0]["skills"] == ["snap"] and catalog["modules"][0]["robots"] == [
        "go2"
    ]
    assert catalog["skills"][0]["module"] == "cam"
    assert any(e.startswith("blueprint missing: ") for e in catalog["errors"])


def test_a_blueprint_change_invalidates_the_scan(
    client: TestClient, scanned: State, sent: list[Any]
) -> None:
    client.get("/dimos/catalog")
    scanned.blueprints_changed(["new"], [])
    assert scanned.scanner.task is None
    assert (
        "ns/dimos/events/blueprints",
        {"type": "blueprints", "added": ["new"], "removed": []},
    ) in sent


def test_blueprint_list_carries_import_status(client: TestClient, scanned: State) -> None:
    scanned.listed = [{"name": "good", "kind": "builtin"}, {"name": "other", "kind": "builtin"}]
    client.get("/dimos/catalog")
    listed = client.get("/dimos/blueprints").json()["blueprints"]
    assert listed[0]["importable"] is True and listed[1]["importable"] is None


def test_extras_status_from_pyproject(tmp_path: Path) -> None:
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\nversion = "1"\n[project.optional-dependencies]\n'
        'sim = ["mujoco>=3"]\nall = ["dimos[sim]", "torch"]\n'
    )
    probe = {"environment": {}, "packages": {"mujoco": "3.1", "torch": "2.0"}}
    status = {e["name"]: e for e in discovery.extras_status(tmp_path, probe)}
    assert status["sim"]["installed"] and status["all"]["includes"] == ["sim"]
    missing = {e["name"]: e for e in discovery.extras_status(tmp_path, {"packages": {}})}
    assert missing["all"]["missing"] == ["torch", "mujoco"]


def test_extras_install_runs_as_a_job(
    client: TestClient, state: State, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (state.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\nversion = "1"\n[project.optional-dependencies]\nsim = ["mujoco"]\n'
    )
    (state.dimos_dir / "uv.lock").write_text("version = 1\n")
    uv = tmp_path / "uv"
    uv.write_text('#!/bin/sh\necho installing "$@"\n')
    uv.chmod(0o755)
    monkeypatch.setattr(discovery, "find_uv", lambda: str(uv))

    async def packages(args: list[str], stdin: Any = None) -> Any:
        return {"python": sys.executable, "environment": {}, "packages": {}}

    monkeypatch.setattr(state, "introspected", packages)
    monkeypatch.setattr(state.scanner, "refresh", lambda reason="": None)
    assert client.post("/dimos/extras/install", json={"extras": ["nope"]}).status_code == 400
    job = client.post("/dimos/extras/install", json={"extras": ["sim"]}).json()["job"]
    until(lambda: client.get(f"/dimos/jobs/{job}/log").json()["done"])
    log = client.get(f"/dimos/jobs/{job}/log").json()
    assert log["ok"] and any("--extra sim" in line for line in log["lines"])
    assert client.get("/dimos/jobs/none/log").status_code == 404


def test_custom_robot_guide(client: TestClient, checkout: Path) -> None:
    assert client.get("/dimos/docs/custom-robot").status_code == 404
    (checkout / "docs").mkdir()
    (checkout / "docs" / "adding_a_new_robot.md").write_text(
        "# Add your own robot\n\nSee [blueprints](b.md).\n"
    )
    (checkout / "mkdocs.yml").write_text("site_url: https://docs.example/\n")
    guide = client.get("/dimos/docs/custom-robot").json()
    assert guide["title"] == "Add your own robot"
    assert "https://docs.example/b/" in guide["markdown"]
    assert guide["html"].startswith("<h1>")


def test_unitree_dds_on_a_machine_without_a_wheel_builds_cyclonedds_with_nix_first(
    client: TestClient, state: State, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (state.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\nversion = "1"\n[project.optional-dependencies]\n'
        'unitree-dds = ["cyclonedds>=0.10"]\n'
    )
    (state.dimos_dir / "uv.lock").write_text(
        '[[package]]\nname = "cyclonedds"\nversion = "0.10.5"\nwheels = [{ url = "https://x/cyclonedds-0.10.5-cp310-cp310-macosx_11_0_arm64.whl" }]\n'
    )
    link = state.dimos_dir / ".venv" / "cyclonedds"
    nix = tmp_path / "nix"
    nix.write_text(f"#!/bin/sh\nmkdir -p {tmp_path}/store/lib && ln -sfn {tmp_path}/store {link}\n")
    nix.chmod(0o755)
    uv = tmp_path / "uv"
    uv.write_text('#!/bin/sh\necho "CYCLONEDDS_HOME=$CYCLONEDDS_HOME" "$@"\n')
    uv.chmod(0o755)
    monkeypatch.setattr(discovery, "find_uv", lambda: str(uv))
    monkeypatch.setattr(discovery, "find_nix", lambda: str(nix))
    monkeypatch.delenv("CYCLONEDDS_HOME", raising=False)
    environment = {
        "platform_system": "Darwin",
        "platform_machine": "arm64",
        "python_version": "3.12",
    }

    async def packages(args: list[str], stdin: Any = None) -> Any:
        return {"python": sys.executable, "environment": environment, "packages": {}}

    monkeypatch.setattr(state, "introspected", packages)
    monkeypatch.setattr(state.scanner, "refresh", lambda reason="": None)
    job = client.post("/dimos/extras/install", json={"extras": ["unitree-dds"]}).json()["job"]
    until(lambda: client.get(f"/dimos/jobs/{job}/log").json()["done"])
    log = client.get(f"/dimos/jobs/{job}/log").json()
    assert log["ok"], log
    assert log["lines"][0].startswith(
        f"$ {nix} --extra-experimental-features nix-command flakes build --out-link {link} "
    )
    assert f"CYCLONEDDS_HOME={(tmp_path / 'store').resolve()} sync --locked --inexact" in "\n".join(
        log["lines"]
    )
