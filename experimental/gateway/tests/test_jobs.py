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
import subprocess
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
          "missing_module": None if ok else "unitree_sdk2py", "doc": "",
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


def test_the_scan_survives_a_restart_and_says_when_the_checkout_changed(
    client: TestClient, scanned: State
) -> None:
    checkout = scanned.dimos_dir
    subprocess.run(["git", "init", "-q", str(checkout)], check=True)
    (checkout / "blueprint.py").write_text("x = 1\n")
    scanned.listed = [{"name": "good", "kind": "builtin"}]
    job = client.post("/catalog/scan").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])

    later = discovery.Scanner(checkout, scanned.jobs, ["false"])
    assert later.catalog()["modules"][0]["name"] == "cam"
    assert later.snapshot()["scanned"] and not later.snapshot()["stale"]
    (checkout / "blueprint.py").write_text("x = 2\n")
    after = discovery.Scanner(checkout, scanned.jobs, ["false"])
    assert after.snapshot()["stale"] and after.catalog()["modules"][0]["name"] == "cam"


def test_the_catalog_answers_from_its_cache_and_only_a_scan_changes_it(
    client: TestClient, scanned: State, sent: list[Any]
) -> None:
    (scanned.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\n[project.optional-dependencies]\nunitree-dds = ["unitree-sdk2py"]\n'
    )
    scanned.listed = [
        {"name": n, "kind": "builtin"} for n in ("good", "crashes", "missing", "other")
    ]
    never = client.get("/catalog").json()
    assert never["modules"] == [] and never["scan"]["scanned"] is False and never["scan"]["stale"]
    assert client.get("/blueprints").json()["scan"]["state"] == "idle"
    assert scanned.scanner.task is None
    response = client.post("/catalog/scan")
    assert response.status_code == 202
    job = response.json()["job"]
    assert client.post("/catalog/scan").json()["job"] == job
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    status = client.get(f"/jobs/{job}").json()
    assert status["ok"] and status["kind"] == "scan"
    assert status["progress"] == {"done": 3, "total": 3, "current": None}
    assert any("crashed" in line for line in status["lines"])
    answer = client.get("/blueprints").json()
    scan = answer["scan"]
    assert {k: scan[k] for k in ("job", "state", "scanned", "stale", "finished_at")} == {
        "job": job,
        "state": "idle",
        "scanned": True,
        "stale": False,
        "finished_at": status["finished_at"],
    }
    assert 0 <= scan["age_seconds"] < 60
    found = {b["name"]: b for b in answer["blueprints"]}
    assert found["good"]["robot"] == "go2" and found["good"]["modules"][0]["class"] == "x.Cam"
    assert "crashed" in found["crashes"]["import_error"]
    assert found["missing"]["suggested_extras"] == ["unitree-dds"]
    assert found["other"] == {"name": "other", "kind": "builtin"}
    catalog = client.get("/catalog").json()
    assert [b["name"] for b in catalog["blueprints"]] == ["good"]
    assert catalog["modules"][0]["skills"] == ["snap"] and catalog["modules"][0]["robots"] == [
        "go2"
    ]
    assert catalog["skills"][0]["module"] == "cam" and catalog["scan"]["scanned"]
    assert any(e.startswith("blueprint missing: ") for e in catalog["errors"])
    one = client.get("/catalog?module=x.Cam").json()
    assert [m["name"] for m in one["modules"]] == ["cam"] and [
        s["name"] for s in one["skills"]
    ] == ["snap"]
    assert client.get("/catalog?module=nope").json()["modules"] == []
    bp = client.get("/catalog?blueprint=good").json()
    assert [b["name"] for b in bp["blueprints"]] == ["good"] and [
        m["name"] for m in bp["modules"]
    ] == ["cam"]
    assert "stale" in bp["scan"] and "age_seconds" in bp["scan"]
    assert client.get("/catalog?blueprint=nope").json()["modules"] == []
    names = client.get("/blueprints?fields=name").json()["blueprints"]
    assert {b["name"] for b in names} == set(found) and all(b.keys() == {"name"} for b in names)
    assert client.get("/blueprints?fields=robot").json()["blueprints"][0].keys() <= {
        "name",
        "robot",
    }
    scanned.blueprints_changed(["new"], [])
    assert client.get("/catalog").json()["modules"]
    assert (
        "ns/dimos/events/blueprints",
        {"type": "blueprints", "added": ["new"], "removed": []},
    ) in sent
    again = client.post("/catalog/scan").json()["job"]
    assert again != job
    during = client.get("/catalog").json()
    assert during["modules"] and during["scan"]["state"] in ("scanning", "idle")


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
    assert client.post("/extras/install", json={"extras": ["nope"]}).status_code == 400
    started = client.post("/extras/install", json={"extras": ["sim"]})
    assert started.status_code == 202
    job = started.json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    log = client.get(f"/jobs/{job}").json()
    assert log["ok"] and any("--extra sim" in line for line in log["lines"])
    assert client.get(f"/jobs/{job}").json()["progress"] is None
    assert client.get("/jobs/none").status_code == 404


def test_docs_search_opens_the_docs_site_on_the_query(client: TestClient, checkout: Path) -> None:
    assert client.get("/docs/search?q=x", follow_redirects=False).status_code == 404
    (checkout / "mkdocs.yml").write_text("site_url: https://docs.example/\n")
    answer = client.get("/docs/search?q=go2 blueprint", follow_redirects=False)
    assert answer.status_code == 307
    assert answer.headers["location"] == "https://docs.example/?q=go2%20blueprint"


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
    job = client.post("/extras/install", json={"extras": ["unitree-dds"]}).json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    log = client.get(f"/jobs/{job}").json()
    assert log["ok"], log
    assert log["lines"][0].startswith(
        f"$ {nix} --extra-experimental-features nix-command flakes build --out-link {link} "
    )
    assert f"CYCLONEDDS_HOME={(tmp_path / 'store').resolve()} sync --locked --inexact" in "\n".join(
        log["lines"]
    )
