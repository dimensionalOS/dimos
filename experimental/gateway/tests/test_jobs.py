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

from experimental.gateway import catalog, constants, discovery, introspect
from experimental.gateway.state import State

FAKE_SCAN = """
import json, os, sys
request = json.loads(sys.stdin.read() or "null") or {{}}
def emit(value):
    print("\\n{marker}" + json.dumps(value), flush=True)
names = request.get("blueprints")
if names is None:
    names = json.loads(os.environ.get("FAKE_SCAN_NAMES", '["good", "crashes", "missing"]'))
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
    if name == "dies_after":
        os._exit(3)
    if ok and request.get("modules") == []:
        emit({{"kind": "module", "name": "cam", "class": "x.Cam", "doc": "A rescanned camera.", "inputs": [],
              "outputs": [], "rpcs": [{{"name": "snap", "doc": "", "params": [], "return_type": None, "skill": True}}]}})
for name in request.get("modules", ["cam"]):
    emit({{"kind": "start", "name": "module:" + name}})
    emit({{"kind": "module", "name": name, "class": "x.Cam", "doc": "A camera.", "inputs": [], "outputs": [],
          "rpcs": [{{"name": "snap", "doc": "", "params": [], "return_type": None, "skill": True}}]}})
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
    # as in a real checkout: the scan's child, run under coverage in CI, writes .coverage.* where it runs
    (checkout / ".gitignore").write_text(".coverage*\n")
    (checkout / "blueprint.py").write_text("x = 1\n")
    scanned.listed = [{"name": "good", "kind": "builtin"}]
    job = client.post("/catalog/scan").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])

    later = discovery.Scanner(checkout, scanned.jobs, ["false"])
    assert catalog.modules(later.cache())[0]["name"] == "cam"
    assert later.snapshot()["scanned"] and not later.snapshot()["stale"]
    (checkout / "blueprint.py").write_text("x = 2\n")
    after = discovery.Scanner(checkout, scanned.jobs, ["false"])
    assert after.snapshot()["stale"] and catalog.modules(after.cache())[0]["name"] == "cam"


def test_a_child_dying_right_after_a_record_still_scans_the_rest(
    client: TestClient, scanned: State, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv("FAKE_SCAN_NAMES", '["dies_after", "good", "missing"]')
    scanned.listed = [
        {"name": name, "kind": "builtin"} for name in ("dies_after", "good", "missing")
    ]
    job = client.post("/catalog/scan").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    assert client.get(f"/jobs/{job}").json()["ok"]
    assert set(scanned.scanner.cache()["results"]) == {"dies_after", "good", "missing"}


def test_a_scan_takes_its_per_blueprint_timeout_from_the_request(
    client: TestClient, state: State, monkeypatch: pytest.MonkeyPatch
) -> None:
    async def no_scan(job: discovery.Job) -> None:
        job.finish(True, None)

    monkeypatch.setattr(state.scanner, "scan", no_scan)
    job = client.post("/catalog/scan").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    assert state.scanner.item_timeout_s == constants.SCAN_ITEM_TIMEOUT_S
    job = client.post("/catalog/scan?timeout=5").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    assert state.scanner.item_timeout_s == 5


def test_the_catalog_answers_from_its_cache_and_only_a_scan_changes_it(
    client: TestClient, scanned: State, sent: list[Any]
) -> None:
    (scanned.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\n[project.optional-dependencies]\nunitree-dds = ["unitree-sdk2py"]\n'
    )
    scanned.listed = [
        {"name": name, "kind": "builtin"} for name in ("good", "crashes", "missing", "other")
    ]
    never = client.get("/catalog").json()
    assert never["modules"] == [] and never["scan"]["scanned"] is False and never["scan"]["stale"]
    assert client.get("/catalog/blueprints").json()["scan"]["state"] == "idle"
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
    answer = client.get("/catalog/blueprints").json()
    scan = answer["scan"]
    assert {key: scan[key] for key in ("job", "state", "scanned", "stale", "finished_at")} == {
        "job": job,
        "state": "idle",
        "scanned": True,
        "stale": False,
        "finished_at": status["finished_at"],
    }
    assert 0 <= scan["age_seconds"] < 60
    found = {blueprint["name"]: blueprint for blueprint in answer["blueprints"]}
    assert found["good"]["robot"] == "go2" and found["good"]["modules"] == ["cam"]
    assert found["good"]["scanned_at"] == status["finished_at"]
    assert "crashed" in found["crashes"]["import_error"]
    assert found["missing"]["suggested_extras"] == ["unitree-dds"]
    assert found["other"]["importable"] is None and found["other"]["scanned_at"] is None
    catalog = client.get("/catalog").json()
    assert [blueprint["name"] for blueprint in catalog["blueprints"]] == [
        "good",
        "crashes",
        "missing",
        "other",
    ]
    assert catalog["modules"][0]["blueprints"] == ["good"] and "go2" in catalog["robots"]
    assert catalog["modules"][0]["rpcs"] == ["snap"] and catalog["modules"][0]["robots"] == ["go2"]
    assert (
        catalog["rpcs"][0]["module"] == "cam"
        and catalog["rpcs"][0]["skill"]
        and catalog["scan"]["scanned"]
    )
    assert any(error.startswith("blueprint missing: ") for error in catalog["errors"])
    one = client.get("/catalog?module=x.Cam").json()
    assert [module["name"] for module in one["modules"]] == ["cam"] and [
        rpc["name"] for rpc in one["rpcs"]
    ] == ["snap"]
    assert client.get("/catalog?module=nope").json()["modules"] == []
    good_only = client.get("/catalog?blueprint=good").json()
    assert [blueprint["name"] for blueprint in good_only["blueprints"]] == ["good"] and [
        module["name"] for module in good_only["modules"]
    ] == ["cam"]
    assert "stale" in good_only["scan"] and "age_seconds" in good_only["scan"]
    assert client.get("/catalog?blueprint=nope").json()["modules"] == []
    names = client.get("/catalog/blueprints?fields=blueprints.name").json()
    assert {blueprint["name"] for blueprint in names["blueprints"]} == set(found)
    assert (
        all(blueprint.keys() == {"name"} for blueprint in names["blueprints"]) and "scan" in names
    )
    picked = client.get("/catalog?fields=modules.name,robots.name").json()
    assert set(picked) == {"modules", "robots", "scan"} and picked["modules"] == [{"name": "cam"}]
    assert picked["robots"]["go2"] == {"name": "Unitree Go2"}
    unknown = client.get("/catalog/blueprints?fields=blueprints.nmae")
    assert unknown.status_code == 400 and "blueprints.nmae" in unknown.json()["error"]
    assert client.get("/catalog/modules").json()["modules"][0]["name"] == "cam"
    assert (
        client.get("/catalog/rpcs?namespace=robot0").json()["rpcs"][0]["rpc"] == "robot0/cam/snap"
    )
    assert client.get("/catalog/rpcs/cam/snap").json()["module"] == "cam"
    assert client.get("/catalog/rpcs/cam/nope").status_code == 404
    assert client.get("/catalog/modules/nope").status_code == 404
    assert client.get("/catalog/msgs/sensor_msgs.Image").json() == {
        "name": "sensor_msgs.Image",
        "definition": None,
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
    status = {extra["name"]: extra for extra in discovery.extras_status(tmp_path, probe)}
    assert status["sim"]["installed"] and status["all"]["includes"] == ["sim"]
    missing = {
        extra["name"]: extra for extra in discovery.extras_status(tmp_path, {"packages": {}})
    }
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


def test_one_blueprint_rescans_without_a_full_scan(client: TestClient, scanned: State) -> None:
    scanned.listed = [{"name": "good", "kind": "builtin"}, {"name": "missing", "kind": "builtin"}]
    assert client.post("/catalog/scan/blueprints/nope").status_code == 404
    job = client.post("/catalog/scan/blueprints/good").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    assert client.get(f"/jobs/{job}").json()["ok"]
    found = {
        blueprint["name"]: blueprint
        for blueprint in client.get("/catalog/blueprints").json()["blueprints"]
    }
    assert found["good"]["importable"] and found["good"]["scanned_at"]
    assert found["missing"]["importable"] is None
    scan = client.get("/catalog").json()
    assert scan["scan"]["scanned"] is False and scan["modules"][0]["doc"] == "A rescanned camera."


def test_a_module_answers_with_its_methods_and_config(
    client: TestClient, scanned: State, monkeypatch: pytest.MonkeyPatch
) -> None:
    scanned.listed = [{"name": "good", "kind": "builtin"}]
    job = client.post("/catalog/scan/blueprints/good").json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])

    async def fake(args: list[str], stdin: Any = None) -> Any:
        assert args[1] == "x.Cam"
        if args[0] == "module_config":
            return {"name": "cam", "class": "x.Cam", "args": {}}
        return {"rpcs": [], "file": "x.py", "line": 1}

    monkeypatch.setattr(scanned, "introspected", fake)
    cam = client.get("/catalog/modules/x.Cam").json()
    assert cam["name"] == "cam" and cam["blueprints"] == ["good"] and cam["file"] == "x.py"
    assert client.get("/catalog/modules/cam/config").json()["class"] == "x.Cam"
    cache = scanned.scanner.cache()
    assert cache is not None
    cache["modules"]["x.f.<locals>.Cam"] = {
        **cache["modules"]["x.Cam"],
        "name": "Cam",
        "class": "x.f.<locals>.Cam",
    }
    assert client.get("/catalog/modules/Cam").status_code == 404


def test_only_the_latest_finished_job_of_each_kind_is_kept() -> None:
    jobs = discovery.Jobs()
    first, second = jobs.add("one", "scan", []), jobs.add("two", "scan", [])
    other, running = jobs.add("three", "install", []), jobs.add("four", "scan", [])
    for job in (first, second, other):
        job.finish(True)
    assert jobs.get(first.id) is None
    assert jobs.get(second.id) is second and jobs.get(other.id) is other
    assert jobs.get(running.id) is running


def test_an_extras_install_forgets_the_cached_package_probe(
    client: TestClient, state: State, monkeypatch: pytest.MonkeyPatch
) -> None:
    (state.dimos_dir / "pyproject.toml").write_text(
        '[project]\nname = "dimos"\n[project.optional-dependencies]\nsim = ["mujoco"]\n'
    )
    probes: list[list[str]] = []

    async def probe(dimos_dir: Path, args: list[str], *rest: Any, **options: Any) -> Any:
        probes.append(args)
        return {"python": sys.executable, "environment": {}, "packages": {}}

    monkeypatch.setattr(introspect, "run_child", probe)
    monkeypatch.setattr(discovery, "find_uv", lambda: "uv")
    monkeypatch.setattr(discovery, "install_command", lambda *args: [sys.executable, "-c", "pass"])
    assert client.get("/extras").status_code == 200
    assert client.get("/extras").status_code == 200
    assert len(probes) == 1
    job = client.post("/extras/install", json={"extras": ["sim"]}).json()["job"]
    until(lambda: client.get(f"/jobs/{job}").json()["done"])
    assert client.get(f"/jobs/{job}").json()["ok"]
    client.get("/extras")
    assert len(probes) == 2
