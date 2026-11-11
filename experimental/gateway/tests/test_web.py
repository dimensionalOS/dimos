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
import re
import subprocess
import sys
from typing import Any

from fastapi.testclient import TestClient
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.deno import ensure_deno
from experimental.gateway import launches
from experimental.gateway.routes import catalog as catalog_routes
from experimental.gateway.state import State

ROUND_TRIP = """
import { decode, decodeChannel, geometry_msgs } from "MODULE"
const bytes = (hex) => Uint8Array.from(hex.match(/../g), (b) => parseInt(b, 16))
const hex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, "0")).join("")
console.log(JSON.stringify({
    pose: decode(bytes(Deno.args[0])),
    byKey: decodeChannel("dimos/odom/geometry_msgs.PoseStamped", bytes(Deno.args[0])),
    twist: hex(geometry_msgs.Twist.encode({ linear: { x: 0.3 }, angular: { z: 0.5 } })),
}, (_, v) => typeof v === "bigint" ? String(v) : v))
"""


def fake_introspection(
    state: State, monkeypatch: pytest.MonkeyPatch, blueprint_file: str = "bp.py"
) -> None:
    async def fake(args: list[str], stdin: Any = None, source: Any = None) -> Any:
        if args[0] == "module":
            return {"rpcs": [], "file": "mapper.py", "line": 5}
        module = {"name": "mapper", "class": "x.Mapper", "file": "mapper.py", "line": 5}
        return {"name": args[1], "file": blueprint_file, "line": 3, "modules": [module]}

    state.listed = [{"name": "demo", "kind": "builtin"}]
    monkeypatch.setattr(state, "introspected", fake)


def test_a_blueprints_and_its_modules_source(
    client: TestClient, state: State, checkout: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (checkout / "bp.py").write_text("demo = 1\n")
    (checkout / "mapper.py").write_text("class Mapper: ...\n")
    fake_introspection(state, monkeypatch)
    assert client.get("/catalog/blueprints/demo/source").json() == {
        "name": "demo",
        "file": "bp.py",
        "line": 3,
        "text": "demo = 1\n",
    }
    found = client.get("/catalog/modules/mapper/source?blueprint=demo").json()
    assert (found["file"], found["line"], found["text"]) == ("mapper.py", 5, "class Mapper: ...\n")
    assert client.get("/catalog/modules/x.Mapper/source?blueprint=demo").status_code == 200
    assert client.get("/catalog/modules/other/source?blueprint=demo").status_code == 404
    assert client.get("/catalog/blueprints/nope/source").status_code == 404
    picked = client.get("/catalog/blueprints/demo?fields=modules.name").json()
    assert picked["modules"] == [{"name": "mapper"}]

    async def importable(state: State, name: str) -> dict[str, Any]:
        return {"name": name, "class": "x.Mapper"}

    monkeypatch.setattr(catalog_routes, "importable_module", importable)
    assert client.get("/catalog/modules/mapper/source").json()["text"] == "class Mapper: ...\n"


@pytest.mark.parametrize("file", ["pyproject.toml", "../outside.py", "gone.py", None])
def test_source_reads_only_python_in_the_checkout(
    client: TestClient, state: State, checkout: Path, monkeypatch: pytest.MonkeyPatch, file: Any
) -> None:
    (checkout / "pyproject.toml").write_text("")
    (checkout.parent / "outside.py").write_text("")
    fake_introspection(state, monkeypatch, file)
    assert client.get("/catalog/blueprints/demo/source").status_code == 404


def test_source_of_a_blueprint_live_from_another_checkout_reads_that_checkout(
    client: TestClient,
    state: State,
    checkout: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    (checkout / "mapper.py").write_text("x = 1\n")
    other = tmp_path / "branch"
    other.mkdir()
    (other / "mapper.py").write_text("x = 2\n")
    found = launches.RunSource(checkout=other, python=sys.executable, run_id="r1")
    monkeypatch.setattr(
        catalog_routes, "running_source", lambda state, name: found if name == "demo" else None
    )
    fake_introspection(state, monkeypatch)
    assert client.get("/catalog/modules/mapper/source?blueprint=demo").json()["text"] == "x = 2\n"


def test_login_page_calls_only_gateway_routes(client: TestClient) -> None:
    page = client.get("/cloud/login/page").text
    paths = client.get("/openapi.json").json()["paths"]
    # the page's fetches are call("METHOD", "/path"), relative to /cloud
    calls = {
        (method.lower(), "/cloud" + path.split("?")[0])
        for method, args in re.findall(r'call\("([A-Z]+)", ([^)]*)\)', page)
        for path in re.findall(r'"(/[^"]*)"', args)
    }
    assert {
        ("get", "/cloud/account"),
        ("post", "/cloud/login"),
        ("delete", "/cloud/login"),
    } <= calls
    assert [call for call in calls if call[0] not in paths.get(call[1], {})] == []


def test_msgs_js_is_cached_and_decodes_what_python_encodes(
    client: TestClient, home: Path, tmp_path: Path
) -> None:
    answer = client.get("/msgs.js")
    (built,) = (home / "state" / "gateway" / "msgs").glob("msgs-*.js")
    assert answer.status_code == 200 and answer.content == built.read_bytes()
    again = client.get("/msgs.js", headers={"if-none-match": answer.headers["etag"]})
    assert again.status_code == 304
    types = built.with_suffix(".d.ts")
    assert set(built.parent.iterdir()) == {built, types}, "a second ask reuses the build"
    assert answer.headers["x-typescript-types"] == "./msgs.d.ts"
    assert client.get("/msgs.d.ts").content == types.read_bytes()
    pose = PoseStamped(
        ts=12.25, frame_id="map", position=[1.0, 2.0, 3.0], orientation=[0, 0, 0.6, 0.8]
    )
    twist = Twist(linear=Vector3(0.3, 0, 0), angular=Vector3(0, 0, 0.5))
    script = tmp_path / "round_trip.mjs"
    script.write_text(ROUND_TRIP.replace("MODULE", built.as_uri()))
    done = subprocess.run(
        [
            str(ensure_deno()),
            "run",
            "--no-config",
            "--allow-read",
            str(script),
            pose.lcm_encode().hex(),
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert done.returncode == 0, done.stderr
    out = json.loads(done.stdout)
    assert out["pose"] == out["byKey"]
    assert out["pose"]["header"]["frame_id"] == "map"
    assert out["pose"]["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert out["twist"] == twist.lcm_encode().hex()
    typed = tmp_path / "typed.ts"
    typed.write_text(
        f'// @ts-types="{types.as_uri()}"\nimport {{ geometry_msgs }} from "{built.as_uri()}"\n'
        "const twist: geometry_msgs.Twist = geometry_msgs.Twist.decode(new Uint8Array())\n"
        "const wrong: string = twist.linear.x\n"
    )
    checked = subprocess.run(
        [str(ensure_deno()), "check", "--no-config", str(typed)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert "Type 'number' is not assignable to type 'string'" in checked.stdout + checked.stderr
    assert checked.stderr.count("TS2") == 1, checked.stderr
