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
import subprocess

from fastapi.testclient import TestClient
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.deno import ensure_deno
from experimental.gateway import web_view
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


def test_blueprint_view_and_its_files(
    client: TestClient, state: State, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # a wheel's shape: a built dist and no sources, so nothing builds
    (tmp_path / "dist" / "blueprint_view").mkdir(parents=True)
    (tmp_path / "dist" / "index.html").write_text("<html></html>")
    (tmp_path / "dist" / "blueprint_view" / "index-Ab_1.js").write_text("export {}")
    monkeypatch.setattr(web_view, "app_dir", lambda: tmp_path)
    state.listed = [{"name": "demo", "kind": "builtin"}]
    page = client.get("/blueprint_view?name=demo")
    assert page.status_code == 200 and "<html" in page.text.lower()
    assert client.get("/blueprint_view?name=nope").status_code == 404
    assert client.get("/blueprint_view?name=demo&view=x").status_code == 400
    script = client.get("/blueprint_view/index-Ab_1.js")
    assert script.headers["content-type"].startswith("text/javascript")
    assert client.get("/blueprint_view/..%2Fmsgs.py").status_code == 404


def test_the_blueprint_view_builds_when_its_sources_change(tmp_path: Path) -> None:
    web = tmp_path / "web"
    (web / "blueprint_view" / "src").mkdir(parents=True)
    (web / "deno.json").write_text("{}")
    (web / "blueprint_view" / "src" / "main.tsx").write_text("1")
    before = web_view.stamp(web, web / "blueprint_view")
    (web / "blueprint_view" / "src" / "main.test.ts").write_text("tests don't count")
    (web / "blueprint_view" / "dist").mkdir()
    (web / "blueprint_view" / "dist" / "index.html").write_text("built output doesn't count")
    assert web_view.stamp(web, web / "blueprint_view") == before
    (web / "blueprint_view" / "src" / "main.tsx").write_text("2")
    assert web_view.stamp(web, web / "blueprint_view") != before


def test_source_reads_only_python_in_the_checkout(client: TestClient, checkout: Path) -> None:
    (checkout / "mod.py").write_text("x = 1\n")
    assert client.get("/source?file=mod.py").json() == {"file": "mod.py", "text": "x = 1\n"}
    assert client.get("/source?file=pyproject.toml").status_code == 400
    assert client.get("/source?file=../outside.py").status_code == 400
    assert client.get("/source?file=gone.py").status_code == 404


def test_login_page(client: TestClient) -> None:
    assert "<html" in client.get("/cloud/login/page").text.lower()


def test_msgs_js_is_cached_and_decodes_what_python_encodes(
    client: TestClient, home: Path, tmp_path: Path
) -> None:
    answer = client.get("/msgs.js")
    (built,) = (home / "state" / "gateway" / "msgs").glob("msgs-*.js")
    assert answer.status_code == 200 and answer.content == built.read_bytes()
    again = client.get("/msgs.js", headers={"if-none-match": answer.headers["etag"]})
    assert again.status_code == 304
    assert list(built.parent.iterdir()) == [built], "a second ask reuses the build"
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
