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
from types import SimpleNamespace
from typing import Any

from fastapi.testclient import TestClient

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.deno import ensure_deno
from experimental.gateway import msgs
from experimental.gateway.state import State
from experimental.gateway.topic_rates import TopicWatch

VIEW = Path(__file__).parents[1] / "assets" / "blueprint_view"

ROUND_TRIP = """
import { decode, decodeChannel, geometry_msgs } from "MODULE"
const input = JSON.parse(Deno.args[0])
const bytes = (hex) => Uint8Array.from(hex.match(/../g), (b) => parseInt(b, 16))
const hex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, "0")).join("")
console.log(JSON.stringify({
    pose: decode(bytes(input.pose)),
    byKey: decodeChannel("dimos/odom/geometry_msgs.PoseStamped", bytes(input.pose)),
    twist: hex(geometry_msgs.Twist.encode({ linear: { x: 0.3 }, angular: { z: 0.5 } })),
}, (_, v) => typeof v === "bigint" ? String(v) : v))
"""


def test_blueprint_view_and_its_files(client: TestClient, state: State) -> None:
    state.listed = [{"name": "demo", "kind": "builtin"}]
    page = client.get("/dimos/blueprint_view?name=demo")
    assert page.status_code == 200 and "<html" in page.text.lower()
    assert client.get("/dimos/blueprint_view?name=nope").status_code == 404
    assert client.get("/dimos/blueprint_view?name=demo&view=x").status_code == 400
    script = client.get("/dimos/blueprint_view/app.js")
    assert script.headers["content-type"].startswith("text/javascript")
    assert client.get("/dimos/blueprint_view/..%2Fmsgs.py").status_code == 404


def test_source_reads_only_python_in_the_checkout(client: TestClient, checkout: Path) -> None:
    (checkout / "mod.py").write_text("x = 1\n")
    assert client.get("/dimos/source?file=mod.py").json() == {"file": "mod.py", "text": "x = 1\n"}
    assert client.get("/dimos/source?file=pyproject.toml").status_code == 400
    assert client.get("/dimos/source?file=../outside.py").status_code == 400
    assert client.get("/dimos/source?file=gone.py").status_code == 404


def test_topic_rates_count_what_is_heard(client: TestClient, state: State) -> None:
    assert client.get("/dimos/topics/rates").json()["up"] is False
    now = [100.0]
    callbacks: list[Any] = []
    session = SimpleNamespace(declare_subscriber=lambda key, callback: callbacks.append(callback))
    state.topics = TopicWatch(lambda: session, clock=lambda: now[0])
    state.topics.start()
    for _ in range(4):
        callbacks[0](SimpleNamespace(key_expr="dimos/odom/nav_msgs.Odometry", payload=b"x" * 10))
    callbacks[0](SimpleNamespace(key_expr="dimos/rpc/Foo/bar", payload=b""))
    (row,) = client.get("/dimos/topics/rates").json()["topics"]
    assert (row["topic"], row["type"], row["hz"], row["bps"]) == (
        "/odom",
        "nav_msgs.Odometry",
        2.0,
        20.0,
    )
    now[0] += 5
    assert client.get("/dimos/topics/rates").json()["topics"][0]["hz"] == 0


def test_login_page(client: TestClient) -> None:
    assert "<html" in client.get("/dimos/cloud/login/page").text.lower()


def test_msgs_js_is_current_and_served_with_an_etag(client: TestClient) -> None:
    assert msgs.bundle_js(msgs.generate_ts(msgs.scan())) == msgs.JS_FILE.read_text()
    answer = client.get("/dimos/msgs.js")
    assert answer.content == msgs.JS_FILE.read_bytes()
    again = client.get("/dimos/msgs.js", headers={"if-none-match": answer.headers["etag"]})
    assert again.status_code == 304


def test_msgs_js_decodes_what_python_encodes(tmp_path: Path) -> None:
    pose = PoseStamped(
        ts=12.25, frame_id="map", position=[1.0, 2.0, 3.0], orientation=[0, 0, 0.6, 0.8]
    )
    twist = Twist(linear=Vector3(0.3, 0, 0), angular=Vector3(0, 0, 0.5))
    script = tmp_path / "round_trip.mjs"
    script.write_text(ROUND_TRIP.replace("MODULE", msgs.JS_FILE.as_uri()))
    frames = {"pose": pose.lcm_encode().hex()}
    done = subprocess.run(
        [str(ensure_deno()), "run", "--no-config", "--allow-read", str(script), json.dumps(frames)],
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


def test_no_graph_layout_overlaps_nodes() -> None:
    out = subprocess.run(
        [str(ensure_deno()), "run", "--no-prompt", "layout_check.js"],
        cwd=VIEW,
        capture_output=True,
        text=True,
        timeout=120,
        check=True,
    ).stdout
    results = json.loads(out)
    assert results and [r for r in results if r["overlaps"]] == []
