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

"""msgs.ts/msgs.js (experimental/gateway/msgs/): current, warning about hand-written messages, served, and decoding what
Python encodes (under deno, when it's there)."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

from fastapi.testclient import TestClient
import pytest

from experimental.gateway.server.app import create_app
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.events import Bus
from experimental.gateway.utils.msgs import codegen as msgs
from experimental.gateway.utils.uploads import Uploads

REPO = Path(__file__).parents[3]


@pytest.fixture(scope="module")
def found() -> msgs.Scan:
    return msgs.scan()


def test_msgs_ts_and_js_are_current(found: msgs.Scan) -> None:
    assert msgs.stale_problems(found) == []


def test_a_stale_file_is_a_problem(
    found: msgs.Scan, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    ts, js = tmp_path / "msgs.ts", tmp_path / "msgs.js"
    ts.write_text(msgs.TS_FILE.read_text() + "// edited\n")
    js.write_text(msgs.JS_FILE.read_text())
    monkeypatch.setattr(msgs, "TS_FILE", ts)
    monkeypatch.setattr(msgs, "JS_FILE", js)
    (problem,) = msgs.stale_problems(found)
    assert "msgs.ts is stale" in problem and msgs.WRITE_COMMAND in problem
    # msgs.js bundled from another msgs.ts
    ts.write_text(msgs.TS_FILE.read_text().replace("// edited\n", ""))
    js.write_text(js.read_text().replace("sha256:", "sha256:0", 1))
    (problem,) = msgs.stale_problems(found)
    assert "msgs.js wasn't bundled from the current msgs.ts" in problem


def test_hand_written_messages_are_warnings_not_types(found: msgs.Scan) -> None:
    missing = {m.type_name for m in found.missing}
    assert {
        "sensor_msgs.JointCommand",
        "sensor_msgs.MotorCommandArray",
        "sensor_msgs.RobotState",
        "trajectory_msgs.JointTrajectory",
    } <= missing
    assert not missing & found.schemas.keys()
    assert {"geometry_msgs.PoseStamped", "geometry_msgs.Twist", "sensor_msgs.JointState"} <= set(
        found.schemas
    )
    # dimos's JointTrajectory is not the ROS one dimos_lcm has under its name
    (trajectory,) = [m for m in found.missing if m.type_name == "trajectory_msgs.JointTrajectory"]
    assert "own fingerprint" in trajectory.reason
    assert any(line.startswith("sensor_msgs.JointCommand (") for line in msgs.warnings(found))
    assert '"sensor_msgs.JointCommand":' in msgs.TS_FILE.read_text()


def test_the_cli_warns_and_passes() -> None:
    done = subprocess.run(
        [sys.executable, "-m", "experimental.gateway.utils.msgs"],
        cwd=REPO,
        capture_output=True,
        text=True,
    )
    assert done.returncode == 0, done.stdout + done.stderr
    assert "warning: no LCM schema: sensor_msgs.JointCommand" in done.stderr
    assert "msgs.ts and msgs.js are current" in done.stdout


def test_served_with_an_etag(tmp_path: Path) -> None:
    bus = Bus()
    app = create_app(
        ServerState(tmp_path, bus, Uploads(tmp_path, bus, None, tmp_path / "log")), background=False
    )
    with TestClient(app) as client:
        answer = client.get("/dimos/msgs.js")
        assert answer.status_code == 200
        assert answer.headers["content-type"].startswith("text/javascript")
        assert answer.headers["cache-control"] == "no-cache"
        assert answer.content == msgs.JS_FILE.read_bytes()
        etag = answer.headers["etag"]
        again = client.get("/dimos/msgs.js", headers={"if-none-match": etag})
        assert (again.status_code, again.content, again.headers["etag"]) == (304, b"", etag)
        assert client.get("/dimos/msgs.js", headers={"if-none-match": '"x"'}).status_code == 200
        ts = client.get("/dimos/msgs.ts")
        assert ts.headers["content-type"].startswith("application/typescript")
        assert ts.headers["etag"] != etag
        assert "/dimos/msgs.js" in client.get("/dimos/openapi.json").json()["paths"]


ROUND_TRIP = """
import * as msgs from "MODULE"
const { decode, decodeChannel, decodeMessage, register, getTypeNames, geometry_msgs, sensor_msgs } = msgs
const input = JSON.parse(Deno.args[0])
const bytes = (hex) => Uint8Array.from(hex.match(/../g), (b) => parseInt(b, 16))
const hex = (b) => Array.from(b, (x) => x.toString(16).padStart(2, "0")).join("")
const error = (f) => { try { f(); return null } catch (e) { return e.message } }
const pose = bytes(input.pose)
const out = {
    pose: decode(pose),
    poseByKey: decodeChannel("dimos/odom/geometry_msgs.PoseStamped", pose),
    // a zenoh-gateway client Message
    poseByMessage: decodeMessage({ key: "dimos/odom/geometry_msgs.PoseStamped", kind: "put", bytes: pose, timestamp: 0, seq: 1 }),
    deleted: decodeMessage({ key: "dimos/odom/geometry_msgs.PoseStamped", kind: "delete", bytes: new Uint8Array(), timestamp: 0, seq: 2 }) ?? null,
    twist: geometry_msgs.Twist.decode(bytes(input.twist)),
    joints: sensor_msgs.JointState.decode(bytes(input.joints)),
    twistEncoded: hex(geometry_msgs.Twist.encode({ linear: { x: 0.3 }, angular: { z: 0.5 } })),
    jointsEncoded: hex(sensor_msgs.JointState.encode({ header: { frame_id: "base" }, name: ["a", "b"], position: [1, 2] })),
    wrongType: error(() => geometry_msgs.Twist.decode(pose)),
    noSchema: error(() => decodeChannel("/joint_command#sensor_msgs.JointCommand", pose)),
    twistKey: geometry_msgs.Twist.zenohKey("dimos/cmd_vel"),
    twistChannel: geometry_msgs.Twist.lcmChannel("/cmd_vel"),
    types: getTypeNames().length,
}
register("sensor_msgs.JointCommand", null, (b) => ({ length: b.length }))
out.registered = decodeChannel("/joint_command#sensor_msgs.JointCommand", pose)
console.log(JSON.stringify(out, (_, v) => typeof v === "bigint" ? String(v) : ArrayBuffer.isView(v) ? Array.from(v) : v))
"""


@pytest.mark.parametrize("module", ["msgs.js", "msgs.ts"])
def test_decodes_what_python_encodes(module: str, tmp_path: Path) -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.msgs.sensor_msgs.JointState import JointState

    deno = msgs.deno()
    if deno is None:
        pytest.skip("deno isn't installed (the JS round trip runs under deno)")
    pose = PoseStamped(
        ts=12.25, frame_id="map", position=[1.0, 2.0, 3.0], orientation=[0, 0, 0.6, 0.8]
    )
    twist = Twist(linear=Vector3(0.3, 0, 0), angular=Vector3(0, 0, 0.5))
    joints = JointState(ts=1.5, frame_id="base", name=["a", "b"], position=[1.0, 2.0])
    script = tmp_path / "round_trip.mjs"
    script.write_text(ROUND_TRIP.replace("MODULE", (msgs.HERE / module).as_uri()))
    frames = {
        "pose": pose.lcm_encode().hex(),
        "twist": twist.lcm_encode().hex(),
        "joints": joints.lcm_encode().hex(),
    }
    done = subprocess.run(
        [deno, "run", "--no-config", "--allow-read", str(script), json.dumps(frames)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert done.returncode == 0, done.stderr
    out = json.loads(done.stdout)

    decoded = out["pose"]
    assert decoded == out["poseByKey"] == out["poseByMessage"]
    assert out["deleted"] is None
    assert decoded["header"]["frame_id"] == "map"
    assert decoded["header"]["stamp"] == {"sec": 12, "nsec": 250_000_000}
    assert decoded["pose"]["position"] == {"x": 1.0, "y": 2.0, "z": 3.0}
    assert decoded["pose"]["orientation"] == {"x": 0, "y": 0, "z": 0.6, "w": 0.8}
    assert out["twist"]["linear"]["x"] == 0.3 and out["twist"]["angular"]["z"] == 0.5
    assert out["joints"]["name"] == ["a", "b"] and out["joints"]["position"] == [1.0, 2.0]
    assert out["joints"]["velocity_length"] == 0

    # what JS encodes, Python decodes (and the fingerprints agree)
    assert out["twistEncoded"] == frames["twist"]
    back = JointState.lcm_decode(bytes.fromhex(out["jointsEncoded"]))
    assert (back.frame_id, back.name, list(back.position)) == ("base", ["a", "b"], [1.0, 2.0])

    assert "fingerprint mismatch" in out["wrongType"]
    assert "sensor_msgs.JointCommand has no LCM schema (hand-written" in out["noSchema"]
    assert out["registered"] == {"length": len(bytes.fromhex(frames["pose"]))}
    assert out["twistKey"] == "dimos/cmd_vel/geometry_msgs.Twist"
    assert out["twistChannel"] == "/cmd_vel#geometry_msgs.Twist"
    assert out["types"] == len(msgs.scan().schemas)


def test_msgs_js_is_msgs_ts_bundled() -> None:
    deno = msgs.deno()
    if deno is None or not msgs.deno_is_pinned(deno):
        pytest.skip(
            "needs the deno dimos pins (dimos/utils/deno.py): another version bundles differently"
        )
    assert msgs.bundle_js(msgs.TS_FILE.read_text(), deno) == msgs.JS_FILE.read_text()
