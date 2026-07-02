#!/usr/bin/env python3
# Integration tests for the pipe plane in pipe.py (the unix-socket feed for the Rust WT sidecar):
# hello-meta on connect, sub-union DATA filtering, new-topic push, teleop → SafetyEgress with a
# per-sid deadman, pipe-drop → all-stop, and the rpc round-trip. A fake sidecar talks over a real
# UDS; no dimos transports (fake publishers injected, as in test_egress.py). The /cert 503-until-
# hash-file behavior lives in app.py's route closure and is exercised in the end-to-end bench run.
#
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_pipe.py -q
import asyncio
import json
import pathlib
import struct
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent))  # dimoscope root → gateway pkg

from gateway.bus import Bus
from gateway.egress import SafetyEgress
from gateway.pipe import KIND_DATA, KIND_JSON, PipePlane


class FakeVec:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class FakeTwist:
    def __init__(self, linear=None, angular=None):
        self.linear, self.angular = linear, angular


class FakePub:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class FakeRpc:
    def call_sync(self, path, args):
        return (f"ok:{path}",)


def _egress() -> tuple[SafetyEgress, FakePub]:
    eg = SafetyEgress()
    pub = FakePub()
    eg._cmd = [pub]
    eg._Twist, eg._Vector3 = FakeTwist, FakeVec
    eg._rpc = FakeRpc()
    eg.has_rpc = True
    return eg, pub


class Harness:
    """Bus + egress + PipePlane server on a temp UDS + a connected fake-sidecar reader/writer."""

    async def __aenter__(self):
        self.bus = Bus()
        self.egress, self.pub = _egress()
        self.plane = PipePlane(self.bus, self.egress)
        self.path = tempfile.mktemp(suffix=".sock")
        self.server_task = asyncio.ensure_future(self.plane.start(self.path))
        for _ in range(100):  # wait for the listener
            try:
                self.reader, self.writer = await asyncio.open_unix_connection(self.path)
                break
            except (ConnectionRefusedError, FileNotFoundError):
                await asyncio.sleep(0.01)
        return self

    async def __aexit__(self, *exc):
        self.writer.close()
        self.server_task.cancel()

    async def read_frame(self, timeout=1.0):
        head = await asyncio.wait_for(self.reader.readexactly(5), timeout)
        length, kind = struct.unpack(">IB", head)
        body = await asyncio.wait_for(self.reader.readexactly(length - 1), timeout)
        return kind, body

    async def read_json(self, timeout=1.0):
        kind, body = await self.read_frame(timeout)
        assert kind == KIND_JSON
        return json.loads(body)

    def send_json(self, obj: dict) -> None:
        payload = json.dumps(obj).encode()
        self.writer.write(struct.pack(">IB", len(payload) + 1, KIND_JSON) + payload)

    def publish(self, topic: str, typ: str, payload: bytes) -> bytes:
        lc02 = self.bus._make_lc02(f"{topic}#{typ}", payload)
        self.bus._publish(topic, typ, lc02, payload)
        return lc02

    async def settle(self) -> None:
        """Let the server task process what we just wrote."""
        for _ in range(3):
            await asyncio.sleep(0)
        await asyncio.sleep(0.02)


def test_hello_meta_on_connect():
    async def run():
        async with Harness() as h:
            hello = await h.read_json()
            assert hello["op"] == "hello"
            assert hello["label"] == "dimoscope/WT-rs"
            assert hello["topics"] == []
            assert isinstance(hello["rpc"], list) and hello["rpc"]  # egress whitelist forwarded

    asyncio.run(run())


def test_sub_union_filters_data():
    async def run():
        async with Harness() as h:
            await h.read_json()  # hello
            h.publish("/pose", "geometry_msgs.Pose", b"pre-sub")  # no subs announced yet
            kind, body = await h.read_frame()  # only the new-topic push, no DATA
            assert (kind, json.loads(body)["op"]) == (KIND_JSON, "topic")

            h.send_json({"op": "subs", "topics": ["/pose"]})
            await h.settle()
            lc02 = h.publish("/pose", "geometry_msgs.Pose", b"wanted")
            h.publish("/other", "x.Y", b"unwanted")  # ∉ union → filtered (its topic push isn't)
            kind, body = await h.read_frame()
            assert (kind, body) == (KIND_DATA, lc02)  # raw LC02, no re-encode, no stamp
            kind, body = await h.read_frame()
            assert (kind, json.loads(body)["op"]) == (KIND_JSON, "topic")  # /other discovery only
            h.publish("/pose", "geometry_msgs.Pose", b"again")
            kind, _ = await h.read_frame()
            assert kind == KIND_DATA  # and nothing from /other queued in between

    asyncio.run(run())


def test_teleop_deadman_per_sid_and_stop():
    async def run():
        async with Harness() as h:
            await h.read_json()
            h.send_json({"op": "teleop", "sid": 1, "linearX": 0.5, "angularZ": 0.1, "ttlMs": 60})
            await h.settle()
            assert len(h.pub.published) == 1
            assert h.pub.published[0].linear.x == 0.5
            await asyncio.sleep(0.15)  # deadman (min 50ms) fires → zero twist
            assert len(h.pub.published) == 2
            assert h.pub.published[1].linear.x == 0.0

            h.send_json({"op": "teleop", "sid": 2, "linearX": 0.3, "angularZ": 0, "ttlMs": 400})
            h.send_json({"op": "stop", "sid": 2})  # explicit stop cancels sid-2's deadman
            await h.settle()
            assert h.pub.published[-1].linear.x == 0.0

    asyncio.run(run())


def test_pipe_drop_stops_all_sids():
    async def run():
        async with Harness() as h:
            await h.read_json()
            h.send_json({"op": "teleop", "sid": 7, "linearX": 0.9, "angularZ": 0, "ttlMs": 5000})
            await h.settle()
            assert len(h.pub.published) == 1
            h.writer.close()  # sidecar dies with a long-TTL teleop armed
            await h.settle()
            # drop_all_sids: zero twist published + deadman cancelled (no later duplicate)
            assert len(h.pub.published) == 2
            assert h.pub.published[1].linear.x == 0.0
            assert not h.egress._deadman

    asyncio.run(run())


def test_rpc_round_trip():
    async def run():
        async with Harness() as h:
            await h.read_json()
            h.send_json(
                {"op": "rpc", "sid": 3, "id": 42, "target": "GO2Connection", "method": "standup", "args": []}
            )
            res = await h.read_json()
            assert res == {"op": "rpc-res", "sid": 3, "id": 42, "res": "ok:GO2Connection/standup"}

            h.send_json({"op": "rpc", "sid": 3, "id": 43, "target": "Nope", "method": "x", "args": []})
            res = await h.read_json()
            assert res["error"].startswith("not allowed")  # server-authoritative whitelist

    asyncio.run(run())


def test_disconnect_op_stops_one_sid():
    async def run():
        async with Harness() as h:
            await h.read_json()
            h.send_json({"op": "teleop", "sid": 1, "linearX": 0.5, "angularZ": 0, "ttlMs": 5000})
            h.send_json({"op": "disconnect", "sid": 1})
            await h.settle()
            assert h.pub.published[-1].linear.x == 0.0  # zero twist for the departing sid
            assert not h.egress._deadman

    asyncio.run(run())
