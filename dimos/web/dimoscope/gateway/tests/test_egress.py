#!/usr/bin/env python3
# Unit tests for the SafetyEgress trust boundary in egress.py — velocity clamp, TTL deadman
# (fire / re-arm / cancel), stop-on-disconnect, and the @rpc whitelist. No dimos transports:
# fake publishers + message classes are injected in place of start().
#
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_egress.py -q
import asyncio
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

from egress import MAX_ANG, MAX_LIN, SafetyEgress


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


def _egress() -> tuple[SafetyEgress, FakePub]:
    eg = SafetyEgress()
    pub = FakePub()
    eg._cmd = [pub]
    eg._Twist, eg._Vector3 = FakeTwist, FakeVec
    return eg, pub


def test_velocity_clamp():
    async def run():
        eg, pub = _egress()
        eg.teleop("k", 5.0, -9.0, 400.0, asyncio.get_running_loop())
        msg = pub.published[0]
        assert msg.linear.x == MAX_LIN
        assert msg.angular.z == -MAX_ANG
        eg.stop("k")  # cancel the armed deadman before the loop closes

    asyncio.run(run())


def test_deadman_fires_zero_twist_after_ttl():
    async def run():
        eg, pub = _egress()
        eg.teleop("k", 0.5, 0.2, 60.0, asyncio.get_running_loop())
        assert len(pub.published) == 1
        await asyncio.sleep(0.15)  # ttl 60ms (floored to 50ms min) has expired
        assert len(pub.published) == 2
        assert pub.published[-1].linear.x == 0.0
        assert pub.published[-1].angular.z == 0.0

    asyncio.run(run())


def test_teleop_rearms_deadman_single_stop():
    async def run():
        eg, pub = _egress()
        loop = asyncio.get_running_loop()
        eg.teleop("k", 0.5, 0.0, 100.0, loop)
        await asyncio.sleep(0.05)
        eg.teleop("k", 0.5, 0.0, 100.0, loop)  # re-arm inside the first ttl
        await asyncio.sleep(0.25)
        # 2 teleop publishes + exactly ONE deadman stop (the first timer was cancelled)
        assert len(pub.published) == 3
        assert pub.published[-1].linear.x == 0.0

    asyncio.run(run())


def test_stop_cancels_deadman_and_zeroes():
    async def run():
        eg, pub = _egress()
        loop = asyncio.get_running_loop()
        eg.teleop("k", 0.5, 0.0, 60.0, loop)
        eg.stop("k")
        assert pub.published[-1].linear.x == 0.0
        n = len(pub.published)
        await asyncio.sleep(0.15)
        assert len(pub.published) == n  # no extra deadman fire — timer was cancelled

    asyncio.run(run())


def test_disconnect_stops_robot():
    async def run():
        eg, pub = _egress()
        loop = asyncio.get_running_loop()
        eg.teleop("k", 0.9, 0.0, 60.0, loop)
        eg.disconnect("k")
        assert pub.published[-1].linear.x == 0.0
        n = len(pub.published)
        await asyncio.sleep(0.15)
        assert len(pub.published) == n

    asyncio.run(run())


def test_rpc_whitelist_rejects_and_no_bridge_reports_unavailable():
    async def run():
        eg, _ = _egress()
        loop = asyncio.get_running_loop()
        # no rpc bridge at all → "rpc unavailable" for anything
        assert "error" in await eg.rpc("GO2Connection", "standup", [], loop)

        class FakeRpc:
            def call_sync(self, path, payload):
                return (f"ok:{path}", None)

        eg._rpc = FakeRpc()
        res = await eg.rpc("GO2Connection", "standup", [], loop)
        assert res == {"res": "ok:GO2Connection/standup"}
        # non-whitelisted target/method never reaches the bridge
        res = await eg.rpc("GO2Connection", "self_destruct", [], loop)
        assert res["error"].startswith("not allowed")

    asyncio.run(run())
