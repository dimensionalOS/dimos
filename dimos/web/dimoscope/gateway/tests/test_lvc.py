#!/usr/bin/env python3
# Unit tests for the last-value cache: the bus keeps each topic's most recent frame (bounded by
# LVC_MAX_BYTES), and a subscribe replays it into the client's priority outbox with a fresh
# gateway-send stamp — late joiners see slow/latched topics immediately.
#
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_lvc.py -q
import asyncio
import pathlib
import struct
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent))  # dimoscope root → gateway pkg

from gateway.bus import Bus, LVC_MAX_BYTES
from gateway.data import DataPlane, _Client
from gateway.egress import SafetyEgress


def _lc02(payload: bytes = b"x") -> bytes:
    return b"LC02....chan\x00" + payload


def test_bus_caches_latest_frame():
    bus = Bus()
    bus._publish("/a", "T", _lc02(b"first"), b"")
    bus._publish("/a", "T", _lc02(b"second"), b"")
    assert bus.last["/a"].lc02 == _lc02(b"second")
    assert list(bus.last) == ["/a"]


def test_bus_skips_oversize_frames():
    bus = Bus()
    bus._publish("/big", "T", b"\x00" * (LVC_MAX_BYTES + 1), b"")
    assert "/big" not in bus.last
    bus._publish("/big", "T", _lc02(), b"")  # a small frame afterwards caches normally
    assert "/big" in bus.last


def _subscribe(plane: DataPlane, st: _Client, topic: str) -> None:
    asyncio.run(plane._on_control(None, st, {"op": "subscribe", "topic": topic}, None))


def test_subscribe_replays_cached_frame_with_fresh_stamp():
    bus = Bus()
    plane = DataPlane(bus, SafetyEgress())
    bus._publish("/map", "nav_msgs.OccupancyGrid", _lc02(b"map"), b"")
    st = _Client()
    before = time.time() * 1000.0
    _subscribe(plane, st, "/map")
    frame = st.q._pick()
    assert frame is not None, "subscribe must replay the cached frame"
    (stamp,) = struct.unpack(">d", frame[:8])
    assert frame[8:] == _lc02(b"map")
    assert stamp >= before - 1  # freshly stamped at replay, not at capture
    assert st.q._pick() is None  # exactly one replay


def test_wildcard_subscribe_replays_all_cached_topics():
    bus = Bus()
    plane = DataPlane(bus, SafetyEgress())
    bus._publish("/a", "T", _lc02(b"a"), b"")
    bus._publish("/b", "T", _lc02(b"b"), b"")
    st = _Client()
    _subscribe(plane, st, "*")
    got = {st.q._pick()[8:], st.q._pick()[8:]}
    assert got == {_lc02(b"a"), _lc02(b"b")}
    assert st.q._pick() is None


def test_subscribe_without_cache_replays_nothing():
    plane = DataPlane(Bus(), SafetyEgress())
    st = _Client()
    _subscribe(plane, st, "/never-published")
    assert st.q._pick() is None
