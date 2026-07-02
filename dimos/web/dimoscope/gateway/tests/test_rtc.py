#!/usr/bin/env python3
# Unit tests for the WebRTC data plane's transport-agnostic core (gateway/transports/webrtc.py):
# RtcSession — on-demand filtering, 1100 B size routing into the pose/bulk outboxes, per-topic
# downsample, client QoS overrides, the read-only control ops — and the bulk chunk framing the
# client reassembles. No aiortc objects involved.
#
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_rtc.py -q
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent.parent))  # dimoscope root → gateway pkg

from gateway.qos import LANE_DEFAULT
from gateway.transports.webrtc import CHUNK, DATAGRAM_MAX, RtcSession, bulk_chunks


def _drain(q, n=10):
    out = []
    for _ in range(n):
        item = q._pick()
        if item is None:
            break
        out.append(item)
    return out


def test_unsubscribed_topics_never_transit():
    s = RtcSession()
    s.offer_sample("/pose", LANE_DEFAULT, b"x" * 50, 0.0)
    assert _drain(s.pose_q) == [] and _drain(s.bulk_q) == []


def test_size_routing_at_datagram_max():
    s = RtcSession()
    s.on_control({"op": "subscribe", "topic": "/a"}, {})
    small = b"s" * DATAGRAM_MAX
    big = b"b" * (DATAGRAM_MAX + 1)
    s.offer_sample("/a", LANE_DEFAULT, small, 0.0)
    s.offer_sample("/a", LANE_DEFAULT, big, 1.0)
    assert _drain(s.pose_q) == [small]
    assert _drain(s.bulk_q) == [big]


def test_maxhz_downsamples():
    s = RtcSession()
    s.on_control({"op": "subscribe", "topic": "/a", "maxHz": 10}, {})
    s.offer_sample("/a", LANE_DEFAULT, b"1", 1000.0)  # delivered
    s.offer_sample("/a", LANE_DEFAULT, b"2", 1010.0)  # < 100 ms window → dropped
    s.offer_sample("/a", LANE_DEFAULT, b"3", 1110.0)  # next window → delivered
    assert _drain(s.pose_q) == [b"1", b"3"]


def test_client_qos_override_conflates():
    s = RtcSession()
    # best-effort → conflate: only the latest frame survives in the outbox
    s.on_control({"op": "subscribe", "topic": "/a", "reliability": "best-effort"}, {})
    for payload in (b"a", b"b", b"c"):
        s.offer_sample("/a", LANE_DEFAULT, payload, 0.0)
    assert _drain(s.pose_q) == [b"c"]


def test_unsubscribe_clears_state():
    s = RtcSession()
    s.on_control({"op": "subscribe", "topic": "/a", "maxHz": 5, "priority": "high"}, {})
    s.on_control({"op": "unsubscribe", "topic": "/a"}, {})
    assert not s.subs and not s.rate and not s.qos
    s.offer_sample("/a", LANE_DEFAULT, b"x", 0.0)
    assert _drain(s.pose_q) == []


def test_list_and_ping_replies():
    s = RtcSession()
    topics = {"/a": "geometry_msgs.Pose"}
    r = s.on_control({"op": "list"}, topics)
    assert r == {"op": "topics", "topics": [{"topic": "/a", "type": "geometry_msgs.Pose"}]}
    r = s.on_control({"op": "ping", "id": 7}, {})
    assert r["op"] == "pong" and r["id"] == 7 and r["serverTs"] > 0


def test_write_path_ops_are_ignored():
    s = RtcSession()
    for op in ("teleop", "goal", "rpc", "bogus"):
        assert s.on_control({"op": op}, {}) is None  # read-only plane


def test_bulk_chunks_round_trip():
    framed = bytes(range(256)) * 700  # ~179 KB → 4 chunks
    chunks = bulk_chunks(framed)
    assert all(len(c) <= CHUNK for c in chunks)
    stream = b"".join(chunks)  # ordered+reliable channel ⇒ client sees the concatenation
    length = struct.unpack(">I", stream[:4])[0]
    assert length == len(framed)
    assert stream[4 : 4 + length] == framed
    assert len(stream) == 4 + length  # no trailing bytes
