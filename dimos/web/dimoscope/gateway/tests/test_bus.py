#!/usr/bin/env python3
# Unit tests for the dimoscope bus pure-logic — the trickiest ported code (LC03→LC02 reassembly),
# plus topic canonicalization + channel parse + LC02 framing. No network, no zenoh, no event loop.
#
# bus.py's logic is import-light; we import it directly off the gateway dir (not a published package).
# Run: uv run pytest dimos/web/dimoscope/gateway/tests/test_bus.py -q
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))

from bus import LC02, LC03, Bus, _canonical, _parse_channel


def test_canonical_strips_zenoh_prefix_and_normalizes_slash():
    assert _canonical("dimos/lidar") == "/lidar"
    assert _canonical("dimos/a/b") == "/a/b"
    assert _canonical("/odom") == "/odom"  # already canonical (LCM style)
    assert _canonical("odom") == "/odom"  # add the leading slash
    assert _canonical("dimosX/y") == "/dimosX/y"  # only the exact "dimos/" prefix is stripped


def test_parse_channel_topic_and_type():
    pkt = struct.pack(">II", LC02, 1) + b"/odom#nav_msgs.Odometry\x00" + b"payload"
    assert _parse_channel(pkt) == ("/odom", "nav_msgs.Odometry")


def test_parse_channel_bare_channel_has_unknown_type():
    pkt = struct.pack(">II", LC02, 1) + b"/raw\x00" + b"x"
    assert _parse_channel(pkt) == ("/raw", "?")


def test_make_lc02_roundtrips_through_parse_channel():
    bus = Bus()
    lc = bus._make_lc02("/p#geometry_msgs.PoseStamped", b"\x01\x02\x03")
    assert _parse_channel(lc) == ("/p", "geometry_msgs.PoseStamped")
    nul = lc.index(0, 8)
    assert lc[nul + 1 :] == b"\x01\x02\x03"  # payload recoverable after the channel NUL


def test_single_lc02_datagram_publishes_one_sample():
    bus = Bus()
    got = []
    bus.subscribe(got.append)
    pkt = bus._make_lc02("/x#std_msgs.String", b"hi")
    bus._on_lcm_datagram(pkt)
    assert len(got) == 1
    s = got[0]
    assert (s.topic, s.type, s.payload) == ("/x", "std_msgs.String", b"hi")


def test_lc03_two_fragment_reassembly():
    bus = Bus()
    got = []
    bus.subscribe(got.append)
    channel = b"/big#sensor_msgs.PointCloud2"
    payload = bytes(range(50))
    seqno, msg_size = 7, len(payload)
    # frag 0 carries the channel + the first 20 message bytes at offset 0
    f0 = struct.pack(">IIIIHH", LC03, seqno, msg_size, 0, 0, 2) + channel + b"\x00" + payload[:20]
    # frag 1 carries the remaining bytes at offset 20 (no channel)
    f1 = struct.pack(">IIIIHH", LC03, seqno, msg_size, 20, 1, 2) + payload[20:]

    bus._on_lcm_datagram(f0)
    assert got == []  # incomplete: only one of two fragments seen

    bus._on_lcm_datagram(f1)
    assert len(got) == 1
    s = got[0]
    assert (s.topic, s.type) == ("/big", "sensor_msgs.PointCloud2")
    assert s.payload == payload  # fully reassembled


def test_new_topic_callback_fires_once_per_topic():
    bus = Bus()
    seen = []
    bus.on_new_topic(lambda t, ty: seen.append((t, ty)))
    pkt = bus._make_lc02("/odom#nav_msgs.Odometry", b"a")
    bus._on_lcm_datagram(pkt)
    bus._on_lcm_datagram(pkt)  # same topic again → no second discovery event
    assert seen == [("/odom", "nav_msgs.Odometry")]
