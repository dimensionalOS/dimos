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

"""WebRTC video (protocol v7) RelayBridgeModule tests: track advertising, the
ICE/offer/answer handshake, and peer teardown/re-offer. Same no-network harness
as test_relay_bridge_module.py (see module_test_support).
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.web.relay_bridge import relay_bridge_module
from dimos.web.relay_bridge.e2e_support import stop_module
from dimos.web.relay_bridge.manifest import TRACK_ENCODING
from dimos.web.relay_bridge.module_test_support import (
    FAKE_INFO_RTC,
    FakeClient,
    fail_peer,
    flush_loop,
    image_transport,
    install_fake_publisher,
    kill_session,
    make_bridge,
    push,
    wait_until,
)
from dimos.web.relay_bridge.protocol import (
    IceServer,
    RtcAnswer,
    RtcIce,
    RtcOffer,
    RtcStalled,
    RtcTrack,
    Subs,
)
from dimos.web.relay_bridge.relay_bridge_module import RelayBridgeModule


def _rtc_bridge(monkeypatch, **kwargs):
    publishers = install_fake_publisher(monkeypatch)
    module, clients = make_bridge(monkeypatch, info=FAKE_INFO_RTC, **kwargs)
    assert wait_until(lambda: bool(clients) and clients[0].hello_args is not None)
    return module, clients, publishers


def _wire_channel(client: FakeClient, ch: str) -> dict[str, Any]:
    assert client.hello_args is not None
    return next(c for c in client.hello_args[1]["channels"] if c["ch"] == ch)


def _offers(client: FakeClient) -> list[RtcOffer]:
    return [m for m in client.control_frames if isinstance(m, RtcOffer)]


def test_rtc_relay_advertises_jpeg_channels_as_tracks(monkeypatch) -> None:
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        cam = _wire_channel(clients[0], "color_image")
        assert cam["encoding"] == TRACK_ENCODING
        assert cam["params"] == {}  # jpeg quality means nothing to a track
        assert _wire_channel(clients[0], "odom")["encoding"] == "pose.json.v1"
        # Only the wire copy changes: the runtime spec keeps its JPEG encoder.
        spec = next(s for s in module._channel_specs if s.ch == "color_image")
        assert spec.encoding == "jpeg.v1" and spec.encoder is not None
    finally:
        stop_module(module)


def test_rtc_ice_offers_the_answer_connects_and_frames_feed_the_track(monkeypatch) -> None:
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        ice = RtcIce(iceServers=[IceServer(urls=["stun:stun.cloudflare.com:3478"])])
        push(module, client, ice)
        assert wait_until(lambda: len(_offers(client)) == 1)
        assert _offers(client)[0].tracks == [RtcTrack(ch="color_image", mid="0")]
        assert publishers[0].ice == ice.iceServers
        push(module, client, RtcIce(iceServers=[]))  # a repeat starts no second peer
        push(module, client, RtcAnswer(sdp="v=0\r\nsfu-answer\r\n"))
        assert wait_until(lambda: publishers[0].accepted == "v=0\r\nsfu-answer\r\n")
        flush_loop(module)
        assert len(publishers) == 1
        # The subs snapshot still gates the input; a frame feeds the track and
        # never a JPEG writer.
        push(module, client, Subs(chs=["color_image"], n=1))
        assert wait_until(lambda: bool(image_transport(module).subscribers))
        image = Image(data=np.zeros((4, 4, 3), np.uint8), format=ImageFormat.RGB)
        image_transport(module).publish(image)
        assert wait_until(lambda: publishers[0].fed == [("color_image", image)])
        assert module.encoded["color_image"] == 1
        assert client.frames == []
    finally:
        stop_module(module)


def test_rtc_unanswered_offer_is_torn_down_and_resent(monkeypatch) -> None:
    monkeypatch.setattr(relay_bridge_module, "_RTC_ANSWER_TIMEOUT_S", 0.1)
    monkeypatch.setattr(relay_bridge_module, "_RTC_RETRY_BASE_S", 0.05)
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        push(module, client, RtcIce(iceServers=[]))
        assert wait_until(lambda: len(_offers(client)) >= 2)
        assert publishers[0].closed == 1  # the unanswered peer was torn down
        push(module, client, RtcAnswer(sdp="late"))
        assert wait_until(lambda: any(p.accepted == "late" for p in publishers))
    finally:
        stop_module(module)


def test_rtc_refreshed_ice_servers_are_used_by_the_next_peer(monkeypatch) -> None:
    monkeypatch.setattr(relay_bridge_module, "_RTC_RETRY_BASE_S", 0.05)
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        push(module, client, RtcIce(iceServers=[IceServer(urls=["stun:s"])]))
        assert wait_until(lambda: len(_offers(client)) == 1)
        # The relay re-minted its TURN credentials: the live peer stays, the
        # next attempt uses the new set.
        turn = [IceServer(urls=["turn:t"], username="u2", credential="c2")]
        push(module, client, RtcIce(iceServers=turn))
        flush_loop(module)
        assert len(publishers) == 1
        publishers[0].accept_error = RuntimeError("PeerConnection failed: failed")
        push(module, client, RtcAnswer(sdp="a"))
        assert wait_until(lambda: len(publishers) == 2 and len(_offers(client)) == 2)
        assert publishers[1].ice == turn
    finally:
        stop_module(module)


def test_rtc_feed_gap_past_the_track_lifetime_is_reported_to_the_relay(monkeypatch) -> None:
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        push(module, client, RtcIce(iceServers=[]))
        assert wait_until(lambda: len(_offers(client)) == 1)
        push(module, client, RtcAnswer(sdp="a"))
        assert wait_until(lambda: publishers[0].accepted == "a")
        push(module, client, Subs(chs=["color_image"], n=1))
        assert wait_until(lambda: bool(image_transport(module).subscribers))
        image = Image(data=np.zeros((4, 4, 3), np.uint8), format=ImageFormat.RGB)
        image_transport(module).publish(image)
        assert wait_until(lambda: len(publishers[0].fed) == 1)
        flush_loop(module)
        assert not any(isinstance(m, RtcStalled) for m in client.control_frames)
        # The next frame ends a gap the SFU does not survive.
        publishers[0].stalled.add("color_image")
        time.sleep(0.2)  # past the channel's rate gate
        image_transport(module).publish(image)
        assert wait_until(lambda: RtcStalled(ch="color_image") in client.control_frames)
        assert module.encoded["color_image"] == 2
    finally:
        stop_module(module)


def test_rtc_off_by_config_or_without_aiortc_keeps_jpeg(monkeypatch) -> None:
    module, clients, _ = _rtc_bridge(monkeypatch, rtc=False)
    try:
        assert _wire_channel(clients[0], "color_image")["encoding"] == "jpeg.v1"
    finally:
        stop_module(module)
    monkeypatch.setattr(relay_bridge_module, "RTC_AVAILABLE", False)
    module, clients, _ = _rtc_bridge(monkeypatch)
    try:
        assert _wire_channel(clients[0], "color_image")["encoding"] == "jpeg.v1"
        assert "color_image" in clients[0].writers  # the JPEG path is wired
    finally:
        stop_module(module)


def test_rtc_peer_dies_with_the_session_and_the_next_one_offers_again(monkeypatch) -> None:
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        push(module, clients[0], RtcIce(iceServers=[]))
        assert wait_until(lambda: len(_offers(clients[0])) == 1)
        push(module, clients[0], RtcAnswer(sdp="a"))
        assert wait_until(lambda: publishers[0].accepted == "a")
        kill_session(module, clients[0])
        assert wait_until(lambda: len(clients) == 2 and clients[1].hello_args is not None)
        assert wait_until(lambda: publishers[0].closed == 1)
        push(module, clients[1], RtcIce(iceServers=[]))
        assert wait_until(lambda: len(publishers) == 2 and len(_offers(clients[1])) == 1)
    finally:
        stop_module(module)


def test_rtc_connect_failure_after_the_answer_is_torn_down_and_reoffered(monkeypatch) -> None:
    monkeypatch.setattr(relay_bridge_module, "_RTC_RETRY_BASE_S", 0.05)
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        push(module, client, RtcIce(iceServers=[]))
        assert wait_until(lambda: len(_offers(client)) == 1)
        # The answer applies but ICE/DTLS never come up.
        publishers[0].accept_error = RuntimeError("PeerConnection failed: failed")
        push(module, client, RtcAnswer(sdp="a"))
        assert wait_until(lambda: publishers[0].closed == 1 and len(_offers(client)) == 2)
        push(module, client, RtcAnswer(sdp="b"))
        assert wait_until(lambda: len(publishers) == 2 and publishers[1].accepted == "b")
    finally:
        stop_module(module)


def test_rtc_peer_loss_after_connect_is_torn_down_and_reoffered(monkeypatch) -> None:
    monkeypatch.setattr(relay_bridge_module, "_RTC_RETRY_BASE_S", 0.05)
    module, clients, publishers = _rtc_bridge(monkeypatch)
    try:
        client = clients[0]
        push(module, client, RtcIce(iceServers=[]))
        assert wait_until(lambda: len(_offers(client)) == 1)
        push(module, client, RtcAnswer(sdp="a"))
        assert wait_until(lambda: publishers[0].accepted == "a")
        push(module, client, Subs(chs=["color_image"], n=1))
        assert wait_until(lambda: bool(image_transport(module).subscribers))
        flush_loop(module)
        fail_peer(module, publishers[0])
        assert wait_until(lambda: publishers[0].closed == 1 and len(_offers(client)) == 2)
        # Frames fed while the peer is down are dropped, not counted.
        image = Image(data=np.zeros((4, 4, 3), np.uint8), format=ImageFormat.RGB)
        image_transport(module).publish(image)
        flush_loop(module)
        assert publishers[1].fed == [] and module.encoded["color_image"] == 0
    finally:
        stop_module(module)


def test_rtc_file_rejected_with_relay_url_and_when_missing(tmp_path: Path) -> None:
    module = RelayBridgeModule(
        relay_url="http://127.0.0.1:1", rtc_file=str(tmp_path / "rtc.json"), open_browser=False
    )
    with pytest.raises(RuntimeError, match="rtc_file requires"):
        module.start()
    stop_module(module)
    module = RelayBridgeModule(
        local_port=0, open_browser=False, rtc_file=str(tmp_path / "nope.json"), robot_id="unit-bot"
    )
    with pytest.raises(RuntimeError, match="rtc_file does not exist"):
        module.start()
    stop_module(module)
