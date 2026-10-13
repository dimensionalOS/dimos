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

"""RtcPublisher against a second in-process aiortc peer standing in for the
SFU: offer shape, then frames fed from a thread arriving decoded on the peer
once accept() returned connected, and the peer's close seen as the loss.
Host candidates only (no ICE servers), so gathering completes offline."""

from __future__ import annotations

import asyncio
import threading
import time

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.web.relay_bridge.protocol import RtcTrack
from dimos.web.relay_bridge.relay_bridge_module import RTC_AVAILABLE

pytestmark = pytest.mark.skipif(not RTC_AVAILABLE, reason="aiortc (the webrtc extra) not installed")


async def test_offer_has_one_sendonly_h264_section_per_channel() -> None:
    from dimos.web.relay_bridge.rtc_publisher import RtcPublisher

    publisher = RtcPublisher(["color_image", "rear"])
    try:
        sdp, tracks = await publisher.start([])
        assert sdp.count("m=video") == 2
        assert sdp.count("a=sendonly") == 2
        assert tracks == [RtcTrack(ch="color_image", mid="0"), RtcTrack(ch="rear", mid="1")]
        for track in tracks:
            assert f"a=mid:{track.mid}" in sdp
        # H.264 is preferred: the first payload type of each video section.
        for section in sdp.split("m=video")[1:]:
            first_pt = section.split()[2]
            assert f"a=rtpmap:{first_pt} H264/90000" in section
    finally:
        await publisher.close()


async def test_feed_reports_a_media_gap_past_the_track_lifetime() -> None:
    from dimos.web.relay_bridge.rtc_publisher import TRACK_GC_S, RtcPublisher

    publisher = RtcPublisher(["color_image"])
    image = Image(data=np.zeros((48, 64, 3), np.uint8), format=ImageFormat.RGB)
    try:
        await publisher.start([])
        assert publisher.feed("color_image", image) is False  # nothing before it
        assert publisher.feed("color_image", image) is False
        # The SFU collected the track during a gap this long.
        publisher._last_fed["color_image"] -= TRACK_GC_S
        assert publisher.feed("color_image", image) is True
        assert publisher.feed("color_image", image) is False
    finally:
        await publisher.close()


async def test_frames_fed_from_a_thread_reach_the_peer() -> None:
    from aiortc import RTCPeerConnection, RTCSessionDescription

    from dimos.web.relay_bridge.rtc_publisher import RtcPublisher

    publisher = RtcPublisher(["color_image"])
    peer = RTCPeerConnection()
    received: asyncio.Queue[object] = asyncio.Queue()
    peer.on("track", received.put_nowait)
    stop = threading.Event()
    image = Image(data=np.full((48, 64, 3), 200, np.uint8), format=ImageFormat.RGB)

    def producer() -> None:
        # The transport thread's cadence: a frame every 50 ms.
        while not stop.is_set():
            publisher.feed("color_image", image)
            time.sleep(0.05)

    thread = threading.Thread(target=producer, daemon=True)
    try:
        sdp, _tracks = await publisher.start([])
        await peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        await peer.setLocalDescription(await peer.createAnswer())
        await publisher.accept(peer.localDescription.sdp)  # returns once connected
        track = await asyncio.wait_for(received.get(), 10)
        thread.start()
        frame = await asyncio.wait_for(track.recv(), 20)  # type: ignore[attr-defined]
        assert (frame.width, frame.height) == (64, 48)
        # The peer's DTLS shutdown drives the publisher's connection to its
        # terminal state: what the bridge waits on to offer again.
        await peer.close()
        assert await asyncio.wait_for(publisher.wait_lost(), 10) == "closed"
    finally:
        stop.set()
        if thread.is_alive():
            thread.join(timeout=2)
        await peer.close()
        await publisher.close()
