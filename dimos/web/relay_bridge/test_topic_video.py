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

"""Direct video peers and the shared encoder, with a fake peer connection."""

import asyncio
from collections.abc import AsyncIterator, Callable, Iterator
from dataclasses import dataclass, field
import json
from pathlib import Path
import threading
import time
from typing import Any

from aiortc import RTCConfiguration, RTCPeerConnection, RTCRtpReceiver
from aiortc.mediastreams import MediaStreamError
import av
from dimos_lcm.sensor_msgs import Image
import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.utils.asyncio_timeout import timeout
from dimos.web.relay_bridge import topic_video
from dimos.web.relay_bridge.topic_profiles import normalize_settings
from dimos.web.relay_bridge.topic_protocol import (
    TopicRtcAnswer,
    TopicRtcCandidate,
    TopicRtcClose,
    TopicRtcOffer,
)
from dimos.web.relay_bridge.topic_transform import TopicTransforms
from dimos.web.relay_bridge.topic_video import (
    FATAL_FAILURES,
    PEER_QUEUE,
    REBUILD_FAILURES,
    PacketTrack,
    SharedEncoder,
    TopicVideo,
    fit_box,
    resolve_encoder,
)

PROFILE = "video.h264.v1"
VECTOR = next(
    v
    for v in json.loads(
        (Path(__file__).parents[3] / "web/shared/fixtures/topic_media.json").read_text()
    )
    if v["name"] == "color_rgb_boxed"
)
OFFER_SDP = (
    "v=0\r\no=- 1 1 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\nm=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
    "c=IN IP4 0.0.0.0\r\na=mid:0\r\na=recvonly\r\na=rtpmap:96 H264/90000\r\n"
)


def frame(seq: int, width: int = 64, height: int = 48) -> bytes:
    message = Image.lcm_decode(bytes.fromhex(VECTOR["lcm"]))
    message.header.seq = seq
    picture = np.full((height, width, 3), seq % 256, dtype=np.uint8)
    picture[:, : width // 2, 1] = (seq * 37) % 256
    message.height, message.width, message.encoding = height, width, "rgb8"
    message.is_bigendian, message.step = False, width * 3
    message.data = picture.tobytes()
    message.data_length = len(message.data)
    return message.lcm_encode()


class FakeSender:
    def _send_keyframe(self) -> None:
        raise AssertionError("aiortc's own keyframe path must be replaced")


@dataclass
class FakeTransceiver:
    track: Any
    kind: str = "video"
    sender: FakeSender = field(default_factory=FakeSender)
    preferences: list[Any] = field(default_factory=list)

    def setCodecPreferences(self, codecs: list[Any]) -> None:
        self.preferences = codecs


class FakePeerConnection:
    """The RTCPeerConnection surface TopicVideo drives, with a scripted state."""

    instances: list["FakePeerConnection"] = []

    def __init__(self, *, fail_offer: bool = False, close_delay: float = 0.0) -> None:
        self.transceivers: list[FakeTransceiver] = []
        self.handlers: dict[str, Callable[[], None]] = {}
        self.remote: str | None = None
        self.candidates: list[Any] = []
        self.connectionState = "new"
        self.closed = False
        self.fail_offer = fail_offer
        self.close_delay = close_delay
        FakePeerConnection.instances.append(self)

    def addTransceiver(self, track: Any, direction: str) -> FakeTransceiver:
        assert direction == "sendonly"
        transceiver = FakeTransceiver(track)
        self.transceivers.append(transceiver)
        return transceiver

    def getTransceivers(self) -> list[FakeTransceiver]:
        return self.transceivers

    def on(self, event: str) -> Callable[[Callable[[], None]], Callable[[], None]]:
        def register(handler: Callable[[], None]) -> Callable[[], None]:
            self.handlers[event] = handler
            return handler

        return register

    async def setRemoteDescription(self, description: Any) -> None:
        if self.fail_offer:
            raise ValueError("bad sdp")
        self.remote = description.sdp

    async def createAnswer(self) -> Any:
        return object()

    async def setLocalDescription(self, _answer: Any) -> None:
        pass

    @property
    def localDescription(self) -> Any:
        return type("Description", (), {"sdp": "v=0 answer"})()

    async def addIceCandidate(self, candidate: Any) -> None:
        self.candidates.append(candidate)

    def connect(self) -> None:
        self.connectionState = "connected"
        self.handlers["connectionstatechange"]()

    def fail(self) -> None:
        self.connectionState = "failed"
        self.handlers["connectionstatechange"]()

    async def close(self) -> None:
        self.closed = True
        await asyncio.sleep(self.close_delay)


@dataclass
class Harness:
    video: TopicVideo
    sent: list[Any]
    transforms: TopicTransforms

    async def offer(self, token: str, variant: str = "v1", **settings: Any) -> FakePeerConnection:
        route = self.video._routes.get("@t:p1") or self.video.route("@t:p1")
        await self.video.offer(
            route,
            normalize_settings(PROFILE, settings),
            TopicRtcOffer(id=token, route="@t:p1", gen=1, sdp=OFFER_SDP, variant=variant),
        )
        return FakePeerConnection.instances[-1]

    def closes(self) -> list[TopicRtcClose]:
        return [m for m in self.sent if isinstance(m, TopicRtcClose)]

    def answers(self) -> list[TopicRtcAnswer]:
        return [m for m in self.sent if isinstance(m, TopicRtcAnswer)]

    async def encode(self, seq: int, variant: str = "v1") -> None:
        route = self.video._routes["@t:p1"]
        async with self.transforms.slots:
            await route.encode(variant, frame(seq))


@pytest.fixture
def peers() -> Iterator[list[FakePeerConnection]]:
    FakePeerConnection.instances = []
    yield FakePeerConnection.instances
    FakePeerConnection.instances = []


@pytest.fixture
async def harness(peers: list[FakePeerConnection]) -> AsyncIterator[Harness]:
    sent: list[Any] = []
    transforms = TopicTransforms()
    video = TopicVideo(
        resolve_encoder("software"),
        max_peers=3,
        max_encoders=2,
        send=sent.append,
        transforms=transforms,
        peer_factory=FakePeerConnection,
    )
    try:
        yield Harness(video, sent, transforms)
    finally:
        await video.close()


def test_resolve_encoder_picks_a_software_codec_and_rejects_the_rest() -> None:
    assert resolve_encoder("auto") == resolve_encoder("software") in ("libopenh264", "libx264")
    assert resolve_encoder("libx264") == "libx264"
    with pytest.raises(ValueError, match="unknown video encoder"):
        resolve_encoder("nope")
    with pytest.raises(ValueError, match="does not encode H.264"):
        resolve_encoder("mjpeg")


def test_fit_box_keeps_even_sides_and_never_upscales() -> None:
    assert fit_box(1920, 1080, [1280, 720]) == (1280, 720)
    assert fit_box(640, 480, [1920, 1080]) == (640, 480)
    assert fit_box(1921, 1081, [1920, 1080]) == (1918, 1080)
    assert fit_box(3, 3, [16, 16]) == (2, 2)


async def test_one_encode_feeds_every_peer_and_a_joiner_starts_from_an_idr(
    harness: Harness, mocker: MockerFixture
) -> None:
    encode = mocker.spy(SharedEncoder, "encode")
    first = await harness.offer("r1")
    second = await harness.offer("r2")
    assert harness.video.encoders() == 1
    assert [m.id for m in harness.answers()] == ["r1", "r2"]
    assert {c.mimeType.lower() for c in first.transceivers[0].preferences} == {
        "video/h264",
        "video/rtx",
    }
    first.connect()
    await harness.encode(1)
    await harness.encode(2)
    assert encode.call_count == 2
    first_track = first.transceivers[0].track
    second_track = second.transceivers[0].track
    assert first_track.waiting_idr is False
    assert len(first_track._queue) == 2
    assert len(second_track._queue) == 0  # not connected: fed nothing
    # The joiner waits for the next IDR, forced on the next frame.
    mocker.patch.object(topic_video, "KEYFRAME_FORCE_S", 0.0)
    second.connect()
    await first_track.recv()
    await first_track.recv()
    await harness.encode(3)
    assert encode.call_count == 3
    joined = await second_track.recv()
    assert joined.is_keyframe
    assert (await first_track.recv()).is_keyframe  # one encode, both got the IDR


async def test_pli_from_one_viewer_forces_the_shared_encoder_within_the_force_window(
    harness: Harness, mocker: MockerFixture
) -> None:
    pc = await harness.offer("r1")
    pc.connect()
    track = pc.transceivers[0].track
    encoder = harness.video.peers["r1"].encoder
    await harness.encode(1)
    assert (await track.recv()).is_keyframe
    await harness.encode(2)
    assert not (await track.recv()).is_keyframe
    # aiortc's RTCP handler calls the sender's hook; it is now the encoder's.
    pc.transceivers[0].sender._send_keyframe()
    mocker.patch.object(topic_video, "KEYFRAME_FORCE_S", 0.0)
    await harness.encode(3)
    assert (await track.recv()).is_keyframe
    assert encoder.keyframes == 2


async def test_overflowing_peer_drops_its_queue_and_waits_for_the_next_idr(
    harness: Harness, mocker: MockerFixture
) -> None:
    mocker.patch.object(topic_video, "KEYFRAME_FORCE_S", 0.0)
    pc = await harness.offer("r1")
    pc.connect()
    track = pc.transceivers[0].track
    encoder = harness.video.peers["r1"].encoder
    for seq in range(1, PEER_QUEUE + 2):
        await harness.encode(seq)
    assert track.dropped == 1
    assert track.waiting_idr is True
    assert len(track._queue) == 0
    await harness.encode(PEER_QUEUE + 2)  # a P-frame would be skipped; the forced IDR is not
    assert len(track._queue) == 1
    assert (await track.recv()).is_keyframe
    assert encoder.frames == PEER_QUEUE + 2


async def test_slow_encode_is_dropped_and_followed_by_a_keyframe_despite_the_force_window(
    harness: Harness, mocker: MockerFixture
) -> None:
    # The production force window stays on: the first frame opened the codec
    # as an IDR a moment ago, so a window-bound request would leave the next
    # frames referencing the dropped one.
    pc = await harness.offer("r1")
    pc.connect()
    track = pc.transceivers[0].track
    encoder = harness.video.peers["r1"].encoder
    await harness.encode(1)
    assert (await track.recv()).is_keyframe
    original = SharedEncoder.encode

    def slow(self: SharedEncoder, payload: bytes) -> list[av.Packet]:
        time.sleep(0.05)
        return original(self, payload)

    mocker.patch.object(SharedEncoder, "encode", slow)
    mocker.patch.object(SharedEncoder, "deadline", property(lambda self: 0.01))
    await harness.encode(2)
    assert encoder.dropped == 1
    assert len(track._queue) == 0
    assert track.waiting_idr
    mocker.patch.object(SharedEncoder, "encode", original)
    await harness.encode(3)
    assert len(track._queue) == 1
    assert (await track.recv()).is_keyframe
    assert encoder.keyframes == 2


async def test_repeated_failures_rebuild_the_encoder_then_end_its_peers(
    harness: Harness, mocker: MockerFixture
) -> None:
    pc = await harness.offer("r1")
    pc.connect()
    encoder = harness.video.peers["r1"].encoder
    await harness.encode(1)
    assert encoder._codec is not None
    mocker.patch.object(SharedEncoder, "encode", side_effect=RuntimeError("boom"))
    for _ in range(REBUILD_FAILURES):
        await harness.encode(2)
    assert encoder._codec is None  # rebuilt on the next frame
    assert harness.closes() == []
    for _ in range(FATAL_FAILURES - REBUILD_FAILURES):
        await harness.encode(3)
    assert harness.closes() == [
        TopicRtcClose(id="r1", route="@t:p1", gen=1, reason="encoder_failed")
    ]
    assert pc.closed
    assert harness.video.encoders() == 0


async def test_encoder_is_released_with_its_last_peer_and_limits_hold(harness: Harness) -> None:
    await harness.offer("r1")
    await harness.offer("r2", "v2", maxResolution=[320, 180])
    assert harness.video.encoders() == 2
    # A third settings set exceeds max_encoders; a fourth peer exceeds max_peers.
    await harness.offer("r3", "v3", maxResolution=[160, 90])
    assert harness.closes()[-1] == TopicRtcClose(id="r3", route="@t:p1", gen=1, reason="rtc_limit")
    await harness.offer("r3")
    await harness.offer("r4")
    assert harness.closes()[-1] == TopicRtcClose(id="r4", route="@t:p1", gen=1, reason="rtc_limit")
    assert len(harness.video.peers) == 3
    await harness.video.close_peer("r1", "route_closed", notify=False)
    assert harness.video.encoders() == 2  # r3 still uses v1
    await harness.video.close_peer("r3", "route_closed", notify=False)
    assert harness.video.encoders() == 1
    await harness.video.close_peer("r2", "route_closed", notify=False)
    assert harness.video.encoders() == 0
    assert len(harness.closes()) == 2  # only the two rejections were reported


async def test_ice_failure_closes_the_peer_within_the_bound_and_tells_the_relay(
    harness: Harness, mocker: MockerFixture
) -> None:
    mocker.patch.object(topic_video, "PEER_CLOSE_S", 0.05)
    pc = FakePeerConnection(close_delay=10)
    harness.video._peer_factory = lambda: pc
    await harness.offer("r1")
    pc.connect()
    started = time.monotonic()
    pc.fail()
    async with timeout(2):
        while "r1" in harness.video.peers or harness.video._tasks:
            await asyncio.sleep(0.01)
    assert time.monotonic() - started < 1
    assert harness.closes() == [TopicRtcClose(id="r1", route="@t:p1", gen=1, reason="ice_failed")]
    assert pc.closed
    with pytest.raises(MediaStreamError):
        await pc.transceivers[0].track.recv()


async def test_bad_offers_and_candidates_end_the_peer_without_an_answer(harness: Harness) -> None:
    harness.video._peer_factory = lambda: FakePeerConnection(fail_offer=True)
    await harness.offer("r1")
    assert harness.answers() == []
    assert harness.closes() == [TopicRtcClose(id="r1", route="@t:p1", gen=1, reason="rtc_failed")]
    harness.video._peer_factory = FakePeerConnection
    pc = await harness.offer("r2")
    candidate = "candidate:1 1 udp 2130706431 192.168.1.20 51234 typ host generation 0"
    await harness.video.candidate(
        TopicRtcCandidate(id="r2", route="@t:p1", gen=1, candidate=candidate, mid="0")
    )
    await harness.video.candidate(
        TopicRtcCandidate(id="r2", route="@t:p1", gen=1, candidate="", mid="0")
    )
    assert [c is None or (c.ip, c.port, c.sdpMid) for c in pc.candidates] == [
        ("192.168.1.20", 51234, "0"),
        True,
    ]
    await harness.video.candidate(
        TopicRtcCandidate(id="r2", route="@t:p1", gen=1, candidate="candidate:junk", mid="0")
    )
    assert harness.closes()[-1].reason == "rtc_failed"
    assert "r2" not in harness.video.peers
    # A relay-initiated close is not echoed back.
    await harness.offer("r3")
    await harness.video.close_peer("r3", "route_closed", notify=False)
    assert len(harness.closes()) == 2


async def test_a_candidate_the_ice_agent_cannot_take_ends_only_its_peer(
    harness: Harness, mocker: MockerFixture
) -> None:
    await harness.offer("r1")
    pc = await harness.offer("r2")
    # What aioice raises for a browser's mDNS host candidate on a host without
    # a multicast route.
    mocker.patch.object(pc, "addIceCandidate", side_effect=OSError(19, "No such device"))
    candidate = "candidate:1 1 udp 2113937151 3f1c8b7e-9a51.local 51234 typ host"
    await harness.video.candidate(
        TopicRtcCandidate(id="r2", route="@t:p1", gen=1, candidate=candidate, mid="0")
    )
    assert harness.closes() == [TopicRtcClose(id="r2", route="@t:p1", gen=1, reason="rtc_failed")]
    assert list(harness.video.peers) == ["r1"]


async def test_packet_track_ends_recv_when_stopped() -> None:
    track = PacketTrack()
    waiting = asyncio.ensure_future(track.recv())
    await asyncio.sleep(0)
    track.stop()
    with pytest.raises(MediaStreamError):
        await waiting


async def viewer_offer(*mime_types: str) -> str:
    """A real aiortc viewer's offer, restricted to the given codecs."""
    viewer = RTCPeerConnection(RTCConfiguration(iceServers=[]))
    transceiver = viewer.addTransceiver("video", direction="recvonly")
    codecs = RTCRtpReceiver.getCapabilities("video").codecs
    transceiver.setCodecPreferences(
        [codec for codec in codecs if codec.mimeType.lower() in mime_types]
    )
    await viewer.setLocalDescription(await viewer.createOffer())
    sdp: str = viewer.localDescription.sdp
    await viewer.close()
    return sdp


@pytest.fixture
def real_peers(harness: Harness) -> Harness:
    harness.video._peer_factory = lambda: RTCPeerConnection(RTCConfiguration(iceServers=[]))
    return harness


async def test_an_offer_without_h264_is_refused_as_unavailable(real_peers: Harness) -> None:
    await real_peers.video.offer(
        real_peers.video.route("@t:p1"),
        normalize_settings(PROFILE, {}),
        TopicRtcOffer(
            id="r1",
            route="@t:p1",
            gen=1,
            sdp=await viewer_offer("video/vp8", "video/rtx"),
            variant="v1",
        ),
    )
    assert real_peers.answers() == []
    assert real_peers.closes() == [
        TopicRtcClose(id="r1", route="@t:p1", gen=1, reason="rtc_unavailable")
    ]
    assert not real_peers.video.peers
    assert real_peers.video.encoders() == 0


async def test_an_answer_names_only_h264_and_its_retransmission(real_peers: Harness) -> None:
    await real_peers.video.offer(
        real_peers.video.route("@t:p1"),
        normalize_settings(PROFILE, {}),
        TopicRtcOffer(
            id="r1",
            route="@t:p1",
            gen=1,
            sdp=await viewer_offer("video/vp8", "video/h264", "video/rtx"),
            variant="v1",
        ),
    )
    [answer] = real_peers.answers()
    assert "H264/90000" in answer.sdp
    assert "rtx/90000" in answer.sdp
    assert "VP8" not in answer.sdp


@pytest.mark.parametrize("encoding", ["mono8", "8UC1"])
def test_grayscale_images_encode(encoding: str) -> None:
    message = Image.lcm_decode(bytes.fromhex(VECTOR["lcm"]))
    message.height, message.width, message.encoding = 32, 32, encoding
    message.is_bigendian, message.step = False, 32
    message.data = bytes(range(256)) * 4
    message.data_length = len(message.data)
    encoder = SharedEncoder(resolve_encoder("software"), normalize_settings(PROFILE, {}))
    packets = encoder.encode(message.lcm_encode())
    assert packets
    assert packets[0].is_keyframe


async def test_closing_routes_returns_the_registry_to_empty(harness: Harness) -> None:
    for cycle in range(3):
        name = f"@t:p{cycle}"
        route = harness.video.route(name)
        await harness.video.offer(
            route,
            normalize_settings(PROFILE, {}),
            TopicRtcOffer(id=f"r{cycle}", route=name, gen=1, sdp=OFFER_SDP, variant="v1"),
        )
        assert harness.video.encoders() == 1
        await route.close()
        assert harness.video.encoders() == 0
        assert not harness.video.peers
        assert not harness.video._routes
    assert {close.reason for close in harness.closes()} == {"route_closed"}


async def test_the_target_bitrate_follows_the_encoded_size_under_the_request(harness):
    pc = await harness.offer("r1")
    pc.connect()
    route = harness.video._routes["@t:p1"]
    encoder = route.encoders["v1"]
    assert encoder.max_bitrate == 4_500_000  # the default box is 1080p
    for size, bitrate in (((640, 360), 1_000_000), ((1280, 720), 2_500_000)):
        async with harness.transforms.slots:
            await route.encode("v1", frame(1, *size))
        assert encoder._codec is not None
        assert encoder._codec.bit_rate == bitrate


async def test_reconfigure_keeps_the_peer_and_emits_an_idr_with_monotonic_pts(harness):
    pc = await harness.offer("r1")
    pc.connect()
    route = harness.video._routes["@t:p1"]
    track = pc.transceivers[0].track
    await harness.encode(1)
    first = await track.recv()
    await harness.encode(2)  # queued old settings must not follow the acknowledgement
    encoder = route.encoders["v1"]
    route.reconfigure(
        "v1",
        normalize_settings(
            PROFILE,
            {
                "maxResolution": [32, 32],
                "maxHz": 15,
                "maxBitrate": 200_000,
            },
        ),
    )
    await harness.encode(3)
    second = await track.recv()
    assert second.is_keyframe
    assert second.pts > first.pts
    decoder = av.CodecContext.create("h264", "r")
    decoded = decoder.decode(second)
    assert (decoded[0].width, decoded[0].height) == (32, 24)
    assert encoder.max_bitrate == 200_000
    assert encoder._codec.bit_rate == 200_000
    assert pc.transceivers[0].track is track
    assert len(harness.answers()) == 1
    assert harness.closes() == []


async def test_a_reconfigure_the_queued_worker_applies_keeps_that_idr(
    harness: Harness, mocker: MockerFixture
) -> None:
    pc = await harness.offer("r1")
    pc.connect()
    route = harness.video._routes["@t:p1"]
    track = pc.transceivers[0].track
    await harness.encode(1)
    await track.recv()
    original = SharedEncoder.encode
    entered, proceed = threading.Event(), threading.Event()

    def held(self: SharedEncoder, payload: bytes) -> list[av.Packet]:
        entered.set()
        assert proceed.wait(5)
        return original(self, payload)

    mocker.patch.object(SharedEncoder, "encode", held)
    encoding = asyncio.create_task(harness.encode(2))
    await asyncio.to_thread(entered.wait, 5)
    # The settings land while the worker is queued, before it applies them:
    # the frame it encodes is the acknowledged configuration's IDR, not a
    # stale one to drop (that left the peer waiting for the safety keyframe).
    route.reconfigure(
        "v1",
        normalize_settings(
            PROFILE, {"maxResolution": [32, 32], "maxHz": 15, "maxBitrate": 200_000}
        ),
    )
    proceed.set()
    await encoding
    async with timeout(2):
        packet = await track.recv()
    assert packet.is_keyframe
    decoded = av.CodecContext.create("h264", "r").decode(packet)
    assert (decoded[0].width, decoded[0].height) == (32, 24)
    assert route.encoders["v1"].frames == 2
