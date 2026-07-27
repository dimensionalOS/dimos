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

from __future__ import annotations

import asyncio
import json
import sys
from types import ModuleType, SimpleNamespace
from typing import Any

import pytest
import typer

from dimos.protocol.pubsub.impl.webrtc import mvp_cli


def test_required_accepts_a_value_and_rejects_missing_input() -> None:
    assert mvp_cli._required("secret", "TOKEN") == "secret"

    with pytest.raises(typer.BadParameter, match="TOKEN is required"):
        mvp_cli._required(None, "TOKEN")


def test_operator_api_sends_auth_and_returns_an_object(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    received: dict[str, Any] = {}

    class Response:
        content = b'{"session_id":"session-1"}'

        def raise_for_status(self) -> None:
            received["status_checked"] = True

        def json(self) -> dict[str, str]:
            return {"session_id": "session-1"}

    def request(method: str, url: str, **kwargs: Any) -> Response:
        received.update(method=method, url=url, **kwargs)
        return Response()

    monkeypatch.setattr(mvp_cli.requests, "request", request)

    result = mvp_cli._operator_api(
        "https://broker.example/",
        "operator-secret",
        "POST",
        "/sessions/demo/join",
        {"role": "operator"},
    )

    assert result == {"session_id": "session-1"}
    assert received == {
        "method": "POST",
        "url": "https://broker.example/api/v1/sessions/demo/join",
        "json": {"role": "operator"},
        "headers": {
            "Authorization": "Bearer operator-secret",
            "Content-Type": "application/json",
        },
        "timeout": 30.0,
        "status_checked": True,
    }


def test_operator_api_handles_empty_and_rejects_non_object_json(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Response:
        def __init__(self, content: bytes, data: Any) -> None:
            self.content = content
            self.data = data

        def raise_for_status(self) -> None:
            return None

        def json(self) -> Any:
            return self.data

    monkeypatch.setattr(
        mvp_cli.requests,
        "request",
        lambda *args, **kwargs: Response(b"", ["ignored"]),
    )
    assert mvp_cli._operator_api("https://broker.example", "token", "GET", "/empty") == {}

    monkeypatch.setattr(
        mvp_cli.requests,
        "request",
        lambda *args, **kwargs: Response(b"[]", []),
    )
    with pytest.raises(RuntimeError, match="non-object JSON"):
        mvp_cli._operator_api("https://broker.example", "token", "GET", "/invalid")


def test_ice_gathering_success_completes_without_waiting() -> None:
    class PeerConnection:
        iceGatheringState = "new"  # noqa: N815

        async def setLocalDescription(self, description: object) -> None:
            assert description == "offer"
            self.iceGatheringState = "complete"

    asyncio.run(
        mvp_cli._set_local_description_with_ice_timeout(
            PeerConnection(),
            "offer",
            timeout=0.1,
        )
    )


def test_ice_gathering_loop_times_out_when_state_never_completes() -> None:
    class PeerConnection:
        iceGatheringState = "gathering"  # noqa: N815

        async def setLocalDescription(self, description: object) -> None:
            del description

    with pytest.raises(RuntimeError, match="ICE gathering did not complete"):
        asyncio.run(
            mvp_cli._set_local_description_with_ice_timeout(
                PeerConnection(),
                object(),
                timeout=0.001,
            )
        )


def test_local_command_emits_loopback_result(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    received: dict[str, Any] = {}

    async def loopback(**kwargs: Any) -> dict[str, Any]:
        received.update(kwargs)
        return {"mode": "local-loopback", "frames_decoded": 12}

    monkeypatch.setattr(mvp_cli, "_local_loopback", loopback)

    mvp_cli.local(duration=1.5, fps=20.0, width=800, height=600)

    assert received == {
        "duration": 1.5,
        "fps": 20.0,
        "width": 800,
        "height": 600,
    }
    assert json.loads(capsys.readouterr().out) == {
        "frames_decoded": 12,
        "mode": "local-loopback",
    }


def test_subscribe_command_forwards_credentials_and_emits_metrics(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    received: dict[str, Any] = {}

    async def subscribe(**kwargs: Any) -> dict[str, Any]:
        received.update(kwargs)
        return {"session_id": kwargs["session_id"], "frames_decoded": 7}

    monkeypatch.setattr(mvp_cli, "_subscribe", subscribe)

    mvp_cli.subscribe(
        session_id="session-7",
        operator_token="operator-token",
        broker_url="https://broker.example",
        duration=3.0,
    )

    assert received == {
        "session_id": "session-7",
        "operator_token": "operator-token",
        "broker_url": "https://broker.example",
        "duration": 3.0,
    }
    assert json.loads(capsys.readouterr().out) == {
        "frames_decoded": 7,
        "session_id": "session-7",
    }


def test_publish_sends_a_stamped_frame_and_stops(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    class Provider:
        session_id = "session-42"

        def __init__(self) -> None:
            self.started = False
            self.stopped = False
            self.frames: list[mvp_cli._SyntheticImage] = []

        def start(self) -> None:
            self.started = True

        def stop(self) -> None:
            self.stopped = True

        def set_video_frame(self, frame: mvp_cli._SyntheticImage) -> None:
            self.frames.append(frame)

    provider = Provider()
    received_config: dict[str, Any] = {}

    class Config:
        def provider(self) -> Provider:
            return provider

    def publisher_config(**kwargs: Any) -> Config:
        received_config.update(kwargs)
        return Config()

    clock = {"now": 0.0}

    def sleep(seconds: float) -> None:
        clock["now"] += seconds

    monkeypatch.setattr(mvp_cli, "_publisher_config", publisher_config)
    monkeypatch.setattr(
        mvp_cli,
        "make_stamped_frame",
        lambda **kwargs: {"stamp": kwargs},
    )
    monkeypatch.setattr(mvp_cli.time, "perf_counter", lambda: 10.0)
    monkeypatch.setattr(mvp_cli.time, "monotonic", lambda: clock["now"])
    monkeypatch.setattr(mvp_cli.time, "sleep", sleep)

    mvp_cli.publish(
        api_key="robot-key",
        broker_url="https://broker.example",
        robot_name="go2-test",
        duration=0.05,
        fps=10.0,
        width=640,
        height=360,
    )

    assert received_config == {
        "broker_url": "https://broker.example",
        "api_key": "robot-key",
        "robot_name": "go2-test",
    }
    assert provider.started is True
    assert provider.stopped is True
    assert len(provider.frames) == 1
    assert provider.frames[0].format == "bgr"
    assert provider.frames[0].data == {"stamp": {"width": 640, "height": 360, "sequence": 0}}
    output = [json.loads(line) for line in capsys.readouterr().out.splitlines()]
    assert output == [
        {
            "broker_url": "https://broker.example",
            "codec": "h264",
            "connect_ms": 0.0,
            "event": "ready",
            "fps": 10.0,
            "session_id": "session-42",
        },
        {"event": "stopped", "frames_sent": 1},
    ]


def test_publish_stops_when_the_broker_returns_no_session(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Provider:
        session_id = None

        def __init__(self) -> None:
            self.stopped = False

        def start(self) -> None:
            return None

        def stop(self) -> None:
            self.stopped = True

    provider = Provider()
    monkeypatch.setattr(
        mvp_cli,
        "_publisher_config",
        lambda **kwargs: type("Config", (), {"provider": lambda self: provider})(),
    )

    with pytest.raises(RuntimeError, match="did not return a session id"):
        mvp_cli.publish(api_key="robot-key", duration=1.0)

    assert provider.stopped is True


def test_publish_stops_when_the_provider_has_no_video(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class Provider:
        session_id = "session-no-video"

        def __init__(self) -> None:
            self.stopped = False

        def start(self) -> None:
            return None

        def stop(self) -> None:
            self.stopped = True

    provider = Provider()
    monkeypatch.setattr(
        mvp_cli,
        "_publisher_config",
        lambda **kwargs: type("Config", (), {"provider": lambda self: provider})(),
    )

    with pytest.raises(RuntimeError, match="does not support video"):
        mvp_cli.publish(
            api_key="robot-key",
            broker_url="https://broker.example",
            robot_name="go2-test",
            duration=1.0,
            fps=15.0,
            width=640,
            height=360,
        )

    assert provider.stopped is True


def test_subscribe_negotiates_video_and_reports_metrics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    class IceServer:
        def __init__(
            self,
            *,
            urls: str | list[str],
            username: str | None = None,
            credential: str | None = None,
        ) -> None:
            self.urls = urls
            self.username = username
            self.credential = credential

    class SessionDescription:
        def __init__(self, *, sdp: str, type: str) -> None:
            self.sdp = sdp
            self.type = type

    class Frame:
        def to_ndarray(self, *, format: str) -> str:
            assert format == "bgr24"
            return "pixels"

    class Track:
        kind = "video"

        def __init__(self) -> None:
            self.delivered = False
            self.blocked = asyncio.Event()

        async def recv(self) -> Frame:
            if not self.delivered:
                self.delivered = True
                return Frame()
            await self.blocked.wait()
            raise AssertionError("blocked receive should be cancelled")

    class PeerConnection:
        iceGatheringState = "complete"  # noqa: N815

        def __init__(self, config: Any) -> None:
            assert len(config.iceServers) == 1
            self.handlers: dict[str, Any] = {}
            self.localDescription = SessionDescription(sdp="local-sdp", type="offer")
            self.closed = False

        def createDataChannel(self, *args: Any, **kwargs: Any) -> None:
            assert args == ("_sctp_init",)
            assert kwargs == {"negotiated": True, "id": 0}

        def on(self, event: str) -> Any:
            def register(callback: Any) -> Any:
                self.handlers[event] = callback
                return callback

            return register

        async def createOffer(self) -> SessionDescription:
            return SessionDescription(sdp="offer", type="offer")

        async def createAnswer(self) -> SessionDescription:
            return SessionDescription(sdp="answer", type="answer")

        async def setLocalDescription(self, description: SessionDescription) -> None:
            self.localDescription = SessionDescription(sdp="local-sdp", type=description.type)

        async def setRemoteDescription(self, description: SessionDescription) -> None:
            if description.type == "offer":
                self.handlers["track"](Track())

        async def getStats(self) -> dict[str, Any]:
            return {
                "inbound-rtp-video": SimpleNamespace(
                    kind="video",
                    packetsReceived=11,
                    packetsLost=2,
                )
            }

        async def close(self) -> None:
            self.closed = True

    class Configuration:
        def __init__(self, *, iceServers: list[IceServer]) -> None:
            self.iceServers = iceServers

    fake_aiortc = ModuleType("aiortc")
    fake_aiortc.RTCConfiguration = Configuration  # type: ignore[attr-defined]
    fake_aiortc.RTCIceServer = IceServer  # type: ignore[attr-defined]
    fake_aiortc.RTCPeerConnection = PeerConnection  # type: ignore[attr-defined]
    fake_aiortc.RTCSessionDescription = SessionDescription  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "aiortc", fake_aiortc)

    from dimos.protocol.pubsub.impl.webrtc.providers import spec

    async def wait_connected(pc: Any, *, timeout: float) -> None:
        assert isinstance(pc, PeerConnection)
        assert timeout == 20.0

    monkeypatch.setattr(spec, "wait_connected", wait_connected)
    monkeypatch.setattr(
        mvp_cli,
        "decode_frame_stamp",
        lambda frame: SimpleNamespace(
            timestamp_ms=mvp_cli.time.time_ns() // 1_000_000 - 5,
            sequence=7,
        ),
    )
    calls: list[tuple[str, str]] = []

    def operator_api(
        broker_url: str,
        operator_token: str,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        assert broker_url == "https://broker.example"
        assert operator_token == "operator-token"
        calls.append((method, path))
        if path.endswith("turn-credentials"):
            return {
                "ice_servers": [
                    {
                        "urls": ["turn:turn.example"],
                        "username": "user",
                        "credential": "secret",
                    }
                ]
            }
        if path.endswith("/join"):
            return {"sdp_answer": "answer-sdp"}
        if path.endswith("/bridge-datachannel"):
            return {"video_offer": "video-offer"}
        return {}

    monkeypatch.setattr(mvp_cli, "_operator_api", operator_api)

    result = asyncio.run(
        mvp_cli._subscribe(
            session_id="session-1",
            operator_token="operator-token",
            broker_url="https://broker.example",
            duration=0.0,
        )
    )

    assert result["session_id"] == "session-1"
    assert result["turn_configured"] is True
    assert result["frames_decoded"] == 1
    assert result["valid_timestamps"] == 1
    assert result["invalid_timestamps"] == 0
    assert result["latency_ms_p50"] == 5.0
    assert result["packets_received"] == 11
    assert result["packets_lost"] == 2
    assert calls == [
        ("GET", "/sessions/turn-credentials"),
        ("POST", "/sessions/session-1/join"),
        ("POST", "/sessions/session-1/bridge-datachannel"),
        ("POST", "/sessions/session-1/renegotiate-answer"),
        ("POST", "/sessions/session-1/leave"),
    ]


def test_local_loopback_negotiates_h264_and_reports_metrics(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    shared: dict[str, Any] = {"connections": []}

    class Configuration:
        def __init__(self, *, iceServers: list[Any]) -> None:
            assert iceServers == []

    class CameraTrack:
        def __init__(self, loop: asyncio.AbstractEventLoop) -> None:
            assert loop is asyncio.get_running_loop()
            self.latest: mvp_cli._SyntheticImage | None = None
            self.ready = asyncio.Event()
            self.armed = False

        def arm(self) -> None:
            self.armed = True

        def set_latest(self, image: mvp_cli._SyntheticImage) -> None:
            assert self.armed is True
            self.latest = image
            self.ready.set()

    class Frame:
        def __init__(self, image: mvp_cli._SyntheticImage) -> None:
            self.image = image

        def to_ndarray(self, *, format: str) -> Any:
            assert format == "bgr24"
            return self.image.data

    class RemoteTrack:
        def __init__(self, track: CameraTrack) -> None:
            self.track = track

        async def recv(self) -> Frame:
            await self.track.ready.wait()
            self.track.ready.clear()
            assert self.track.latest is not None
            return Frame(self.track.latest)

    class Transceiver:
        kind = "video"

        def __init__(self) -> None:
            self.codecs: list[Any] = []

        def setCodecPreferences(self, codecs: list[Any]) -> None:
            self.codecs = codecs

    class PeerConnection:
        def __init__(self, config: Configuration) -> None:
            self.role = "sender" if not shared["connections"] else "receiver"
            shared["connections"].append(self)
            self.handlers: dict[str, Any] = {}
            self.transceiver = Transceiver()
            self.localDescription = SimpleNamespace(sdp="", type="")
            self.track: CameraTrack | None = None
            self.closed = False

        def addTrack(self, track: CameraTrack) -> None:
            self.track = track
            shared["track"] = track

        def getTransceivers(self) -> list[Transceiver]:
            return [self.transceiver]

        def on(self, event: str) -> Any:
            def register(callback: Any) -> Any:
                self.handlers[event] = callback
                return callback

            return register

        async def createOffer(self) -> Any:
            return SimpleNamespace(sdp="offer", type="offer")

        async def createAnswer(self) -> Any:
            return SimpleNamespace(sdp="answer", type="answer")

        async def setLocalDescription(self, description: Any) -> None:
            self.localDescription = description

        async def setRemoteDescription(self, description: Any) -> None:
            if self.role == "receiver" and "track" in self.handlers:
                self.handlers["track"](RemoteTrack(shared["track"]))

        async def getStats(self) -> dict[str, Any]:
            return {
                "inbound-rtp-video": SimpleNamespace(
                    kind="video",
                    packetsReceived=9,
                    packetsLost=1,
                )
            }

        async def close(self) -> None:
            self.closed = True

    class SenderCapabilities:
        @staticmethod
        def getCapabilities(kind: str) -> Any:
            assert kind == "video"
            return SimpleNamespace(
                codecs=[
                    SimpleNamespace(mimeType="video/VP8"),
                    SimpleNamespace(mimeType="video/H264"),
                ]
            )

    fake_aiortc = ModuleType("aiortc")
    fake_aiortc.RTCConfiguration = Configuration  # type: ignore[attr-defined]
    fake_aiortc.RTCPeerConnection = PeerConnection  # type: ignore[attr-defined]
    fake_aiortc.RTCRtpSender = SenderCapabilities  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "aiortc", fake_aiortc)

    fake_video_track = ModuleType("dimos.protocol.pubsub.impl.webrtc.providers.video_track")
    fake_video_track.CameraVideoTrack = CameraTrack  # type: ignore[attr-defined]
    monkeypatch.setitem(
        sys.modules,
        "dimos.protocol.pubsub.impl.webrtc.providers.video_track",
        fake_video_track,
    )

    from dimos.protocol.pubsub.impl.webrtc.providers import spec

    async def wait_connected(pc: Any, *, timeout: float) -> None:
        assert isinstance(pc, PeerConnection)
        assert timeout == 10.0

    monkeypatch.setattr(spec, "wait_connected", wait_connected)
    monkeypatch.setattr(
        mvp_cli,
        "decode_frame_stamp",
        lambda frame: SimpleNamespace(
            timestamp_ms=mvp_cli.time.time_ns() // 1_000_000,
            sequence=3,
        ),
    )

    result = asyncio.run(
        mvp_cli._local_loopback(
            duration=0.0,
            fps=30.0,
            width=640,
            height=360,
        )
    )

    assert result["mode"] == "local-loopback"
    assert result["codec"] == "h264"
    assert result["frames_decoded"] == 1
    assert result["valid_timestamps"] == 1
    assert result["packets_received"] == 9
    assert result["packets_lost"] == 1
    assert [codec.mimeType for codec in shared["connections"][0].transceiver.codecs] == [
        "video/H264"
    ]
    assert all(connection.closed for connection in shared["connections"])


def test_metric_helpers_handle_missing_values_rounding_and_wraparound() -> None:
    assert mvp_cli._elapsed_ms(None, 2.0) is None
    assert mvp_cli._elapsed_ms(1.0, 2.23456) == 1234.56
    assert mvp_cli._rounded_percentile([], 0.5) is None
    assert mvp_cli._rounded_percentile([1.111, 2.222, 3.333], 0.5) == 2.22
    assert mvp_cli._sequence_gaps([10, 11, 14]) == 2
    assert mvp_cli._sequence_gaps([65534, 1]) == 2


def test_main_invokes_the_webrtc_app(monkeypatch: pytest.MonkeyPatch) -> None:
    called = False

    def fake_app() -> None:
        nonlocal called
        called = True

    monkeypatch.setattr(mvp_cli, "webrtc_mvp_app", fake_app)
    mvp_cli.main()

    assert called
