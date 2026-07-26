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


def test_metric_helpers_handle_missing_values_rounding_and_wraparound() -> None:
    assert mvp_cli._elapsed_ms(None, 2.0) is None
    assert mvp_cli._elapsed_ms(1.0, 2.23456) == 1234.56
    assert mvp_cli._rounded_percentile([], 0.5) is None
    assert mvp_cli._rounded_percentile([1.111, 2.222, 3.333], 0.5) == 2.22
    assert mvp_cli._sequence_gaps([10, 11, 14]) == 2
    assert mvp_cli._sequence_gaps([65534, 1]) == 2
