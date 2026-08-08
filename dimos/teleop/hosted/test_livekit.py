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
from types import SimpleNamespace
from unittest.mock import AsyncMock, MagicMock

import numpy as np
import pytest

from dimos.core.module import Module
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.hosted.blueprints.livekit import (
    teleop_hosted_go2_livekit,
    teleop_hosted_xarm6_livekit,
    teleop_hosted_xarm7_livekit,
)
from dimos.teleop.hosted.livekit import LiveKitTeleopConfig, LiveKitTeleopModule
from dimos.teleop.hosted.livekit_broker_client import LiveKitSession
from dimos.teleop.hosted.robot_type import RobotType


class LiveKitTestModule(LiveKitTeleopModule):
    state_json: MagicMock
    camera_select: MagicMock
    cmd_raw: MagicMock
    cmd_vel_in: MagicMock
    mux_image: MagicMock
    telemetry_out: MagicMock
    cmd_ack: MagicMock
    map_out: MagicMock


@pytest.fixture
def module(monkeypatch: pytest.MonkeyPatch) -> LiveKitTestModule:
    monkeypatch.setattr(
        Module,
        "__init__",
        lambda self, **kwargs: setattr(self, "config", LiveKitTeleopConfig(**kwargs)),
    )
    result = LiveKitTestModule(
        broker_url="https://broker.example",
        api_key="robot-key",
        robot_type=RobotType.GO2,
    )
    for name in (
        "state_json",
        "camera_select",
        "cmd_raw",
        "cmd_vel_in",
        "mux_image",
        "telemetry_out",
        "cmd_ack",
        "map_out",
    ):
        setattr(result, name, MagicMock())
    return result


def test_state_packets_fan_out_to_command_and_camera_selection(module: LiveKitTestModule) -> None:
    module._on_data(
        "state_reliable", b'{"type":"camera_select"}', SimpleNamespace(identity="op-user")
    )

    module.state_json.publish.assert_called_once_with(b'{"type":"camera_select"}')
    module.camera_select.publish.assert_called_once_with(b'{"type":"camera_select"}')


def test_command_packets_reach_raw_and_typed_command_ports(
    module: LiveKitTestModule, mocker: pytest.MockFixture
) -> None:
    command = MagicMock(spec=TwistStamped)
    mocker.patch.object(TwistStamped, "lcm_decode", return_value=command)

    module._on_data("cmd_unreliable", b"command", SimpleNamespace(identity="op-user"))

    module.cmd_raw.publish.assert_called_once_with(b"command")
    module.cmd_vel_in.publish.assert_called_once_with(command)


def test_non_twist_command_still_reaches_raw_port(
    module: LiveKitTestModule, mocker: pytest.MockFixture
) -> None:
    mocker.patch.object(TwistStamped, "lcm_decode", side_effect=ValueError("not a twist"))

    module._on_data("cmd_unreliable", b"arm-command", SimpleNamespace(identity="op-user"))

    module.cmd_raw.publish.assert_called_once_with(b"arm-command")
    module.cmd_vel_in.publish.assert_not_called()


def test_viewer_packets_are_ignored(module: LiveKitTestModule) -> None:
    module._on_data("cmd_unreliable", b"command", SimpleNamespace(identity="viewer-user-1234"))

    module.cmd_raw.publish.assert_not_called()
    module.cmd_vel_in.publish.assert_not_called()


def test_operator_loss_is_emitted_once(module: LiveKitTestModule) -> None:
    module._operator_present = True

    module._on_operator_lost()
    module._on_operator_lost()

    module.state_json.publish.assert_called_once_with(b'{"type": "operator_lost"}')


def test_broker_session_request_uses_robot_metadata(
    module: LiveKitTestModule, mocker: pytest.MockFixture
) -> None:
    response = MagicMock(status_code=201)
    response.json.return_value = {
        "session_id": "session",
        "url": "wss://livekit.example",
        "token": "jwt",
        "room": "room",
    }
    client = MagicMock()
    client.post = AsyncMock(return_value=response)
    client.__aenter__ = AsyncMock(return_value=client)
    client.__aexit__ = AsyncMock(return_value=None)
    mocker.patch("httpx.AsyncClient", return_value=client)

    session = asyncio.run(
        module._broker.create_session(
            module.config.robot_id, module.config.robot_name, module.config.robot_type
        )
    )

    assert session == LiveKitSession(
        session_id="session", url="wss://livekit.example", token="jwt", room="room"
    )
    assert client.post.await_args.kwargs["json"] == {
        "transport": "livekit",
        "robot_name": "robot",
        "robot_type": "go2",
    }


def test_outbound_data_preserves_topic_reliability(
    module: LiveKitTestModule, mocker: pytest.MockFixture
) -> None:
    room = MagicMock()
    room.local_participant.publish_data = AsyncMock()
    loop = MagicMock()
    loop.is_running.return_value = True
    future = MagicMock()
    submit = mocker.patch("asyncio.run_coroutine_threadsafe", return_value=future)
    module._room = room
    module._loop = loop

    module._publish_data("map_unreliable", b"map", reliable=False)

    coro = submit.call_args.args[0]
    assert submit.call_args.args[1] is loop
    coro.close()
    room.local_participant.publish_data.assert_called_once_with(
        b"map", reliable=False, topic="map_unreliable"
    )
    future.add_done_callback.assert_called_once()


def test_image_conversion_produces_rgba() -> None:
    image = MagicMock(spec=Image)
    image.data = np.array([[[1, 2, 3]]], dtype=np.uint8)
    image.format = ImageFormat.BGR

    width, height, rgba = LiveKitTeleopModule._image_to_rgba(image)

    assert (width, height) == (1, 1)
    assert rgba == bytes([3, 2, 1, 255])


def test_video_frames_are_dropped_until_the_room_connects(module: LiveKitTestModule) -> None:
    module._loop = MagicMock()
    module._loop.is_running.return_value = True

    module._publish_video(MagicMock(spec=Image))

    assert module._loop.call_soon_threadsafe.call_count == 0


def test_livekit_blueprints_are_exposed() -> None:
    assert teleop_hosted_go2_livekit is not None
    assert teleop_hosted_xarm6_livekit is not None
    assert teleop_hosted_xarm7_livekit is not None


def test_connect_registers_data_and_operator_lifecycle(
    module: LiveKitTestModule, mocker: pytest.MockFixture
) -> None:
    room = MagicMock()
    room.connect = AsyncMock()
    room.remote_participants = {}
    handlers: dict[str, object] = {}

    def on(event: str):
        def register(handler: object) -> object:
            handlers[event] = handler
            return handler

        return register

    room.on.side_effect = on
    mocker.patch("livekit.rtc.Room", return_value=room)
    disconnected = asyncio.Event()

    asyncio.run(
        module._connect_room(
            LiveKitSession("session", "wss://livekit.example", "jwt", "room"), disconnected
        )
    )

    assert set(handlers) == {
        "data_received",
        "participant_connected",
        "participant_disconnected",
        "disconnected",
    }
    handlers["data_received"](  # type: ignore[operator]
        SimpleNamespace(
            topic="state_reliable", data=b"state", participant=SimpleNamespace(identity="op-user")
        )
    )
    room.remote_participants = {"operator": SimpleNamespace(identity="op-user")}
    module._operator_present = True
    handlers["participant_disconnected"](SimpleNamespace(identity="op-user"))  # type: ignore[operator]
    handlers["disconnected"](None)  # type: ignore[operator]
    assert disconnected.is_set()
    assert [call.args[0] for call in module.state_json.publish.call_args_list] == [
        b"state",
        b'{"type": "operator_lost"}',
    ]
