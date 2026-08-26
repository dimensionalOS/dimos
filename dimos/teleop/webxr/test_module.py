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

import asyncio
from collections.abc import Awaitable, Callable, Iterator
import json
from types import SimpleNamespace
from typing import Any

from fastapi.testclient import TestClient
import pytest
import pytest_mock

from dimos.imitation.collection.episode_monitor import EpisodeStatus
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.webxr.body_tracking import BodyTrackingSnapshot
from dimos.teleop.webxr.controller_types import (
    Buttons,
    Hand,
    ThumbstickState,
    WebXRControllerState,
)
from dimos.teleop.webxr.extensions import ArmTeleopModule, Go2TeleopModule, HandTeleopModule
from dimos.teleop.webxr.module import WebXRTeleopModule, _ws_send_text
from dimos.web.robot_web_interface import RobotWebInterface


@pytest.fixture
def module() -> Iterator[WebXRTeleopModule]:
    module = WebXRTeleopModule(server_port=9443)
    try:
        yield module
    finally:
        module.stop()


def test_webxr_web_server_is_initialized_during_start(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    web_interface = mocker.patch("dimos.teleop.webxr.module.RobotWebInterface")
    setup_routes = mocker.patch.object(module, "_setup_routes")
    start_server = mocker.patch.object(module, "_start_server")
    start_control_loop = mocker.patch.object(module, "_start_control_loop")

    module.start()

    web_interface.assert_called_once_with(host="0.0.0.0", port=9443)
    setup_routes.assert_called_once_with()
    start_server.assert_called_once_with()
    start_control_loop.assert_called_once_with()


def test_build_subscribes_to_episode_status(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    module.status._transport = mocker.MagicMock()
    subscribe = mocker.patch.object(module.status, "subscribe", return_value=mocker.MagicMock())

    module.build()

    subscribe.assert_called_once_with(module._on_episode_status)


def test_unknown_joy_controller_identity_is_rejected(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch(
        "dimos.teleop.webxr.module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )

    with pytest.raises(ValueError, match="Unexpected frame_id"):
        module._on_joy_bytes(b"data")


def test_websocket_text_message_is_sent(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    ws = mocker.MagicMock()
    ws.send_text = mocker.AsyncMock()

    assert module._loop is not None
    asyncio.run_coroutine_threadsafe(_ws_send_text(ws, '{"type":"status"}'), module._loop).result(
        timeout=5
    )

    ws.send_text.assert_awaited_once_with('{"type":"status"}')


def _episode_status() -> EpisodeStatus:
    return EpisodeStatus(
        ts=123.0,
        state="recording",
        episodes_saved=12,
        episodes_discarded=1,
        last_event="start",
        task_label="Pick up red mug",
    )


def test_episode_status_is_cached_and_broadcast(
    module: WebXRTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")
    mocker.patch("dimos.teleop.webxr.module.time.time", return_value=165.5)

    module._on_episode_status(_episode_status())

    assert module._latest_episode_status == _episode_status()
    payload = json.loads(broadcast.call_args.args[0])
    assert payload == {
        "type": "episode_status",
        "elapsed_s": 42.5,
        "ts": 123.0,
        "state": "recording",
        "episodes_saved": 12,
        "episodes_discarded": 1,
        "last_event": "start",
        "task_label": "Pick up red mug",
    }


def test_connected_client_receives_latest_episode_status(
    module: WebXRTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    module._latest_episode_status = _episode_status()
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    payload = json.loads(broadcast.call_args.args[0])
    assert payload["type"] == "episode_status"
    assert payload["episodes_saved"] == 12


def test_connected_client_without_episode_status_does_not_show_collection_hud(
    module: WebXRTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    broadcast.assert_not_called()


def test_control_client_disconnect_clears_state(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    first = mocker.MagicMock()
    published: list[Buttons] = []
    module.teleop_buttons.subscribe(published.append)
    pose = mocker.MagicMock(spec=PoseStamped)
    assert module._client_connected(first) is True
    with module._lock:
        for hand in Hand:
            module._is_engaged[hand] = True
            module._initial_poses[hand] = pose
            module._current_poses[hand] = pose
            module._controllers[hand] = WebXRControllerState(primary=True)

    module._client_disconnected(first)

    status = module.get_status()
    assert status.left_engaged is False
    assert status.right_engaged is False
    assert status.left_pose is None
    assert status.right_pose is None
    assert status.buttons.data == 0
    assert published[-1].data == 0


def test_websocket_rejects_additional_control_client(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    endpoint: Callable[[Any], Awaitable[None]] | None = None
    app = mocker.MagicMock()
    app.get.side_effect = lambda *_args, **_kwargs: lambda fn: fn

    def capture_websocket(*_args: Any, **_kwargs: Any) -> Callable[[Any], Any]:
        def decorator(fn: Callable[[Any], Awaitable[None]]) -> Callable[[Any], Awaitable[None]]:
            nonlocal endpoint
            endpoint = fn
            return fn

        return decorator

    app.websocket.side_effect = capture_websocket
    web_server = mocker.MagicMock()
    web_server.app = app
    module._web_server = web_server
    module._setup_routes()
    assert module._client_connected(mocker.MagicMock()) is True
    ws = mocker.MagicMock()
    ws.accept = mocker.AsyncMock()
    ws.close = mocker.AsyncMock()
    ws.receive_bytes = mocker.AsyncMock()

    assert endpoint is not None
    assert module._loop is not None
    asyncio.run_coroutine_threadsafe(endpoint(ws), module._loop).result(timeout=5.0)

    ws.accept.assert_awaited_once_with()
    ws.close.assert_awaited_once_with(
        code=1008, reason="A WebXR control client is already connected"
    )
    ws.receive_bytes.assert_not_awaited()


def test_first_client_connection_rejects_stale_cached_state(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    with module._lock:
        module._is_engaged[Hand.RIGHT] = True
        module._current_poses[Hand.RIGHT] = mocker.MagicMock(spec=PoseStamped)
        module._controllers[Hand.RIGHT] = WebXRControllerState(primary=True)

    assert module._client_connected(mocker.MagicMock()) is True

    status = module.get_status()
    assert status.right_engaged is False
    assert status.right_pose is None
    assert status.buttons.data == 0


def test_stale_controller_input_disengages_hand(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    pose = mocker.MagicMock(spec=PoseStamped)
    now = 10.0
    with module._lock:
        module._is_engaged[Hand.RIGHT] = True
        module._initial_poses[Hand.RIGHT] = pose
        module._current_poses[Hand.RIGHT] = pose
        module._controllers[Hand.RIGHT] = WebXRControllerState(primary=True)
        module._last_pose_update[Hand.RIGHT] = now
        module._last_controller_update[Hand.RIGHT] = now - module.config.input_timeout_s - 0.1
        module._expire_stale_state(now)

    status = module.get_status()
    assert status.right_engaged is False
    assert status.right_pose is pose
    assert status.buttons.data == 0


def test_stop_publishes_safe_button_state(
    module: WebXRTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    published: list[Buttons] = []
    module.teleop_buttons.subscribe(published.append)
    module._controllers[Hand.RIGHT] = WebXRControllerState(primary=True)
    module._is_engaged[Hand.RIGHT] = True
    mocker.patch.object(module, "_stop_control_loop")
    mocker.patch.object(module, "_stop_server")

    module.stop()

    assert module.get_status().right_engaged is False
    assert published[-1].data == 0


def test_go2_stale_input_publishes_zero_velocity(mocker: pytest_mock.MockerFixture) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    try:
        with module._lock:
            module._controllers[Hand.LEFT] = WebXRControllerState(primary=True)
            module._last_controller_update[Hand.LEFT] = 1.0
            module._expire_stale_state(1.0 + module.config.input_timeout_s + 0.1)

        twist = publish.call_args.args[0]
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0
    finally:
        module.stop()


def test_default_webxr_config_does_not_request_body_tracking(
    module: WebXRTeleopModule,
) -> None:
    assert module._webxr_client_config() == {
        "body_tracking_mode": "off",
        "session_modes": ["immersive-ar", "immersive-vr"],
        "session_options": {
            "requiredFeatures": ["local-floor"],
            "optionalFeatures": ["hand-tracking"],
        },
    }


@pytest.mark.parametrize(
    ("mode", "session_modes", "required_features", "optional_features"),
    [
        (
            "optional",
            ["immersive-ar", "immersive-vr"],
            ["local-floor"],
            ["hand-tracking", "bounded-floor", "body-tracking"],
        ),
        (
            "required",
            ["immersive-ar"],
            ["local-floor", "body-tracking"],
            ["hand-tracking", "bounded-floor"],
        ),
    ],
)
def test_enabled_webxr_config_requests_body_tracking(
    mode,
    session_modes,
    required_features,
    optional_features,
) -> None:
    module = WebXRTeleopModule(body_tracking_mode=mode)
    try:
        assert module._webxr_client_config() == {
            "body_tracking_mode": mode,
            "session_modes": session_modes,
            "session_options": {
                "requiredFeatures": required_features,
                "optionalFeatures": optional_features,
            },
        }
    finally:
        module.stop()


def test_webxr_config_route_exposes_body_tracking_mode() -> None:
    module = WebXRTeleopModule(body_tracking_mode="required")
    module._web_server = RobotWebInterface(host="127.0.0.1", port=9443)
    module._setup_routes()

    try:
        with TestClient(module._web_server.app) as client:
            response = client.get("/teleop/config")

        assert response.status_code == 200
        assert response.json() == module._webxr_client_config()
    finally:
        module.stop()


def test_go2_accepts_pico_six_button_joystick(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    joy = Joy(
        ts=1.0,
        frame_id="left",
        axes=[0.25, -0.75, 0.0, 0.0],
        buttons=[0, 0, 0, 0, 0, 0],
    )
    try:
        assert module._on_joy_bytes(joy.lcm_encode()) is True

        twist = publish.call_args.args[0]
        assert twist.linear.x == pytest.approx(0.75 * module.config.linear_speed)
        assert twist.linear.y == pytest.approx(-0.25 * module.config.linear_speed)
        assert twist.angular.z == 0.0
    finally:
        module.stop()


def test_go2_malformed_joy_clears_stale_state_and_publishes_zero_velocity(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.webxr.module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="left", axes=[], buttons=[]),
    )
    module._controllers[Hand.LEFT] = WebXRControllerState(thumbstick=ThumbstickState(y=-1.0))
    try:
        assert module._on_joy_bytes(b"malformed") is False

        assert module._controllers[Hand.LEFT] is None
        publish.assert_called_once()
        twist = publish.call_args.args[0]
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0
    finally:
        module.stop()


def test_webxr_body_reader_is_served_as_javascript() -> None:
    module = WebXRTeleopModule()
    module._web_server = RobotWebInterface(host="127.0.0.1", port=9443)
    module._setup_routes()

    try:
        with TestClient(module._web_server.app) as client:
            response = client.get("/static/webxr_body.mjs")

        assert response.status_code == 200
        assert response.headers["content-type"].startswith("text/javascript")
        assert "export function captureBody" in response.text
    finally:
        module.stop()


def test_go2_unknown_controller_identity_publishes_zero_velocity(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.webxr.module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )
    module._controllers[Hand.LEFT] = WebXRControllerState(thumbstick=ThumbstickState(y=-1.0))
    try:
        with pytest.raises(ValueError, match="Unexpected frame_id"):
            module._on_joy_bytes(b"unknown")

        publish.assert_called_once()
        twist = publish.call_args.args[0]
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0
    finally:
        module.stop()


def test_text_body_tracking_snapshot_is_published(
    module: WebXRTeleopModule,
    mocker,
) -> None:
    publish = mocker.patch.object(module.body_tracking, "publish")
    payload = json.dumps(
        {
            "type": "body_tracking_snapshot",
            "capture_time_s": 3.0,
            "frame_id": "bounded-floor",
            "joints": {
                "hips": {
                    "position": [1.0, 2.0, 3.0],
                    "orientation": [0.0, 0.0, 0.0, 1.0],
                }
            },
        }
    )

    accepted = module._dispatch_text_message(payload)

    assert accepted
    snapshot = publish.call_args.args[0]
    assert isinstance(snapshot, BodyTrackingSnapshot)
    assert snapshot.frame_id == "bounded-floor"
    assert snapshot.joints is not None
    assert snapshot.joints["hips"].position.to_list() == [1.0, 2.0, 3.0]


def test_malformed_text_message_is_dropped(
    module: WebXRTeleopModule,
    mocker,
) -> None:
    publish = mocker.patch.object(module.body_tracking, "publish")

    accepted = module._dispatch_text_message('{"type": "unknown"}')

    assert not accepted
    publish.assert_not_called()


def test_binary_pose_dispatch_remains_on_existing_decoder(
    module: WebXRTeleopModule,
    mocker,
) -> None:
    body_publish = mocker.patch.object(module.body_tracking, "publish")
    pose = PoseStamped(ts=1.0, frame_id="left", position=[1.0, 2.0, 3.0])

    accepted = module._dispatch_binary_message(pose.lcm_encode())

    assert accepted
    assert module._current_poses[Hand.LEFT] is not None
    body_publish.assert_not_called()


def test_translation_scale_changes_pose_delta(module: WebXRTeleopModule) -> None:
    module._initial_poses[Hand.RIGHT] = PoseStamped(position=[1.0, 2.0, 3.0])
    module._current_poses[Hand.RIGHT] = PoseStamped(position=[1.2, 1.5, 4.0])

    module._set_translation_scale(2.0)

    output = module._get_output_pose(Hand.RIGHT)
    assert output is not None
    assert output.position.x == pytest.approx(0.4)
    assert output.position.y == pytest.approx(-1.0)
    assert output.position.z == pytest.approx(2.0)


@pytest.mark.parametrize("translation_scale", [0.0, -1.0, float("inf")])
def test_translation_scale_must_be_positive_and_finite(
    module: WebXRTeleopModule, translation_scale: float
) -> None:
    with pytest.raises(ValueError):
        module._set_translation_scale(translation_scale)

    assert module._translation_scale == 1.0


def test_arm_teleop_publishes_absolute_controller_pose() -> None:
    module = ArmTeleopModule()
    try:
        pose = PoseStamped(frame_id="left", position=[1.0, 2.0, 3.0])
        module._current_poses[Hand.LEFT] = pose
        module._initial_poses[Hand.LEFT] = PoseStamped(position=[0.5, 0.5, 0.5])

        assert module._get_output_pose(Hand.LEFT) is pose
    finally:
        module.stop()


def test_arm_teleop_publishes_normalized_gripper_opening_for_engaged_hand(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = ArmTeleopModule()
    try:
        left_publish = mocker.patch.object(module.left_gripper_command, "publish")
        right_publish = mocker.patch.object(module.right_gripper_command, "publish")
        left = WebXRControllerState(is_left=True, trigger=0.25)
        right = WebXRControllerState(is_left=False, trigger=0.75)
        module._is_engaged[Hand.LEFT] = True

        module._publish_button_state(left, right)

        assert left_publish.call_args.args[0].data == pytest.approx(0.75)
        right_publish.assert_not_called()
    finally:
        module.stop()


def test_hand_teleop_pinch_toggles_engagement(mocker: pytest_mock.MockerFixture) -> None:
    module = HandTeleopModule()
    try:
        publish = mocker.patch.object(module.teleop_buttons, "publish")
        module._current_poses[Hand.RIGHT] = mocker.Mock()
        module._controllers[Hand.RIGHT] = WebXRControllerState(
            is_left=False, primary=True, trigger=1.0
        )

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        assert publish.call_args.args[0].right_trigger_analog == pytest.approx(1.0)

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]

        module._controllers[Hand.RIGHT] = WebXRControllerState(is_left=False, primary=False)
        module._handle_engage()
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        module._controllers[Hand.RIGHT] = WebXRControllerState(is_left=False, primary=True)
        module._handle_engage()

        assert not module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert not publish.call_args.args[0].right_primary
    finally:
        module.stop()
