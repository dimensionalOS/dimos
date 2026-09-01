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

import pytest
import pytest_mock

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.imitation_msgs.EpisodeStatus import EpisodeStatus
from dimos.teleop.quest.quest_extensions import (
    ArmTeleopModule,
    Go2TeleopModule,
    HandTeleopModule,
    MobileVideoArmTeleopModule,
)
from dimos.teleop.quest.quest_teleop_module import QuestTeleopModule, _ws_send_text
from dimos.teleop.quest.quest_types import (
    Buttons,
    Hand,
    QuestControllerState,
    ThumbstickState,
)


@pytest.fixture
def module() -> Iterator[QuestTeleopModule]:
    module = QuestTeleopModule(server_port=9443)
    try:
        yield module
    finally:
        module.stop()


def test_quest_web_server_is_initialized_during_start(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    web_interface = mocker.patch("dimos.teleop.quest.quest_teleop_module.RobotWebInterface")
    setup_routes = mocker.patch.object(module, "_setup_routes")
    start_server = mocker.patch.object(module, "_start_server")
    start_control_loop = mocker.patch.object(module, "_start_control_loop")

    module.start()

    web_interface.assert_called_once_with(host="0.0.0.0", port=9443)
    setup_routes.assert_called_once_with()
    start_server.assert_called_once_with()
    start_control_loop.assert_called_once_with()


def test_build_subscribes_to_episode_status(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    module.status._transport = mocker.MagicMock()
    subscribe = mocker.patch.object(module.status, "subscribe", return_value=mocker.MagicMock())

    module.build()

    subscribe.assert_called_once_with(module._on_episode_status)


def test_unknown_joy_controller_identity_is_rejected(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )

    with pytest.raises(ValueError, match="Unexpected frame_id"):
        module._on_joy_bytes(b"data")


def test_websocket_text_message_is_sent(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
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
    module: QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")
    mocker.patch("dimos.teleop.quest.quest_teleop_module.time.time", return_value=165.5)

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
    module: QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    module._latest_episode_status = _episode_status()
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    payload = json.loads(broadcast.call_args.args[0])
    assert payload["type"] == "episode_status"
    assert payload["episodes_saved"] == 12


def test_connected_client_without_episode_status_does_not_show_collection_hud(
    module: QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    broadcast = mocker.patch.object(module, "_broadcast_text")

    assert module._client_connected(mocker.MagicMock()) is True

    broadcast.assert_not_called()


def test_control_client_disconnect_clears_state(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
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
            module._controllers[hand] = QuestControllerState(primary=True)

    module._client_disconnected(first)

    status = module.get_status()
    assert status.left_engaged is False
    assert status.right_engaged is False
    assert status.left_pose is None
    assert status.right_pose is None
    assert status.buttons.data == 0
    assert published[-1].data == 0


def test_websocket_rejects_additional_control_client(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
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
        code=1008, reason="A Quest control client is already connected"
    )
    ws.receive_bytes.assert_not_awaited()


def test_first_client_connection_rejects_stale_cached_state(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    with module._lock:
        module._is_engaged[Hand.RIGHT] = True
        module._current_poses[Hand.RIGHT] = mocker.MagicMock(spec=PoseStamped)
        module._controllers[Hand.RIGHT] = QuestControllerState(primary=True)

    assert module._client_connected(mocker.MagicMock()) is True

    status = module.get_status()
    assert status.right_engaged is False
    assert status.right_pose is None
    assert status.buttons.data == 0


def test_stale_controller_input_disengages_hand(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    pose = mocker.MagicMock(spec=PoseStamped)
    now = 10.0
    with module._lock:
        module._is_engaged[Hand.RIGHT] = True
        module._initial_poses[Hand.RIGHT] = pose
        module._current_poses[Hand.RIGHT] = pose
        module._controllers[Hand.RIGHT] = QuestControllerState(primary=True)
        module._last_pose_update[Hand.RIGHT] = now
        module._last_controller_update[Hand.RIGHT] = now - module.config.input_timeout_s - 0.1
        module._expire_stale_state(now)

    status = module.get_status()
    assert status.right_engaged is False
    assert status.right_pose is pose
    assert status.buttons.data == 0


def test_stop_publishes_safe_button_state(
    module: QuestTeleopModule, mocker: pytest_mock.MockerFixture
) -> None:
    published: list[Buttons] = []
    module.teleop_buttons.subscribe(published.append)
    module._controllers[Hand.RIGHT] = QuestControllerState(primary=True)
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
            module._controllers[Hand.LEFT] = QuestControllerState(primary=True)
            module._last_controller_update[Hand.LEFT] = 1.0
            module._expire_stale_state(1.0 + module.config.input_timeout_s + 0.1)

        twist = publish.call_args.args[0]
        assert twist.linear.x == 0.0
        assert twist.linear.y == 0.0
        assert twist.angular.z == 0.0
    finally:
        module.stop()


def test_go2_malformed_joy_clears_stale_state_and_publishes_zero_velocity(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="left", axes=[], buttons=[]),
    )
    module._controllers[Hand.LEFT] = QuestControllerState(thumbstick=ThumbstickState(y=-1.0))
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


def test_go2_unknown_controller_identity_publishes_zero_velocity(
    mocker: pytest_mock.MockerFixture,
) -> None:
    module = Go2TeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )
    module._controllers[Hand.LEFT] = QuestControllerState(thumbstick=ThumbstickState(y=-1.0))
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


def test_translation_scale_changes_pose_delta(module: QuestTeleopModule) -> None:
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
    module: QuestTeleopModule, translation_scale: float
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
        left = QuestControllerState(is_left=True, trigger=0.25)
        right = QuestControllerState(is_left=False, trigger=0.75)
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
        module._controllers[Hand.RIGHT] = QuestControllerState(
            is_left=False, primary=True, trigger=1.0
        )

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        assert publish.call_args.args[0].right_trigger_analog == pytest.approx(1.0)

        module._handle_engage()

        assert module._is_engaged[Hand.RIGHT]

        module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=False)
        module._handle_engage()
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert publish.call_args.args[0].right_primary
        module._controllers[Hand.RIGHT] = QuestControllerState(is_left=False, primary=True)
        module._handle_engage()

        assert not module._is_engaged[Hand.RIGHT]
        module._publish_button_state(None, module._controllers[Hand.RIGHT])
        assert not publish.call_args.args[0].right_primary
    finally:
        module.stop()


def _controller(
    *,
    is_left: bool,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    thumbstick_press: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
        is_left=is_left,
        thumbstick_press=thumbstick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def test_mobile_arm_teleop_publishes_yaw_drive_and_one_neutral_stop(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_y=-1.0)
        right = _controller(is_left=False, stick_x=0.5)

        module._publish_cmd_vel(left, right)
        moving = publish.call_args.args[0]
        assert isinstance(moving, Twist)
        assert moving.linear.x == pytest.approx(module.config.linear_scale)
        assert moving.linear.y == 0.0
        assert moving.angular.z == pytest.approx(-0.5 * module.config.yaw_scale)

        idle_left = _controller(is_left=True)
        idle_right = _controller(is_left=False)
        module._publish_cmd_vel(idle_left, idle_right)
        module._publish_cmd_vel(idle_left, idle_right)

        assert publish.call_count == 2
        assert publish.call_args.args[0] == Twist.zero()
    finally:
        module.stop()


def test_mobile_arm_teleop_strafe_mode_and_deadzone(mocker) -> None:
    module = MobileVideoArmTeleopModule(right_stick_mode="strafe")
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_x=0.5, stick_y=0.1)
        right = _controller(is_left=False, stick_x=-0.5)

        module._publish_cmd_vel(left, right)

        moving = publish.call_args.args[0]
        assert moving.linear.x == 0.0
        assert moving.linear.y == pytest.approx(0.5 * module.config.strafe_scale)
        assert moving.angular.z == pytest.approx(-0.5 * module.config.yaw_scale)
    finally:
        module.stop()


def test_mobile_arm_teleop_stick_press_publishes_one_stop_per_press(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    try:
        publish = mocker.patch.object(module.cmd_vel, "publish")
        left = _controller(is_left=True, stick_y=-1.0)
        pressed = _controller(is_left=False, thumbstick_press=True)

        module._publish_cmd_vel(left, pressed)
        module._publish_cmd_vel(left, pressed)

        assert publish.call_count == 1
        assert publish.call_args.args[0] == Twist.zero()
    finally:
        module.stop()


def test_mobile_arm_stale_input_publishes_zero_velocity(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    try:
        with module._lock:
            module._controllers[Hand.LEFT] = _controller(is_left=True, stick_y=-1.0)
            module._last_controller_update[Hand.LEFT] = 1.0
            module._cmd_vel_moving = True
            module._expire_stale_state(1.0 + module.config.input_timeout_s + 0.1)

        publish.assert_called_once_with(Twist.zero())
        assert module._cmd_vel_moving is False
    finally:
        module.stop()


def test_mobile_arm_disconnect_publishes_zero_velocity(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    client = mocker.MagicMock()
    try:
        assert module._client_connected(client) is True
        publish.reset_mock()
        module._cmd_vel_moving = True

        module._client_disconnected(client)

        publish.assert_called_once_with(Twist.zero())
        assert module._cmd_vel_moving is False
    finally:
        module.stop()


def test_mobile_arm_malformed_joy_publishes_zero_velocity(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="left", axes=[], buttons=[]),
    )
    module._cmd_vel_moving = True
    try:
        assert module._on_joy_bytes(b"malformed") is False

        publish.assert_called_once_with(Twist.zero())
        assert module._cmd_vel_moving is False
    finally:
        module.stop()


def test_mobile_arm_unknown_controller_identity_publishes_zero_velocity(mocker) -> None:
    module = MobileVideoArmTeleopModule()
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch(
        "dimos.teleop.quest.quest_teleop_module.Joy.lcm_decode",
        return_value=SimpleNamespace(frame_id="unknown"),
    )
    module._cmd_vel_moving = True
    try:
        with pytest.raises(ValueError, match="Unexpected frame_id"):
            module._on_joy_bytes(b"unknown")

        publish.assert_called_once_with(Twist.zero())
        assert module._cmd_vel_moving is False
    finally:
        module.stop()
