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

"""Safety and mapping tests for G1 Quest joystick locomotion."""

from collections.abc import Iterator

from pydantic import ValidationError
import pytest
import pytest_mock

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule
from dimos.teleop.quest.quest_extensions import VideoArmTeleopModule
from dimos.teleop.quest.quest_types import Buttons, Hand, QuestControllerState


@pytest.fixture
def module() -> Iterator[G1QuestTeleopModule]:
    module = G1QuestTeleopModule()
    try:
        yield module
    finally:
        module.stop()


def _joy(hand: str, *, x: float = 0.0, y: float = 0.0) -> bytes:
    return Joy(
        frame_id=hand,
        axes=[x, y, 0.0, 0.0],
        buttons=[0, 0, 0, 0, 0, 0, 0],
    ).lcm_encode()


def _tick(module: G1QuestTeleopModule) -> None:
    module._publish_button_state(
        module._controllers[Hand.LEFT],
        module._controllers[Hand.RIGHT],
    )


def _arm_drive_gate(module: G1QuestTeleopModule) -> None:
    assert module._on_joy_bytes(_joy("left"))
    assert module._on_joy_bytes(_joy("right"))
    _tick(module)
    assert module.state_snapshot()["drive_ready"] is True


def _last_command(publish: pytest_mock.MockType) -> Twist:
    command = publish.call_args.args[0]
    assert isinstance(command, Twist)
    return command


def test_g1_quest_teleop_preserves_video_arm_behavior(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    assert issubclass(G1QuestTeleopModule, VideoArmTeleopModule)
    assert G1QuestTeleopModule.handle_color_image is VideoArmTeleopModule.handle_color_image

    pose = PoseStamped(frame_id="left", position=[1.0, 2.0, 3.0])
    module._current_poses[Hand.LEFT] = pose
    assert module._get_output_pose(Hand.LEFT) is pose

    buttons: list[Buttons] = []
    module.teleop_buttons.subscribe(buttons.append)
    gripper_publish = mocker.patch.object(module.left_gripper_command, "publish")
    module._is_engaged[Hand.LEFT] = True
    module._publish_button_state(QuestControllerState(primary=True, trigger=0.25), None)

    assert buttons[-1].left_primary is True
    assert buttons[-1].left_trigger_analog == pytest.approx(0.25, abs=0.01)
    assert gripper_publish.call_args.args[0].data == pytest.approx(0.75)


def test_drive_stays_zero_until_both_fresh_controllers_are_neutral(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")

    assert module._on_joy_bytes(_joy("left", x=-1.0))
    assert module._on_joy_bytes(_joy("right"))
    _tick(module)

    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is False

    assert module._on_joy_bytes(_joy("left"))
    _tick(module)

    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is True


def test_drive_maps_forward_lateral_and_yaw_at_control_loop_tick(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    _arm_drive_gate(module)
    publish.reset_mock()
    assert module._on_joy_bytes(_joy("left", x=-0.4, y=-0.5))
    assert module._on_joy_bytes(_joy("right", x=-0.25))

    _tick(module)
    _tick(module)

    assert publish.call_count == 2
    command = _last_command(publish)
    assert command.linear.x == pytest.approx(0.1)
    assert command.linear.y == pytest.approx(0.08)
    assert command.angular.z == pytest.approx(0.125)


def test_control_loop_continuously_publishes_at_configured_period(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    mocker.patch.object(module._stop_event, "is_set", side_effect=[False, False, True])
    wait = mocker.patch.object(module._stop_event, "wait")

    module._control_loop()

    assert publish.call_count == 2
    assert wait.call_count == 2
    for call in wait.call_args_list:
        assert call.args[0] == pytest.approx(1.0 / module.config.control_loop_hz, abs=0.005)


def test_deadzone_is_hard_and_inputs_are_clamped_before_scaling(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    _arm_drive_gate(module)

    assert module._on_joy_bytes(_joy("left", x=-module.config.deadzone, y=module.config.deadzone))
    assert module._on_joy_bytes(_joy("right", x=-module.config.deadzone))
    _tick(module)
    assert _last_command(publish).is_zero()

    assert module._on_joy_bytes(_joy("left", x=2.0, y=-4.0))
    assert module._on_joy_bytes(_joy("right", x=3.0))
    _tick(module)
    command = _last_command(publish)
    assert command.linear.x == pytest.approx(module.config.forward_speed_mps)
    assert command.linear.y == pytest.approx(-module.config.lateral_speed_mps)
    assert command.angular.z == pytest.approx(-module.config.yaw_speed_rad_s)


def test_stale_input_relocks_drive_and_requires_new_neutral_samples(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    _arm_drive_gate(module)
    assert module._on_joy_bytes(_joy("left", y=-1.0))
    _tick(module)
    assert not _last_command(publish).is_zero()

    with module._lock:
        module._last_controller_update[Hand.LEFT] = 1.0
        module._expire_stale_state(1.0 + module.config.input_timeout_s + 0.1)

    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is False

    assert module._on_joy_bytes(_joy("left", y=-1.0))
    assert module._on_joy_bytes(_joy("right"))
    _tick(module)
    assert _last_command(publish).is_zero()


@pytest.mark.parametrize(
    "payload",
    [
        Joy(frame_id="left", axes=[], buttons=[]).lcm_encode(),
        Joy(
            frame_id="left",
            axes=[0.0, float("nan"), 0.0, 0.0],
            buttons=[0, 0, 0, 0, 0, 0, 0],
        ).lcm_encode(),
    ],
)
def test_malformed_or_nonfinite_joy_immediately_zeros_and_relocks(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
    payload: bytes,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    _arm_drive_gate(module)
    assert module._on_joy_bytes(_joy("left", y=-1.0))
    _tick(module)
    assert not _last_command(publish).is_zero()

    assert module._on_joy_bytes(payload) is False

    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is False


def test_unknown_controller_immediately_zeros_and_relocks(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    _arm_drive_gate(module)

    with pytest.raises(ValueError, match="Unexpected frame_id"):
        module._on_joy_bytes(_joy("unknown"))

    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is False


def test_disconnect_and_stop_publish_zero(module: G1QuestTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    client = mocker.MagicMock()
    assert module._client_connected(client)
    _arm_drive_gate(module)

    module._client_disconnected(client)
    assert _last_command(publish).is_zero()
    assert module.state_snapshot()["drive_ready"] is False

    mocker.patch.object(module, "_stop_control_loop")
    mocker.patch.object(module, "_stop_server")
    module.stop()
    assert _last_command(publish).is_zero()


def test_state_snapshot_exposes_drive_inputs_ages_and_command(
    module: G1QuestTeleopModule,
    mocker: pytest_mock.MockerFixture,
) -> None:
    _arm_drive_gate(module)
    assert module._on_joy_bytes(_joy("left", x=-0.4, y=-0.5))
    assert module._on_joy_bytes(_joy("right", x=0.25))
    _tick(module)
    now = max(value for value in module._last_controller_update.values() if value is not None)
    mocker.patch("dimos.robot.unitree.g1.quest_teleop.time.monotonic", return_value=now + 0.1)

    snapshot = module.state_snapshot()

    assert snapshot["drive_ready"] is True
    assert snapshot["left_stick_x"] == pytest.approx(-0.4)
    assert snapshot["left_stick_y"] == -0.5
    assert snapshot["right_stick_x"] == 0.25
    assert snapshot["left_input_age_s"] == pytest.approx(0.1, abs=0.01)
    assert snapshot["right_input_age_s"] == pytest.approx(0.1, abs=0.01)
    assert snapshot["command"] == pytest.approx([0.1, 0.08, -0.125])


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("forward_speed_mps", -0.1),
        ("forward_speed_mps", float("inf")),
        ("lateral_speed_mps", -0.1),
        ("lateral_speed_mps", float("inf")),
        ("yaw_speed_rad_s", float("nan")),
        ("deadzone", 1.0),
    ],
)
def test_drive_configuration_rejects_unsafe_values(field: str, value: float) -> None:
    with pytest.raises(ValidationError):
        G1QuestTeleopModule(**{field: value})
