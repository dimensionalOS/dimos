# Copyright 2025-2026 Dimensional Inc.
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

from collections.abc import Iterator

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.g1.quest_teleop import G1QuestTeleopModule
from dimos.teleop.quest.quest_teleop_module import Hand
from dimos.teleop.quest.quest_types import QuestControllerState, ThumbstickState


@pytest.fixture
def module() -> Iterator[G1QuestTeleopModule]:
    module = G1QuestTeleopModule()
    try:
        yield module
    finally:
        module.stop()


def _xr_matrix(x: float, y: float, z: float) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, 3] = [x, y, z]
    return mat


def _controller(
    *,
    is_left: bool,
    stick_x: float = 0.0,
    stick_y: float = 0.0,
    trigger: float = 0.0,
    thumbstick_press: bool = False,
) -> QuestControllerState:
    return QuestControllerState(
        is_left=is_left,
        trigger=trigger,
        thumbstick_press=thumbstick_press,
        thumbstick=ThumbstickState(x=stick_x, y=stick_y),
    )


def test_no_output_pose_until_head_and_hand_tracked(module: G1QuestTeleopModule) -> None:
    assert not module._should_publish(Hand.LEFT)
    module._hand_xr[Hand.LEFT] = _xr_matrix(0.0, 1.0, -0.3)
    assert not module._should_publish(Hand.LEFT)
    module._head_xr = _xr_matrix(0.0, 1.5, 0.0)
    assert module._should_publish(Hand.LEFT)


def test_output_pose_routes_to_dual_arm_task(module: G1QuestTeleopModule) -> None:
    module._head_xr = _xr_matrix(0.0, 1.5, 0.0)
    module._hand_xr[Hand.LEFT] = _xr_matrix(-0.2, 1.1, -0.4)
    module._hand_xr[Hand.RIGHT] = _xr_matrix(0.2, 1.1, -0.4)

    left = module._get_output_pose(Hand.LEFT)
    right = module._get_output_pose(Hand.RIGHT)
    assert left is not None and right is not None
    assert left.frame_id == "dual_arm_ik/left"
    assert right.frame_id == "dual_arm_ik/right"


def test_retarget_anchors_hand_ahead_of_waist(module: G1QuestTeleopModule) -> None:
    # Head 1.5 m up at the XR origin; hand 40 cm in front (XR -Z is forward),
    # 40 cm below the head. In the robot frame the hand-head delta is
    # (0.4 forward, 0, -0.4 up), scaled by workspace_scale and anchored at
    # waist_offset.
    module._head_xr = _xr_matrix(0.0, 1.5, 0.0)
    module._hand_xr[Hand.RIGHT] = _xr_matrix(0.0, 1.1, -0.4)

    pose = module._get_output_pose(Hand.RIGHT)
    assert pose is not None

    scale = module.config.workspace_scale
    waist_x, _waist_y, waist_z = module.config.waist_offset
    assert pose.position.x == pytest.approx(waist_x + 0.4 * scale)
    assert pose.position.y == pytest.approx(module.config.shoulder_y_correction)
    assert pose.position.z == pytest.approx(waist_z - 0.4 * scale)


def test_head_yaw_is_normalized_out(module: G1QuestTeleopModule) -> None:
    # Rotate the whole operator (head + hand) 90° about XR up: the
    # retargeted wrist must be identical to the unrotated case.
    module._head_xr = _xr_matrix(0.0, 1.5, 0.0)
    module._hand_xr[Hand.RIGHT] = _xr_matrix(0.1, 1.1, -0.4)
    baseline = module._get_output_pose(Hand.RIGHT)
    assert baseline is not None

    yaw = np.array(
        [[0, 0, -1, 0], [0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 0, 1]],
        dtype=np.float64,
    )
    module._head_xr = yaw @ module._head_xr
    module._hand_xr[Hand.RIGHT] = yaw @ module._hand_xr[Hand.RIGHT]
    rotated = module._get_output_pose(Hand.RIGHT)
    assert rotated is not None

    assert rotated.position.x == pytest.approx(baseline.position.x, abs=1e-9)
    assert rotated.position.y == pytest.approx(baseline.position.y, abs=1e-9)
    assert rotated.position.z == pytest.approx(baseline.position.z, abs=1e-9)


def test_degenerate_pose_matrix_is_dropped(module: G1QuestTeleopModule) -> None:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion

    module._head_xr = _xr_matrix(0.0, 1.5, 0.0)
    good = module._head_xr

    # NaN orientation (tracking loss) must not replace the last good pose.
    msg = PoseStamped()
    msg.frame_id = "head"
    msg.orientation = Quaternion(float("nan"), 0.0, 0.0, 1.0)
    module._on_pose_bytes(msg.lcm_encode())

    assert module._head_xr is good


def test_thumbsticks_map_to_cmd_vel(module: G1QuestTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    left = _controller(is_left=True, stick_y=-1.0)
    right = _controller(is_left=False, stick_x=0.5)

    module._publish_cmd_vel(left, right)

    msg = publish.call_args.args[0]
    assert isinstance(msg, Twist)
    assert msg.linear.x == pytest.approx(module.config.linear_scale)
    assert msg.angular.z == pytest.approx(-0.5 * module.config.yaw_scale)


def test_deadzone_publishes_single_stop_not_zero_spam(module: G1QuestTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    left_moving = _controller(is_left=True, stick_y=-1.0)
    left_idle = _controller(is_left=True)
    right_idle = _controller(is_left=False)

    module._publish_cmd_vel(left_moving, right_idle)
    module._publish_cmd_vel(left_idle, right_idle)
    module._publish_cmd_vel(left_idle, right_idle)
    module._publish_cmd_vel(left_idle, right_idle)

    # One moving Twist + exactly one zero Twist on the transition.
    assert publish.call_count == 2
    stop = publish.call_args.args[0]
    assert stop.linear.x == 0.0 and stop.angular.z == 0.0


def test_right_stick_press_is_estop(module: G1QuestTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.cmd_vel, "publish")
    left_moving = _controller(is_left=True, stick_y=-1.0)
    right_idle = _controller(is_left=False)
    right_pressed = _controller(is_left=False, thumbstick_press=True)

    module._publish_cmd_vel(left_moving, right_idle)
    module._publish_cmd_vel(left_moving, right_pressed)

    assert publish.call_count == 2
    stop = publish.call_args.args[0]
    assert stop.linear.x == 0.0 and stop.angular.z == 0.0


def test_buttons_carry_analog_triggers(module: G1QuestTeleopModule, mocker) -> None:
    publish = mocker.patch.object(module.teleop_buttons, "publish")
    mocker.patch.object(module.cmd_vel, "publish")
    left = _controller(is_left=True, trigger=1.0)
    right = _controller(is_left=False, trigger=0.75)

    module._publish_button_state(left, right)

    buttons = publish.call_args.args[0]
    assert buttons.left_trigger_analog == pytest.approx(1.0, abs=0.01)
    assert buttons.right_trigger_analog == pytest.approx(0.75, abs=0.01)
