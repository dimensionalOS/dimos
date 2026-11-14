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
import math
from typing import Any

import pytest

from dimos.msgs.std_msgs.Bool import Bool
from dimos.robot.manipulators.feetech.connection import GUARDED_JOINT_FRAME_ID, FeetechConnection
from dimos.robot.manipulators.feetech.robot import (
    FeetechBus,
    FeetechMotor,
    FeetechRobot,
    FeetechTrigger,
)


class _Leaders(FeetechConnection):
    robot = FeetechRobot(
        buses=(
            FeetechBus(
                "left",
                (
                    FeetechMotor("joint_1", "left_joint1", lower=-1.0, upper=1.0),
                    FeetechMotor("joint_2", "left_joint2", lower=0.0, upper=2.0),
                ),
                trigger=FeetechTrigger("joint_7", travel_radians=1.5),
            ),
            FeetechBus("right", (FeetechMotor("joint_1", "right_joint1", lower=-1.0, upper=1.0),)),
        )
    )

    def serial_ports(self) -> dict[str, str]:
        return {}


def test_a_reading_becomes_clamped_follower_targets_and_a_jaw_opening(mocker: Any) -> None:
    connection = _Leaders(mock_positions={})
    joints = mocker.patch.object(connection.joint_state, "publish")
    left = mocker.patch.object(connection.left_gripper_command, "publish")
    right = mocker.patch.object(connection.right_gripper_command, "publish")
    try:
        # The right leader is not connected; the left trigger is half squeezed.
        connection.publish_reading({"left": {"joint_1": 1.4, "joint_2": -0.3, "joint_7": -0.75}})
    finally:
        connection.stop()

    targets = joints.call_args.args[0]
    assert targets.name == ["arm/left_joint1", "arm/left_joint2"]
    assert targets.position == pytest.approx([1.0, 0.0])
    assert targets.frame_id == GUARDED_JOINT_FRAME_ID
    assert left.call_args.args[0].data == pytest.approx(0.5)
    right.assert_not_called()


def test_the_filter_moves_each_target_a_fraction_toward_the_reading(mocker: Any) -> None:
    connection = _Leaders(mock_positions={}, filter_cutoff_hz=6.0)
    joints = mocker.patch.object(connection.joint_state, "publish")
    try:
        connection.publish_reading({"right": {"joint_1": 0.0}})
        connection.publish_reading({"right": {"joint_1": 0.5}})
    finally:
        connection.stop()

    assert 0.0 < joints.call_args.args[0].position[0] < 0.5


def test_follow_off_holds_the_follower_until_follow_is_on_again(mocker: Any) -> None:
    connection = _Leaders(mock_positions={})
    joints = mocker.patch.object(connection.joint_state, "publish")
    try:
        connection._on_follow(Bool(data=False))
        connection.publish_reading({"right": {"joint_1": 0.5}})
        sent_while_off = [call.args[0] for call in joints.call_args_list]
        connection._on_follow(Bool(data=True))
        connection.publish_reading({"right": {"joint_1": 0.5}})
    finally:
        connection.stop()

    # One hold for every joint (names, zero speed, no positions), then nothing.
    [hold] = sent_while_off
    assert hold.name == ["arm/left_joint1", "arm/left_joint2", "arm/right_joint1"]
    assert hold.position == []
    assert hold.velocity == [0.0, 0.0, 0.0]
    assert joints.call_args.args[0].position == pytest.approx([0.5])


def test_a_joint_whose_range_crosses_half_a_turn_follows_the_leader_across_it() -> None:
    # The OpenArm left shoulder reaches -3.45 rad, past -pi, where the encoder wraps.
    shoulder = FeetechMotor("joint_1", "left_joint1", lower=-3.45, upper=1.35)
    just_inside = shoulder.target(-3.10)
    # Just past -pi the encoder reports +3.10; the target keeps going the same way.
    just_past = shoulder.target(3.10)
    assert just_inside == pytest.approx(-3.10)
    assert just_past == pytest.approx(3.10 - 2 * math.pi)
