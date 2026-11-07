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
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.assets.model import LoadedRobotModel
from dimos.robot.galaxea.r1pro.sim.config import JOINTS, SERVO_JOINTS, UPPER_JOINTS
from dimos.robot.galaxea.r1pro.sim.control import R1ProSimControl, steer_targets
from dimos.robot.galaxea.r1pro.sim.model import (
    FINGER_JOINTS,
    FINGER_SPEED_M_S,
    FINGER_TRAVEL_M,
    STEER_JOINTS,
    WHEEL_RADIUS,
)


@pytest.mark.parametrize(
    "vx,vy,expected_angle,expected_speed",
    [(0.3, 0, 0, 0.3), (-0.3, 0, 0, -0.3), (0, 0.3, math.pi / 2, 0.3)],
)
def test_translation_rolls_in_requested_direction(vx, vy, expected_angle, expected_speed):
    angle, speed = steer_targets(vx, vy, 0, 0.17, 0.28, expected_angle)
    assert angle == pytest.approx(expected_angle)
    assert speed * WHEEL_RADIUS == pytest.approx(expected_speed)


def test_rotation_points_each_wheel_along_its_local_tangent():
    for x, y in ((0.17, 0.28), (0.17, -0.28), (-0.33, 0)):
        tangent = math.atan2(0.4 * x, -0.4 * y)
        angle, speed = steer_targets(0, 0, 0.4, x, y, tangent)
        np.testing.assert_allclose(
            [math.cos(angle) * speed * WHEEL_RADIUS, math.sin(angle) * speed * WHEEL_RADIUS],
            [-0.4 * y, 0.4 * x],
            atol=1e-12,
        )


def test_unaligned_wheel_waits_for_steering():
    _, speed = steer_targets(0, 0.3, 0, 0.17, 0.28, 0)
    assert abs(speed) < 1e-12


@pytest.fixture
def controller():
    root = ET.Element("robot", name="controller_fixture")
    ET.SubElement(root, "link", name="base")
    for index, name in enumerate((*SERVO_JOINTS, *STEER_JOINTS)):
        child = f"link{index}"
        ET.SubElement(root, "link", name=child)
        joint = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(joint, "parent", link="base")
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz="0.17 0.28 0")
        ET.SubElement(joint, "axis", xyz="0 0 1")
        ET.SubElement(joint, "limit", lower="-3.14", upper="3.14", effort="30", velocity="2")
    model = LoadedRobotModel(ET.tostring(root, encoding="unicode"), None, {})
    module = R1ProSimControl()
    module.config_receive([("test", {"sim_clock_id": "test", "robot_models": {"r1pro": model}})])
    yield module
    module.dispose()


def test_deadman_brakes_and_duplicate_samples_keep_original_action(controller, mocker):
    clock = mocker.patch("dimos.robot.galaxea.r1pro.sim.control.time.monotonic", return_value=0)
    controller.move(Twist(linear=Vector3(0.3, 0, 0)), duration=0.3)

    def sample(t):
        return JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=t)

    first = controller.compute(sample(0))
    assert first is not None
    assert controller.compute(sample(0)) is first
    for i in range(1, 50):
        action = controller.compute(sample(i * 0.02))
    assert action.joint_commands[1].values[-1] > 0
    clock.return_value = 1
    for i in range(50, 100):
        action = controller.compute(sample(i * 0.02))
    assert action.joint_commands[1].values[-3:] == [0, 0, 0]
    assert controller.compute(sample(0.5)) is None
    assert action.header.clock_id == "test"
    assert action.valid_until_ns - action.apply_at_ns == 20_000_000


def test_upper_body_targets_are_bounded_and_rate_limited(controller):
    with pytest.raises(ValueError, match="outside model limits"):
        controller.set_joint(UPPER_JOINTS[4], 4)
    controller.set_joint(UPPER_JOINTS[4], -1)
    action = controller.compute(JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=0))
    assert action.joint_commands[0].values[4] == pytest.approx(-0.01)
    assert action.joint_commands[0].values[:4] == [0, 0, 0, 0]


@pytest.mark.parametrize("measured,expected", [(3.15, 3.14), (-3.15, -3.14)])
def test_stopped_steering_keeps_commands_legal_after_soft_stop_overshoot(
    controller, measured, expected
):
    positions = [measured if name in STEER_JOINTS else 0.0 for name in JOINTS]
    action = controller.compute(JointState(name=list(JOINTS), position=positions, ts=0))
    assert action.joint_commands[0].values[-6:-3] == [expected] * 3
    assert action.joint_commands[1].values[-3:] == [0.0] * 3


async def test_arm_teleop_does_not_recapture_torso_sag(controller, mocker):
    mocker.patch.object(controller.config, "upper_command_joints", UPPER_JOINTS[4:])
    await controller.handle_upper_command(
        JointState(name=list(UPPER_JOINTS), position=[0.2] * len(UPPER_JOINTS))
    )
    action = controller.compute(JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=0))
    assert action.joint_commands[0].values[:4] == [0.0] * 4
    assert action.joint_commands[0].values[4:18] == [0.01] * 14


async def test_grippers_move_opposite_fingers_independently_and_ignore_arm_holds(controller):
    await controller.handle_left_gripper_command(Float32(data=1))
    await controller.handle_right_gripper_command(Float32(data=0.5))
    # A coordinator's passive joint hold must never overwrite the gripper target.
    await controller.handle_upper_command(JointState(name=list(FINGER_JOINTS), position=[0] * 4))
    first = controller.compute(JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=0))
    assert first.joint_commands[0].values[18:22] == pytest.approx(
        [FINGER_SPEED_M_S * 0.02, -FINGER_SPEED_M_S * 0.02] * 2
    )
    for i in range(1, 60):
        action = controller.compute(
            JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=i * 0.02)
        )
    assert action.joint_commands[0].values[18:22] == pytest.approx(
        [FINGER_TRAVEL_M, -FINGER_TRAVEL_M, FINGER_TRAVEL_M / 2, -FINGER_TRAVEL_M / 2]
    )
    assert action.joint_commands[0].values[:18] == [0] * 18
    await controller.handle_left_gripper_command(Float32(data=0))
    for i in range(60, 120):
        action = controller.compute(
            JointState(name=list(JOINTS), position=[0.0] * len(JOINTS), ts=i * 0.02)
        )
    assert action.joint_commands[0].values[18:22] == pytest.approx(
        [0, 0, FINGER_TRAVEL_M / 2, -FINGER_TRAVEL_M / 2]
    )


@pytest.mark.parametrize("opening", [float("nan"), -0.1, 1.1])
async def test_invalid_gripper_command_is_rejected(controller, opening):
    with pytest.raises(ValueError, match="gripper opening"):
        await controller.handle_left_gripper_command(Float32(data=opening))
