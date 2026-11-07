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

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.connection import R1PRO_WHOLE_BODY_JOINTS, R1ProConnection
from dimos.robot.galaxea.r1pro.spatial_control import R1SpatialControl, height_step
from dimos.teleop.webxr.controller_types import Buttons


@pytest.fixture
def control(mocker):
    module = R1SpatialControl()
    module._limits = np.array([[-3.1, 3.1]] * 18)
    for port in (module.motor_command, module.hardware_cmd_vel, module.joint_state):
        mocker.patch.object(port, "publish")
    clock = mocker.patch(
        "dimos.robot.galaxea.r1pro.spatial_control.time.monotonic", return_value=10.0
    )
    mocker.patch("dimos.robot.galaxea.r1pro.spatial_control.time.time", return_value=1000.0)
    yield module, clock
    module.stop()


def feedback(arm: float = 0.4) -> JointState:
    return JointState(
        name=R1PRO_WHOLE_BODY_JOINTS, position=[0] * 4 + [arm] * 14 + [85, 60], ts=1000
    )


async def test_startup_never_homes_or_opens_grippers(control):
    module, _ = control
    await module.handle_motor_states(feedback())
    module.tick(10)
    module.motor_command.publish.assert_not_called()
    # If the robot moved before first engagement, capture its current posture.
    await module.handle_motor_states(feedback(0.6))
    await module.handle_teleop_buttons(Buttons(data=1 << Buttons.BITS["left_grip"]))
    module.tick(10)
    np.testing.assert_allclose(
        module.motor_command.publish.call_args.args[0].q, feedback(0.6).position
    )


async def test_arm_rate_limit_release_and_stale_ik(control):
    module, clock = control
    await module.handle_motor_states(feedback())
    await module.handle_teleop_buttons(Buttons(data=1 << Buttons.BITS["left_grip"]))
    await module.handle_upper_command(JointState(name=[R1PRO_WHOLE_BODY_JOINTS[4]], position=[1]))
    module.tick(10)
    assert module.motor_command.publish.call_args.args[0].q[4] == pytest.approx(0.405)
    # A still-held grip cannot keep moving toward an old IK target after the IK worker dies.
    clock.return_value = 10.2
    await module.handle_motor_states(feedback())
    clock.return_value = 10.31
    await module.handle_motor_states(feedback())
    await module.handle_teleop_buttons(Buttons(data=1 << Buttons.BITS["left_grip"]))
    module.tick(10.31)
    assert module.motor_command.publish.call_args.args[0].q[4] == pytest.approx(0.4)
    await module.handle_upper_command(JointState(name=[R1PRO_WHOLE_BODY_JOINTS[4]], position=[1]))
    await module.handle_teleop_buttons(Buttons())
    module.tick(10.31)
    assert module.motor_command.publish.call_args.args[0].q[4] == pytest.approx(0.4)


async def test_hands_use_vendor_percentage_without_replacing_other_targets(control):
    module, _ = control
    await module.handle_motor_states(feedback())
    await module.handle_left_gripper_command(Float32(data=0.2))
    await module.handle_right_gripper_command(Float32(data=float("nan")))
    module.tick(10)
    assert list(module.motor_command.publish.call_args.args[0].q[-2:]) == pytest.approx([20, 60])
    assert list(module.motor_command.publish.call_args.args[0].q[:18]) == pytest.approx(
        feedback().position[:18]
    )


async def test_feedback_loss_stops_drive_and_recovery_drops_old_targets(control):
    module, clock = control
    await module.handle_motor_states(feedback())
    await module.handle_cmd_vel(Twist(linear=Vector3(0.15, 0, 0)))
    await module.handle_left_gripper_command(Float32(data=0))
    module.tick(10)
    assert module.hardware_cmd_vel.publish.call_args.args[0].linear.x == 0.15
    module.motor_command.publish.reset_mock()
    module.tick(10.4)
    assert module.hardware_cmd_vel.publish.call_args.args[0].linear.x == 0
    module.motor_command.publish.assert_not_called()
    clock.return_value = 10.5
    await module.handle_motor_states(feedback(0.8))
    module.tick(10.5)
    module.motor_command.publish.assert_not_called()
    assert module.hardware_cmd_vel.publish.call_args.args[0].linear.x == 0


def torso_pose(q):
    return (
        0.35 * (math.sin(q[0]) + math.sin(q[0] + q[1])),
        0.35 * (math.cos(q[0]) + math.cos(q[0] + q[1])),
        q[0] + q[1] - q[2],
        q[3],
    )


@pytest.mark.parametrize("axis", [-1, 1])
def test_torso_vertical_step_preserves_reach_pitch_and_yaw(axis):
    q = np.array([0.4, -0.8, -0.4, 0.3])
    initial = torso_pose(q)
    for _ in range(20):
        next_q = height_step(q, axis, 0.02, np.array([[-3, 3]] * 4))
        assert np.max(np.abs(next_q - q)) <= 0.15 * 0.02 + 1e-12
        q = next_q
    final = torso_pose(q)
    assert (final[1] - initial[1]) * axis > 0
    np.testing.assert_allclose(
        [final[0], final[2], final[3]], [initial[0], initial[2], initial[3]], atol=1e-8
    )


def test_gripper_bridge_holds_measured_opening_until_changed():
    connection = R1ProConnection(enable_grippers=True, boot_vendor_stack=False)
    try:
        connection._latest_gripper_q = [101.1, 50]
        assert connection._gripper_targets([100, 50]) == []
        assert connection._gripper_targets([-2, 200]) == [(0, 0), (1, 100)]
        assert connection._gripper_targets([0, 100]) == []
        assert connection._gripper_targets([float("nan"), 100]) == []
    finally:
        connection.stop()
