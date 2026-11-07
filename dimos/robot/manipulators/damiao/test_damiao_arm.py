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

"""The motor layer on its own: commands, teaching, passive jaws and limits."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest

from dimos.robot.manipulators.damiao._damiao_native_test_helpers import TEST_ROBOT
from dimos.robot.manipulators.damiao.arm import DamiaoArm
from dimos.robot.manipulators.damiao.robot import DamiaoRobot


def _arm(boundary, *, teach: bool = False, robot: DamiaoRobot = TEST_ROBOT) -> DamiaoArm:
    arm = DamiaoArm(robot, ["can1", "can0"], teach=teach)
    arm.connect()
    arm.enable()
    return arm


def test_command_composes_one_tick_with_declared_gains_and_held_targets(boundary, mocker):
    arm = _arm(boundary)
    native = boundary[0]
    arm.command({"joint2": -0.5})
    rows = native.groups["left_arm"].commands[-1]
    np.testing.assert_equal(rows, [[80, 5, 0.2, 0, 0], [10, 1.5, -0.5, 0, 0]])
    assert native.groups["right_arm"].commands[-1][0, 2] == 0.2
    arm.command({"left_gripper": 0.75})
    assert native.groups["left_gripper"].commands[-1] == 0.75


def test_command_rejects_unknown_joints_and_gripper_openings(boundary):
    arm = _arm(boundary)
    with pytest.raises(ValueError, match="unknown Damiao joints"):
        arm.command({"missing": 0.1})
    with pytest.raises(ValueError, match="gripper opening"):
        arm.command({"left_gripper": 1.5})


def test_command_and_hold_are_inert_on_an_unenabled_arm(boundary):
    arm = DamiaoArm(TEST_ROBOT, ["can1", "can0"])
    arm.connect()
    with pytest.raises(RuntimeError, match="inactive"):
        arm.command({"joint1": 0.1})
    arm.hold()
    arm.disable()
    arm.disable()
    assert boundary[0].events.count("disable") == 2


def test_hold_replaces_targets_with_measured_positions(boundary):
    arm = _arm(boundary)
    native = boundary[0]
    arm.command({"joint1": 0.7})
    native.groups["left_arm"].q[:] = [0.3, 0.4]
    arm.hold()
    np.testing.assert_equal(native.groups["left_arm"].commands[-1][:, 2], [0.3, 0.4])


def test_teach_uses_zero_stiffness_teaching_damping_and_measured_gravity(boundary, mocker):
    robot = replace(TEST_ROBOT, teach_kd=[2.0, 2.0, 0.5])
    arm = _arm(boundary, teach=True, robot=robot)
    gravity = mocker.patch.object(arm._physical, "gravity", return_value=np.array([1.0, 2.0, 3.0]))
    arm.command({"joint1": 0.5})
    np.testing.assert_equal(gravity.call_args.args[0], [0.2, 0.2, 0.2])
    left = boundary[0].groups["left_arm"].commands[-1]
    np.testing.assert_equal(left[:, 0], [0.0, 0.0])
    np.testing.assert_equal(left[:, 1], [2.0, 2.0])
    np.testing.assert_equal(left[:, 4], [1.0, 2.0])
    assert boundary[0].groups["right_arm"].commands[-1][0, 1] == 0.5
    assert boundary[0].groups["right_arm"].commands[-1][0, 4] == 3.0


def test_teach_keeps_gravity_fresh_without_a_command(boundary, mocker):
    robot = replace(TEST_ROBOT, teach_kd=[2.0, 2.0, 0.5])
    arm = _arm(boundary, teach=True, robot=robot)
    native = boundary[0]
    native.groups["left_arm"].q[:] = [0.5, -0.5]
    gravity = mocker.patch.object(arm._physical, "gravity", return_value=np.array([4.0, 5.0, 6.0]))
    sent = len(native.groups["left_arm"].commands)
    arm.read()
    assert len(native.groups["left_arm"].commands) == sent + 1
    np.testing.assert_equal(gravity.call_args.args[0], [0.5, -0.5, 0.2])
    np.testing.assert_equal(native.groups["left_arm"].commands[-1][:, 4], [4.0, 5.0])


def test_teach_makes_the_jaw_passive_and_ignores_openings(boundary):
    robot = replace(
        TEST_ROBOT, teach_kd=[2.0, 2.0, 0.5], passive_grippers=frozenset({"left_gripper"})
    )
    arm = _arm(boundary, teach=True, robot=robot)
    gripper = boundary[0].groups["left_gripper"]
    assert gripper.modes == ["mit"]
    assert set(gripper.mit_commands) == {(0.0, 0.0, 0.4, 0.0, 0.0)}
    arm.command({"left_gripper": 2.0})
    assert gripper.commands == []


def test_passive_jaw_stays_positioned_without_teaching(boundary):
    robot = replace(TEST_ROBOT, passive_grippers=frozenset({"left_gripper"}))
    arm = _arm(boundary, robot=robot)
    gripper = boundary[0].groups["left_gripper"]
    assert gripper.modes == []
    arm.command({"left_gripper": 0.25})
    assert gripper.commands[-1] == 0.25
    assert gripper.mit_commands == []


def test_strict_out_of_limits_error(boundary):
    arm = _arm(boundary)
    upper = float(arm._physical._upper[0])
    boundary[0].groups["left_arm"].q[0] = upper + 0.01
    with pytest.raises(RuntimeError, match="outside"):
        arm.read()
    with pytest.raises(RuntimeError, match="outside"):
        arm.hold()


@pytest.mark.parametrize(
    "ports",
    [["can0"], ["can0", "can0"], ["can0", " "], []],
)
def test_can_selectors_must_cover_the_buses_exactly(ports):
    with pytest.raises(ValueError, match="CAN selectors"):
        DamiaoArm(TEST_ROBOT, ports)


def test_teaching_without_damping_is_rejected():
    with pytest.raises(ValueError, match="teach_kd"):
        DamiaoArm(TEST_ROBOT, ["can1", "can0"], teach=True)


def test_unknown_motor_type_fails_before_a_bus_is_opened(boundary):
    bus = replace(
        TEST_ROBOT.buses[0],
        arm=[replace(TEST_ROBOT.buses[0].arm[0], type="XX")],
        kp=[1.0],
        kd=[1.0],
        gripper=None,
    )
    robot = replace(TEST_ROBOT, buses=[bus], joint_names=["joint1"])
    with pytest.raises(ValueError, match="unknown Damiao motor type"):
        DamiaoArm(robot, ["can0"])._build()


def test_missing_targets_and_nonfinite_commands_are_rejected(boundary):
    arm = _arm(boundary)
    arm._targets = {}
    with pytest.raises(RuntimeError, match="no target"):
        arm.command({"joint2": 0.1})
    arm._targets = {
        "joint1": 0.1,
        "joint2": 0.1,
        "joint3": 0.1,
        "left_gripper": float("nan"),
        "right_gripper": 0.0,
    }
    with pytest.raises(ValueError, match="non-finite"):
        arm.command({"joint1": 0.2})


def test_read_before_connect_is_rejected():
    with pytest.raises(RuntimeError, match="disconnected"):
        DamiaoArm(TEST_ROBOT, ["can1", "can0"]).read()
    with pytest.raises(RuntimeError, match="disconnected"):
        DamiaoArm(TEST_ROBOT, ["can1", "can0"]).enable()
