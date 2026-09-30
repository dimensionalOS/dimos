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

"""Tests that the presets are a shortcut and nothing more.

The first three tests are the important ones. ``conftest.py`` writes out three
descriptions by hand -- an arm, a humanoid body, a base -- and each test
rebuilds one through its preset and checks they match exactly.

The rest cover what each preset decides: which limits an arm keeps, that a
gripper and an orientation sensor are added as asked, and that a base is one
part whose speed limits clamp.
"""

from __future__ import annotations

import pickle

import pytest

from dimos.control.contract.conftest import ARM_JOINTS, G1_JOINTS
from dimos.control.contract.description import ControlDescription, Limits, ResourceKind
from dimos.control.contract.keys import EFFORT, KD, KP, POSITION, VELOCITY, VX, VY, WZ, Key, Unit
from dimos.control.contract.presets import (
    GripperSpec,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.control.contract.validate import (
    CommandBatch,
    DescriptionError,
    Rejected,
    validate_command,
    validate_description,
)
from dimos.msgs.control_msgs.ControlValues import ControlValues

ARM_LIMITS = {Key.of("arm", j, POSITION): Limits(-3.14, 3.14) for j in ARM_JOINTS} | {
    Key.of("arm", j, VELOCITY): Limits(-1.0, 1.0) for j in ARM_JOINTS
}
G1_LIMITS = {Key.of("g1", j, POSITION): Limits(-2.0, 2.0) for j in G1_JOINTS}
CHASSIS_LIMITS = {
    Key.of("chassis", "base", VX): Limits(-1.5, 1.5),
    Key.of("chassis", "base", VY): Limits(-1.0, 1.0),
    Key.of("chassis", "base", WZ): Limits(-2.0, 2.0),
}


def preset_arm() -> ControlDescription:
    """The conftest arm, built through the preset."""
    return manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        state=(POSITION, EFFORT),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
    )


def preset_g1() -> ControlDescription:
    """The conftest humanoid, built through the preset."""
    return pd_joint_description("g1", G1_JOINTS, limits=G1_LIMITS, imu="imu")


def preset_chassis() -> ControlDescription:
    """The conftest base, built through the preset."""
    return twist_base_description("chassis", limits=CHASSIS_LIMITS)


def command(values: dict[str, float]) -> ControlValues:
    return ControlValues(
        source="coordinator",
        epoch=0,
        sequence=1,
        interface_names=list(values),
        values=list(values.values()),
    )


def test_manipulator_preset_rebuilds_the_hand_written_arm(xarm: ControlDescription) -> None:
    assert preset_arm() == xarm


def test_pd_joint_preset_rebuilds_the_hand_written_g1(g1: ControlDescription) -> None:
    assert preset_g1() == g1


def test_twist_base_preset_rebuilds_the_hand_written_chassis(chassis: ControlDescription) -> None:
    assert preset_chassis() == chassis


@pytest.mark.parametrize("build", [preset_arm, preset_g1, preset_chassis])
def test_every_preset_output_passes_the_description_check(build) -> None:
    validate_description(build())


@pytest.mark.parametrize("build", [preset_arm, preset_g1, preset_chassis])
def test_every_preset_output_survives_a_pickle(build) -> None:
    # Descriptions are sent between processes.
    described = build()
    assert pickle.loads(pickle.dumps(described)) == described


def test_an_arm_is_told_position_and_velocity_unless_it_says_otherwise() -> None:
    both = manipulator_description("arm", ARM_JOINTS, limits={})
    assert both.resource("joint1").command_interfaces == (POSITION, VELOCITY)
    only = manipulator_description("arm", ARM_JOINTS, limits={}, command=(POSITION,))
    assert only.resource("joint1").command_interfaces == (POSITION,)


def test_an_arm_leaves_out_limits_on_what_it_is_not_told() -> None:
    # A robot model gives effort limits too, but this arm is never told an
    # effort, so there is nothing for that limit to apply to.
    limits = ARM_LIMITS | {Key.of("arm", "joint1", EFFORT): Limits(-40.0, 40.0)}
    described = manipulator_description("arm", ARM_JOINTS, limits=limits)
    assert set(described.limits) == set(ARM_LIMITS)


def test_a_limit_for_a_joint_the_arm_does_not_have_is_an_error() -> None:
    limits = {Key.of("arm", "joint99", POSITION): Limits(-1.0, 1.0)}
    with pytest.raises(DescriptionError, match="joint99"):
        manipulator_description("arm", ARM_JOINTS, limits=limits)


def test_an_arm_refuses_a_command_past_a_limit() -> None:
    refused = validate_command(
        preset_arm(), command({"arm/joint1/position": 9.0}), last_sequence=None
    )
    assert isinstance(refused, Rejected) and refused.reason == "limit"


def test_a_gripper_keeps_its_own_unit_and_range() -> None:
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits={},
        gripper=GripperSpec(name="hand", unit=Unit.NORMALIZED, lo=0.0, hi=1.0),
    )
    assert described.unit_of("arm/hand/position") is Unit.NORMALIZED
    assert described.limits["arm/hand/position"] == Limits(0.0, 1.0)
    assert "arm/hand" in described.joint_names()


def test_an_arm_with_no_joints_or_nothing_to_command_raises() -> None:
    with pytest.raises(ValueError, match="no joints"):
        manipulator_description("arm", (), limits={})
    with pytest.raises(ValueError, match="nothing to command"):
        manipulator_description("arm", ARM_JOINTS, limits={}, command=())


def test_an_interface_the_preset_has_no_unit_for_raises() -> None:
    with pytest.raises(ValueError, match="no preset unit"):
        manipulator_description("arm", ARM_JOINTS, limits={}, state=(POSITION, "temperature"))


def test_a_body_is_told_stiffness_and_damping_with_every_target() -> None:
    described = pd_joint_description("g1", G1_JOINTS, limits={})
    assert described.resource("joint1").command_interfaces == (POSITION, VELOCITY, EFFORT, KP, KD)
    # No orientation sensor unless asked for.
    assert [r.kind for r in described.resources] == [ResourceKind.JOINT] * len(G1_JOINTS)


def test_an_orientation_sensor_only_reports() -> None:
    described = pd_joint_description("g1", G1_JOINTS, limits={}, imu="chest_imu")
    imu = described.resource("chest_imu")
    assert imu.kind is ResourceKind.SENSOR and imu.command_interfaces == ()
    assert "g1/chest_imu/qw" in described.state_keys()
    assert "g1/chest_imu" not in described.joint_names()


def test_a_body_with_no_joints_raises() -> None:
    with pytest.raises(ValueError, match="no joints"):
        pd_joint_description("g1", (), limits={})


def test_a_base_is_one_part_told_three_speeds() -> None:
    described = preset_chassis()
    [base] = described.resources
    assert base.name == "base" and base.kind is ResourceKind.BASE
    assert base.command_interfaces == (VX, VY, WZ)
    assert described.joint_names() == ()


def test_a_base_clamps_every_limit_it_is_given() -> None:
    described = preset_chassis()
    assert set(described.limits) == set(CHASSIS_LIMITS)
    assert all(limit.clamp for limit in described.limits.values())


def test_a_base_told_to_go_too_fast_goes_at_its_top_speed() -> None:
    accepted = validate_command(
        preset_chassis(),
        command({"chassis/base/vx": 9.0, "chassis/base/vy": 0.0, "chassis/base/wz": -9.0}),
        last_sequence=None,
    )
    assert accepted == CommandBatch(
        values={"chassis/base/vx": 1.5, "chassis/base/vy": 0.0, "chassis/base/wz": -2.0},
        clamped=("chassis/base/vx", "chassis/base/wz"),
    )


def test_a_base_that_cannot_move_sideways_limits_that_speed_to_zero() -> None:
    limits = CHASSIS_LIMITS | {Key.of("chassis", "base", VY): Limits(0.0, 0.0)}
    described = twist_base_description("chassis", limits=limits)
    accepted = validate_command(described, command({"chassis/base/vy": 0.5}), last_sequence=None)
    assert accepted == CommandBatch(values={"chassis/base/vy": 0.0}, clamped=("chassis/base/vy",))


def test_a_base_limit_needs_both_bounds() -> None:
    with pytest.raises(DescriptionError, match="not bounded on both sides"):
        twist_base_description("chassis", limits={Key.of("chassis", "base", VX): Limits(hi=1.5)})


def test_a_base_reports_only_what_it_can() -> None:
    no_pose = twist_base_description("chassis", limits={}, odometry=False)
    assert no_pose.resources[0].state_interfaces == (VX, VY, WZ)
    # One that can only repeat back what it was told does not claim to measure.
    no_speed = twist_base_description("chassis", limits={}, measured_velocity=False)
    assert no_speed.resources[0].state_interfaces == ("x", "y", "yaw")
