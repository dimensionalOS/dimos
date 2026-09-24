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
rebuilds one through its preset and checks they match exactly. If a preset
ever starts making a decision of its own, one of these fails.

The rest cover what the presets add on top: grippers, spreading one gain
across every joint, bases that can only move some ways, and the promise that
an invalid description can never come out of one.
"""

from __future__ import annotations

import pickle

import pytest

from dimos.control.contract.conftest import ARM_JOINTS, G1_JOINTS
from dimos.control.contract.description import (
    ActivationPolicy,
    ControlDescription,
    Estop,
    EstopKind,
    EstopRecovery,
    LimitPolicy,
    Limits,
    Omission,
    ProcessLoss,
    ResourceKind,
    SafeStop,
    SafeStopKind,
    Timing,
)
from dimos.control.contract.keys import (
    AX,
    AZ,
    EFFORT,
    GX,
    KD,
    KP,
    PITCH,
    POSITION,
    QW,
    ROLL,
    VELOCITY,
    VX,
    VY,
    VZ,
    WX,
    WY,
    WZ,
    YAW,
    Unit,
    X,
    Y,
    Z,
    make_key,
)
from dimos.control.contract.presets import (
    GripperSpec,
    ImuSpec,
    manipulator_description,
    pd_joint_description,
    twist_base_description,
)
from dimos.control.contract.validate import DescriptionError, validate_description

ARM_LIMITS = {make_key("arm", j, POSITION): Limits(-3.14, 3.14) for j in ARM_JOINTS} | {
    make_key("arm", j, VELOCITY): Limits(-1.0, 1.0) for j in ARM_JOINTS
}
G1_LIMITS = {make_key("g1", j, POSITION): Limits(-2.0, 2.0, LimitPolicy.CLAMP) for j in G1_JOINTS}
CHASSIS_LIMITS = {
    make_key("chassis", "base", VX): Limits(-1.5, 1.5),
    make_key("chassis", "base", VY): Limits(-1.0, 1.0),
    make_key("chassis", "base", WZ): Limits(-2.0, 2.0),
}


def preset_arm() -> ControlDescription:
    """The conftest xArm, built through the preset."""
    return manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        state=(POSITION, EFFORT),
        groups=(frozenset({POSITION}), frozenset({VELOCITY})),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
        safe_stop=SafeStop(kind=SafeStopKind.HOLD, stable_state="holds position"),
        estop=Estop(
            kind=EstopKind.VENDOR, recovery=EstopRecovery.CLEAR, stable_state="brakes engage"
        ),
    )


def preset_g1() -> ControlDescription:
    """The conftest G1, built through the preset."""
    return pd_joint_description(
        "g1",
        G1_JOINTS,
        limits=G1_LIMITS,
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        omission={make_key("g1", j, VELOCITY): Omission.UNSET for j in G1_JOINTS},
        safe_stop=SafeStop(kind=SafeStopKind.DAMP, stable_state="sinks to the floor"),
        estop=Estop(
            kind=EstopKind.DISABLE,
            recovery=EstopRecovery.PREPARE_ARM_REQUIRED,
            stable_state="limp",
        ),
    )


def preset_chassis() -> ControlDescription:
    """The conftest chassis, built through the preset."""
    return twist_base_description(
        "chassis",
        limits=CHASSIS_LIMITS,
        safe_stop=SafeStop(kind=SafeStopKind.ZERO_RAMP, ramp_s=0.3, stable_state="rolls to a stop"),
        estop=Estop(kind=EstopKind.ZERO, recovery=EstopRecovery.CLEAR, stable_state="stops dead"),
        timing=Timing(
            state_rate_hz=50.0,
            stale_timeout_s=0.2,
            watchdog_timeout_s=0.2,
            write_rate_hz=50.0,
        ),
        process_loss=ProcessLoss.EXTERNAL_SUPERVISOR,
        meta={"command_frame": "body", "yaw_convention": "unwrapped"},
    )


def test_manipulator_preset_rebuilds_the_hand_written_arm(xarm: ControlDescription) -> None:
    assert preset_arm() == xarm


def test_pd_joint_preset_rebuilds_the_hand_written_g1(g1: ControlDescription) -> None:
    assert preset_g1() == g1


def test_twist_base_preset_rebuilds_the_hand_written_chassis(
    chassis: ControlDescription,
) -> None:
    assert preset_chassis() == chassis


@pytest.mark.parametrize("build", [preset_arm, preset_g1, preset_chassis])
def test_every_preset_output_validates(build) -> None:
    # They must: each preset validates before returning. This is the guard
    # against somebody deleting that call.
    validate_description(build())


@pytest.mark.parametrize("build", [preset_arm, preset_g1, preset_chassis])
def test_every_preset_output_survives_a_pickle(build) -> None:
    # Descriptions cross the RPC boundary to reach the coordinator.
    described = build()
    assert pickle.loads(pickle.dumps(described)) == described


def test_group_names_come_from_the_interfaces_they_drive() -> None:
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        groups=(frozenset({POSITION, VELOCITY}),),
    )
    assert [group.name for group in described.mode_groups] == ["position+velocity"]


def test_joints_command_the_union_of_their_groups() -> None:
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        groups=(frozenset({POSITION}), frozenset({VELOCITY})),
    )
    joint = described.resource("joint1")
    assert joint is not None
    # Canonical order, not alphabetical: position before velocity.
    assert joint.command_interfaces == (POSITION, VELOCITY)


def test_gripper_lands_in_its_own_non_exclusive_group() -> None:
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        groups=(frozenset({POSITION}), frozenset({VELOCITY})),
        gripper=GripperSpec(unit=Unit.M, lo=0.0, hi=0.085),
    )
    groups = {group.name: group for group in described.mode_groups}
    assert groups["gripper"].resources == ("gripper",)
    assert groups["gripper"].exclusive is False
    # The arm groups stay exclusive of each other and do not reach the gripper.
    assert groups["position"].exclusive is True
    assert "gripper" not in groups["position"].resources


def test_gripper_keeps_its_own_unit_and_limit_policy() -> None:
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        gripper=GripperSpec(
            name="hand", unit=Unit.NORMALIZED, lo=0.0, hi=1.0, policy=LimitPolicy.CLAMP
        ),
    )
    # A Damiao gripper speaks normalized, not metres, and says so here (D19).
    assert described.unit_of("arm/hand/position") is Unit.NORMALIZED
    assert described.limits["arm/hand/position"] == Limits(0.0, 1.0, LimitPolicy.CLAMP)


def test_caller_limits_are_passed_through_untouched() -> None:
    described = manipulator_description("arm", ARM_JOINTS, limits=ARM_LIMITS)
    for key, limit in ARM_LIMITS.items():
        assert described.limits[key] == limit


def test_omission_overrides_reach_the_description() -> None:
    # How an xArm or R1Pro opts a key into UNSET (D11).
    override = {make_key("arm", j, VELOCITY): Omission.UNSET for j in ARM_JOINTS}
    described = manipulator_description(
        "arm",
        ARM_JOINTS,
        limits=ARM_LIMITS,
        groups=(frozenset({POSITION}), frozenset({VELOCITY})),
        omission=override,
    )
    assert described.omission_of("arm/joint1/velocity") is Omission.UNSET
    # Everything unmentioned keeps the default.
    assert described.omission_of("arm/joint1/position") is Omission.RETAIN_LAST


def test_a_manipulator_with_no_joints_raises() -> None:
    with pytest.raises(ValueError, match="no joints"):
        manipulator_description("arm", (), limits={})


def test_a_manipulator_with_no_groups_raises() -> None:
    with pytest.raises(ValueError, match="no mode groups"):
        manipulator_description("arm", ARM_JOINTS, limits=ARM_LIMITS, groups=())


def test_overlapping_groups_cannot_leave_a_preset() -> None:
    # position would be in two groups, so it is ambiguous which one commanding
    # it selects. The preset validates, so this never reaches a caller.
    with pytest.raises(DescriptionError, match="several mode groups"):
        manipulator_description(
            "arm",
            ARM_JOINTS,
            limits=ARM_LIMITS,
            groups=(frozenset({POSITION}), frozenset({POSITION, VELOCITY})),
        )


def test_an_interface_with_no_preset_unit_raises() -> None:
    with pytest.raises(ValueError, match="no preset unit"):
        manipulator_description(
            "arm", ARM_JOINTS, limits=ARM_LIMITS, state=(POSITION, "temperature")
        )


def test_a_float_gain_broadcasts_to_every_joint() -> None:
    described = pd_joint_description(
        "g1", G1_JOINTS, limits=G1_LIMITS, kp=60.0, kd=1.5, damp_kd=5.0
    )
    assert all(described.initial_values[make_key("g1", j, KP)] == 60.0 for j in G1_JOINTS)
    assert all(described.initial_values[make_key("g1", j, KD)] == 1.5 for j in G1_JOINTS)


def test_a_gain_table_is_read_per_joint() -> None:
    kp = {joint: float(i) for i, joint in enumerate(G1_JOINTS)}
    described = pd_joint_description("g1", G1_JOINTS, limits=G1_LIMITS, kp=kp, kd=1.5, damp_kd=5.0)
    assert described.initial_values[make_key("g1", G1_JOINTS[3], KP)] == 3.0


def test_a_partial_gain_table_raises() -> None:
    # A joint left out of the table would be given no stiffness, so it would
    # hang slack while every other joint held its position.
    partial = {joint: 60.0 for joint in G1_JOINTS[:-1]}
    with pytest.raises(ValueError, match=r"kp does not cover joint\(s\)"):
        pd_joint_description("g1", G1_JOINTS, limits=G1_LIMITS, kp=partial, kd=1.5, damp_kd=5.0)


def test_a_gain_table_naming_an_unknown_joint_raises() -> None:
    stray = {joint: 60.0 for joint in G1_JOINTS} | {"typo_joint": 60.0}
    with pytest.raises(ValueError, match=r"kp names undeclared joint\(s\)"):
        pd_joint_description("g1", G1_JOINTS, limits=G1_LIMITS, kp=stray, kd=1.5, damp_kd=5.0)


def test_a_partial_damp_table_raises() -> None:
    partial = {joint: 5.0 for joint in G1_JOINTS[:-1]}
    with pytest.raises(ValueError, match=r"damp_kd does not cover joint\(s\)"):
        pd_joint_description("g1", G1_JOINTS, limits=G1_LIMITS, kp=60.0, kd=1.5, damp_kd=partial)


def test_the_damping_safe_stop_is_built_from_damp_kd() -> None:
    described = pd_joint_description(
        "g1", G1_JOINTS, limits=G1_LIMITS, kp=60.0, kd=1.5, damp_kd=5.0
    )
    assert described.safe_stop.kind is SafeStopKind.DAMP
    assert described.safe_stop.kd == {make_key("g1", j, KD): 5.0 for j in G1_JOINTS}


def test_an_explicit_damp_stop_keeps_its_own_gains() -> None:
    mine = {make_key("g1", j, KD): 9.0 for j in G1_JOINTS}
    described = pd_joint_description(
        "g1",
        G1_JOINTS,
        limits=G1_LIMITS,
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        safe_stop=SafeStop(kind=SafeStopKind.DAMP, kd=mine),
    )
    assert described.safe_stop.kd == mine


def test_a_non_damping_safe_stop_is_left_alone() -> None:
    described = pd_joint_description(
        "g1",
        G1_JOINTS,
        limits=G1_LIMITS,
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        safe_stop=SafeStop(kind=SafeStopKind.HOLD),
    )
    assert described.safe_stop == SafeStop(kind=SafeStopKind.HOLD)


def test_an_imu_is_a_state_only_sensor() -> None:
    described = pd_joint_description(
        "g1",
        G1_JOINTS,
        limits=G1_LIMITS,
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        imu=ImuSpec(frame_id="g1_pelvis"),
    )
    imu = described.resource("imu")
    assert imu is not None
    assert imu.kind is ResourceKind.SENSOR
    assert imu.command_interfaces == ()
    assert "g1/imu/qw" in described.state_keys()
    assert described.unit_of(f"g1/imu/{QW}") is Unit.UNITLESS
    assert described.unit_of(f"g1/imu/{GX}") is Unit.RAD_PER_S
    assert described.unit_of(f"g1/imu/{AX}") is Unit.M_PER_S2
    assert described.unit_of(f"g1/imu/{AZ}") is Unit.M_PER_S2
    assert described.meta["imu_frame_id"] == "g1_pelvis"
    # A sensor is never claimed, so it is not a joint and not in the pd group.
    assert "g1/imu" not in described.joint_names()
    assert described.groups_for("imu") == ()


def test_imu_meta_joins_the_caller_meta() -> None:
    described = pd_joint_description(
        "g1",
        G1_JOINTS,
        limits=G1_LIMITS,
        kp=60.0,
        kd=1.5,
        damp_kd=5.0,
        imu=ImuSpec(frame_id="g1_pelvis"),
        meta={"vendor": "unitree"},
    )
    assert described.meta == {"vendor": "unitree", "imu_frame_id": "g1_pelvis"}


def test_a_pd_body_with_no_joints_raises() -> None:
    with pytest.raises(ValueError, match="no joints"):
        pd_joint_description("g1", (), limits={}, kp=1.0, kd=1.0, damp_kd=1.0)


def test_a_six_dof_base_declares_the_full_pose() -> None:
    axes = (VX, VY, VZ, WX, WY, WZ)
    described = twist_base_description(
        "drone",
        axes=axes,
        limits={make_key("drone", "base", axis): Limits(-1.0, 1.0) for axis in axes},
    )
    base = described.resource("base")
    assert base is not None
    assert base.command_interfaces == axes
    assert base.state_interfaces == (*axes, X, Y, Z, ROLL, PITCH, YAW)
    assert described.unit_of("drone/base/vz") is Unit.M_PER_S
    assert described.unit_of("drone/base/wx") is Unit.RAD_PER_S
    assert described.unit_of("drone/base/z") is Unit.M
    assert described.unit_of("drone/base/roll") is Unit.RAD
    assert described.unit_of("drone/base/pitch") is Unit.RAD


def test_a_differential_base_reaches_the_whole_plane() -> None:
    # It drives forward and turns, so it cannot strafe -- but driving forward
    # while turning traces an arc, which reaches world y just the same. A pose
    # term per commanded axis would declare only x and then reject an honest
    # odometry frame carrying y.
    described = twist_base_description(
        "diff",
        axes=(VX, WZ),
        limits={make_key("diff", "base", VX): Limits(-1.0, 1.0)},
    )
    base = described.resource("base")
    assert base is not None
    assert base.command_interfaces == (VX, WZ)
    assert base.state_interfaces == (VX, WZ, X, Y, YAW)
    assert described.unit_of("diff/base/y") is Unit.M
    # Reaching world y is not the same as having a lateral velocity to report:
    # vy stays undeclared rather than published as a fabricated zero.
    assert VY not in base.state_interfaces
    assert VY not in base.command_interfaces


def test_a_base_that_cannot_turn_keeps_its_heading() -> None:
    # A gantry: two linear axes, no rotation, so no yaw to report.
    described = twist_base_description(
        "gantry",
        axes=(VX, VY),
        limits={make_key("gantry", "base", VX): Limits(-1.0, 1.0)},
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (VX, VY, X, Y)


def test_a_single_linear_axis_is_a_rail() -> None:
    described = twist_base_description(
        "rail",
        axes=(VX,),
        limits={make_key("rail", "base", VX): Limits(-1.0, 1.0)},
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (VX, X)


def test_a_base_that_only_turns_reports_only_its_heading() -> None:
    # A turret: it spins in place and never goes anywhere.
    described = twist_base_description(
        "turret",
        axes=(WZ,),
        limits={make_key("turret", "base", WZ): Limits(-1.0, 1.0)},
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (WZ, YAW)


def test_two_rotations_compose_to_reach_every_orientation() -> None:
    # Roll and pitch generate yaw between them, and each rotation opens the
    # linear plane perpendicular to it, so this reaches the full pose.
    described = twist_base_description(
        "gimbal",
        axes=(VX, WX, WY),
        limits={make_key("gimbal", "base", VX): Limits(-1.0, 1.0)},
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (VX, WX, WY, X, Y, Z, ROLL, PITCH, YAW)


def test_a_base_without_odometry_reports_only_its_twist() -> None:
    described = twist_base_description(
        "diff",
        axes=(VX, WZ),
        limits={make_key("diff", "base", VX): Limits(-1.0, 1.0)},
        odometry=False,
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (VX, WZ)


def test_a_base_that_cannot_measure_reports_only_its_pose() -> None:
    # FlowBase echoes the command back today; that is not a measurement, so it
    # is not declared as one.
    described = twist_base_description(
        "flow",
        axes=(VX, WZ),
        limits={make_key("flow", "base", VX): Limits(-1.0, 1.0)},
        measured_velocity=False,
    )
    base = described.resource("base")
    assert base is not None
    assert base.state_interfaces == (X, Y, YAW)


def test_an_axis_outside_the_six_raises() -> None:
    with pytest.raises(ValueError, match="non-twist axes"):
        twist_base_description("base", axes=(VX, POSITION), limits={})


def test_a_repeated_axis_raises() -> None:
    with pytest.raises(ValueError, match="repeats axes"):
        twist_base_description("base", axes=(VX, VX), limits={})


def test_a_base_with_no_axes_raises() -> None:
    with pytest.raises(ValueError, match="no axes"):
        twist_base_description("base", axes=(), limits={})


def test_an_unclaimed_base_axis_falls_to_zero() -> None:
    # D17: no retain-last on a base. A task that wants a hold keeps claiming.
    described = preset_chassis()
    assert described.omission_of("chassis/base/vx") is Omission.ZERO


def test_a_base_is_one_resource_and_not_a_set_of_virtual_joints() -> None:
    described = preset_chassis()
    assert [resource.name for resource in described.resources] == ["base"]
    assert described.resources[0].kind is ResourceKind.BASE
    assert described.joint_names() == ()


def test_the_base_activation_policy_defaults_to_direct() -> None:
    described = preset_chassis()
    assert described.activation_policy is ActivationPolicy.DIRECT
